// Event-driven logic analyser served through the ipdbg-la TCP protocol.
// GPIO and bridge state changes are timestamped as they happen; the fixed-rate
// sample array is only produced when a client requests a capture.
//
// Channels:
//   0 input wire, 1 WS2812 data, 2..4/5..7/8..10 phase A/B/C mode,
//   11 comparator output, 12..13 sensed phase.
// Phase mode is SITL_PHASE_*: 0 float, 1 low, 2 PWM, 3 PWM without
// complementary drive, 4 proportional brake.
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_Sigrok : IDoubleWordPeripheral, IKnownSize,
                              IGPIOReceiver, IAM32LogicAnalyzer, IDisposable
    {
        public AM32_Sigrok(IMachine machine)
        {
            this.machine = machine;
            sampleRate = DefaultSampleRate;
        }

        public long Size => 0x100;

        // The analyser is bench equipment, so a firmware reset does not close
        // its socket or discard the waveform already on the wire.
        public void Reset()
        {
        }

        public void Dispose()
        {
            lock(lifecycle)
            {
                Close();
            }
        }

        public int Port
        {
            get { return port; }
            set
            {
                if(value < 0 || value > 65535)
                {
                    throw new RecoverableException("sigrok port must be 0..65535");
                }
                lock(lifecycle)
                {
                    if(value == port)
                    {
                        return;
                    }
                    Close();
                    if(value != 0)
                    {
                        Open(value);
                    }
                }
            }
        }

        public uint SampleRate
        {
            get { return sampleRate; }
            set
            {
                if(value == 0 || value > 1000000000u)
                {
                    throw new RecoverableException(
                        "sigrok sample rate must be 1..1000000000 Hz");
                }
                sampleRate = value;
            }
        }

        // Capture depth as log2 of the sample count, which is what the
        // protocol carries. It bounds the window a capture can cover
        // (2^n / SampleRate) and PulseView's largest offered sample
        // count, but the whole buffer is transferred on every capture,
        // so deeper is not free.
        public uint AddressWidth
        {
            get { return addressWidth; }
            set
            {
                if(value < MinAddressWidth || value > MaxAddressWidth)
                {
                    throw new RecoverableException(string.Format(
                        "sigrok address width must be {0}..{1}",
                        MinAddressWidth, MaxAddressWidth));
                }
                addressWidth = value;
            }
        }

        // Debug window: port, nominal sample rate, edges retained, captures.
        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return (uint)port;
            case 0x04: return sampleRate;
            case 0x08: return (uint)edgeCount;
            case 0x0C: return captures;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0x00)
            {
                Port = (int)value;
            }
            else if(offset == 0x04)
            {
                SampleRate = value;
            }
        }

        // Inputs 0 and 1 are direct fan-outs of the throttle and LED wires.
        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number > 1)
            {
                return;
            }
            var now = NowNs;
            ActivateRequest(now);
            UpdateState(1u << number, value ? 1u << number : 0, now);
            CompleteIfReady(now);
        }

        public void ObserveBridge(int phaseA, int phaseB, int phaseC,
                                  int sensedPhase, bool comparator)
        {
            var now = NowNs;
            ActivateRequest(now);
            uint value = ((uint)phaseA & 7u) << 2;
            value |= ((uint)phaseB & 7u) << 5;
            value |= ((uint)phaseC & 7u) << 8;
            if(comparator)
            {
                value |= 1u << 11;
            }
            value |= ((uint)Math.Max(0, sensedPhase) & 3u) << 12;
            UpdateState(BridgeMask, value, now);
            CompleteIfReady(now);
        }

        private long NowNs => (long)(machine.ElapsedVirtualTime.TimeElapsed
            .TotalMicroseconds * 1000.0);

        private void UpdateState(uint mask, uint value, long now)
        {
            var before = currentState;
            var after = (before & ~mask) | (value & mask);
            var changed = before ^ after;
            if(changed != 0)
            {
                for(var channel = 0; channel < DataWidth; channel++)
                {
                    var bit = 1u << channel;
                    if((changed & bit) != 0)
                    {
                        AddEdge(new Edge(now, channel, (after & bit) != 0));
                    }
                }
                currentState = after;
            }
            EvaluateTrigger(before, after, changed, now);
        }

        private void AddEdge(Edge edge)
        {
            if(edgeCount == edges.Length)
            {
                baseState = edges[edgeHead].Apply(baseState);
                edgeHead = (edgeHead + 1) % edges.Length;
                edgeCount--;
            }
            edges[(edgeHead + edgeCount) % edges.Length] = edge;
            edgeCount++;
        }

        private void ActivateRequest(long now)
        {
            if(active != null)
            {
                return;
            }
            do
            {
                if(!requests.TryDequeue(out active))
                {
                    return;
                }
                if(active.Cancelled)
                {
                    active.Ready.Set();
                    active = null;
                }
            }
            while(active == null);
            active.SampleRate = sampleRate;
            active.Delay = Math.Min(active.Delay, SampleCount - 1);
            EvaluateTrigger(currentState, currentState, 0, now);
        }

        private void EvaluateTrigger(uint before, uint after, uint changed,
                                     long now)
        {
            if(active == null || active.Triggered)
            {
                return;
            }
            var any = active.MaskCurrent | active.MaskLast | active.MaskEdge;
            var currentMatch = (after & active.MaskCurrent)
                == (active.ValueCurrent & active.MaskCurrent);
            var lastMatch = (before & active.MaskLast)
                == (active.ValueLast & active.MaskLast);
            var edgeMatch = active.MaskEdge == 0
                || (changed & active.MaskEdge) == active.MaskEdge;
            if(any == 0 || (currentMatch && lastMatch && edgeMatch))
            {
                active.Triggered = true;
                active.TriggerNs = now;
            }
        }

        private void CompleteIfReady(long now)
        {
            if(active == null)
            {
                return;
            }
            if(active.Cancelled)
            {
                active.Ready.Set();
                active = null;
                return;
            }
            if(!active.Triggered)
            {
                return;
            }
            var post = SampleCount - 1 - active.Delay;
            var end = active.TriggerNs + SamplesToNs(post, active.SampleRate);
            if(now < end)
            {
                return;
            }
            active.Data = Rasterize(active);
            captures++;
            active.Ready.Set();
            active = null;
        }

        private byte[] Rasterize(CaptureRequest request)
        {
            var data = new byte[SampleCount * DataBytes];
            var start = request.TriggerNs
                - SamplesToNs(request.Delay, request.SampleRate);
            var state = baseState;
            var edgeIndex = 0;
            while(edgeIndex < edgeCount)
            {
                var edge = edges[(edgeHead + edgeIndex) % edges.Length];
                if(edge.TimeNs > start)
                {
                    break;
                }
                state = edge.Apply(state);
                edgeIndex++;
            }
            for(var sample = 0; sample < SampleCount; sample++)
            {
                var when = start + SamplesToNs((uint)sample, request.SampleRate);
                while(edgeIndex < edgeCount)
                {
                    var edge = edges[(edgeHead + edgeIndex) % edges.Length];
                    if(edge.TimeNs > when)
                    {
                        break;
                    }
                    state = edge.Apply(state);
                    edgeIndex++;
                }
                var offset = sample * DataBytes;
                data[offset] = (byte)state;
                data[offset + 1] = (byte)(state >> 8);
            }
            return data;
        }

        private static long SamplesToNs(uint samples, uint rate)
        {
            return (long)((ulong)samples * 1000000000ul / rate);
        }

        private void Open(int value)
        {
            var listener = new Socket(AddressFamily.InterNetwork,
                                      SocketType.Stream, ProtocolType.Tcp);
            try
            {
                listener.SetSocketOption(SocketOptionLevel.Socket,
                                         SocketOptionName.ReuseAddress, true);
                listener.Bind(new IPEndPoint(IPAddress.Loopback, value));
                listener.Listen(1);
            }
            catch(SocketException e)
            {
                listener.Close();
                throw new RecoverableException(string.Format(
                    "could not bind the sigrok port {0}: {1}", value, e.Message));
            }
            listenerSocket = listener;
            port = value;
            listenerThread = new Thread(() => AcceptLoop(listener))
            {
                IsBackground = true,
                Name = "am32 sigrok " + value,
            };
            listenerThread.Start();
            this.Log(LogLevel.Info, "ipdbg-la listening on tcp 127.0.0.1:{0}",
                     value);
        }

        private void Close()
        {
            port = 0;
            var listener = listenerSocket;
            var client = clientSocket;
            var thread = listenerThread;
            listenerSocket = null;
            clientSocket = null;
            listenerThread = null;
            if(client != null)
            {
                client.Close();
            }
            if(listener != null)
            {
                listener.Close();
            }
            if(thread != null && !thread.Join(2000))
            {
                this.Log(LogLevel.Warning,
                         "sigrok listener thread did not stop in time");
            }
        }

        private void AcceptLoop(Socket listener)
        {
            try
            {
                while(listenerSocket == listener)
                {
                    Socket client;
                    try
                    {
                        client = listener.Accept();
                    }
                    catch(SocketException)
                    {
                        if(listenerSocket != listener)
                        {
                            return;
                        }
                        continue;
                    }
                    client.NoDelay = true;
                    clientSocket = client;
                    try
                    {
                        new ProtocolSession(this, client).Run();
                    }
                    catch(SocketException)
                    {
                        // Client disconnect is the normal scan/open sequence.
                    }
                    catch(ObjectDisposedException)
                    {
                    }
                    catch(Exception e)
                    {
                        this.Log(LogLevel.Error, "sigrok client stopped: {0}", e);
                    }
                    finally
                    {
                        if(clientSocket == client)
                        {
                            clientSocket = null;
                        }
                        client.Close();
                    }
                }
            }
            catch(ObjectDisposedException)
            {
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Error, "sigrok listener stopped: {0}", e);
            }
        }

        private CaptureRequest RequestCapture(ProtocolSession session)
        {
            var request = new CaptureRequest(session);
            requests.Enqueue(request);
            return request;
        }

        private sealed class ProtocolSession
        {
            public ProtocolSession(AM32_Sigrok owner, Socket socket)
            {
                this.owner = owner;
                this.socket = socket;
            }

            public uint MaskCurrent { get; private set; }
            public uint ValueCurrent { get; private set; }
            public uint MaskLast { get; private set; }
            public uint ValueLast { get; private set; }
            public uint MaskEdge { get; private set; }
            public uint Delay { get; private set; }

            public void Run()
            {
                var buf = new byte[4096];
                while(true)
                {
                    var count = socket.Receive(buf);
                    if(count <= 0)
                    {
                        return;
                    }
                    for(var i = 0; i < count; i++)
                    {
                        Feed(buf[i]);
                    }
                }
            }

            private void Feed(byte value)
            {
                if(escaped)
                {
                    escaped = false;
                    Command(value);
                    return;
                }
                if(value == Escape)
                {
                    escaped = true;
                    return;
                }
                if(value == ResetCommand)
                {
                    Reset();
                    return;
                }
                Command(value);
            }

            private void Reset()
            {
                if(captureRequest != null)
                {
                    captureRequest.Cancelled = true;
                    captureRequest.Ready.Set();
                    captureRequest = null;
                }
                state = State.Idle;
                payload = 0;
                payloadRemaining = 0;
                MaskCurrent = ValueCurrent = MaskLast = ValueLast = MaskEdge = 0;
                Delay = 0;
            }

            private void Command(byte value)
            {
                if(state == State.Payload)
                {
                    payload = (payload << 8) | value;
                    if(--payloadRemaining == 0)
                    {
                        StorePayload();
                        state = State.Idle;
                    }
                    return;
                }
                if(state == State.Trigger)
                {
                    state = value == 0xF1 ? State.Current
                        : value == 0xF9 ? State.Last
                        : value == 0xF5 ? State.Edge : State.Idle;
                    return;
                }
                if(state == State.Current)
                {
                    BeginPayload(value == 0xF3 ? Target.MaskCurrent
                        : value == 0xF7 ? Target.ValueCurrent : Target.None,
                        DataBytes);
                    return;
                }
                if(state == State.Last)
                {
                    BeginPayload(value == 0xFB ? Target.MaskLast
                        : value == 0xFF ? Target.ValueLast : Target.None,
                        DataBytes);
                    return;
                }
                if(state == State.Edge)
                {
                    BeginPayload(value == 0xF6 ? Target.MaskEdge : Target.None,
                                 DataBytes);
                    return;
                }
                if(state == State.LogicAnalyzer)
                {
                    BeginPayload(value == 0x1F ? Target.Delay : Target.None,
                                 owner.AddressBytes);
                    return;
                }

                switch(value)
                {
                case 0xBB:
                    Send(new byte[] { (byte)'I', (byte)'D', (byte)'B', (byte)'G' });
                    break;
                case 0xAA:
                    SendWidths();
                    break;
                case 0x10:
                    Send(new byte[4]);
                    break;
                case 0x60:
                    Send(new byte[1]);
                    break;
                case 0xF0:
                    state = State.Trigger;
                    break;
                case 0x0F:
                    state = State.LogicAnalyzer;
                    break;
                case 0xFE:
                    Capture();
                    break;
                case 0x00:
                    break;
                }
            }

            private void BeginPayload(Target target, int bytes)
            {
                if(target == Target.None)
                {
                    state = State.Idle;
                    return;
                }
                this.target = target;
                payload = 0;
                payloadRemaining = bytes;
                state = State.Payload;
            }

            private void StorePayload()
            {
                var value = payload & ValidMask;
                switch(target)
                {
                case Target.MaskCurrent: MaskCurrent = value; break;
                case Target.ValueCurrent: ValueCurrent = value; break;
                case Target.MaskLast: MaskLast = value; break;
                case Target.ValueLast: ValueLast = value; break;
                case Target.MaskEdge: MaskEdge = value; break;
                case Target.Delay: Delay = payload; break;
                }
            }

            private void Capture()
            {
                var request = owner.RequestCapture(this);
                captureRequest = request;
                var buf = new byte[256];
                while(!request.Ready.WaitOne(10))
                {
                    if(owner.clientSocket != socket)
                    {
                        request.Cancelled = true;
                        return;
                    }
                    if(socket.Poll(0, SelectMode.SelectRead))
                    {
                        var count = socket.Receive(buf);
                        if(count <= 0)
                        {
                            request.Cancelled = true;
                            return;
                        }
                        for(var i = 0; i < count; i++)
                        {
                            Feed(buf[i]);
                        }
                    }
                }
                captureRequest = null;
                if(!request.Cancelled && request.Data != null)
                {
                    Send(request.Data);
                }
                request.Ready.Close();
            }

            private void SendWidths()
            {
                var data = new byte[8];
                WriteLittleEndian(data, 0, DataWidth);
                WriteLittleEndian(data, 4, owner.addressWidth);
                Send(data);
            }

            private static void WriteLittleEndian(byte[] data, int offset,
                                                  uint value)
            {
                data[offset] = (byte)value;
                data[offset + 1] = (byte)(value >> 8);
                data[offset + 2] = (byte)(value >> 16);
                data[offset + 3] = (byte)(value >> 24);
            }

            private void Send(byte[] data)
            {
                var offset = 0;
                while(offset < data.Length)
                {
                    var sent = socket.Send(data, offset, data.Length - offset,
                                           SocketFlags.None);
                    if(sent <= 0)
                    {
                        throw new SocketException();
                    }
                    offset += sent;
                }
            }

            private readonly AM32_Sigrok owner;
            private readonly Socket socket;
            private bool escaped;
            private State state;
            private Target target;
            private uint payload;
            private int payloadRemaining;
            private CaptureRequest captureRequest;

            private enum State { Idle, Trigger, Current, Last, Edge,
                                 LogicAnalyzer, Payload }
            private enum Target { None, MaskCurrent, ValueCurrent, MaskLast,
                                  ValueLast, MaskEdge, Delay }
        }

        private sealed class CaptureRequest
        {
            public CaptureRequest(ProtocolSession session)
            {
                MaskCurrent = session.MaskCurrent;
                ValueCurrent = session.ValueCurrent;
                MaskLast = session.MaskLast;
                ValueLast = session.ValueLast;
                MaskEdge = session.MaskEdge;
                Delay = session.Delay;
            }

            public readonly uint MaskCurrent;
            public readonly uint ValueCurrent;
            public readonly uint MaskLast;
            public readonly uint ValueLast;
            public readonly uint MaskEdge;
            public uint Delay;
            public uint SampleRate;
            public bool Triggered;
            public volatile bool Cancelled;
            public long TriggerNs;
            public byte[] Data;
            public readonly ManualResetEvent Ready = new ManualResetEvent(false);
        }

        private struct Edge
        {
            public Edge(long timeNs, int channel, bool value)
            {
                TimeNs = timeNs;
                Channel = channel;
                Value = value;
            }

            public uint Apply(uint state)
            {
                var bit = 1u << Channel;
                return Value ? state | bit : state & ~bit;
            }

            public readonly long TimeNs;
            public readonly int Channel;
            public readonly bool Value;
        }

        private const int DataWidth = 14;
        private const int DataBytes = (DataWidth + 7) / 8;
        private const uint DefaultAddressWidth = 20;
        private const uint MinAddressWidth = 8;
        private const uint MaxAddressWidth = 26;
        private const uint ValidMask = (1u << DataWidth) - 1;
        private const uint BridgeMask = ValidMask & ~3u;
        private const uint DefaultSampleRate = 10000000;
        private const byte Escape = 0x55;
        private const byte ResetCommand = 0xEE;

        private uint addressWidth = DefaultAddressWidth;
        private int AddressBytes => (int)((addressWidth + 7) / 8);
        private uint SampleCount => 1u << (int)addressWidth;

        private readonly IMachine machine;
        private readonly object lifecycle = new object();
        private readonly ConcurrentQueue<CaptureRequest> requests
            = new ConcurrentQueue<CaptureRequest>();
        private readonly Edge[] edges = new Edge[1 << 20];
        private volatile Socket listenerSocket;
        private volatile Socket clientSocket;
        private Thread listenerThread;
        private CaptureRequest active;
        private uint sampleRate;
        private int port;
        private int edgeHead;
        private int edgeCount;
        private uint baseState;
        private uint currentState;
        private uint captures;
    }
}
