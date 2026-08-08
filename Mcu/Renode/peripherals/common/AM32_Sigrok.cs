// Event-driven logic analyser served through the renode-la TCP protocol
// (src/hardware/renode-la/PROTOCOL.md in libsigrok). GPIO and bridge state
// changes are timestamped as they happen and expanded into the fixed-rate
// stream the client asked for, so the sample rate costs nothing to raise:
// there is no per-sample emulation event, only a flush tick.
//
// Capture is continuous. Once the client sends START the stream follows
// simulated time until STOP, which is what makes it behave like a bench
// analyser rather than a one-shot buffer.
//
// Channels are named on the wire (see ChannelNames). Phase mode is
// SITL_PHASE_*: 0 float, 1 low, 2 PWM, 3 PWM without complementary drive,
// 4 proportional brake.
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
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
            // Advancing the stream needs a tick of its own: an idle wire
            // still has to produce samples or the client's time stands
            // still. One per millisecond of simulated time is negligible
            // beside the physics batch, and each one emits however many
            // samples the selected rate calls for.
            flush = new LimitTimer(machine.ClockSource, 1000000, this, "sigrok",
                                   FlushPeriodUs, direction: Direction.Ascending,
                                   enabled: false, autoUpdate: true,
                                   eventEnabled: true);
            flush.LimitReached += OnFlush;
        }

        public long Size => 0x100;

        // The analyser is bench equipment, so a firmware reset does not close
        // its socket or interrupt the stream already running.
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

        // The rate advertised in the greeting. The client may pick another
        // one when it starts, and that is the rate the stream then uses.
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

        // Shown as the device name in the frontend's device list.
        public string DeviceName
        {
            get { return deviceName; }
            set { deviceName = string.IsNullOrEmpty(value) ? DefaultName : value; }
        }

        // Debug window: port, advertised rate, streamed samples, clients.
        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return (uint)port;
            case 0x04: return sampleRate;
            case 0x08: return (uint)streamedSamples;
            case 0x0C: return clients;
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
            UpdateState(1u << number, value ? 1u << number : 0);
        }

        public void ObserveBridge(int phaseA, int phaseB, int phaseC,
                                  int sensedPhase, bool comparator)
        {
            uint value = ((uint)phaseA & 7u) << 2;
            value |= ((uint)phaseB & 7u) << 5;
            value |= ((uint)phaseC & 7u) << 8;
            if(comparator)
            {
                value |= 1u << 11;
            }
            value |= ((uint)Math.Max(0, sensedPhase) & 3u) << 12;
            UpdateState(BridgeMask, value);
        }

        // ---- capture, on the emulation thread ----

        private long NowNs => (long)(machine.ElapsedVirtualTime.TimeElapsed
            .TotalMicroseconds * 1000.0);

        private void UpdateState(uint mask, uint value)
        {
            var after = (currentState & ~mask) | (value & mask);
            if(after == currentState)
            {
                return;
            }
            // Emit everything up to this instant at the old level first, so
            // the change lands in the right sample rather than being
            // back-dated to the last flush.
            var stream = active;
            if(stream != null)
            {
                Emit(stream, NowNs);
            }
            currentState = after;
        }

        private void OnFlush()
        {
            ServicePending();
            var stream = active;
            if(stream == null)
            {
                return;
            }
            Emit(stream, NowNs);
            stream.Push();
        }

        // Expand the interval [emitted, until) into fixed-rate samples of the
        // level held over it. Cost is the sample count, not the edge count,
        // and nothing here runs per emulated sample.
        private void Emit(Stream stream, long until)
        {
            if(until <= stream.EmittedNs)
            {
                return;
            }
            var wanted = (long)((decimal)(until - stream.StartNs)
                                * stream.Rate / 1000000000m);
            var count = wanted - stream.EmittedSamples;
            if(count <= 0)
            {
                stream.EmittedNs = until;
                return;
            }
            if(count > MaxSamplesPerEmit)
            {
                // A long pause (the emulation was stopped, or the client
                // stalled) would otherwise allocate without bound.
                count = MaxSamplesPerEmit;
            }
            stream.Append(currentState, (int)count);
            stream.EmittedSamples += count;
            stream.EmittedNs = until;
            streamedSamples += count;
        }

        // ---- the socket side ----

        private void Open(int value)
        {
            var socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream,
                                    ProtocolType.Tcp);
            try
            {
                socket.Bind(new IPEndPoint(IPAddress.Loopback, value));
                socket.Listen(1);
            }
            catch(SocketException e)
            {
                socket.Close();
                throw new RecoverableException(string.Format(
                    "could not listen on sigrok port {0}: {1}", value, e.Message));
            }
            port = value;
            listenerSocket = socket;
            flush.Enabled = true;
            listenerThread = new Thread(() => ListenLoop(socket))
            {
                IsBackground = true,
                Name = "am32 sigrok listener",
            };
            listenerThread.Start();
            this.Log(LogLevel.Info, "renode-la listening on tcp 127.0.0.1:{0}",
                     value);
        }

        private void Close()
        {
            StopStream();
            flush.Enabled = false;
            var socket = listenerSocket;
            listenerSocket = null;
            if(socket != null)
            {
                socket.Close();
            }
            var client = clientSocket;
            clientSocket = null;
            if(client != null)
            {
                client.Close();
            }
            port = 0;
        }

        private void ListenLoop(Socket socket)
        {
            while(true)
            {
                Socket client;
                try
                {
                    client = socket.Accept();
                }
                catch(Exception)
                {
                    return;
                }
                try
                {
                    client.NoDelay = true;
                    clients++;
                    clientSocket = client;
                    Serve(client);
                }
                catch(Exception e)
                {
                    this.Log(LogLevel.Warning, "sigrok client dropped: {0}",
                             e.Message);
                }
                finally
                {
                    StopStream();
                    clientSocket = null;
                    try
                    {
                        client.Close();
                    }
                    catch(Exception)
                    {
                    }
                }
            }
        }

        // One connection: greeting, then commands until the peer goes away.
        // A scan connects, reads the greeting and disconnects, so this has
        // to survive being dropped at any point.
        private void Serve(Socket client)
        {
            SendAll(client, Greeting());
            var header = new byte[FrameHeader];
            while(true)
            {
                if(!ReadAll(client, header, FrameHeader))
                {
                    return;
                }
                var type = header[0];
                var length = (int)ReadLe32(header, 4);
                if(length < 0 || length > MaxFramePayload)
                {
                    return;
                }
                var payload = length > 0 ? new byte[length] : Array.Empty<byte>();
                if(length > 0 && !ReadAll(client, payload, length))
                {
                    return;
                }
                if(type == CmdStart)
                {
                    var rate = length >= 8 ? ReadLe64(payload, 0) : sampleRate;
                    StartStream(client, rate);
                }
                else if(type == CmdStop)
                {
                    StopStream();
                }
            }
        }

        private byte[] Greeting()
        {
            var meta = new List<byte>();
            AppendString(meta, deviceName);
            foreach(var name in ChannelNames)
            {
                AppendString(meta, name);
            }
            var packet = new byte[GreetingSize + meta.Count];
            Encoding.ASCII.GetBytes(Magic, 0, Magic.Length, packet, 0);
            WriteLe16(packet, 8, ProtocolVersion);
            WriteLe16(packet, 10, (ushort)DataWidth);
            WriteLe32(packet, 12, (uint)meta.Count);
            WriteLe64(packet, 16, sampleRate);
            meta.CopyTo(packet, GreetingSize);
            return packet;
        }

        private static void AppendString(List<byte> to, string value)
        {
            var bytes = Encoding.UTF8.GetBytes(value);
            to.Add((byte)(bytes.Length & 0xFF));
            to.Add((byte)(bytes.Length >> 8));
            to.AddRange(bytes);
        }

        // Socket thread: latch the request only. Virtual time and the
        // flush timer belong to the emulation thread, so the stream is
        // actually created in OnFlush.
        private void StartStream(Socket client, ulong rate)
        {
            if(rate == 0 || rate > 1000000000ul)
            {
                SendError(client, "unsupported sample rate");
                return;
            }
            lock(pendingLock)
            {
                pendingClient = client;
                pendingRate = rate;
                pendingStart = true;
                pendingStop = true;
            }
        }

        private void StopStream()
        {
            lock(pendingLock)
            {
                pendingStart = false;
                pendingClient = null;
                pendingStop = true;
            }
        }

        // Emulation thread: apply whatever the socket thread asked for.
        private void ServicePending()
        {
            Socket client = null;
            ulong rate = 0;
            bool start, stop;
            lock(pendingLock)
            {
                start = pendingStart;
                stop = pendingStop;
                client = pendingClient;
                rate = pendingRate;
                pendingStart = false;
                pendingStop = false;
                pendingClient = null;
            }
            if(stop)
            {
                var previous = active;
                active = null;
                if(previous != null)
                {
                    previous.Stop();
                }
            }
            if(start && client != null)
            {
                active = new Stream(this, client, rate, NowNs);
                this.Log(LogLevel.Info, "sigrok streaming at {0} Hz", rate);
            }
        }

        private void SendError(Socket client, string text)
        {
            var body = Encoding.UTF8.GetBytes(text);
            var packet = new byte[FrameHeader + body.Length];
            packet[0] = MsgError;
            WriteLe32(packet, 4, (uint)body.Length);
            Array.Copy(body, 0, packet, FrameHeader, body.Length);
            SendAll(client, packet);
        }

        private static void SendAll(Socket socket, byte[] data)
        {
            var sent = 0;
            while(sent < data.Length)
            {
                sent += socket.Send(data, sent, data.Length - sent,
                                    SocketFlags.None);
            }
        }

        private static bool ReadAll(Socket socket, byte[] into, int length)
        {
            var got = 0;
            while(got < length)
            {
                int n;
                try
                {
                    n = socket.Receive(into, got, length - got, SocketFlags.None);
                }
                catch(Exception)
                {
                    return false;
                }
                if(n <= 0)
                {
                    return false;
                }
                got += n;
            }
            return true;
        }

        // A started stream: samples are packed on the emulation thread and
        // written by a thread of its own, so a client that reads slowly
        // applies backpressure through the queue rather than stalling the
        // emulation inside a socket write.
        private sealed class Stream
        {
            public Stream(AM32_Sigrok owner, Socket socket, ulong rate,
                          long startNs)
            {
                this.owner = owner;
                this.socket = socket;
                Rate = rate;
                StartNs = startNs;
                EmittedNs = startNs;
                sender = new Thread(SendLoop)
                {
                    IsBackground = true,
                    Name = "am32 sigrok sender",
                };
                sender.Start();
            }

            public ulong Rate { get; private set; }
            public long StartNs { get; private set; }
            public long EmittedNs { get; set; }
            public long EmittedSamples { get; set; }

            public void Append(uint state, int count)
            {
                var low = (byte)state;
                var high = (byte)(state >> 8);
                for(var i = 0; i < count; i++)
                {
                    pending.Add(low);
                    pending.Add(high);
                }
                if(pending.Count >= PushThreshold)
                {
                    Push();
                }
            }

            public void Push()
            {
                if(pending.Count == 0 || stopped)
                {
                    return;
                }
                var payload = pending.ToArray();
                pending.Clear();
                var packet = new byte[FrameHeader + payload.Length];
                packet[0] = MsgData;
                WriteLe32(packet, 4, (uint)payload.Length);
                Array.Copy(payload, 0, packet, FrameHeader, payload.Length);
                try
                {
                    queue.Add(packet);
                }
                catch(Exception)
                {
                }
            }

            public void Stop()
            {
                stopped = true;
                pending.Clear();
                try
                {
                    queue.CompleteAdding();
                }
                catch(Exception)
                {
                }
            }

            private void SendLoop()
            {
                try
                {
                    foreach(var packet in queue.GetConsumingEnumerable())
                    {
                        SendAll(socket, packet);
                    }
                }
                catch(Exception)
                {
                    // the client went away; ListenLoop cleans up
                }
                finally
                {
                    stopped = true;
                }
            }

            private readonly AM32_Sigrok owner;
            private readonly Socket socket;
            private readonly List<byte> pending = new List<byte>();
            private readonly BlockingCollection<byte[]> queue
                = new BlockingCollection<byte[]>(QueueDepth);
            private readonly Thread sender;
            private volatile bool stopped;
        }

        private static void WriteLe16(byte[] to, int at, ushort value)
        {
            to[at] = (byte)value;
            to[at + 1] = (byte)(value >> 8);
        }

        private static void WriteLe32(byte[] to, int at, uint value)
        {
            for(var i = 0; i < 4; i++)
            {
                to[at + i] = (byte)(value >> (8 * i));
            }
        }

        private static void WriteLe64(byte[] to, int at, ulong value)
        {
            for(var i = 0; i < 8; i++)
            {
                to[at + i] = (byte)(value >> (8 * i));
            }
        }

        private static uint ReadLe32(byte[] from, int at)
        {
            return (uint)(from[at] | (from[at + 1] << 8)
                          | (from[at + 2] << 16) | (from[at + 3] << 24));
        }

        private static ulong ReadLe64(byte[] from, int at)
        {
            ulong value = 0;
            for(var i = 7; i >= 0; i--)
            {
                value = (value << 8) | from[at + i];
            }
            return value;
        }

        // The names the frontend shows. They are part of the greeting and
        // the driver rejects a device whose metadata changes between the
        // scan and the open, so this stays fixed.
        private static readonly string[] ChannelNames =
        {
            "input", "ws2812",
            "A.mode0", "A.mode1", "A.mode2",
            "B.mode0", "B.mode1", "B.mode2",
            "C.mode0", "C.mode1", "C.mode2",
            "comparator", "sense0", "sense1",
        };

        private const string Magic = "RenodeLA";
        private const string DefaultName = "AM32 in Renode";
        private const ushort ProtocolVersion = 1;
        private const int GreetingSize = 24;
        private const int FrameHeader = 8;
        private const int MaxFramePayload = 16 * 1024 * 1024;
        private const byte CmdStart = 0x01;
        private const byte CmdStop = 0x02;
        private const byte MsgData = 0x80;
        private const byte MsgError = 0x81;
        private const int DataWidth = 14;
        private const int DataBytes = (DataWidth + 7) / 8;
        private const uint ValidMask = (1u << DataWidth) - 1;
        private const uint BridgeMask = ValidMask & ~3u;
        private const uint DefaultSampleRate = 10000000;
        private const uint FlushPeriodUs = 1000;
        private const int PushThreshold = 64 * 1024;
        private const int QueueDepth = 64;
        private const long MaxSamplesPerEmit = 4 * 1000 * 1000;

        private readonly IMachine machine;
        private readonly object lifecycle = new object();
        private readonly LimitTimer flush;

        private volatile Socket listenerSocket;
        private volatile Socket clientSocket;
        private Thread listenerThread;
        private volatile Stream active;
        private readonly object pendingLock = new object();
        private Socket pendingClient;
        private ulong pendingRate;
        private bool pendingStart;
        private bool pendingStop;
        private uint currentState;
        private uint sampleRate;
        private string deviceName = DefaultName;
        private int port;
        private uint clients;
        private long streamedSamples;
    }
}
