//
// Serves the SITL's two UDP wire protocols from inside the emulator, so
// Mcu/SITL/sitl_gui.py drives an emulated ESC with no idea which backend
// is on the other end:
//   input port  (Mcu/SITL/Src/sitl_input.c) - throttle in, BDShot replies out
//   state port  (Mcu/SITL/Src/sitl_state.c) - physics samples, eeprom, model
//
// Setpoints, not a wire recording. This is the one deliberate deviation
// from what the SITL does, and it is forced by the clock: Renode runs at
// about 0.11x real time, so a GUI streaming servo frames at 50Hz of wall
// clock is a 5.5Hz signal as the firmware experiences it, well under the
// rate detectInput() needs, and nothing ever arms. An incoming packet
// therefore sets the generator's throttle rather than becoming one frame
// on the wire, and AM32ThrottleGenerator synthesises correctly timed
// frames in virtual time. The frames the firmware decodes are still real
// pin edges through the real capture and DMA path - what is lost is the
// ability to drive malformed or oddly rated signals from the GUI, which
// stays a SITL job.
//
// Frame rate is a property here rather than something the GUI sets, for
// the same reason: the sender's rate is wall clock and means nothing in
// virtual time.
//
// Everything that touches emulated state happens on the emulation
// thread, in Tick. The socket threads only latch: one slot for the
// newest setpoint (so a paused emulation cannot build a backlog - the
// newest setpoint is the only one that matters) and a bounded queue for
// state commands.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    //   0x00  input frames received
    //   0x04  BDShot replies sent
    //   0x08  state samples sent
    //   0x0C  replies dropped because the client polled too slowly
    public class AM32_GuiLink : IDoubleWordPeripheral, IKnownSize
    {
        public AM32_GuiLink(IMachine machine, ulong eepromAddress = 0,
                            uint eepromSize = 0)
        {
            this.machine = machine;
            this.eepromAddress = eepromAddress;
            this.eepromSize = eepromSize;
            FrameUs = 20000;
            DshotFrameUs = 250;
            SignalTimeoutMs = 250;

            tick = new LimitTimer(machine.ClockSource, 1000000, this, "guilink",
                                  IdleTickUs, direction: Direction.Ascending,
                                  enabled: false, autoUpdate: true,
                                  eventEnabled: true);
            tick.LimitReached += Tick;
        }

        public long Size => 0x100;

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return framesIn;
            case 0x04: return repliesOut;
            case 0x08: return samplesOut;
            case 0x0C: return repliesLost;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        // udp port the GUI sends throttle to and receives telemetry on,
        // 57733 by default in the SITL. Setting it opens the socket.
        public int InputPort
        {
            set
            {
                inputSocket = Listen(value, InputLoop, "input");
                // The generator self-starts in servo mode, which is right
                // for a scripted run but poisons this one: nothing stops
                // the firmware detecting servo in the seconds before the
                // GUI attaches, and detectInput() then only ever calls
                // checkServo() again, so a dshot stream arriving later is
                // never looked at and the ESC never arms. With a client on
                // the wire the client owns it - silence until it speaks,
                // which is also what a bench ESC with no FC plugged in
                // looks like.
                ownsWire = inputSocket != null;
            }
        }

        // udp port serving physics samples, eeprom and model commands
        public int StatePort
        {
            set { stateSocket = Listen(value, StateLoop, "state"); }
        }

        // Servo frame period, and the dshot frame period, in virtual
        // microseconds. The GUI's rate control cannot reach here - it is a
        // wall clock rate - so this is what the emulated wire actually
        // carries.
        public uint FrameUs { get; set; }

        // Dshot frame period, also virtual microseconds. Every edge is a
        // timer event, so this is the cheapest speed lever the link has:
        // 250us is 4kHz as a flight controller would send, 1000us is a
        // quarter of the emulation cost for a quarter of the telemetry
        // rate.
        public uint DshotFrameUs { get; set; }

        // Where the firmware says what it is: the `filename` symbol, a
        // fixed 30 byte string in its own flash section, which is where a
        // configurator reads it from too. Read out of emulated flash
        // rather than out of the ELF, so it is what is actually loaded.
        public ulong FirmwareNameAddress { get; set; }

        // armed_timeout_count in SRAM, a uint16 counted up by
        // tenKhzRoutine() while the input is at zero, and the loop rate it
        // is counted at. Together they are "how far through the one second
        // of zero throttle arming needs".
        public ulong ArmedCountAddress { get; set; }
        public uint LoopHz { get; set; }

        // the armed flag itself, so a client does not have to infer it
        // from the motor turning
        public ulong ArmedAddress { get; set; }

        // the firmware's in-RAM eepromBuffer. Settings writes land in
        // the flash page AND here, which is what changing a setting at
        // runtime over DroneCAN does - the firmware acts on the RAM
        // copy, so the change is live without a reboot
        public ulong EepromBufferAddress { get; set; }

        // where the application starts; below it is the bootloader region,
        // so the PC says which of the two is executing
        public ulong AppBase { get; set; }

        // Wall clock silence that counts as the sender having gone away,
        // after which the generator stops driving and the firmware sees
        // signal loss. Wall clock is right despite everything else being
        // virtual: it measures whether the GUI is still there.
        public int SignalTimeoutMs { get; set; }

        public uint FramesIn => framesIn;
        public uint RepliesOut => repliesOut;
        public uint SamplesOut => samplesOut;
        // replies decoded but never sent, because the client polls slower
        // than the reply history is long
        public uint RepliesLost => repliesLost;

        // what the last setpoint asked for: a pulse width in servo mode,
        // the raw 11 bit value in dshot
        public uint Throttle => generator == null ? 0
            : (generator.Protocol == 0 ? generator.PulseUs : generator.DshotValue);
        public bool Driving => driving;

        public void Reset()
        {
            // the sockets survive: the firmware reboots itself on signal
            // loss while armed, and a link that died with it would leave
            // the GUI looking at a dead ESC for the rest of the session
            generator = null;
            bridge = null;
            capture = null;
            lastReplyCount = 0;
            batchCount = 0;
            // the generator is reset too, so the next setpoint has to
            // start it driving again
            driving = false;
            silenced = false;
            lock(sync)
            {
                haveSetpoint = false;
            }
            // the pace setting survives a firmware reboot, like the
            // SITL's does; only the anchor is dropped
            paceValid = false;
            // the watch subscription survives too (the addresses are
            // still valid, it is the same firmware), but the change
            // detection restarts so the rebooted values re-emit
            for(var i = 0; i < watchCount; i++)
            {
                watches[i].HaveLast = false;
            }
            watchBatchCount = 0;
            tick.Limit = IdleTickUs;
            tick.Enabled = inputSocket != null || stateSocket != null;
        }

        private Socket Listen(int port, Action<Socket> loop, string what)
        {
            if(port <= 0)
            {
                return null;
            }
            var s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram,
                               ProtocolType.Udp);
            try
            {
                // no address reuse: a second instance on the same port has
                // to fail loudly rather than silently steal datagrams,
                // which is also how the SITL binds
                s.Bind(new IPEndPoint(IPAddress.Loopback, port));
            }
            catch(SocketException e)
            {
                s.Close();
                throw new RecoverableException(string.Format(
                    "could not bind the {0} port {1}: {2}", what, port, e.Message));
            }
            // the socket is handed to the thread rather than read back out
            // of the field it is about to be assigned to: the thread wins
            // that race often enough to be a reliable crash
            var t = new Thread(() =>
            {
                try
                {
                    loop(s);
                }
                catch(Exception e)
                {
                    // an unhandled exception on a background thread takes
                    // the whole emulator down with it
                    this.Log(LogLevel.Error, "{0} thread stopped: {1}", what, e);
                }
            });
            t.IsBackground = true;
            t.Name = "am32 guilink " + what;
            t.Start();
            tick.Enabled = true;
            this.Log(LogLevel.Info, "{0} port on udp {1}", what, port);
            return s;
        }

        // ---- input port: setpoints in, telemetry replies out ----

        private void InputLoop(Socket sock)
        {
            var buf = new byte[256];
            EndPoint from = new IPEndPoint(IPAddress.Any, 0);
            while(true)
            {
                int n;
                try
                {
                    n = sock.ReceiveFrom(buf, ref from);
                }
                catch(SocketException)
                {
                    return;
                }
                catch(ObjectDisposedException)
                {
                    return;
                }
                if(n < 8 || BitConverter.ToUInt16(buf, 0) != InputMagic
                   || buf[3] != 4)
                {
                    continue;
                }
                lock(sync)
                {
                    setType = buf[2];
                    setFlags = BitConverter.ToUInt16(buf, 4);
                    setData = BitConverter.ToUInt16(buf, 6);
                    haveSetpoint = true;
                    replyTo = from;
                }
                // wall clock, deliberately: it answers "is the sender
                // still there", which virtual time cannot
                Volatile.Write(ref lastInputMs, Environment.TickCount);
                framesIn++;
            }
        }

        private void ApplySetpoint()
        {
            byte type;
            ushort flags, data;
            bool had;
            lock(sync)
            {
                type = setType;
                flags = setFlags;
                data = setData;
                had = haveSetpoint;
                haveSetpoint = false;
            }
            if(!had)
            {
                // gone quiet: stop driving, so the firmware runs its real
                // signal loss path. Subtraction rather than a comparison
                // so a TickCount wrap cannot look like an eternity of
                // silence.
                if(driving && Environment.TickCount
                   - Volatile.Read(ref lastInputMs) > SignalTimeoutMs)
                {
                    generator.Enabled = false;
                    driving = false;
                    silenced = true;
                }
                return;
            }

            // Protocol and Bidirectional restart the frame when written,
            // so they are only written on a change. Assigning the same
            // value every setpoint restarts the transmission thousands of
            // times a simulated second and the wire never carries a whole
            // frame.
            switch(type)
            {
            case TypePwm:
                if(generator.Protocol != 0)
                {
                    generator.Protocol = 0;
                }
                generator.PulseUs = data;
                break;
            case TypeDshot150:
            case TypeDshot300:
            case TypeDshot600:
                // the sender composed a whole frame; the generator builds
                // its own each time it transmits, so unpack what it needs
                var bidir = (flags & FlagIdleHigh) != 0;
                if(generator.Bidirectional != bidir)
                {
                    generator.Bidirectional = bidir;
                }
                generator.TelemetryBit = ((data >> 4) & 1) != 0;
                generator.DshotValue = (uint)((data >> 5) & 0x7FF);
                var proto = type == TypeDshot150 ? 150u
                    : (type == TypeDshot300 ? 300u : 600u);
                if(generator.Protocol != proto)
                {
                    generator.Protocol = proto;
                }
                // replies carry the protocol they were asked for
                replyType = type;
                break;
            default:
                // types 4 (serial) and 5 (line level) are bootloader and
                // wire-hold tests, which have no setpoint meaning
                return;
            }
            generator.FrameUs = FrameUs;
            generator.DshotFrameUs = DshotFrameUs;
            if(!driving)
            {
                generator.Enabled = true;
                driving = true;
            }
        }

        // The reply the firmware actually drove onto the wire, decoded by
        // the capture timer from the levels it saw - not read out of the
        // firmware's gcr[] buffer, so this covers the transmit path
        // instead of restating it.
        private void PumpReplies()
        {
            EndPoint to;
            lock(sync)
            {
                to = replyTo;
            }
            if(capture == null || to == null)
            {
                return;
            }
            var count = capture.ReplyCount;
            if(count == lastReplyCount)
            {
                return;
            }
            // every frame since the last tick, not just the newest: the
            // tick is far slower than the reply rate, and extended
            // telemetry interleaves its kinds between the eRPM frames
            var pkt = new byte[8];
            Array.Copy(BitConverter.GetBytes(InputMagic), 0, pkt, 0, 2);
            pkt[2] = replyType;
            pkt[3] = 4;
            Array.Copy(BitConverter.GetBytes(FlagIdleHigh), 0, pkt, 4, 2);
            while(lastReplyCount < count)
            {
                uint frame;
                if(capture.TryGetReply(lastReplyCount, out frame))
                {
                    Array.Copy(BitConverter.GetBytes((ushort)frame), 0, pkt, 6, 2);
                    Send(inputSocket, pkt, pkt.Length, to);
                    repliesOut++;
                }
                else
                {
                    // older than the history: a lost frame, as a busy wire
                    // would produce
                    repliesLost++;
                }
                lastReplyCount++;
            }
        }

        // ---- state port: physics samples, eeprom, model ----

        private void StateLoop(Socket sock)
        {
            var buf = new byte[1024];
            EndPoint from = new IPEndPoint(IPAddress.Any, 0);
            while(true)
            {
                int n;
                try
                {
                    n = sock.ReceiveFrom(buf, ref from);
                }
                catch(SocketException)
                {
                    return;
                }
                catch(ObjectDisposedException)
                {
                    return;
                }
                if(n < 4 || BitConverter.ToUInt16(buf, 0) != StateMagicCmd)
                {
                    continue;
                }
                // The pace target lands here as well as in the queue: the
                // queue is only serviced by the emulation thread, which is
                // exactly the thread Pace() may be holding asleep for
                // seconds at the lowest settings, so a new setting has to
                // be able to cut a sleep short from this thread.
                if(buf[2] == 2 && n >= 8)
                {
                    paceTarget = BitConverter.ToSingle(buf, 4);
                }
                if(commands.Count > 32)
                {
                    // a paused emulation is not a reason to accumulate
                    // work; the GUI resends everything that matters
                    continue;
                }
                var copy = new byte[n];
                Array.Copy(buf, copy, n);
                commands.Enqueue(new Command { Data = copy, From = from });
            }
        }

        private void ServiceCommands()
        {
            Command c;
            while(commands.TryDequeue(out c))
            {
                var d = c.Data;
                switch(d[2])
                {
                case 0: // subscribe, with the wanted sample period
                    if(d.Length < 8)
                    {
                        break;
                    }
                    var wanted = BitConverter.ToUInt32(d, 4) / 1000;
                    var averaged = (d[3] & 1) != 0;
                    if(!subscribed || !c.From.Equals(sampleTo))
                    {
                        batchCount = 0;
                    }
                    sampleTo = c.From;
                    subscribed = true;
                    subscribeMs = Environment.TickCount;
                    SetSamplePeriod(wanted, averaged);
                    break;
                case 1: // load a motor model
                    LoadModel(Encoding.UTF8.GetString(d, 4, d.Length - 4)
                                      .TrimEnd('\0'), c.From);
                    break;
                case 2: // pace the emulation, simulated time over wall
                    // time. The receive thread already stored the value;
                    // this is the acknowledgement.
                    var pace = paceTarget;
                    if(pace > 0 && pace <= 1.0f)
                    {
                        this.Log(LogLevel.Info, "pacing to {0:F3}x", pace);
                        Reply(c.From, true,
                              string.Format("pacing to {0:F3}x", pace));
                    }
                    else
                    {
                        this.Log(LogLevel.Info, "pacing off");
                        Reply(c.From, true, "unpaced: the emulator runs as "
                              + "fast as it can");
                    }
                    break;
                case 8: // variable watch subscribe. Same command number,
                    // reply and data format as the SITL's state port, but
                    // the entries carry (u8 size, u32 address) instead of
                    // (u8 size, name\0): there is no symbol table in here,
                    // the client resolves names against the ELF and sends
                    // addresses
                    WatchSubscribe(c.Data, c.From);
                    break;
                case 9: // restart the ESC
                    // AM32 latches the input protocol it detected and only
                    // ever re-checks that one, so a client that changes
                    // protocol - or writes the eeprom - needs a reboot,
                    // exactly as it would on the bench. Under the SITL that
                    // is relaunching the process; here it is a machine
                    // reset, which is the emulator's power cycle.
                    this.Log(LogLevel.Info, "restarting the ESC");
                    Reply(c.From, true, "restarting the ESC");
                    machine.RequestReset();
                    break;
                case 10: // what firmware is running, and where it is
                    DeviceInfo(c.From);
                    break;
                case 5:
                    EepromFetch(c.From);
                    break;
                case 6:
                    if(d.Length >= 8)
                    {
                        EepromSet(BitConverter.ToUInt16(d, 4),
                                  BitConverter.ToUInt16(d, 6), d, 8, c.From);
                    }
                    break;
                case 7: // stuck rotor fraction
                    if(d.Length >= 8)
                    {
                        var stuck = BitConverter.ToSingle(d, 4);
                        if(stuck >= 0 && stuck <= 1)
                        {
                            am32sim_set_stuck(stuck);
                            this.Log(LogLevel.Info, "stuck rotor {0:F2}", stuck);
                        }
                    }
                    break;
                default:
                    // 3 and 4 are the tone and audio streams, which have no
                    // source here: the SITL derives them from its own fake
                    // timer, and TIM1 beeps are not modelled
                    break;
                }
            }
        }

        // The subscriber's period is honoured rather than capped, because
        // it is exactly the cost of looking: a tick is one call into the
        // physics library, and nobody pays for it when nothing is
        // subscribed. Floored at 20us because finer than the bridge's own
        // batch would only resample the same state.
        private void SetSamplePeriod(uint us, bool averaged)
        {
            tick.Limit = Math.Max(20u, Math.Min(us, 100000u));
            if(averaged != averaging)
            {
                averaging = averaged;
                am32sim_set_averaging(averaged ? 1 : 0);
            }
        }

        private void SampleState()
        {
            if(!subscribed || bridge == null || !bridge.Started)
            {
                return;
            }
            if(Environment.TickCount - subscribeMs > SubscriberTimeoutMs)
            {
                subscribed = false;
                batchCount = 0;
                if(averaging)
                {
                    averaging = false;
                    am32sim_set_averaging(0);
                }
                tick.Limit = IdleTickUs;
                return;
            }

            float omega = 0, theta = 0, thetaE = 0, vbus = 0, ibus = 0;
            am32sim_get_live_state(ref omega, ref theta, ref thetaE, phaseI,
                                   phaseV, ref vbus, ref ibus);
            double[] mean = null;
            if(averaging && am32sim_take_signals(signals) != 0)
            {
                mean = signals;
            }

            var nowNs = (ulong)machine.ElapsedVirtualTime.TimeElapsed
                .TotalMicroseconds * 1000;
            var o = BatchHeader + batchCount * SampleSize;
            Array.Copy(BitConverter.GetBytes(nowNs), 0, batch, o, 8);
            Array.Copy(BitConverter.GetBytes(omega), 0, batch, o + 8, 4);
            Array.Copy(BitConverter.GetBytes(theta), 0, batch, o + 12, 4);
            Array.Copy(BitConverter.GetBytes(thetaE), 0, batch, o + 16, 4);
            for(var k = 0; k < 3; k++)
            {
                Array.Copy(BitConverter.GetBytes(mean == null ? phaseI[k]
                                                 : (float)mean[k]),
                           0, batch, o + 20 + 4 * k, 4);
                Array.Copy(BitConverter.GetBytes(mean == null ? phaseV[k]
                                                 : (float)mean[3 + k]),
                           0, batch, o + 32 + 4 * k, 4);
            }
            Array.Copy(BitConverter.GetBytes(mean == null ? vbus : (float)mean[6]),
                       0, batch, o + 44, 4);
            Array.Copy(BitConverter.GetBytes(mean == null ? ibus : (float)mean[7]),
                       0, batch, o + 48, 4);
            for(var p = 0; p < 3; p++)
            {
                batch[o + 52 + p] = (byte)bridge.LastPhaseMode(p);
            }
            batch[o + 55] = (byte)bridge.LastSensedPhase;
            batch[o + 56] = (byte)(bridge.LastCompOut ? 1 : 0);
            batch[o + 57] = batch[o + 58] = batch[o + 59] = 0;
            batchCount++;

            // flush on a full batch or every 5ms of simulated time, so a
            // coarse sample period still arrives promptly
            if(batchCount >= BatchSamples || nowNs - lastFlushNs > 5000000UL)
            {
                batch[0] = (byte)(StateMagicData & 0xFF);
                batch[1] = (byte)(StateMagicData >> 8);
                batch[2] = 2; // sample layout version
                batch[3] = (byte)batchCount;
                Send(stateSocket, batch, BatchHeader + batchCount * SampleSize,
                     sampleTo);
                samplesOut += (uint)batchCount;
                batchCount = 0;
                lastFlushNs = nowNs;
            }
        }

        // ---- variable watch (state cmd 8) ----
        //
        // The client subscribes with a list of (size, address) entries;
        // every tick each watched location is read from the system bus
        // and a change - honouring the per-variable min period - becomes
        // an event (t_ns, index, raw) in a batch packet, exactly the
        // packets the SITL's watch emits, so the same client reads both.

        private void WatchSubscribe(byte[] d, EndPoint from)
        {
            if(d.Length < 8)
            {
                return;
            }
            // an unchanged re-send is a keepalive: extend the expiry
            // without disturbing the change detection state
            if(watchCount > 0 && watchReq != null
               && d.Length == watchReq.Length && from.Equals(watchTo)
               && d.SequenceEqual(watchReq))
            {
                watchSubscribeMs = Environment.TickCount;
                return;
            }
            var count = Math.Min((int)d[3], WatchMax);
            watchMinPeriodNs = BitConverter.ToUInt32(d, 4);
            watchCount = 0;
            var reply = new byte[4 + count];
            Array.Copy(BitConverter.GetBytes(StateMagicWatchReply), 0, reply, 0, 2);
            reply[2] = 1; // version
            var off = 8;
            for(var k = 0; k < count && off + 5 <= d.Length; k++)
            {
                var size = d[off];
                var addr = BitConverter.ToUInt32(d, off + 1);
                off += 5;
                var ok = size == 1 || size == 2 || size == 4 || size == 8;
                if(ok)
                {
                    watches[watchCount].Address = addr;
                    watches[watchCount].Size = size;
                    watches[watchCount].HaveLast = false;
                    watches[watchCount].NextNs = 0;
                    watchCount++;
                }
                reply[3]++;
                reply[4 + k] = (byte)(ok ? 1 : 0);
            }
            watchTo = from;
            watchSubscribeMs = Environment.TickCount;
            watchBatchCount = 0;
            watchReq = (byte[])d.Clone();
            this.Log(LogLevel.Info, "watching {0} variables", watchCount);
            Send(stateSocket, reply, 4 + reply[3], from);
        }

        private ulong WatchRead(int i)
        {
            var addr = watches[i].Address;
            switch(watches[i].Size)
            {
            case 1: return machine.SystemBus.ReadByte(addr);
            case 2: return machine.SystemBus.ReadWord(addr);
            case 8: return machine.SystemBus.ReadQuadWord(addr);
            default: return machine.SystemBus.ReadDoubleWord(addr);
            }
        }

        private void WatchStep()
        {
            if(watchCount == 0)
            {
                return;
            }
            if(Environment.TickCount - watchSubscribeMs > SubscriberTimeoutMs)
            {
                watchCount = 0;
                watchReq = null;
                return;
            }
            var nowNs = (ulong)machine.ElapsedVirtualTime.TimeElapsed
                .TotalMicroseconds * 1000;
            for(var i = 0; i < watchCount; i++)
            {
                if(nowNs < watches[i].NextNs)
                {
                    continue;
                }
                var v = WatchRead(i);
                if(watches[i].HaveLast && v == watches[i].Last)
                {
                    continue;
                }
                watches[i].Last = v;
                watches[i].HaveLast = true;
                watches[i].NextNs = nowNs + watchMinPeriodNs;
                var o = 4 + watchBatchCount * WatchEventSize;
                Array.Copy(BitConverter.GetBytes(nowNs), 0, watchBatch, o, 8);
                Array.Copy(BitConverter.GetBytes((uint)i), 0, watchBatch, o + 8, 4);
                Array.Copy(BitConverter.GetBytes(v), 0, watchBatch, o + 12, 8);
                watchBatchCount++;
                if(watchBatchCount >= WatchBatchMax)
                {
                    break; // the rest go next tick; ordering stays intact
                }
            }
            if(watchBatchCount > 0
               && (watchBatchCount >= WatchBatchMax
                   || nowNs - watchLastFlushNs > 5000000UL))
            {
                watchBatch[0] = (byte)(StateMagicWatchData & 0xFF);
                watchBatch[1] = (byte)(StateMagicWatchData >> 8);
                watchBatch[2] = 1; // version
                watchBatch[3] = (byte)watchBatchCount;
                Send(stateSocket, watchBatch, 4 + watchBatchCount * WatchEventSize,
                     watchTo);
                watchBatchCount = 0;
                watchLastFlushNs = nowNs;
            }
        }

        private void LoadModel(string path, EndPoint to)
        {
            var ok = am32sim_reload_config(path) != 0;
            var name = path.Substring(path.LastIndexOfAny(PathSeparators) + 1);
            Reply(to, ok, string.Format(ok ? "loaded {0}" : "failed to load {0}",
                                        name));
        }

        // Everything a client cannot see from the wire: which firmware is
        // loaded, whether the core is executing at all and where, and how
        // far through arming it is. All of it read here rather than
        // inferred, because the interesting cases are exactly the ones
        // where the wire has gone quiet and there is nothing to infer from.
        private void DeviceInfo(EndPoint to)
        {
            var cpu = machine.SystemBus.GetCPUs().FirstOrDefault();
            ulong pc = 0;
            var halted = false;
            if(cpu != null)
            {
                pc = cpu.PC.RawValue;
                halted = cpu.IsHalted;
            }
            uint armedCount = 0;
            if(ArmedCountAddress != 0)
            {
                armedCount = machine.SystemBus.ReadWord(ArmedCountAddress);
            }
            var name = FirmwareName();
            var text = Encoding.UTF8.GetBytes(name);
            // the LED block rides behind the name's terminator, where a
            // client that predates it never looks
            if(ws2812 == null)
            {
                ws2812 = machine.GetPeripheralsOfType<AM32_Ws2812>()
                                .FirstOrDefault();
            }
            var ledBytes = ws2812 != null && ws2812.Count > 0 ? 5 : 0;
            var pkt = new byte[20 + text.Length + 1 + ledBytes];
            Array.Copy(BitConverter.GetBytes(StateMagicInfo), 0, pkt, 0, 2);
            pkt[2] = 9;
            var armed = ArmedAddress != 0
                && machine.SystemBus.ReadByte(ArmedAddress) != 0;
            pkt[3] = (byte)((halted ? 1 : 0)
                            | (AppBase != 0 && pc != 0 && pc < AppBase ? 2 : 0)
                            | (armed ? 4 : 0));
            Array.Copy(BitConverter.GetBytes((uint)pc), 0, pkt, 4, 4);
            Array.Copy(BitConverter.GetBytes((uint)AppBase), 0, pkt, 8, 4);
            Array.Copy(BitConverter.GetBytes(armedCount), 0, pkt, 12, 4);
            Array.Copy(BitConverter.GetBytes(LoopHz), 0, pkt, 16, 4);
            Array.Copy(text, 0, pkt, 20, text.Length);
            if(ledBytes > 0)
            {
                var off = 20 + text.Length + 1;
                var color = ws2812.Color;
                pkt[off] = (byte)'L';
                pkt[off + 1] = (byte)Math.Min(ws2812.Count, 255);
                pkt[off + 2] = (byte)(color >> 16);
                pkt[off + 3] = (byte)(color >> 8);
                pkt[off + 4] = (byte)color;
            }
            Send(stateSocket, pkt, pkt.Length, to);
        }

        private string FirmwareName()
        {
            if(FirmwareNameAddress == 0)
            {
                return "";
            }
            var raw = machine.SystemBus.ReadBytes(FirmwareNameAddress, FirmwareNameMax);
            var end = Array.IndexOf(raw, (byte)0);
            if(end < 0)
            {
                end = raw.Length;
            }
            // it is flash: an unprogrammed or unloaded region reads as
            // filler rather than text, and a control character in it is
            // the giveaway
            for(var i = 0; i < end; i++)
            {
                if(raw[i] < 0x20 || raw[i] > 0x7E)
                {
                    return "";
                }
            }
            return Encoding.ASCII.GetString(raw, 0, end);
        }

        // The eeprom is the emulated flash page, which is what the
        // firmware read its settings from, rather than a copy kept here.
        private void EepromFetch(EndPoint to)
        {
            if(eepromSize == 0)
            {
                return;
            }
            double kv = 0;
            var poles = 0;
            am32sim_get_model(ref kv, ref poles);
            var pkt = new byte[16 + eepromSize];
            Array.Copy(BitConverter.GetBytes(StateMagicEeprom), 0, pkt, 0, 2);
            pkt[2] = 5;
            Array.Copy(BitConverter.GetBytes((ushort)eepromSize), 0, pkt, 4, 2);
            Array.Copy(BitConverter.GetBytes((float)kv), 0, pkt, 8, 4);
            pkt[12] = (byte)poles;
            var image = machine.SystemBus.ReadBytes(eepromAddress, (int)eepromSize);
            Array.Copy(image, 0, pkt, 16, eepromSize);
            Send(stateSocket, pkt, pkt.Length, to);
        }

        private void EepromSet(ushort off, ushort len, byte[] data, int dataOffset,
                               EndPoint to)
        {
            var avail = data.Length - dataOffset;
            if(eepromSize == 0)
            {
                Reply(to, false, "no eeprom region configured");
            }
            else if(len > avail)
            {
                Reply(to, false, string.Format("truncated: {0} bytes for length {1}",
                                               avail, len));
            }
            else if(off + len > eepromSize)
            {
                Reply(to, false, string.Format(
                    "range {0}+{1} past the eeprom ({2} bytes)", off, len, eepromSize));
            }
            else
            {
                var bytes = new byte[len];
                Array.Copy(data, dataOffset, bytes, 0, len);
                machine.SystemBus.WriteBytes(bytes, eepromAddress + off);
                // the RAM copy too, as a runtime DroneCAN parameter
                // write would: the firmware acts on eepromBuffer, so
                // the change is live. Derived values that only
                // loadEEpromSettings() computes still need a reset,
                // exactly as they would on the bench.
                if(EepromBufferAddress != 0)
                {
                    machine.SystemBus.WriteBytes(bytes, EepromBufferAddress + off);
                }
                this.Log(LogLevel.Info, "eeprom wrote {0} bytes at {1}", len, off);
                Reply(to, true, string.Format(
                    EepromBufferAddress != 0
                    ? "wrote {0} bytes at {1} (live)"
                    : "wrote {0} bytes at {1}; reset the ESC to read them",
                    len, off));
            }
        }

        private void Reply(EndPoint to, bool ok, string msg)
        {
            var text = Encoding.UTF8.GetBytes(msg);
            var pkt = new byte[4 + text.Length + 1];
            Array.Copy(BitConverter.GetBytes(StateMagicReply), 0, pkt, 0, 2);
            pkt[2] = (byte)(ok ? 1 : 0);
            Array.Copy(text, 0, pkt, 4, text.Length);
            Send(stateSocket, pkt, pkt.Length, to);
        }

        // ---- the emulation thread ----

        private void Tick()
        {
            if(generator == null)
            {
                generator = machine.GetPeripheralsOfType<AM32ThrottleGenerator>()
                    .FirstOrDefault();
                bridge = machine.GetPeripheralsOfType<AM32_F051_Bridge>()
                    .FirstOrDefault();
                capture = machine.GetPeripheralsOfType<IAM32ReplySource>()
                    .FirstOrDefault(s => s.DecodesReplies);
                if(generator == null)
                {
                    this.Log(LogLevel.Error,
                             "no throttle generator in the platform; link disabled");
                    tick.Enabled = false;
                    return;
                }
            }
            if(ownsWire && !driving && !silenced)
            {
                generator.Enabled = false;
                silenced = true;
            }
            ServiceCommands();
            ApplySetpoint();
            PumpReplies();
            SampleState();
            WatchStep();
            Pace();
        }

        // Hold simulated time to paceTarget times wall time by sleeping
        // the emulation thread at the end of its own tick - the GUI's
        // slow motion, which the emulator otherwise has no equivalent
        // of. The anchor slides whenever the emulation cannot keep up,
        // so a target at or above what the host achieves costs nothing
        // and accumulates no debt to sprint off later. Sleeps are sliced
        // so a new setting from the wire cuts them short.
        private void Pace()
        {
            var target = paceTarget;
            if(float.IsNaN(target) || target <= 0 || target > 1.0f)
            {
                paceValid = false;
                return;
            }
            var virtMs = machine.ElapsedVirtualTime.TimeElapsed.TotalMilliseconds;
            var wallMs = paceClock.Elapsed.TotalMilliseconds;
            if(!paceValid || target != paceApplied)
            {
                paceValid = true;
                paceApplied = target;
                paceVirtMs = virtMs;
                paceWallMs = wallMs;
                return;
            }
            var wantWall = paceWallMs + (virtMs - paceVirtMs) / target;
            if(wantWall <= wallMs)
            {
                paceVirtMs = virtMs;
                paceWallMs = wallMs;
                return;
            }
            while(true)
            {
                var ahead = wantWall - paceClock.Elapsed.TotalMilliseconds;
                if(ahead <= 0 || paceTarget != target)
                {
                    break;
                }
                Thread.Sleep((int)Math.Min(Math.Ceiling(ahead), 50));
            }
        }

        private void Send(Socket s, byte[] data, int length, EndPoint to)
        {
            if(s == null || to == null)
            {
                return;
            }
            try
            {
                s.SendTo(data, length, SocketFlags.None, to);
            }
            catch(SocketException)
            {
                // a datagram to a GUI that has gone away; the wire drops
                // frames too
            }
        }

        private struct Command
        {
            public byte[] Data;
            public EndPoint From;
        }

        private const ushort InputMagic = 0x4453;
        private const byte TypePwm = 0;
        private const byte TypeDshot150 = 1;
        private const byte TypeDshot300 = 2;
        private const byte TypeDshot600 = 3;
        private const ushort FlagIdleHigh = 0x0001;

        private const ushort StateMagicCmd = 0x5353;
        private const ushort StateMagicData = 0x5354;
        private const ushort StateMagicReply = 0x5355;
        private const ushort StateMagicEeprom = 0x5358;
        private const ushort StateMagicInfo = 0x5359;
        // shared with the info reply magic, as the SITL does: a watch
        // reply has version 1 in byte 2 where the info packet has 9
        private const ushort StateMagicWatchReply = 0x5359;
        private const ushort StateMagicWatchData = 0x535a;
        private const int WatchMax = 16;
        private const int WatchBatchMax = 32;
        private const int WatchEventSize = 20; // u64 t_ns, u32 index, u64 raw
        // const char filename[30] in Src/main.c
        private const int FirmwareNameMax = 30;

        private const int SampleSize = 60;
        private const int BatchHeader = 4;
        private const int BatchSamples = 16;
        // what the tick costs when nobody is watching: enough to keep
        // setpoints and telemetry moving, cheap enough to ignore
        private const uint IdleTickUs = 1000;
        // the SITL drops a subscriber after 2 wall seconds of silence and
        // the GUI resubscribes every second
        private const int SubscriberTimeoutMs = 2000;

        private static readonly char[] PathSeparators = { '/', '\\' };

        [DllImport("am32sim")]
        private static extern void am32sim_get_live_state(
            ref float omega, ref float theta, ref float thetaE,
            [Out] float[] i, [Out] float[] v, ref float vbus, ref float ibus);
        [DllImport("am32sim")]
        private static extern void am32sim_set_stuck(double stuck);
        [DllImport("am32sim")]
        private static extern void am32sim_set_averaging(int on);
        [DllImport("am32sim")]
        private static extern int am32sim_take_signals([Out] double[] mean);
        [DllImport("am32sim")]
        private static extern int am32sim_reload_config(string path);
        [DllImport("am32sim")]
        private static extern void am32sim_get_model(ref double kv, ref int poles);

        private readonly IMachine machine;
        private readonly ulong eepromAddress;
        private readonly uint eepromSize;
        private readonly LimitTimer tick;
        private readonly object sync = new object();
        private readonly ConcurrentQueue<Command> commands =
            new ConcurrentQueue<Command>();
        private readonly byte[] batch = new byte[BatchHeader + BatchSamples * SampleSize];
        private readonly float[] phaseI = new float[3];
        private readonly float[] phaseV = new float[3];
        private readonly double[] signals = new double[8];

        private Socket inputSocket;
        private Socket stateSocket;
        private AM32ThrottleGenerator generator;
        private AM32_F051_Bridge bridge;
        private IAM32ReplySource capture;
        private AM32_Ws2812 ws2812;

        // guarded by sync: the newest setpoint, and where its sender is
        private bool haveSetpoint;
        private byte setType;
        private ushort setFlags;
        private ushort setData;
        private EndPoint replyTo;

        private bool ownsWire;
        private bool silenced;
        private byte replyType = TypeDshot300;
        private int lastInputMs;
        private bool driving;
        private uint framesIn;
        private uint repliesOut;
        private uint samplesOut;
        private uint repliesLost;

        private struct WatchEntry
        {
            public ulong Address;
            public byte Size;
            public ulong Last;
            public bool HaveLast;
            public ulong NextNs;
        }

        private readonly WatchEntry[] watches = new WatchEntry[WatchMax];
        private readonly byte[] watchBatch =
            new byte[4 + WatchBatchMax * WatchEventSize];
        private int watchCount;
        private uint watchMinPeriodNs;
        private EndPoint watchTo;
        private int watchSubscribeMs;
        private int watchBatchCount;
        private ulong watchLastFlushNs;
        private byte[] watchReq;

        private EndPoint sampleTo;
        private bool subscribed;
        private bool averaging;
        private int subscribeMs;
        private int batchCount;
        private ulong lastFlushNs;
        private uint lastReplyCount;

        // pacing (cmd 2): simulated over wall time to hold, 0 or >1 is
        // unpaced. Written by the state socket thread, read in Pace().
        private volatile float paceTarget;
        private readonly Stopwatch paceClock = Stopwatch.StartNew();
        private bool paceValid;
        private float paceApplied;
        private double paceVirtMs;
        private double paceWallMs;
    }
}
