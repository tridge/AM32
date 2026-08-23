//
// Generates a servo PWM throttle signal as pin edges.
//
// Servo is deliberately pin-level rather than writing capture values into
// dma_buffer directly: the point is to exercise the real detectInput() and
// checkServo() wrap arithmetic in Src/signal.c, which is the auto-detection
// logic most likely to carry an MCU porting bug. DShot has an optional
// frame-batch path, but it still supplies every captured edge through the
// timer and modeled DMA rather than injecting a decoded command.
//
// The output drives two things, and both are needed: the capture
// timer's channel 1 input, and the GPIO pin, because
// transfercomplete() in Src/signal.c calls getInputPinState() in servo
// mode to decide whether the next transfer wants two edges or three.
//
// Servo before dshot: same hardware path, but a servo frame is two
// edges at 50Hz against dshot's 16 bits at 600kbaud.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System.Collections.Generic;
using System.Linq;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Registered on the sysbus at an address no F051 peripheral occupies,
    // rather than at "none". That is not cosmetic: Renode 1.16 gives an
    // unregistered peripheral no path in the machine's name tree, so
    // "throttle PulseUs 1200" fails with "No such command or device" and
    // there is no way to change throttle mid-run. Registering it makes
    // both the monitor properties and the registers below work.
    //   0x00  servo pulse width, microseconds
    //   0x04  frame period, microseconds
    //   0x08  protocol: 0 servo, or a dshot bitrate in kbaud (150/300/600)
    //   0x0C  dshot throttle, the raw 11 bit value: 0 stops, 48..2047 drives
    //   0x10  bidirectional dshot: idle high, pulses low, CRC inverted
    // Also an IGPIOReceiver, which is how the shared bidirectional wire
    // is modelled: input 0 is the ESC driving the line back. Renode GPIO
    // has no contention, so this peripheral arbitrates - it drives its
    // own frame while transmitting, and otherwise passes the ESC's level
    // through to the pin. That matches the real half duplex wire, where
    // the flight controller releases it after each frame.
    public class AM32ThrottleGenerator : IDoubleWordPeripheral, IKnownSize,
                                         INumberedGPIOOutput, IGPIOReceiver
    {
        public AM32ThrottleGenerator(IMachine machine,
                                     bool batchDshotFrames = false,
                                     INumberedGPIOOutput escPort = null,
                                     int escPin = -1)
        {
            this.machine = machine;
            BatchDshotFrames = batchDshotFrames;
            // The ESC's own drive of the signal pin, needed to decode the
            // bootloader's bit banged serial reply. Wired here rather than
            // in the platform file because re-opening a peripheral block
            // in a repl does not add connections; Connect() appends an
            // endpoint alongside the port's existing EXTI wiring.
            escPinNumber = escPin < 0 ? 0 : escPin;
            if(escPort != null && escPin >= 0)
            {
                escPort.Connections[escPin].Connect(this, 1);
            }
            var conns = new Dictionary<int, IGPIO>();
            conns[0] = new GPIO();
            Connections = conns;

            // nanosecond ticks: dshot600's short high time is 625ns, so
            // the microsecond timebase the servo path used cannot express
            // a dshot bit at all
            frameTimer = new LimitTimer(machine.ClockSource, 1000000000, this,
                                        "throttle", DefaultFrameUs * 1000,
                                        direction: Direction.Ascending,
                                        enabled: false, autoUpdate: false,
                                        workMode: WorkMode.OneShot,
                                        eventEnabled: true);
            frameTimer.LimitReached += OnTimer;
            // the serial bit clock and the receive sampler: separate from
            // the frame timer so a serial session cannot disturb the
            // throttle state machine's own scheduling
            serialTimer = new LimitTimer(machine.ClockSource, 1000000000, this,
                                         "serialtx", 1000,
                                         direction: Direction.Ascending,
                                         enabled: false, autoUpdate: false,
                                         workMode: WorkMode.OneShot,
                                         eventEnabled: true);
            serialTimer.LimitReached += SerialStep;
            rxTimer = new LimitTimer(machine.ClockSource, 1000000000, this,
                                     "serialrx", 1000,
                                     direction: Direction.Ascending,
                                     enabled: false, autoUpdate: false,
                                     workMode: WorkMode.OneShot,
                                     eventEnabled: true);
            rxTimer.LimitReached += SerialRxSample;
            // Renode propagates only GPIO changes, and the guest's own
            // writes to the pin overwrite the port's view of it, so the
            // level an idle adapter holds has to be put back
            // periodically or it quietly stops being there. Both edges of
            // the refresh land in one host callback, so no guest
            // instruction can observe the intermediate level.
            holdTimer = new LimitTimer(machine.ClockSource, 1000000000, this,
                                       "serialhold", HoldNs,
                                       direction: Direction.Ascending,
                                       enabled: false, autoUpdate: true,
                                       workMode: WorkMode.Periodic,
                                       eventEnabled: true);
            holdTimer.LimitReached += HoldStep;
            // a free-running nanosecond clock: unlike
            // machine.ElapsedVirtualTime, a timer's value is
            // interpolated by the reporting CPU's own progress, so a
            // read from guest context gives the guest's clock at
            // sub-quantum precision - which is what lets AdvanceWire
            // place wire levels exactly where the bootloader samples
            clockTimer = new LimitTimer(machine.ClockSource, 1000000000,
                                        this, "serialclock",
                                        long.MaxValue,
                                        direction: Direction.Ascending,
                                        enabled: true, autoUpdate: true,
                                        workMode: WorkMode.Periodic,
                                        eventEnabled: false);
            PulseUs = 1000;
            FrameUs = DefaultFrameUs;
            Protocol = 0;
            DshotValue = 0;
            // self-starting: a @none peripheral is not addressable from
            // the monitor, and zero throttle is what arming needs anyway
            Enabled = true;
        }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }
        public long Size => 0x100;

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return PulseUs;
            case 0x04: return FrameUs;
            case 0x08: return Protocol;
            case 0x0C: return DshotValue;
            case 0x10: return Bidirectional ? 1u : 0u;
            case 0x24: return PinReadSkip();
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case 0x00: PulseUs = value; break;
            case 0x04: FrameUs = value; break;
            case 0x08: Protocol = value; break;
            case 0x0C: DshotValue = value; break;
            case 0x10: Bidirectional = value != 0; break;
            case 0x20: SkipDelay(value); break;
            }
        }

        // servo high time. 1000us is zero throttle on a default config;
        // Src/signal.c accepts 800 < pulse < 2200
        public uint PulseUs { get; set; }

        // frame period; 50Hz servo
        public uint FrameUs { get; set; }

        // 0 for servo, or a dshot bitrate in kbaud: 150, 300 or 600
        public uint Protocol
        {
            get { return protocol; }
            set
            {
                if(value != 0 && value != 150 && value != 300 && value != 600)
                {
                    throw new RecoverableException(string.Format(
                        "protocol {0} is not 0 (servo), 150, 300 or 600", value));
                }
                protocol = value;
                if(enabled)
                {
                    // restart cleanly rather than finish the frame in the
                    // old protocol's timing
                    Enabled = true;
                }
            }
        }

        // Bidirectional (inverted) dshot. The line idles high and pulses
        // low, which is how computeDshotDMA() recognises it: it counts
        // getInputPinState() being high between frames and sets
        // dshot_telemetry after 100. The CRC nibble is sent inverted to
        // match, since the firmware compares against ~received.
        public bool Bidirectional
        {
            get { return bidirectional; }
            set
            {
                bidirectional = value;
                if(enabled)
                {
                    Enabled = true;
                }
            }
        }

        // the raw 11 bit dshot throttle: 0 stops, 1..47 are commands,
        // 48..2047 drive. Not a pulse width - PulseUs stays the servo one.
        public uint DshotValue
        {
            get { return dshotValue; }
            set
            {
                if(value > 2047)
                {
                    throw new RecoverableException(string.Format(
                        "dshot value {0} does not fit in 11 bits", value));
                }
                dshotValue = value;
            }
        }

        // the telemetry request bit. AM32 does not need it set to process
        // a command, but a flight controller sets it, so carry what the
        // sender asked for rather than a fixed zero.
        public bool TelemetryBit { get; set; }

        // Batch a complete DShot input frame into the capture timer. This
        // removes 32 host clock callbacks per frame, while the timer still
        // generates all 32 CCR values and DMA requests for guest firmware.
        // It is optional so pin-level porting tests can retain every edge.
        public bool BatchDshotFrames
        {
            get { return batchDshotFrames; }
            set
            {
                batchDshotFrames = value;
                if(!value)
                {
                    CancelBatchedFrame();
                }
            }
        }

        public bool Enabled
        {
            get { return enabled; }
            set
            {
                enabled = value;
                if(!enabled)
                {
                    // stopping is a flight controller unplugging, not one
                    // holding the wire down: on a bidirectional link that
                    // means releasing it, so the ESC still owns its half
                    // of the frame and the firmware sees signal loss
                    // rather than a stuck-low line.
                    frameTimer.Enabled = false;
                    CancelBatchedFrame();
                    transmitting = false;
                    DriveIdle();
                    high = false;
                    return;
                }
                StartHigh();
            }
        }

        public void Reset()
        {
            if(serialMode)
            {
                // The wire side is the bench, not the ESC: a reboot does
                // not unplug the adapter holding the line, and the
                // bootloader decides whether to jump by reading exactly
                // that level as it starts. Drop any half sent frame, keep
                // driving the line.
                lock(serialSync)
                {
                    txBits.Clear();
                }
                serialTimer.Enabled = false;
                rxTimer.Enabled = false;
                txDriving = false;
                rxActive = false;
                DriveSerialIdle();
                // The GPIO port clears its input state in its own reset,
                // and Renode's GPIO drops a Set that matches what it last
                // propagated, so the level we hold would silently stop
                // being visible to the guest. The bootloader decides
                // whether to jump to the application by reading exactly
                // this pin, so push it again once the ports have reset
                // too - a short delay, since peripheral reset order is
                // not defined.
                rxBlankUntilNs = NowNs() + RxBlankNs;
                // once now, in case the GPIO port reset before us and
                // is ready to hear it...
                ForceWire(serialIdleHigh);
                // ...and once shortly after resume, in case it resets
                // after us and wipes the level again. The CAN bootloader
                // samples the pin within its first millisecond to decide
                // whether a signal is present, and a stale LOW there
                // reads as one - which waives its raw-command boot gate
                // and boots the app out from under the config session.
                reassertPending = true;
                escQuietTicks = 0;
                holdTimer.Enabled = true;
                serialTimer.Limit = ReassertNs;
                serialTimer.Enabled = true;
                return;
            }
            // The generator is the bench-side transmitter: an ESC
            // reboot (NVIC_SystemReset -> machine reset) does not
            // silence the radio or flight controller on the other end
            // of the wire, so the frame train restarts with its current
            // settings instead of going quiet - without this the
            // firmware's signal-loss reboot came back deaf and
            // re-reset every two seconds forever.
            high = false;
            bitIndex = 0;
            CancelBatchedFrame();
            transmitting = false;
            frameTimer.Enabled = false;
            Connections[0].Unset();
            if(enabled)
            {
                StartHigh();
            }
        }

        private void StartHigh()
        {
            if(protocol != 0)
            {
                StartDshotFrame();
                return;
            }
            high = true;
            Connections[0].Set();
            frameTimer.Limit = (ulong)PulseUs * 1000;
            frameTimer.Enabled = true;
        }

        private void OnTimer()
        {
            if(!enabled)
            {
                return;
            }
            if(protocol != 0)
            {
                DshotStep();
                return;
            }
            if(high)
            {
                // end of the pulse; stay low for the rest of the frame
                high = false;
                Connections[0].Unset();
                var rest = FrameUs > PulseUs ? FrameUs - PulseUs : 1;
                frameTimer.Limit = (ulong)rest * 1000;
                frameTimer.Enabled = true;
            }
            else
            {
                StartHigh();
            }
        }

        // 16 bits MSB first, each bit one bit period long with the line
        // high for 3/4 of it for a 1 and 3/8 for a 0. computeDshotDMA()
        // in Src/dshot.c decides each bit by comparing its high time
        // against a thirty-secondth of the whole frame, so the exact
        // fractions matter less than staying either side of that.
        private void StartDshotFrame()
        {
            frame = DshotFrame();
            bitIndex = 0;
            if(BatchDshotFrames)
            {
                if(dshotFrameSink == null)
                {
                    dshotFrameSink = machine.GetPeripheralsOfType<IAM32DshotFrameSink>()
                        .FirstOrDefault();
                }
                // The wire remains at its idle level for the host-side
                // batch. CompleteDshotFrame supplies the transitions as
                // capture timestamps after the simulated frame duration.
                transmitting = false;
                DriveIdle();
                if(dshotFrameSink != null
                   && dshotFrameSink.BeginDshotFrame(frame, BitPeriodNs))
                {
                    batchedFrame = true;
                    transmitting = true;
                    high = false;
                    frameTimer.Limit = BitPeriodNs * 16;
                    frameTimer.Enabled = true;
                    return;
                }
            }
            BeginBit();
        }

        private void BeginBit()
        {
            high = true;
            transmitting = true;
            DriveActive();
            var bitNs = BitPeriodNs;
            var oneBit = (frame & (0x8000u >> bitIndex)) != 0;
            frameTimer.Limit = oneBit ? bitNs * 3 / 4 : bitNs * 3 / 8;
            frameTimer.Enabled = true;
        }

        // "active" is high for plain dshot and low for bidirectional,
        // which inverts the whole waveform
        private void DriveActive()
        {
            if(bidirectional)
            {
                Connections[0].Unset();
            }
            else
            {
                Connections[0].Set();
            }
        }

        private void DriveIdle()
        {
            if(bidirectional)
            {
                // released: the ESC may be replying, so the wire follows
                // whatever it is driving
                Connections[0].Set(escLevel);
            }
            else
            {
                Connections[0].Unset();
            }
        }

        // the ESC's end of the shared wire: input 0 is the capture
        // timer's reply tap (bidirectional dshot), input 1 the signal
        // pin's own level (the bootloader's bit banged serial)
        public void OnGPIO(int number, bool value)
        {
            if(number != 0 && number != 1)
            {
                return;
            }
            escLevel = value;
            if(serialMode)
            {
                if(forcingWire)
                {
                    return;
                }
                if(value != serialIdleHigh)
                {
                    escQuietTicks = EscQuietTicks;
                }
                // the ESC's reply is decoded from its own pin rather than
                // from the shared wire, so our idle drive cannot mask it
                SerialRxEdge(value);
                return;
            }
            // only forward it while we are not driving a frame ourselves
            if(bidirectional && !transmitting)
            {
                Connections[0].Set(value);
            }
        }

        // --- one wire serial ------------------------------------------
        //
        // The bootloader talks 19200 8N1 bit-banged on this same wire, so
        // a configurator can be driven against an emulated ESC exactly as
        // against hardware. Transmission is a queue of line levels clocked
        // out one bit at a time; reception is a soft UART sampling the
        // ESC's own pin at the bit centres, started by its falling start
        // edge. Half duplex is the protocol's business, as on the wire.

        // bytes for the ESC. gap prepends idle bit times so the bootloader
        // sees a frame boundary, which is what inter-command latency does
        // on a real adapter
        public void QueueSerial(byte[] data, bool gap)
        {
            if(data == null || data.Length == 0)
            {
                return;
            }
            lock(serialSync)
            {
                if(gap)
                {
                    for(var i = 0; i < SerialGapBits; i++)
                    {
                        txBits.Enqueue(true);
                    }
                }
                foreach(var b in data)
                {
                    txBits.Enqueue(false);              // start
                    for(var i = 0; i < 8; i++)
                    {
                        txBits.Enqueue(((b >> i) & 1) != 0);   // lsb first
                    }
                    txBits.Enqueue(true);               // stop
                }
            }
            EnterSerialMode();
            SerialKick();
        }

        // whatever the ESC has bit-banged back since the last call
        // one-shot: the TX bit queue ran dry since the last call
        public bool TakeTxDrained()
        {
            lock(serialSync)
            {
                var was = txDrainedEvent;
                txDrainedEvent = false;
                return was;
            }
        }

        public byte[] TakeSerialRx()
        {
            lock(serialSync)
            {
                if(serialRx.Count == 0)
                {
                    return null;
                }
                var outBytes = serialRx.ToArray();
                serialRx.Clear();
                return outBytes;
            }
        }

        // a constant line state, as an adapter holding the wire at boot
        public void SetLineLevel(bool high, bool floating)
        {
            EnterSerialMode();
            lock(serialSync)
            {
                txBits.Clear();
            }
            serialTimer.Enabled = false;
            txDriving = false;
            // floating releases the wire; the bootloader's own pull-up
            // decides, which here means leaving the line where the ESC has
            // it rather than driving a level of our own
            serialIdleHigh = floating ? escLevel : high;
            DriveSerialIdle();
        }

        public uint SerialBaud
        {
            get { return serialBaud; }
            set { serialBaud = value == 0 ? DefaultSerialBaud : value; }
        }

        // The bootloader's delayMicroseconds busy-polls the utility
        // timer's CNT at one native-to-managed transition per four
        // instructions, which is what makes bootloader serial run
        // several times slower than its own wire. The generated target
        // script patches the function to write its argument here and
        // return; jumping virtual time forward costs one IO access per
        // delay. (An execution hook on the function was tried first: a
        // hook fire costs ~2ms of translation machinery, far more than
        // the busy-wait it replaces.) No interrupt gate: the plain
        // bootloaders never enable one, and the CAN bootloaders only
        // queue frames from theirs, so delivery at the end of a skipped
        // interval instead of part-way through changes nothing.
        public ulong SkipDelayTimerCnt { get; set; }
        public ulong SkipDelayElapsedVar { get; set; }

        // The bootloader's serial wait loops poll the signal pin's IDR
        // (and the utility timer) once per handful of instructions, the
        // same native-to-managed burn as the delay loops. The generated
        // target script redirects gpio_read's IDR load here: the read
        // returns the real IDR content, and - only while nothing is on
        // the wire and nothing is queued to go onto it - jumps virtual
        // time forward a little, so an idle wait costs one iteration
        // per SkipWaitUs of virtual time instead of one per ~200ns.
        // The budget for the skip: the guest samples each bit half a
        // bit (26us) after the start edge it detected, and detection
        // can already be late by the skip plus up to one global quantum
        // (20us) of scheduling skew, so skip + 26 + 20 must stay inside
        // the 52us bit. 15us failed exactly that bound in practice
        // (reads corrupted, connects flaky); 5us holds it with margin.
        // The skip is withheld whenever the generator is driving bits
        // (the guest is then mid-byte, sampling at exact
        // delayMicroseconds offsets).
        public ulong SkipPinIdr { get; set; }
        public uint SkipWaitUs { get; set; } = 5;

        private uint PinReadSkip()
        {
            if(SkipPinIdr == 0)
            {
                return 0;
            }
            var now = NowNs();
            if(machine.SystemBus.TryGetCurrentCPU(out var icpu)
               && icpu is TranslationCPU tcpu)
            {
                ulong skip = 0;
                if(!txDriving)
                {
                    bool pending;
                    lock(serialSync)
                    {
                        pending = txBits.Count > 0;
                    }
                    if(!pending)
                    {
                        skip = SkipWaitUs;
                    }
                }
                else if(now - lastDelayNs > 200000
                        && txDeadlineNs > now + 1000)
                {
                    // Mid-transmission the guest is either sampling bits
                    // (always within a delayMicroseconds or two of the
                    // last one - do not disturb its timing) or spinning
                    // in a start-bit wait across a level run, where time
                    // can be skipped as long as no edge is crossed: the
                    // level cannot change before the run's deadline.
                    skip = System.Math.Min((ulong)SkipWaitUs,
                                           (txDeadlineNs - now) / 1000);
                }
                if(skip > 0)
                {
                    // skip first, then sample: an edge landing inside
                    // the skipped stretch is visible to this very read
                    tcpu.SkipTime(TimeInterval.FromMicroseconds(skip));
                    now = NowNs();
                }
            }
            AdvanceWire(now);
            if(serialMode && !txDriving && !rxActive)
            {
                // an idle wire always sits at the held level; a mismatch
                // means a machine reset wiped the port's view of it, so
                // re-drive before the guest samples a stale LOW
                var idr = machine.SystemBus.ReadDoubleWord(SkipPinIdr);
                var bit = (idr >> escPinNumber) & 1;
                if((bit != 0) != serialIdleHigh)
                {
                    bool pending;
                    lock(serialSync)
                    {
                        pending = txBits.Count > 0;
                    }
                    if(!pending)
                    {
                        ForceWire(serialIdleHigh);
                    }
                }
            }
            return machine.SystemBus.ReadDoubleWord(SkipPinIdr);
        }

        private void SkipDelay(uint us)
        {
            if(us == 0 || !machine.SystemBus.TryGetCurrentCPU(out var icpu)
               || !(icpu is TranslationCPU tcpu))
            {
                return;
            }
            tcpu.SkipTime(TimeInterval.FromMicroseconds(us));
            lastDelayNs = NowNs();
            if(SkipDelayTimerCnt != 0 && SkipDelayElapsedVar != 0)
            {
                // the patched-out function stored the timer count in
                // us_start on entry, and receiveBuffer's inter-byte gap
                // check measures from it; reproduce the side effect as
                // of the delay's end
                var cnt = machine.SystemBus.ReadDoubleWord(
                    SkipDelayTimerCnt) & 0xFFFFu;
                machine.SystemBus.WriteWord(
                    SkipDelayElapsedVar, (ushort)((cnt + us) & 0xFFFF));
            }
        }

        // the wire is one or the other: a throttle train and a serial
        // session cannot share it, which is true of the hardware too
        private void EnterSerialMode()
        {
            if(serialMode)
            {
                return;
            }
            serialMode = true;
            frameTimer.Enabled = false;
            CancelBatchedFrame();
            transmitting = false;
            high = false;
            serialIdleHigh = true;
            DriveSerialIdle();
            holdTimer.Enabled = true;
        }

        public void LeaveSerialMode()
        {
            if(!serialMode)
            {
                return;
            }
            serialMode = false;
            holdTimer.Enabled = false;
            serialTimer.Enabled = false;
            rxTimer.Enabled = false;
            rxActive = false;
            txDriving = false;
            lock(serialSync)
            {
                txBits.Clear();
            }
            if(enabled)
            {
                StartHigh();
            }
        }

        private ulong SerialBitNs()
        {
            return 1000000000UL / serialBaud;
        }

        private void DriveSerialIdle()
        {
            Connections[0].Set(serialIdleHigh);
        }

        // Renode's GPIO only propagates a change, so a level the far end
        // has forgotten needs the flip to get there. The flip comes back
        // to us through the port's echo, and must not be mistaken for the
        // ESC starting a byte, so the receiver is gated while it runs.
        private void ForceWire(bool level)
        {
            forcingWire = true;
            Connections[0].Set(!level);
            Connections[0].Set(level);
            forcingWire = false;
        }

        private void HoldStep()
        {
            if(!serialMode || txDriving || rxActive)
            {
                return;
            }
            if(escQuietTicks > 0)
            {
                // the ESC is talking: the wire is its own until it stops
                escQuietTicks--;
                return;
            }
            ForceWire(serialIdleHigh);
        }

        private void SerialKick()
        {
            if(txDriving)
            {
                return;
            }
            bool any;
            lock(serialSync)
            {
                any = txBits.Count > 0;
            }
            if(!any)
            {
                return;
            }
            txDriving = true;
            txDeadlineNs = NowNs();
            AdvanceWire(txDeadlineNs, reschedule: true);
        }

        private ulong NowNs()
        {
            return clockTimer.Value;
        }

        private void SerialStep()
        {
            if(reassertPending)
            {
                reassertPending = false;
                ForceWire(serialIdleHigh);
                SerialKick();
                return;
            }
            AdvanceWire(NowNs(), reschedule: true);
        }

        // Drive the wire through every transition whose time has come.
        // The timer only guarantees progress: timer events land on
        // quantum boundaries, so a bootloader sampling at exact
        // delayMicroseconds offsets would see edges up to a quantum
        // late and misread bits near the boundary. The patched
        // gpio_read calls in here too, so the level a sample returns is
        // computed for the guest's precise virtual time. Deadlines are
        // virtual, one per EDGE (a run of equal bits is one level held
        // for run*bitNs), so lateness never accumulates into the byte.
        private void AdvanceWire(ulong now, bool reschedule = false)
        {
            // dirty fast path: the guest calls this on every pin read,
            // mostly finding nothing due yet
            if(!txDriving || (!reschedule && now < txDeadlineNs))
            {
                return;
            }
            var edges = new System.Collections.Generic.List<bool>();
            var drained = false;
            lock(serialSync)
            {
                if(!txDriving)
                {
                    return;
                }
                while(now >= txDeadlineNs)
                {
                    if(txBits.Count == 0)
                    {
                        txDriving = false;
                        // everything queued has left the wire: tell the
                        // serial client, whose echo pacing runs on our
                        // virtual clock, not its own
                        txDrainedEvent = true;
                        drained = true;
                        break;
                    }
                    var bit = txBits.Dequeue();
                    var run = 1;
                    while(txBits.Count > 0 && txBits.Peek() == bit)
                    {
                        txBits.Dequeue();
                        run++;
                    }
                    edges.Add(bit);
                    txDeadlineNs += (ulong)run * SerialBitNs();
                }
            }
            if(edges.Count > 2)
            {
                this.Log(LogLevel.Warning,
                         "COLLAPSE {0} runs at now={1} dl={2}",
                         edges.Count, now, txDeadlineNs);
            }
            foreach(var bit in edges)
            {
                Connections[0].Set(bit);
            }
            if(drained)
            {
                // release: the ESC answers into the idle line
                DriveSerialIdle();
                return;
            }
            // guest-context calls keep the wire advanced by themselves;
            // re-arming the timer thousands of times per frame would
            // only churn the clock source
            if(reschedule || !serialTimer.Enabled)
            {
                serialTimer.Limit = txDeadlineNs > now ? txDeadlineNs - now
                    : 1;
                serialTimer.Enabled = true;
            }
        }

        // Decode the ESC's reply from its own edges rather than by
        // sampling every bit centre: the edges arrive as GPIO events
        // anyway (placed precisely in virtual time by the guest's pin
        // writes), so the level between two edges decides all the bits
        // in between and the only timer needed is one per byte, at the
        // stop bit. This is also more tolerant of the bit-banged
        // sender's timing than fixed-phase sampling, since every edge
        // re-anchors the decoding.
        private void SerialRxEdge(bool level)
        {
            var now = NowNs();
            if(now < rxBlankUntilNs)
            {
                // A machine reset makes the GPIO port emit its cleared
                // pin state, which looks exactly like the ESC pulling
                // the line for a start bit; treating it as one wedges
                // the receiver (no further edges ever come) with the
                // wire released low. A real MCU's pin goes high-Z
                // through reset and the adapter keeps the line at idle,
                // so ignore esc-side edges for the first moments after
                // a reset.
                return;
            }
            if(txDriving)
            {
                // The guest samples our final stop bit mid-bit and can
                // start its reply before that bit's period has formally
                // ended; bring the drain bookkeeping up to its clock or
                // the reply's start edge would be discarded as our own.
                AdvanceWire(now);
            }
            if(!rxActive)
            {
                if(level || txDriving)
                {
                    return;
                }
                RxStart(now);
                return;
            }
            var bitNs = SerialBitNs();
            // which bit boundary this edge lands on, counted from the
            // start bit's falling edge
            var k = (int)(((now - rxStartNs) + bitNs / 2) / bitNs);
            RxFillTo(k - 1);
            if(k >= 10)
            {
                RxFinish();
                if(!level && !txDriving)
                {
                    // the sender ran on into the next byte's start bit
                    RxStart(now);
                    return;
                }
            }
            rxLastLevel = level;
        }

        private void RxStart(ulong now)
        {
            rxActive = true;
            rxStartNs = now;
            rxByte = 0;
            rxFilled = 0;
            rxLastLevel = false;
            // fires in the middle of the stop bit
            rxTimer.Limit = SerialBitNs() * 19 / 2;
            rxTimer.Enabled = true;
        }

        // bits rxFilled..upto-1 carry the level held since the last edge
        private void RxFillTo(int upto)
        {
            if(upto > 8)
            {
                upto = 8;
            }
            while(rxFilled < upto)
            {
                if(rxLastLevel)
                {
                    rxByte |= (byte)(1 << rxFilled);
                }
                rxFilled++;
            }
        }

        private void RxFinish()
        {
            rxTimer.Enabled = false;
            RxFillTo(8);
            // the stop bit is whatever level the wire held after the
            // last edge; low means we mis-tracked, drop the byte
            if(rxLastLevel)
            {
                lock(serialSync)
                {
                    serialRx.Enqueue(rxByte);
                }
            }
            rxActive = false;
        }

        // the per-byte timer, in the middle of the stop bit
        private void SerialRxSample()
        {
            if(!rxActive)
            {
                return;
            }
            RxFinish();
        }

        private void DshotStep()
        {
            if(batchedGap)
            {
                batchedGap = false;
                StartDshotFrame();
                return;
            }
            if(batchedFrame)
            {
                batchedFrame = false;
                transmitting = false;
                high = false;
                DriveIdle();
                dshotFrameSink.CompleteDshotFrame();
                batchedGap = true;
                frameTimer.Limit = FrameGapNs;
                frameTimer.Enabled = true;
                return;
            }
            var bitNs = BitPeriodNs;
            if(high)
            {
                high = false;
                DriveIdle();
                var oneBit = (frame & (0x8000u >> bitIndex)) != 0;
                var highNs = oneBit ? bitNs * 3 / 4 : bitNs * 3 / 8;
                frameTimer.Limit = bitNs - highNs;
                frameTimer.Enabled = true;
                return;
            }
            if(bitIndex >= 16)
            {
                // the gap just ended; the next frame starts now
                StartDshotFrame();
                return;
            }
            bitIndex++;
            if(bitIndex < 16)
            {
                BeginBit();
                return;
            }
            // frame done. On a bidirectional wire this is where the line
            // is released so the ESC can answer inside the gap.
            transmitting = false;
            DriveIdle();
            frameTimer.Limit = FrameGapNs;
            frameTimer.Enabled = true;
        }

        private ulong BitPeriodNs
        {
            get
            {
                switch(protocol)
                {
                case 150: return 6667;
                case 300: return 3333;
                default: return 1667;
                }
            }
        }

        // Dshot frame period. The default 250us is 4kHz, a rate a flight
        // controller would really use, and cheap: every edge is a timer
        // event, so the 19kHz that falls out of a minimal gap costs nearly
        // five times as much to simulate for no added coverage. Raising it
        // is the cheapest speed lever there is when the wire is not what
        // is under test - the cost is proportional.
        public uint DshotFrameUs
        {
            get { return dshotFrameUs; }
            set { dshotFrameUs = value == 0 ? 1u : value; }
        }

        // Idle between frames. Falls back to a bit period if the frame
        // alone is longer than the requested period.
        private ulong FrameGapNs
        {
            get
            {
                var frameNs = BitPeriodNs * 16;
                var periodNs = (ulong)dshotFrameUs * 1000;
                return periodNs > frameNs ? periodNs - frameNs : BitPeriodNs;
            }
        }

        private uint dshotFrameUs = 250;

        // 11 bit value, telemetry request, then a 4 bit CRC over the
        // three nibbles above it
        private uint DshotFrame()
        {
            var payload = (dshotValue << 1) | (TelemetryBit ? 1u : 0u);
            var crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0xF;
            if(bidirectional)
            {
                // computeDshotDMA() compares its own CRC against
                // ~received, so the wire carries the complement
                crc = ~crc & 0xF;
            }
            return ((payload << 4) | crc) & 0xFFFF;
        }

        private void CancelBatchedFrame()
        {
            batchedFrame = false;
            batchedGap = false;
            if(dshotFrameSink != null)
            {
                dshotFrameSink.CancelDshotFrame();
            }
        }

        private const uint DefaultFrameUs = 20000;

        private readonly IMachine machine;
        private readonly LimitTimer frameTimer;
        private readonly LimitTimer serialTimer;
        private readonly LimitTimer rxTimer;
        private readonly LimitTimer holdTimer;
        private readonly LimitTimer clockTimer;
        private readonly object serialSync = new object();
        private readonly Queue<bool> txBits = new Queue<bool>();
        private readonly Queue<byte> serialRx = new Queue<byte>();
        private const uint DefaultSerialBaud = 19200;
        // a frame separator the bootloader's byte timeout can see
        private const int SerialGapBits = 20;
        private uint serialBaud = DefaultSerialBaud;
        private bool serialMode;
        private bool serialIdleHigh = true;
        private bool txDriving;
        private bool txDrainedEvent;
        private readonly int escPinNumber;
        private ulong rxBlankUntilNs;
        // well past every reset-time GPIO emission, well short of the
        // earliest possible real reply (the guest is still in clock
        // init): 200us
        private const ulong RxBlankNs = 200000;
        private ulong lastDelayNs;
        private bool rxActive;
        private ulong rxStartNs;
        private int rxFilled;
        private bool rxLastLevel;
        private ulong txDeadlineNs;
        private bool reassertPending;
        private const ulong ReassertNs = 20000;   // 20us after a reset
        private const ulong HoldNs = 250000;      // idle level refresh
        private const int EscQuietTicks = 8;      // refreshes to skip after
        private int escQuietTicks;                // the ESC drove the wire
        private bool forcingWire;
        private byte rxByte;
        private bool high;
        private bool enabled;
        private uint protocol;
        private uint dshotValue;
        private bool bidirectional;
        // true while we hold the shared wire for our own frame
        private bool transmitting;
        // last level the ESC drove, followed when we are not transmitting
        private bool escLevel = true;
        private uint frame;
        private int bitIndex;
        private IAM32DshotFrameSink dshotFrameSink;
        private bool batchedFrame;
        private bool batchedGap;
        private bool batchDshotFrames;
    }
}
