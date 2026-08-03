//
// Generates a servo PWM throttle signal as pin edges.
//
// Deliberately pin-level rather than writing capture values into
// dma_buffer directly: the point is to exercise the real detectInput()
// and checkServo() wrap arithmetic in Src/signal.c, which is the
// auto-detection logic most likely to carry an MCU porting bug. Poking
// the buffer would bypass exactly the code under test.
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
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System.Collections.Generic;

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
        public AM32ThrottleGenerator(IMachine machine)
        {
            this.machine = machine;
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

        public bool Enabled
        {
            get { return enabled; }
            set
            {
                enabled = value;
                if(!enabled)
                {
                    frameTimer.Enabled = false;
                    Connections[0].Unset();
                    high = false;
                    return;
                }
                StartHigh();
            }
        }

        public void Reset()
        {
            frameTimer.Enabled = false;
            Connections[0].Unset();
            high = false;
            enabled = false;
            bitIndex = 0;
            transmitting = false;
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

        // the ESC's end of the shared wire
        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                return;
            }
            escLevel = value;
            // only forward it while we are not driving a frame ourselves
            if(bidirectional && enabled && !transmitting)
            {
                Connections[0].Set(value);
            }
        }

        private void DshotStep()
        {
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

        // Idle between frames, sized to give a 4kHz frame rate - a rate a
        // flight controller would really use, and cheap: every edge is a
        // timer event, so the 19kHz that falls out of a minimal gap costs
        // nearly five times as much to simulate for no added coverage.
        // Falls back to a bit period if the frame alone is longer.
        private ulong FrameGapNs
        {
            get
            {
                var frameNs = BitPeriodNs * 16;
                return DshotPeriodNs > frameNs ? DshotPeriodNs - frameNs
                                               : BitPeriodNs;
            }
        }

        private const ulong DshotPeriodNs = 250000;

        // 11 bit value, telemetry request, then a 4 bit CRC over the
        // three nibbles above it
        private uint DshotFrame()
        {
            var payload = (dshotValue << 1) | 0u; // no telemetry request
            var crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0xF;
            if(bidirectional)
            {
                // computeDshotDMA() compares its own CRC against
                // ~received, so the wire carries the complement
                crc = ~crc & 0xF;
            }
            return ((payload << 4) | crc) & 0xFFFF;
        }

        private const uint DefaultFrameUs = 20000;

        private readonly IMachine machine;
        private readonly LimitTimer frameTimer;
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
    }
}
