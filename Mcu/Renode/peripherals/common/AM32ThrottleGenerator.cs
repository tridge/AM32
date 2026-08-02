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
    //   0x00  pulse width, microseconds
    //   0x04  frame period, microseconds
    public class AM32ThrottleGenerator : IDoubleWordPeripheral, IKnownSize,
                                         INumberedGPIOOutput
    {
        public AM32ThrottleGenerator(IMachine machine)
        {
            this.machine = machine;
            var conns = new Dictionary<int, IGPIO>();
            conns[0] = new GPIO();
            Connections = conns;

            frameTimer = new LimitTimer(machine.ClockSource, 1000000, this,
                                        "throttle", DefaultFrameUs,
                                        direction: Direction.Ascending,
                                        enabled: false, autoUpdate: false,
                                        workMode: WorkMode.OneShot,
                                        eventEnabled: true);
            frameTimer.LimitReached += OnTimer;
            PulseUs = 1000;
            FrameUs = DefaultFrameUs;
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
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case 0x00: PulseUs = value; break;
            case 0x04: FrameUs = value; break;
            }
        }

        // servo high time. 1000us is zero throttle on a default config;
        // Src/signal.c accepts 800 < pulse < 2200
        public uint PulseUs { get; set; }

        // frame period; 50Hz servo
        public uint FrameUs { get; set; }

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
        }

        private void StartHigh()
        {
            high = true;
            Connections[0].Set();
            frameTimer.Limit = PulseUs;
            frameTimer.Enabled = true;
        }

        private void OnTimer()
        {
            if(!enabled)
            {
                return;
            }
            if(high)
            {
                // end of the pulse; stay low for the rest of the frame
                high = false;
                Connections[0].Unset();
                var rest = FrameUs > PulseUs ? FrameUs - PulseUs : 1;
                frameTimer.Limit = rest;
                frameTimer.Enabled = true;
            }
            else
            {
                StartHigh();
            }
        }

        private const uint DefaultFrameUs = 20000;

        private readonly IMachine machine;
        private readonly LimitTimer frameTimer;
        private bool high;
        private bool enabled;
    }
}
