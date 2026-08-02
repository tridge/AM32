//
// Couples the emulated bridge to the SITL motor physics.
//
// Everything this reads comes from registers the real Mcu/f051/Src code
// wrote, which is the reason the project exists:
//   - GPIO MODER and ODR give the per-phase bridge mode, exactly as
//     phaseouts.c set them. Renode's stock STM32_GPIOPort does implement
//     MODER, so no GPIO model of our own is needed.
//   - TIM1 gives ARR, the three compare values, the prescaler and the
//     dead time.
//   - COMP1's CSR says which phase is being sensed, and the result goes
//     back into COMP1OUT, from where the real EXTI raises the interrupt.
//
// The physics itself is Mcu/SITL/sim/motor.c, unmodified, reached
// through Mcu/Renode/sim/am32sim_shim.c. See that file for why it is not
// transliterated into C#.
//
// Phase modes and the pin map are for HARDWARE_GROUP_F0_A. A second
// target group means a second pin map, which is the natural place for a
// per-target .repl overlay to plug in.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System;
using System.Linq;
using System.Runtime.InteropServices;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Registered on the bus rather than at "none" because Renode gives an
    // unregistered peripheral no path in the machine name tree, so the
    // .resc could not set LibraryPath or ConfigPath on it.
    //   0x00  motor rpm, mechanical, read only
    public class AM32_F051_Bridge : IDoubleWordPeripheral, IKnownSize
    {
        public AM32_F051_Bridge(IMachine machine, uint batchUs = 2)
        {
            this.machine = machine;
            this.batchUs = batchUs == 0 ? 1u : batchUs;

            batch = new LimitTimer(machine.ClockSource, 1000000, this, "bridge",
                                   this.batchUs,
                                   direction: Direction.Ascending,
                                   enabled: false, autoUpdate: true,
                                   eventEnabled: true);
            batch.LimitReached += Tick;
        }

        // absolute path to libam32sim.so. Setting it loads the library
        // RTLD_GLOBAL so the DllImports below resolve to it, which is
        // how the physics gets in without putting the build tree on the
        // system library path.
        public string LibraryPath
        {
            set
            {
                if(dlopen(value, RtldNow | RtldGlobal) == IntPtr.Zero)
                {
                    throw new RecoverableException(string.Format(
                        "could not load the motor library from '{0}'", value));
                }
                loaded = true;
            }
        }

        // model JSON, the same files Mcu/SITL/models holds. Empty uses
        // the built-in defaults.
        public string ConfigPath
        {
            set
            {
                if(!loaded)
                {
                    throw new RecoverableException("set LibraryPath before ConfigPath");
                }
                am32sim_init(value ?? "");
                started = true;
                batch.Enabled = true;
            }
        }

        public long Size => 0x100;

        public uint ReadDoubleWord(long offset)
        {
            return offset == 0 ? (uint)Math.Max(0.0, Rpm) : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public double Rpm
        {
            get
            {
                double omega = 0, theta = 0, rpm = 0;
                if(started)
                {
                    am32sim_get_state(ref omega, ref theta, ref rpm);
                }
                return rpm;
            }
        }

        public void Reset()
        {
            batch.Enabled = false;
            started = false;
        }

        private void Tick()
        {
            if(!started)
            {
                return;
            }
            if(timer == null)
            {
                timer = machine.GetPeripheralsOfType<AM32_STM32_AdvancedTimer>().FirstOrDefault();
                comp = machine.GetPeripheralsOfType<AM32_STM32F0_SysCfgComp>().FirstOrDefault();
                if(timer == null || comp == null)
                {
                    this.Log(LogLevel.Error, "no TIM1 or COMP in the platform; bridge disabled");
                    batch.Enabled = false;
                    return;
                }
            }

            var moderA = machine.SystemBus.ReadDoubleWord(GpioABase);
            var odrA = machine.SystemBus.ReadDoubleWord(GpioABase + OdrOffset);
            var moderB = machine.SystemBus.ReadDoubleWord(GpioBBase);
            var odrB = machine.SystemBus.ReadDoubleWord(GpioBBase + OdrOffset);

            var moe = timer.MainOutputEnabled;
            var a = PhaseMode(moe, moderA, odrA, 10, moderB, odrB, 1);
            var b = PhaseMode(moe, moderA, odrA, 9, moderB, odrB, 0);
            var c = PhaseMode(moe, moderA, odrA, 8, moderA, odrA, 7);
            am32sim_set_bridge(a, b, c);

            // TIM1 channel to phase: CH1 is C, CH2 is B, CH3 is A on this
            // pin map, so the compare values are read in that order
            var arr = machine.SystemBus.ReadDoubleWord(Tim1Base + 0x2C);
            var psc = machine.SystemBus.ReadDoubleWord(Tim1Base + 0x28);
            var ccrC = machine.SystemBus.ReadDoubleWord(Tim1Base + 0x34);
            var ccrB = machine.SystemBus.ReadDoubleWord(Tim1Base + 0x38);
            var ccrA = machine.SystemBus.ReadDoubleWord(Tim1Base + 0x3C);
            am32sim_set_tim1(arr, ccrA, ccrB, ccrC, (psc + 1) * TickPs, timer.DeadTimeNs);

            var sensed = comp.SensedPhase;
            if(sensed >= 0)
            {
                am32sim_set_comp_phase(sensed);
            }

            var nowNs = (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds * 1000;
            // a bridge that is off cannot change a motor that is not
            // turning; the shim skips those steps, which is most of boot
            var driven = (a != 0 || b != 0 || c != 0) ? 1 : 0;
            comp.CompOutput = am32sim_advance(nowNs, driven) != 0;
        }

        // SITL_PHASE_*: 0 float, 1 low, 2 pwm, 3 pwm without
        // complementary, 4 proportional brake
        private static int PhaseMode(bool moe, uint moderHi, uint odrHi, int pinHi,
                                     uint moderLo, uint odrLo, int pinLo)
        {
            if(!moe)
            {
                return 0;
            }
            var hi = (moderHi >> (2 * pinHi)) & 3;
            var lo = (moderLo >> (2 * pinLo)) & 3;
            if(hi == ModeAlternate)
            {
                return lo == ModeAlternate ? 2 : 3;
            }
            if(lo == ModeAlternate)
            {
                // high side held off, low side switching: proportionalBrake()
                return 4;
            }
            return ((odrLo >> pinLo) & 1) != 0 ? 1 : 0;
        }

        private const uint ModeAlternate = 2;
        private const ulong GpioABase = 0x48000000;
        private const ulong GpioBBase = 0x48000400;
        private const ulong OdrOffset = 0x14;
        private const ulong Tim1Base = 0x40012C00;
        // one 48MHz tick in picoseconds
        private const uint TickPs = 20833;

        private const int RtldNow = 2;
        private const int RtldGlobal = 0x100;

        [DllImport("dl", EntryPoint = "dlopen")]
        private static extern IntPtr dlopen(string path, int flags);

        [DllImport("am32sim")]
        private static extern int am32sim_init(string configPath);
        [DllImport("am32sim")]
        private static extern void am32sim_set_bridge(int a, int b, int c);
        [DllImport("am32sim")]
        private static extern void am32sim_set_tim1(uint arr, uint ccrA, uint ccrB,
                                                    uint ccrC, uint tickPs, uint deadNs);
        [DllImport("am32sim")]
        private static extern void am32sim_set_comp_phase(int phase);
        [DllImport("am32sim")]
        private static extern int am32sim_advance(ulong nowNs, int driven);
        [DllImport("am32sim")]
        private static extern void am32sim_get_state(ref double omega, ref double theta,
                                                     ref double rpm);

        private readonly IMachine machine;
        private readonly uint batchUs;
        private readonly LimitTimer batch;
        private AM32_STM32_AdvancedTimer timer;
        private AM32_STM32F0_SysCfgComp comp;
        private bool loaded;
        private bool started;
    }
}
