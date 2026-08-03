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
// The phase pin map is a constructor parameter, written as "PA10"/"PB1".
// It is not the same for every F051 target: most put phase A on
// PA10/PB1, but a third of them rotate the phases across the same six
// pins, and using the wrong map is silent. It comes from PHASE_x_GPIO_*
// in Inc/targets.h, via Mcu/Renode/gen_target.py.
//
// The TIM1 compare register for each phase is derived rather than
// passed: the high side pin fixes it, since TIM1_CH1 is PA8, CH2 is PA9
// and CH3 is PA10. (Trapezoidal drive writes all three compares the same
// value, so this only bites in sine mode.)
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
        // gpio bases default to the F0 map; the G0 puts its ports at
        // 0x50000000 instead, so the platform states them there.
        //
        // On an "enable" topology target the two pins per phase are the
        // gate driver's PWM and ENABLE rather than a high and low side;
        // see PhaseMode for what each combination means. The pins are
        // still passed as High (=PWM) and Low (=ENABLE) so the platform
        // shape is the same either way.
        public AM32_F051_Bridge(IMachine machine, string phaseAHigh, string phaseALow,
                                string phaseBHigh, string phaseBLow,
                                string phaseCHigh, string phaseCLow, uint batchUs = 2,
                                ulong gpioABase = 0x48000000, ulong gpioBBase = 0x48000400,
                                ulong gpioCBase = 0x48000800,
                                uint timerHz = 48000000,
                                bool invertedLow = false, bool invertedHigh = false,
                                string topology = "highlow", uint timerAf = 2,
                                ulong syscfgBase = 0)
        {
            this.machine = machine;
            this.batchUs = batchUs == 0 ? 1u : batchUs;
            this.timerAf = timerAf;
            this.syscfgBase = syscfgBase;
            gpioBase = new[] { gpioABase, gpioBBase, gpioCBase };
            this.invertedLow = invertedLow;
            // no AM32 target defines USE_INVERTED_HIGH today, so rather
            // than model it untested, refuse loudly if one appears
            if(invertedHigh)
            {
                throw new RecoverableException(
                    "USE_INVERTED_HIGH is not modelled by the bridge");
            }
            if(topology != "highlow" && topology != "enable")
            {
                throw new RecoverableException(string.Format(
                    "topology '{0}' is not 'highlow' or 'enable'", topology));
            }
            enableBridge = topology == "enable";
            if(timerHz == 0)
            {
                throw new RecoverableException("timerHz must not be zero");
            }
            // one TIM1 tick in picoseconds: 20833 at 48MHz on the F0,
            // 15625 at 64MHz on the G0
            tickPs = (uint)(1000000000000ul / timerHz);

            phases = new[]
            {
                new Phase(phaseAHigh, phaseALow),
                new Phase(phaseBHigh, phaseBLow),
                new Phase(phaseCHigh, phaseCLow),
            };

            // matches sitl_comp_phase's initial value in the shim
            LastSensedPhase = 2;

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
                configPath = value ?? "";
                am32sim_init(configPath);
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

        // rotor angle, radians mechanical
        public double Theta
        {
            get
            {
                double omega = 0, theta = 0, rpm = 0;
                if(started)
                {
                    am32sim_get_state(ref omega, ref theta, ref rpm);
                }
                return theta;
            }
        }

        // What the ADC model samples: bus voltage, bus current into the
        // bridge, and temperature. Same source the SITL's ADC.c reads.
        public double BusVoltage => Sensor(0);
        public double BusCurrent => Sensor(1);
        public double TemperatureC => Sensor(2);

        private double Sensor(int which)
        {
            if(!started)
            {
                return 0;
            }
            double volts = 0, amps = 0, degrees = 0;
            am32sim_get_sensors(ref volts, ref amps, ref degrees);
            return which == 0 ? volts : (which == 1 ? amps : degrees);
        }

        // What the last tick made of the bridge, kept so the state stream
        // can publish it without recomputing: the per-phase mode
        // (SITL_PHASE_*), which phase the comparator watches, and what it
        // answered.
        public bool Started => started;
        public int LastPhaseMode(int phase) => lastMode[phase];
        public int LastSensedPhase { get; private set; }
        public bool LastCompOut { get; private set; }

        // phase currents in amps, motor truth rather than anything the
        // firmware measures
        public double CurrentA => Current(0);
        public double CurrentB => Current(1);
        public double CurrentC => Current(2);

        private double Current(int phase)
        {
            if(!started)
            {
                return 0;
            }
            var i = new double[3];
            am32sim_get_currents(i);
            return i[phase];
        }

        // A machine reset must put the motor back at standstill and keep
        // driving it, not switch the physics off: the firmware reboots
        // itself on signal loss while armed, and a bridge that stayed
        // dead after that would report a frozen rotor and zero sensors
        // for the rest of the run.
        public void Reset()
        {
            batch.Enabled = false;
            started = false;
            timer = null;
            comp = null;
            syscfg = null;
            Array.Clear(gpio, 0, gpio.Length);
            Array.Clear(lastMode, 0, lastMode.Length);
            LastSensedPhase = 2;
            LastCompOut = false;
            if(loaded && configPath != null)
            {
                am32sim_init(configPath);
                started = true;
                batch.Enabled = true;
            }
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
                comp = machine.GetPeripheralsOfType<IAM32Comparator>().FirstOrDefault();
                if(timer == null || comp == null)
                {
                    this.Log(LogLevel.Error, "no TIM1 or COMP in the platform; bridge disabled");
                    batch.Enabled = false;
                    return;
                }
                // The registers this tick polls are read on the
                // peripheral objects, not through the bus: a SystemBus
                // read pays a range lookup, locking and an allocation on
                // every access, and at 100k ticks a simulated second the
                // twelve GPIO reads alone were over half of all bus
                // traffic in the emulation.
                for(var i = 0; i < 3; i++)
                {
                    gpio[i] = machine.SystemBus.WhatPeripheralIsAt(gpioBase[i])
                        as IDoubleWordPeripheral;
                    if(gpio[i] == null)
                    {
                        this.Log(LogLevel.Error,
                                 "no GPIO port at 0x{0:X}; bridge disabled",
                                 gpioBase[i]);
                        batch.Enabled = false;
                        return;
                    }
                }
                if(syscfgBase != 0)
                {
                    syscfg = machine.SystemBus.WhatPeripheralIsAt(syscfgBase)
                        as IDoubleWordPeripheral;
                    if(syscfg == null)
                    {
                        this.Log(LogLevel.Error,
                                 "no SYSCFG at 0x{0:X}; bridge disabled",
                                 syscfgBase);
                        batch.Enabled = false;
                        return;
                    }
                }
            }

            for(var i = 0; i < 3; i++)
            {
                moder[i] = gpio[i].ReadDoubleWord(0);
                odr[i] = gpio[i].ReadDoubleWord(OdrOffset);
                afrl[i] = gpio[i].ReadDoubleWord(AfrlOffset);
                afrh[i] = gpio[i].ReadDoubleWord(AfrhOffset);
            }

            // CEN gates everything: a counter that never advances never
            // matches a compare, whatever the pins say
            var moe = timer.MainOutputEnabled && timer.CounterEnabled;
            var mode = tickMode;
            var ccr = tickCcr;
            for(var p = 0; p < 3; p++)
            {
                var ph = phases[p];
                // a pin only carries the timer output when it is in
                // alternate mode AND selects the timer's AF AND the
                // channel is connected to it
                var hiTimer = TimerDrives(ph.HighPort, ph.HighPin)
                    && timer.ChannelEnabled(ph.CcrChannel)
                    && RemapOk(ph.RemapBit);
                var loTimer = TimerDrives(ph.LowPort, ph.LowPin)
                    && timer.ComplementaryEnabled(ph.CcrChannel);
                mode[p] = PhaseMode(moe, hiTimer, odr[ph.HighPort], ph.HighPin,
                                    loTimer, odr[ph.LowPort], ph.LowPin);
                // the shadow, not the register: see ActiveCcr
                ccr[p] = timer.ActiveCcr(ph.CcrChannel);
            }
            am32sim_set_bridge(mode[0], mode[1], mode[2]);
            mode.CopyTo(lastMode, 0);

            var arr = timer.ActiveArr;
            am32sim_set_tim1(arr, ccr[0], ccr[1], ccr[2],
                             (timer.Prescaler + 1) * tickPs,
                             timer.DeadTimeNs);

            var sensed = comp.SensedPhase;
            if(sensed >= 0)
            {
                am32sim_set_comp_phase(sensed);
                LastSensedPhase = sensed;
            }

            var nowNs = (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds * 1000;
            // a bridge that is off cannot change a motor that is not
            // turning; the shim skips those steps, which is most of boot
            var driven = (mode[0] != 0 || mode[1] != 0 || mode[2] != 0) ? 1 : 0;
            LastCompOut = am32sim_advance(nowNs, driven) != 0;
            comp.CompOutput = LastCompOut;
        }

        // one phase's high and low side pins, e.g. "PA10" and "PB1"
        private class Phase
        {
            public Phase(string high, string low)
            {
                Decode(high, out HighPort, out HighPin);
                Decode(low, out LowPort, out LowPin);
                // TIM1_CH1 is PA8, CH2 is PA9, CH3 is PA10. AM32 turns on
                // the G0's PA11/PA12 remap, which carries PA9 and PA10
                // out on those pads, so a target naming PA11 or PA12
                // means channel 2 or 3.
                var channelPin = HighPin == 11 ? 9 : (HighPin == 12 ? 10 : HighPin);
                // that alias only holds while the remap is on, so
                // remember which SYSCFG bit has to be set for it
                RemapBit = HighPin == 11 ? Pa11Rmp
                    : (HighPin == 12 ? Pa12Rmp : 0u);
                if(HighPort != 0 || channelPin < 8 || channelPin > 10)
                {
                    throw new RecoverableException(string.Format(
                        "phase high side '{0}' is not a TIM1 output; expected PA8, "
                        + "PA9, PA10, or PA11/PA12 under the remap", high));
                }
                CcrChannel = channelPin - 8;
            }

            private static void Decode(string pin, out int port, out int number)
            {
                var n = 0;
                if(pin == null || pin.Length < 3 || pin[0] != 'P'
                   || pin[1] < 'A' || pin[1] > 'C'
                   || !int.TryParse(pin.Substring(2), out n) || n < 0 || n > 15)
                {
                    throw new RecoverableException(string.Format(
                        "'{0}' is not a pin name like PA10, PB1 or PC6", pin));
                }
                port = pin[1] - 'A';
                number = n;
            }

            // 0=A 1=B 2=C
            public readonly int HighPort;
            public readonly int HighPin;
            public readonly int LowPort;
            public readonly int LowPin;
            // 0, 1 or 2 for TIM1_CH1..CH3
            public readonly int CcrChannel;
            public readonly uint RemapBit;
        }

        // SITL_PHASE_*: 0 float, 1 low, 2 pwm, 3 pwm without
        // complementary, 4 proportional brake
        private int PhaseMode(bool moe, bool hiTimer, uint odrHi, int pinHi,
                              bool loTimer, uint odrLo, int pinLo)
        {
            if(!moe)
            {
                return 0;
            }

            if(enableBridge)
            {
                // gate driver with one PWM in and one enable: "low" is
                // the enable pin. Enable off floats the phase whatever
                // the PWM pin is doing; enable on with the PWM pin still
                // a plain output means it is held low.
                if(((odrLo >> pinLo) & 1) == 0)
                {
                    return 0;
                }
                return hiTimer ? 2 : 1;
            }

            if(hiTimer)
            {
                return loTimer ? 2 : 3;
            }
            if(loTimer)
            {
                // high side held off, low side switching: proportionalBrake()
                return 4;
            }
            // USE_INVERTED_LOW targets turn the low FET on by writing BRR,
            // so ODR low means on; see phaseouts.c LOW_BITREG_ON
            var lowOn = ((odrLo >> pinLo) & 1) != 0;
            if(invertedLow)
            {
                lowOn = !lowOn;
            }
            if(!lowOn)
            {
                return 0;
            }
            // with an inverted high side the "off" write is BSRR, so a
            // high ODR on the high pin is still off; only a driven low
            // side counts as phase low either way
            return 1;
        }

        // A pin carries the timer output only in alternate mode and with
        // the AF that selects this timer. MODER alone is not enough: an
        // alternate pin pointing at the wrong AF drives nothing useful,
        // and that is a live porting bug on a new target.
        private bool TimerDrives(int port, int pin)
        {
            if(((moder[port] >> (2 * pin)) & 3) != ModeAlternate)
            {
                return false;
            }
            var afr = pin < 8 ? afrl[port] : afrh[port];
            return ((afr >> (4 * (pin & 7))) & 0xF) == timerAf;
        }

        // PA11/PA12 carry TIM1_CH2 and CH3 only while the G0's SYSCFG
        // remap is set. Without it those pins are not channels 2 and 3
        // at all, so a target that forgot the remap must not drive.
        private bool RemapOk(uint bit)
        {
            if(bit == 0 || syscfg == null)
            {
                return true;
            }
            return (syscfg.ReadDoubleWord(0) & bit) != 0;
        }

        private const uint Pa11Rmp = 1u << 3;
        private const uint Pa12Rmp = 1u << 4;
        private const uint ModeAlternate = 2;
        private const long OdrOffset = 0x14;
        private const long AfrlOffset = 0x20;
        private const long AfrhOffset = 0x24;

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
        [DllImport("am32sim")]
        private static extern void am32sim_get_currents([Out] double[] i);
        [DllImport("am32sim")]
        private static extern void am32sim_get_sensors(ref double volts, ref double amps,
                                                       ref double degrees);

        private readonly IMachine machine;
        private readonly Phase[] phases;
        private readonly ulong[] gpioBase;
        private readonly uint[] moder = new uint[3];
        private readonly uint[] odr = new uint[3];
        private readonly uint[] afrl = new uint[3];
        private readonly uint[] afrh = new uint[3];
        private readonly int[] lastMode = new int[3];
        private readonly bool invertedLow;
        // AF number that routes TIM1 to a pin; 2 on both F0 and G0
        private readonly uint timerAf;
        // SYSCFG_CFGR1, 0 on families without the remap
        private readonly ulong syscfgBase;
        private readonly bool enableBridge;
        private readonly uint batchUs;
        private readonly uint tickPs;
        private readonly LimitTimer batch;
        // scratch for Tick, hoisted: two fresh arrays per tick is 200k
        // allocations a simulated second
        private readonly int[] tickMode = new int[3];
        private readonly uint[] tickCcr = new uint[3];
        private readonly IDoubleWordPeripheral[] gpio = new IDoubleWordPeripheral[3];
        private IDoubleWordPeripheral syscfg;
        private AM32_STM32_AdvancedTimer timer;
        private IAM32Comparator comp;
        private string configPath;
        private bool loaded;
        private bool started;
    }
}
