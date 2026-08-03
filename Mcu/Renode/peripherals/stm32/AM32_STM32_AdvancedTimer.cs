//
// TIM1, the motor PWM timer, modelled as registers plus an analytic
// output query rather than as a source of pin events.
//
// That choice is deliberate. AM32 never uses TIM1's outputs as GPIO
// events - Mcu/f051/Src/stm32f0xx_it.c's TIM1 handler only clears flags,
// and doPWMChanges() is commented out - so the outputs matter to exactly
// one consumer, the motor simulation, which samples "is the high side
// commanded on right now" the same way Mcu/SITL/sim/motor.c does through
// sitl_tim1_pwm_out(). Emitting 24kHz x 3 channels of GPIO edges to
// synthesise a signal nothing listens to would cost a lot and buy
// nothing.
//
// What does have to be faithful is the register behaviour, because the
// real Mcu/f051 code drives it: ARR and CCRx are preloaded (MX_TIM1_Init
// enables ARPE and per-channel OCxPE), so a duty written mid-period only
// takes effect at the next update event. Shadowing that is the
// difference between modelling the hardware and modelling the C.
//
// Renode's Timers.STM32_Timer keeps its register fields private and has
// no notion of MOE, complementary outputs or dead time, so subclassing
// is not an option.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Timers
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32_AdvancedTimer : IDoubleWordPeripheral, IKnownSize,
                                            INumberedGPIOOutput
    {
        public AM32_STM32_AdvancedTimer(IMachine machine, ulong frequency = 48000000)
        {
            this.frequency = frequency;
            var conns = new Dictionary<int, IGPIO>();
            conns[UpdateIrqLine] = new GPIO();
            conns[CaptureIrqLine] = new GPIO();
            Connections = conns;

            counter = new LimitTimer(machine.ClockSource, frequency, this,
                                     "tim1", MaxCount + 1,
                                     direction: Direction.Ascending,
                                     enabled: false, autoUpdate: true,
                                     eventEnabled: true);
            counter.LimitReached += OnUpdateEvent;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            for(var i = 0; i < regs.Length; i++)
            {
                regs[i] = 0;
            }
            regs[ARR / 4] = MaxCount;
            arrShadow = MaxCount;
            for(var i = 0; i < 4; i++)
            {
                ccrShadow[i] = 0;
            }
            counter.Enabled = false;
            counter.Divider = 1;
            counter.Limit = MaxCount + 1;
            counter.Value = 0;
            Connections[UpdateIrqLine].Unset();
            Connections[CaptureIrqLine].Unset();
        }

        // pulsed by RCC APB2RSTR
        public void PeripheralReset()
        {
            Reset();
        }

        // OCxREF for a channel, 0-based: is the high side commanded on
        // right now, before dead time. Polarity and PWM1/PWM2 applied;
        // gated by MOE and the channel enable, so a disabled bridge
        // reads off rather than reading the counter.
        public bool OutputRef(int channel)
        {
            if(channel < 0 || channel > 3 || !MainOutputEnabled)
            {
                return false;
            }
            if((regs[CCER / 4] & (CCxE << (4 * channel))) == 0)
            {
                return false;
            }
            var mode = OutputMode(channel);
            var active = counter.Value < ccrShadow[channel];
            if(mode == OcModePwm2)
            {
                active = !active;
            }
            else if(mode != OcModePwm1)
            {
                // forced-inactive/active and the compare modes AM32 never
                // selects; treat anything else as inactive
                active = mode == OcModeForceActive;
            }
            var invert = (regs[CCER / 4] & (CCxP << (4 * channel))) != 0;
            return invert ? !active : active;
        }

        // true when the low-side complementary output of this channel is
        // driven by the timer rather than left off (CCxNE)
        public bool ComplementaryEnabled(int channel)
        {
            if(channel < 0 || channel > 3 || !MainOutputEnabled)
            {
                return false;
            }
            return (regs[CCER / 4] & (CCxNE << (4 * channel))) != 0;
        }

        public bool MainOutputEnabled => (regs[BDTR / 4] & MOE) != 0;

        // CR1.CEN. Without it the counter never advances, so no compare
        // ever matches and the outputs sit idle however the pins and
        // CCER are set up.
        public bool CounterEnabled => (regs[CR1 / 4] & CEN) != 0;

        // CCxE: the channel's main output is connected to its pin
        public bool ChannelEnabled(int channel)
        {
            if(channel < 0 || channel > 3 || !MainOutputEnabled)
            {
                return false;
            }
            return (regs[CCER / 4] & (CCxE << (4 * channel))) != 0;
        }

        // The register, not the shadow: AM32 writes PSC once at init
        // and never again, so they agree. The bridge reads this every
        // tick, so it must not go through the bus.
        public uint Prescaler => regs[PSC / 4] & MaxCount;

        // The period and compare values the hardware is actually
        // counting against, which with ARPE/OCxPE set are not what a
        // read of ARR or CCRx returns - those give the preload the
        // firmware wrote, which takes effect at the next update event.
        // The motor model has to use these or it sees a duty change a
        // PWM period before the bridge really makes it.
        public uint ActiveArr => arrShadow;

        public uint ActiveCcr(int channel)
        {
            return (channel >= 0 && channel < ccrShadow.Length)
                ? ccrShadow[channel] : 0;
        }

        // BDTR.DTG, decoded per RM0091 and scaled by the CR1.CKD
        // prescale. AM32 sets this from the per-target DEAD_TIME, and
        // the motor model needs it in nanoseconds to open both fets
        // across a PWM edge.
        public uint DeadTimeNs
        {
            get
            {
                var dtg = regs[BDTR / 4] & 0xFF;
                uint ticks;
                if((dtg & 0x80) == 0)
                {
                    ticks = dtg;
                }
                else if((dtg & 0xC0) == 0x80)
                {
                    ticks = (64 + (dtg & 0x3F)) * 2;
                }
                else if((dtg & 0xE0) == 0xC0)
                {
                    ticks = (32 + (dtg & 0x1F)) * 8;
                }
                else
                {
                    ticks = (32 + (dtg & 0x1F)) * 16;
                }
                var ckd = (regs[CR1 / 4] >> 8) & 3;
                var tdts = ckd == 0 ? 1u : (ckd == 1 ? 2u : 4u);
                return (uint)(ticks * tdts * 1000000000UL / frequency);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == CNT)
            {
                return (uint)(counter.Value & MaxCount);
            }
            var idx = offset / 4;
            return (idx >= 0 && idx < regs.Length) ? regs[idx] : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return;
            }
            switch(offset)
            {
            case CR1:
                regs[idx] = value;
                counter.Enabled = (value & CEN) != 0;
                return;
            case EGR:
                regs[idx] = 0;
                if((value & UG) != 0)
                {
                    ApplyPrescaler();
                    LatchPreloads();
                    counter.Value = 0;
                }
                return;
            case SR:
                regs[idx] &= value; // rc_w0
                UpdateIrqs();
                return;
            case CNT:
                counter.Value = value & MaxCount;
                return;
            case ARR:
                regs[idx] = value & MaxCount;
                // ARPE off means it takes effect immediately
                if((regs[CR1 / 4] & ARPE) == 0)
                {
                    arrShadow = regs[idx];
                    counter.Limit = arrShadow + 1;
                }
                return;
            case CCR1:
            case CCR2:
            case CCR3:
            case CCR4:
                regs[idx] = value & MaxCount;
                {
                    var ch = (int)((offset - CCR1) / 4);
                    if(!PreloadEnabled(ch))
                    {
                        ccrShadow[ch] = regs[idx];
                    }
                }
                return;
            case PSC:
                regs[idx] = value & MaxCount;
                // takes effect at the next update event
                return;
            default:
                regs[idx] = value;
                if(offset == DIER)
                {
                    UpdateIrqs();
                }
                return;
            }
        }

        private void OnUpdateEvent()
        {
            ApplyPrescaler();
            LatchPreloads();
            regs[SR / 4] |= UIF;
            UpdateIrqs();
        }

        private void ApplyPrescaler()
        {
            counter.Divider = (ulong)((regs[PSC / 4] & MaxCount) + 1);
        }

        private void LatchPreloads()
        {
            arrShadow = regs[ARR / 4] & MaxCount;
            counter.Limit = arrShadow + 1;
            for(var ch = 0; ch < 4; ch++)
            {
                ccrShadow[ch] = regs[(CCR1 / 4) + ch] & MaxCount;
            }
        }

        private bool PreloadEnabled(int channel)
        {
            var ccmr = regs[((channel < 2) ? CCMR1 : CCMR2) / 4];
            var shift = (channel & 1) == 0 ? 0 : 8;
            return ((ccmr >> shift) & OCxPE) != 0;
        }

        private uint OutputMode(int channel)
        {
            var ccmr = regs[((channel < 2) ? CCMR1 : CCMR2) / 4];
            var shift = (channel & 1) == 0 ? 4 : 12;
            return (ccmr >> shift) & 7;
        }

        private void UpdateIrqs()
        {
            var dier = regs[DIER / 4];
            var sr = regs[SR / 4];
            Connections[UpdateIrqLine].Set((sr & dier & UIF) != 0);
            Connections[CaptureIrqLine].Set((sr & dier & CC1IF) != 0);
        }

        private const long CR1 = 0x00;
        private const long DIER = 0x0C;
        private const long SR = 0x10;
        private const long EGR = 0x14;
        private const long CCMR1 = 0x18;
        private const long CCMR2 = 0x1C;
        private const long CCER = 0x20;
        private const long CNT = 0x24;
        private const long PSC = 0x28;
        private const long ARR = 0x2C;
        private const long CCR1 = 0x34;
        private const long CCR2 = 0x38;
        private const long CCR3 = 0x3C;
        private const long CCR4 = 0x40;
        private const long BDTR = 0x44;

        private const uint CEN = 1u << 0;
        private const uint ARPE = 1u << 7;
        private const uint UG = 1u << 0;
        private const uint UIF = 1u << 0;
        private const uint CC1IF = 1u << 1;
        private const uint MOE = 1u << 15;
        private const uint OCxPE = 1u << 3;
        private const uint CCxE = 1u << 0;
        private const uint CCxP = 1u << 1;
        private const uint CCxNE = 1u << 2;
        private const uint OcModeForceActive = 5;
        private const uint OcModePwm1 = 6;
        private const uint OcModePwm2 = 7;
        private const uint MaxCount = 0xFFFF;

        private const int UpdateIrqLine = 0;
        private const int CaptureIrqLine = 1;

        private readonly ulong frequency;
        private readonly LimitTimer counter;
        private readonly uint[] regs = new uint[0x100];
        private readonly uint[] ccrShadow = new uint[4];
        private uint arrShadow;
    }
}
