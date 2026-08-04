//
// WCH core SysTick (STK) at 0xE000F000: not the ARM SysTick despite the
// name. 64-bit CNT and CMP, a control register with enable / interrupt
// enable / clock select / auto-reload, and a one-bit SR whose CNTIF is
// cleared by writing zero.
//
// AM32 programs it once as the 20 kHz loop timer: CTLR=0xF (enabled,
// interrupt on, HCLK direct, auto-reload at CMP), CMP=SystemCoreClock/
// LOOP_FREQUENCY_HZ. The handler writes SR=0 and the PFIC line follows
// SR, which is what ends the interrupt request.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using System;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class AM32_WCH_SysTick : IDoubleWordPeripheral, IKnownSize
    {
        public AM32_WCH_SysTick(IMachine machine, ulong frequency)
        {
            this.frequency = frequency;
            IRQ = new GPIO();
            timer = new LimitTimer(machine.ClockSource, frequency, this, "stk",
                                   limit: uint.MaxValue, direction: Direction.Ascending,
                                   enabled: false, workMode: WorkMode.Periodic,
                                   eventEnabled: true, autoUpdate: true);
            timer.LimitReached += OnLimitReached;
            Reset();
        }

        public long Size => 0x100;
        public GPIO IRQ { get; }

        public void Reset()
        {
            ctlr = 0;
            sr = 0;
            cmpLo = 0;
            cmpHi = 0;
            timer.Enabled = false;
            timer.Limit = uint.MaxValue;
            timer.Value = 0;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Ctlr: return ctlr;
            case Sr: return sr;
            case CntLo: return (uint)timer.Value;
            case CntHi: return (uint)(timer.Value >> 32);
            case CmpLo: return cmpLo;
            case CmpHi: return cmpHi;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Ctlr:
                ctlr = value;
                timer.Frequency = (value & Stclk) != 0 ? frequency : frequency / 8;
                timer.Enabled = (value & Ste) != 0;
                UpdateIrq();
                return;
            case Sr:
                sr &= value & Cntif; // write 0 clears CNTIF
                UpdateIrq();
                return;
            case CntLo:
                timer.Value = (timer.Value & ~0xFFFFFFFFul) | value;
                return;
            case CntHi:
                timer.Value = (timer.Value & 0xFFFFFFFFul) | ((ulong)value << 32);
                return;
            case CmpLo:
                cmpLo = value;
                ApplyLimit();
                return;
            case CmpHi:
                cmpHi = value;
                ApplyLimit();
                return;
            }
        }

        private void ApplyLimit()
        {
            var cmp = ((ulong)cmpHi << 32) | cmpLo;
            // LimitTimer cannot have a zero limit; the firmware writes
            // CMP before starting, so this only guards the reset state
            timer.Limit = Math.Max(1, cmp);
        }

        private void OnLimitReached()
        {
            // STRE=0 (count past CMP without wrapping) is not modelled;
            // AM32 always sets it
            sr |= Cntif;
            UpdateIrq();
        }

        private void UpdateIrq()
        {
            IRQ.Set((sr & Cntif) != 0 && (ctlr & Stie) != 0 && (ctlr & Ste) != 0);
        }

        private const long Ctlr = 0x00;
        private const long Sr = 0x04;
        private const long CntLo = 0x08;
        private const long CntHi = 0x0C;
        private const long CmpLo = 0x10;
        private const long CmpHi = 0x14;

        private const uint Ste = 1u << 0;
        private const uint Stie = 1u << 1;
        private const uint Stclk = 1u << 2;
        private const uint Cntif = 1u << 0;

        private readonly ulong frequency;
        private readonly LimitTimer timer;
        private uint ctlr, sr, cmpLo, cmpHi;
    }
}
