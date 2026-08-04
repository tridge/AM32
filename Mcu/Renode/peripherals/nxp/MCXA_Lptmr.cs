//
// MCXA153 LPTMR: the 20kHz loop timer. CSR (TEN/TIE/TCF/TFC), PSR
// (PBYP/PCS), CMR compare, CNR counter. AM32 runs it prescaler-bypassed
// from CLK_1M with CMR=50 and reset-on-compare, so this is a 1MHz
// LimitTimer raising its interrupt at the compare and clearing on the
// firmware's TCF write-1-to-clear.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using System;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class MCXA_Lptmr : IDoubleWordPeripheral, IKnownSize
    {
        public MCXA_Lptmr(IMachine machine, ulong frequency = 1000000)
        {
            IRQ = new GPIO();
            timer = new LimitTimer(machine.ClockSource, frequency, this, "lptmr",
                                   limit: uint.MaxValue, direction: Direction.Ascending,
                                   enabled: false, workMode: WorkMode.Periodic,
                                   eventEnabled: true, autoUpdate: true);
            timer.LimitReached += OnCompare;
            Reset();
        }

        public long Size => 0x100;
        public GPIO IRQ { get; }

        public void Reset()
        {
            csr = 0;
            psr = 0;
            cmr = 0;
            timer.Enabled = false;
            timer.Limit = uint.MaxValue;
            timer.Value = 0;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Csr: return csr;
            case Psr: return psr;
            case Cmr: return cmr;
            case Cnr: return (uint)timer.Value;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Csr:
                // TCF (bit 7) is write-1-to-clear; TEN/TIE/TFC plain
                if((value & Tcf) != 0)
                {
                    csr &= ~Tcf;
                }
                csr = (csr & Tcf) | (value & ~Tcf);
                timer.Enabled = (csr & Ten) != 0;
                UpdateIrq();
                return;
            case Psr: psr = value; return;
            case Cmr:
                cmr = value & 0xFFFF;
                timer.Limit = Math.Max(1, cmr);
                return;
            }
        }

        private void OnCompare()
        {
            // TFC=0: reset on compare, which the periodic LimitTimer is
            csr |= Tcf;
            UpdateIrq();
        }

        private void UpdateIrq()
        {
            IRQ.Set((csr & Tcf) != 0 && (csr & Tie) != 0 && (csr & Ten) != 0);
        }

        private const long Csr = 0x00;
        private const long Psr = 0x04;
        private const long Cmr = 0x08;
        private const long Cnr = 0x0C;

        private const uint Ten = 1u << 0;
        private const uint Tie = 1u << 6;
        private const uint Tcf = 1u << 7;

        private readonly LimitTimer timer;
        private uint csr, psr, cmr;
    }
}
