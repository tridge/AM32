//
// MCXA153 LPCMP pair. Unlike the STM32 families the two comparators are
// separate bus peripherals with separate NVIC lines, and the firmware
// commutates by enabling exactly one of them (changeCompInput() disables
// both, retargets MSEL on the phase's unit, then enables the main one).
// The phase key is therefore (unit, MSEL): on the FRDM_A153 phase A is
// CMP0/IN3, phase B is CMP1/IN3, phase C is CMP0/IN1, with the virtual
// neutral on the plus input of both.
//
// The bridge wants one IAM32Comparator, so the pair hangs off a shared
// MCXA_LpcmpMux hub (a tiny bus stub so GetPeripheralsOfType finds it):
// the units own the registers and IRQs, the hub owns the phase map and
// the motor-model output level.
//
// Edges are latched as CSR.CFR/CFF flags recomputed from
// "enabled && output" - so an enable with the level already high flags
// a rising edge, which is what the hardware's transition detector does
// coming out of the disabled (low) state. The firmware's IRQ handler
// blanks false edges by leaving the flag set and re-entering until the
// interval check passes, so spurious edges at commutation boundaries
// are its problem to filter, as on silicon.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // the IAM32Comparator facade over the two units
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class MCXA_LpcmpMux : IDoubleWordPeripheral, IKnownSize, IAM32Comparator
    {
        public MCXA_LpcmpMux(int phaseAComp = 0, int phaseAMsel = 3,
                             int phaseBComp = 1, int phaseBMsel = 3,
                             int phaseCComp = 0, int phaseCMsel = 1)
        {
            phaseOf = new Dictionary<int, int>();
            Map(phaseAComp, phaseAMsel, 0);
            Map(phaseBComp, phaseBMsel, 1);
            Map(phaseCComp, phaseCMsel, 2);
        }

        public long Size => 0x10;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset()
        {
            output = false;
        }

        public void Attach(MCXA_Lpcmp unit, int index)
        {
            units[index] = unit;
        }

        // the enabled unit's minus-input selection names the phase
        public int SensedPhase
        {
            get
            {
                for(var i = 0; i < units.Length; i++)
                {
                    var u = units[i];
                    if(u == null || !u.Enabled)
                    {
                        continue;
                    }
                    var phase = 0;
                    if(phaseOf.TryGetValue(Key(i, u.Msel), out phase))
                    {
                        return phase;
                    }
                    return -1;
                }
                return -1;
            }
        }

        // set by the motor model: true when the virtual neutral is above
        // the floating phase
        public bool CompOutput
        {
            get { return output; }
            set
            {
                output = value;
                foreach(var u in units)
                {
                    if(u != null)
                    {
                        u.Recompute(output);
                    }
                }
            }
        }

        public bool Output => output;

        private static int Key(int comp, int msel) => comp * 8 + msel;

        private void Map(int comp, int msel, int phase)
        {
            if(comp != 0 && comp != 1)
            {
                throw new RecoverableException(string.Format(
                    "comparator number {0} is not 0 or 1", comp));
            }
            if(msel < 0 || msel > 7)
            {
                throw new RecoverableException(string.Format(
                    "comparator MSEL code {0} is out of range, expected 0-7", msel));
            }
            var key = Key(comp, msel);
            if(phaseOf.ContainsKey(key))
            {
                throw new RecoverableException(string.Format(
                    "CMP{0} MSEL code {1} is assigned to two phases", comp, msel));
            }
            phaseOf[key] = phase;
        }

        private readonly Dictionary<int, int> phaseOf;
        private readonly MCXA_Lpcmp[] units = new MCXA_Lpcmp[2];
        private bool output;
    }

    // one LPCMP unit
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class MCXA_Lpcmp : IDoubleWordPeripheral, IKnownSize
    {
        public MCXA_Lpcmp(MCXA_LpcmpMux mux, int index)
        {
            this.mux = mux;
            IRQ = new GPIO();
            mux.Attach(this, index);
            Reset();
        }

        public long Size => 0x100;
        public GPIO IRQ { get; }

        public void Reset()
        {
            ccr0 = ccr1 = ccr2 = ier = csr = 0;
            cout = false;
            IRQ.Unset();
        }

        public bool Enabled => (ccr0 & CmpEn) != 0;
        public int Msel => (int)((ccr2 >> MselShift) & 0x7);

        // recompute the latched output from the mux level; a change
        // flags the matching edge
        public void Recompute(bool output)
        {
            var now = Enabled && output;
            if(now == cout)
            {
                return;
            }
            cout = now;
            csr |= now ? Cfr : Cff;
            UpdateIrq();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Ccr0: return ccr0;
            case Ccr1: return ccr1;
            case Ccr2: return ccr2;
            case Ier: return ier;
            case Csr: return csr | (cout ? Cout : 0);
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Ccr0:
                ccr0 = value;
                Recompute(mux.Output);
                UpdateIrq();
                return;
            case Ccr1: ccr1 = value; return;
            case Ccr2: ccr2 = value; return;
            case Ier:
                ier = value;
                UpdateIrq();
                return;
            case Csr:
                // CFR/CFF write-1-to-clear (the firmware writes 0x7)
                csr &= ~(value & (Cfr | Cff));
                UpdateIrq();
                return;
            default: return;
            }
        }

        private void UpdateIrq()
        {
            IRQ.Set(((csr & Cfr) != 0 && (ier & CfrIe) != 0)
                || ((csr & Cff) != 0 && (ier & CffIe) != 0));
        }

        private const long Ccr0 = 0x08;
        private const long Ccr1 = 0x0C;
        private const long Ccr2 = 0x10;
        private const long Ier = 0x1C;
        private const long Csr = 0x20;

        private const uint CmpEn = 1u << 0;
        private const int MselShift = 20;
        private const uint CfrIe = 1u << 0;
        private const uint CffIe = 1u << 1;
        private const uint Cfr = 1u << 0;
        private const uint Cff = 1u << 1;
        private const uint Cout = 1u << 8;

        private readonly MCXA_LpcmpMux mux;
        private uint ccr0, ccr1, ccr2, ier, csr;
        private bool cout;
    }
}
