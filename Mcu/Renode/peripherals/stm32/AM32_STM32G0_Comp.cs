//
// STM32G0 comparators. Two of them, one CSR each, at 0x40010200 and
// 0x40010204 - adjacent registers rather than the single shared page the
// F051 has, and a different layout: the output level is bit 30 (not 14),
// INMSEL is four bits at [7:4] (not three at [6:4]), and bit 31 is the
// lock.
//
// AM32 uses COMP2 alone on most G0 targets (MAIN_COMP), and both on
// N_VARIANT ones, where changeCompInput() moves active_COMP between them
// per commutation step and arms EXTI line 17 or 18 to match. The active
// comparator is tracked as the one most recently written, which is exactly
// what active_COMP means: the firmware writes its CSR on every step.
//
// The non-inverting input is the resistor-star virtual neutral, so the
// output is high when neutral is above the floating phase - the same sense
// as Mcu/SITL/sim/motor.c, which is what lets one physics model drive
// both families.
//
// The phase map is (comparator, INMSEL) rather than INMSEL alone: on an
// N_VARIANT target two phases can sit on the same input number of
// different comparators, so INMSEL by itself is ambiguous.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32G0_Comp : IDoubleWordPeripheral, IKnownSize,
                                     INumberedGPIOOutput, IAM32Comparator
    {
        // phaseXInmsel is the CSR[7:4] code the target's PHASE_X_COMP
        // selects: LL_COMP_INPUT_MINUS_IO1 is 6, IO2 is 7, IO3 is 8.
        // phaseXComp is 1 or 2, from PHASE_X_COMP_NUMBER; on targets
        // without N_VARIANT every phase is on MAIN_COMP.
        public AM32_STM32G0_Comp(IMachine machine,
                                 int phaseAInmsel, int phaseBInmsel, int phaseCInmsel,
                                 int phaseAComp = 2, int phaseBComp = 2,
                                 int phaseCComp = 2, int mainComp = 2)
        {
            phaseOf = new Dictionary<int, int>();
            Map(phaseAComp, phaseAInmsel, 0);
            Map(phaseBComp, phaseBInmsel, 1);
            Map(phaseCComp, phaseCInmsel, 2);

            if(mainComp != 1 && mainComp != 2)
            {
                throw new RecoverableException(string.Format(
                    "mainComp must be 1 or 2, not {0}", mainComp));
            }
            this.mainComp = mainComp;

            var conns = new Dictionary<int, IGPIO>();
            conns[Comp1Line] = new GPIO();
            conns[Comp2Line] = new GPIO();
            Connections = conns;
            Reset();
        }

        // two CSRs, but the region is registered as a page so a stray
        // access nearby is caught here rather than warning as unmapped
        public long Size => 0x100;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            csr1 = csr2 = 0;
            output = false;
            active = mainComp;
            Connections[Comp1Line].Unset();
            Connections[Comp2Line].Unset();
        }

        // which phase the active comparator's inverting input is watching
        public int SensedPhase
        {
            get
            {
                var phase = 0;
                var key = Key(active, (int)((Csr(active) >> InmselShift) & InmselMask));
                return phaseOf.TryGetValue(key, out phase) ? phase : -1;
            }
        }

        // Driven by the motor model. Only the active comparator's line
        // moves: the other keeps its last level, and the firmware has its
        // EXTI line masked off anyway.
        public bool CompOutput
        {
            get { return output; }
            set
            {
                if(output == value)
                {
                    return;
                }
                output = value;
                Drive();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == Csr1)
            {
                return Level(1) ? (csr1 | ValueBit) : (csr1 & ~ValueBit);
            }
            if(offset == Csr2)
            {
                return Level(2) ? (csr2 | ValueBit) : (csr2 & ~ValueBit);
            }
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset != Csr1 && offset != Csr2)
            {
                return;
            }
            var which = offset == Csr1 ? 1 : 2;
            // the output level is read-only; the lock bit is kept as
            // written but not enforced, as AM32 never sets it
            value &= ~ValueBit;
            if(which == 1)
            {
                csr1 = value;
            }
            else
            {
                csr2 = value;
            }
            // active_COMP is whichever the firmware last configured
            active = which;
            Drive();
        }

        private uint Csr(int which) => which == 1 ? csr1 : csr2;

        private bool Enabled(int which) => (Csr(which) & EnBit) != 0;

        private bool Inverted(int which) => (Csr(which) & PolarityBit) != 0;

        private bool Level(int which)
        {
            if(which != active || !Enabled(which))
            {
                return false;
            }
            return Inverted(which) ? !output : output;
        }

        private void Drive()
        {
            Connections[Comp1Line].Set(Level(1));
            Connections[Comp2Line].Set(Level(2));
        }

        private static int Key(int comp, int inmsel) => comp * 16 + inmsel;

        private void Map(int comp, int inmsel, int phase)
        {
            if(comp != 1 && comp != 2)
            {
                throw new RecoverableException(string.Format(
                    "comparator number {0} is not 1 or 2", comp));
            }
            if(inmsel < 0 || inmsel > 15)
            {
                throw new RecoverableException(string.Format(
                    "comparator INMSEL code {0} is out of range, expected 0-15", inmsel));
            }
            var key = Key(comp, inmsel);
            if(phaseOf.ContainsKey(key))
            {
                throw new RecoverableException(string.Format(
                    "COMP{0} INMSEL code {1} is assigned to two phases", comp, inmsel));
            }
            phaseOf[key] = phase;
        }

        private const long Csr1 = 0x00;
        private const long Csr2 = 0x04;

        private const uint EnBit = 1u << 0;
        private const int InmselShift = 4;
        private const uint InmselMask = 0xF;
        private const uint PolarityBit = 1u << 15;
        private const uint ValueBit = 1u << 30;

        // COMP1 is EXTI line 17, COMP2 is line 18
        private const int Comp1Line = 0;
        private const int Comp2Line = 1;

        private readonly Dictionary<int, int> phaseOf;
        private readonly int mainComp;
        private uint csr1, csr2;
        private int active;
        private bool output;
    }
}
