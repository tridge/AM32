//
// STM32L4 comparators. Two of them, one CSR each, at 0x40010200 and
// 0x40010204, like the G0 - but the inverting input select is two
// fields, not one: INMSEL is three bits at [6:4] (bit 7 is INPSEL here,
// which is why the G0 model's four-bit read cannot serve), and INMESEL
// at [26:25] extends it. The LL driver's IO1..IO5 map to (INMSEL,
// INMESEL) = (6,0), (7,0), (7,1), (7,2), (7,3): four of the five
// choices collide on INMSEL 7 and only INMESEL tells them apart, so the
// phase map is keyed on the full triple.
//
// The output level is bit 30 and the polarity bit 15, as on the G0.
// COMP1 is EXTI line 21 and COMP2 line 22 - the F051's lines, not the
// G0's 17/18.
//
// AM32's L431 targets use a single comparator (MAIN_COMP, COMP1 or
// COMP2 per hardware group), and the L431 firmware has no N_VARIANT
// split-comparator path - the per-phase comparator parameters exist for
// symmetry with the G0 model, not as working support for one. The
// active comparator is the one most recently written, which is what the
// firmware does on every commutation step.
//
// The non-inverting input is the resistor-star virtual neutral, so the
// output is high when neutral is above the floating phase - the same
// sense as Mcu/SITL/sim/motor.c.
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
    public class AM32_STM32L4_Comp : IDoubleWordPeripheral, IKnownSize,
                                     INumberedGPIOOutput, IAM32Comparator
    {
        // phaseXInmsel/phaseXInmesel are the CSR[6:4] and CSR[26:25]
        // codes the target's PHASE_X_COMP selects. phaseXComp is 1 or 2;
        // every current L431 target keeps all phases on mainComp.
        public AM32_STM32L4_Comp(IMachine machine,
                                 int phaseAInmsel, int phaseAInmesel,
                                 int phaseBInmsel, int phaseBInmesel,
                                 int phaseCInmsel, int phaseCInmesel,
                                 int phaseAComp = 2, int phaseBComp = 2,
                                 int phaseCComp = 2, int mainComp = 2)
        {
            phaseOf = new Dictionary<int, int>();
            Map(phaseAComp, phaseAInmsel, phaseAInmesel, 0);
            Map(phaseBComp, phaseBInmsel, phaseBInmesel, 1);
            Map(phaseCComp, phaseCInmsel, phaseCInmesel, 2);

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
                var csr = Csr(active);
                var key = Key(active,
                              (int)((csr >> InmselShift) & InmselMask),
                              (int)((csr >> InmeselShift) & InmeselMask));
                return phaseOf.TryGetValue(key, out phase) ? phase : -1;
            }
        }

        // Driven by the motor model. Only the active comparator can
        // read or drive high: the inactive one's output is held low,
        // and the firmware has its EXTI line masked off anyway.
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

        private static int Key(int comp, int inmsel, int inmesel)
            => (comp * 4 + inmesel) * 8 + inmsel;

        private void Map(int comp, int inmsel, int inmesel, int phase)
        {
            if(comp != 1 && comp != 2)
            {
                throw new RecoverableException(string.Format(
                    "comparator number {0} is not 1 or 2", comp));
            }
            if(inmsel < 0 || inmsel > 7)
            {
                throw new RecoverableException(string.Format(
                    "comparator INMSEL code {0} is out of range, expected 0-7", inmsel));
            }
            if(inmesel < 0 || inmesel > 3)
            {
                throw new RecoverableException(string.Format(
                    "comparator INMESEL code {0} is out of range, expected 0-3", inmesel));
            }
            var key = Key(comp, inmsel, inmesel);
            if(phaseOf.ContainsKey(key))
            {
                throw new RecoverableException(string.Format(
                    "COMP{0} input {1}/{2} is assigned to two phases",
                    comp, inmsel, inmesel));
            }
            phaseOf[key] = phase;
        }

        private const long Csr1 = 0x00;
        private const long Csr2 = 0x04;

        private const uint EnBit = 1u << 0;
        private const int InmselShift = 4;
        private const uint InmselMask = 0x7;
        private const int InmeselShift = 25;
        private const uint InmeselMask = 0x3;
        private const uint PolarityBit = 1u << 15;
        private const uint ValueBit = 1u << 30;

        // COMP1 is EXTI line 21, COMP2 is line 22
        private const int Comp1Line = 0;
        private const int Comp2Line = 1;

        private readonly Dictionary<int, int> phaseOf;
        private readonly int mainComp;
        private uint csr1, csr2;
        private int active;
        private bool output;
    }
}
