//
// Artery AT32 comparator (CMP), shared by the F415 and F421 ports. The
// block is nearly the F051's COMP1 half-register - enable at bit 0,
// inverting-input select CMPINVSEL at [6:4] - but the two families
// place it differently, so the placement is constructor parameters
// rather than two models:
//
// - the F415 gives it a page of its own at 0x40002400, output CMP1VALUE
//   at bit 14, polarity at bit 11 (Mcu/f415/Src/comparator.c);
// - the F421 keeps it an SCFG-page tenant, F051 style: ctrlsts at page
//   offset 0x1C, output CMPVALUE at bit 30, polarity CMPP at bit 15
//   (Mcu/f421/Src/comparator.c), with the rest of the page being plain
//   SCFG storage.
//
// Both firmwares whole-assign the register with the PHASE_x_COMP
// constants from Inc/targets.h to select which phase the inverting
// input watches; the codes are 4=PA4, 5=PA5, 6=PA0, 7=PA2 (F421 only).
// Which code is phase A, B or C differs per hardware group, so the map
// is constructor parameters with no default - a wrong map is silent,
// the firmware would commutate against the wrong phase and simply
// never run well. Stray bits the constants set above the modelled ones
// (the F415's bit 30 lands in its unused CMP2 half) are stored and
// ignored. Neither firmware ever sets the polarity bit; it is honoured
// anyway so a target that did would not silently invert BEMF sensing.
//
// The non-inverting input is the resistor-star virtual neutral, so the
// output is high when neutral is above the floating phase - the same
// sense as Mcu/SITL/sim/motor.c and every other family's comparator
// model, which is what lets one physics model drive them all.
//
// Connections: [0] is the EXTI line the comparator gates (19 on the
// F415, 21 on the F421), wired by the target overlay. The EXTI model
// decides whether the edge direction is armed, so the rising/falling
// selection stays in the real EXINT registers where the firmware put
// it.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_AT32_Cmp : IDoubleWordPeripheral, IKnownSize,
                                 INumberedGPIOOutput, IAM32Comparator
    {
        public AM32_AT32_Cmp(IMachine machine, int phaseAInmsel,
                             int phaseBInmsel, int phaseCInmsel,
                             int outputBit = 14, int polarityBit = 11,
                             long ctrlstsOffset = 0)
        {
            if(outputBit < 0 || outputBit > 31)
            {
                throw new RecoverableException("outputBit must be 0..31");
            }
            if(polarityBit < 0 || polarityBit > 31)
            {
                throw new RecoverableException("polarityBit must be 0..31");
            }
            if(ctrlstsOffset < 0 || ctrlstsOffset >= 0x400 || (ctrlstsOffset & 3) != 0)
            {
                throw new RecoverableException(
                    "ctrlstsOffset must be a word offset inside the page");
            }
            this.outputBit = 1u << outputBit;
            this.polarityBit = 1u << polarityBit;
            ctrlsts = ctrlstsOffset;
            inmselToPhase = new int[8];
            for(var i = 0; i < inmselToPhase.Length; i++)
            {
                inmselToPhase[i] = -1;
            }
            SetPhase(phaseAInmsel, 0);
            SetPhase(phaseBInmsel, 1);
            SetPhase(phaseCInmsel, 2);

            var conns = new Dictionary<int, IGPIO>();
            conns[ExtiLine] = new GPIO();
            Connections = conns;
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
            output = false;
            Connections[ExtiLine].Unset();
        }

        // which phase the inverting input is watching, 0=A 1=B 2=C, or
        // -1 when the selection is not one of the three phase pins
        public int SensedPhase => inmselToPhase[(regs[ctrlsts / 4] >> 4) & 7];

        public bool Enabled => (regs[ctrlsts / 4] & EnBit) != 0;

        // driven by the motor model
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
                Connections[ExtiLine].Set(Level);
            }
        }

        private bool PolarityInverted => (regs[ctrlsts / 4] & polarityBit) != 0;

        // a disabled comparator drives nothing, as on the other
        // families' models; both firmwares enable before use
        private bool Level => Enabled && (PolarityInverted ? !output : output);

        public uint ReadDoubleWord(long offset)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return 0;
            }
            if(offset == ctrlsts)
            {
                var v = regs[idx] & ~outputBit;
                return Level ? (v | outputBit) : v;
            }
            return regs[idx];
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return;
            }
            if(offset == ctrlsts)
            {
                // the output level is read-only
                regs[idx] = value & ~outputBit;
                // re-evaluate the line: changeCompInput() whole-assigns
                // this register on every commutation step
                Connections[ExtiLine].Set(Level);
                return;
            }
            regs[idx] = value;
        }

        private void SetPhase(int inmsel, int phase)
        {
            if(inmsel < 0 || inmsel > 7)
            {
                throw new RecoverableException(string.Format(
                    "comparator INMSEL code {0} is out of range, expected 0-7", inmsel));
            }
            if(inmselToPhase[inmsel] >= 0)
            {
                throw new RecoverableException(string.Format(
                    "comparator INMSEL code {0} is assigned to two phases", inmsel));
            }
            inmselToPhase[inmsel] = phase;
        }

        private const uint EnBit = 1u << 0;

        private const int ExtiLine = 0;

        // one 0x400 page: the comparator register plus, on the F421,
        // the SCFG registers it shares the page with (write-readback)
        private readonly uint[] regs = new uint[0x100];
        private readonly int[] inmselToPhase;
        private readonly uint outputBit;
        private readonly uint polarityBit;
        private readonly long ctrlsts;
        private bool output;
    }
}
