//
// Artery AT32 comparator (CMP). First Artery family in the harness; the
// block is nearly the F051's COMP1 half-register - enable at bit 0,
// inverting-input select at [6:4], polarity at bit 11 - but it lives on
// a page of its own (0x40002400 on the F415, not the SYSCFG page) and
// the F415 firmware whole-assigns CTRLSTS1 with the PHASE_x_COMP
// constants from Inc/targets.h: 0xC5 selects PA4, 0xD5 PA5, 0xE5 PA0.
// Those constants also set bit 30, which lands in the unused CMP2
// half of the register and is ignored here, as CMP2 itself is: no AM32
// target touches it.
//
// The output level bit is a constructor parameter because the two
// Artery families place it differently: the F415 reads CMP1VALUE at
// bit 14 of this register (getCompOutputLevel() in
// Mcu/f415/Src/comparator.c), while the F421's comparator lives in the
// SCFG page with its output at bit 30. Shared with the F421 port -
// keep the name and parameters in sync when merging.
//
// The non-inverting input is the resistor-star virtual neutral, so the
// output is high when neutral is above the floating phase - the same
// sense as Mcu/SITL/sim/motor.c and every other family's comparator
// model, which is what lets one physics model drive them all.
//
// Connections: [0] is the EXTI line the comparator gates (line 19 on
// the F415, wired by the target overlay).
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
        // phaseXInmsel is the CTRLSTS1[6:4] code the target's
        // PHASE_X_COMP constant selects: 4 is PA4, 5 is PA5, 6 is PA0.
        // outputBit is where the output level reads back: 14 on the
        // F415, 30 on the F421's SCFG-page block.
        public AM32_AT32_Cmp(IMachine machine, int phaseAInmsel,
                             int phaseBInmsel, int phaseCInmsel,
                             int outputBit = 14)
        {
            if(outputBit < 0 || outputBit > 31)
            {
                throw new RecoverableException("outputBit must be 0..31");
            }
            this.outputBit = 1u << outputBit;
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
        public int SensedPhase => inmselToPhase[(regs[Ctrlsts1 / 4] >> 4) & 7];

        public bool Enabled => (regs[Ctrlsts1 / 4] & EnBit) != 0;

        // Driven by the motor model. A change moves the EXTI line; the
        // EXTI model itself decides whether this edge direction is
        // armed, so the rising/falling selection stays in the real
        // EXINT registers where the firmware put it.
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

        private bool PolarityInverted => (regs[Ctrlsts1 / 4] & PolBit) != 0;

        private bool Level => PolarityInverted ? !output : output;

        public uint ReadDoubleWord(long offset)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return 0;
            }
            if(offset == Ctrlsts1)
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
            if(offset == Ctrlsts1)
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

        private const long Ctrlsts1 = 0x00;

        private const uint EnBit = 1u << 0;
        private const uint PolBit = 1u << 11;

        private const int ExtiLine = 0;

        // CTRLSTS1 and CTRLSTS2; the rest of the page reads back writes
        private readonly uint[] regs = new uint[0x100];
        private readonly int[] inmselToPhase;
        private readonly uint outputBit;
        private bool output;
    }
}
