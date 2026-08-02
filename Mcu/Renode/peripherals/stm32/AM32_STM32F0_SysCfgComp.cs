//
// SYSCFG + COMP block. On the F051 these share one register page, and
// the stock platform tags the whole thing, so the comparator reads back
// as nothing.
//
// AM32 senses BEMF entirely through COMP1: comStep() in
// Mcu/f051/Src/phaseouts.c whole-assigns COMP->CSR to select which phase
// the inverting input watches, getCompOutputLevel() polls COMP1OUT, and
// changeCompInput() in comparator.c arms EXTI line 21 for one edge
// direction at a time.
//
// The non-inverting input is PA1, the resistor-star virtual neutral, so
// COMP1OUT is high when neutral is above the floating phase. That is the
// same sense as Mcu/SITL/sim/motor.c, which compares
// (v_neutral - v_float); keeping the two identical is what lets the same
// physics drive both.
//
// The phase selection lives in CSR[6:4] (COMP1 INMSEL): 101 is PA5,
// 100 is PA4, 110 is PA0, which the F0_A pin map assigns to phases A, B
// and C respectively.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32F0_SysCfgComp : IDoubleWordPeripheral, IKnownSize,
                                           INumberedGPIOOutput
    {
        public AM32_STM32F0_SysCfgComp(IMachine machine)
        {
            var conns = new Dictionary<int, IGPIO>();
            conns[Comp1Line] = new GPIO();
            conns[Comp2Line] = new GPIO();
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
            Connections[Comp1Line].Unset();
            Connections[Comp2Line].Unset();
        }

        // Which phase COMP1's inverting input is watching, 0=A 1=B 2=C,
        // or -1 when the selection is not one of the three phase pins.
        public int SensedPhase
        {
            get
            {
                switch((regs[CompCsr / 4] >> 4) & 7)
                {
                case 5: return 0;  // PA5
                case 4: return 1;  // PA4
                case 6: return 2;  // PA0
                default: return -1;
                }
            }
        }

        public bool Enabled => (regs[CompCsr / 4] & Comp1En) != 0;

        // Driven by the motor model. Setting it moves COMP1OUT and, on a
        // change, pulses EXTI line 21 - the EXTI model itself decides
        // whether this edge direction is armed, so rising/falling
        // selection stays in the real EXTI registers where the firmware
        // put it.
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
                Connections[Comp1Line].Set(PolarityInverted ? !output : output);
            }
        }

        private bool PolarityInverted => (regs[CompCsr / 4] & Comp1Pol) != 0;

        public uint ReadDoubleWord(long offset)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return 0;
            }
            if(offset == CompCsr)
            {
                var v = regs[idx] & ~Comp1Out;
                var level = PolarityInverted ? !output : output;
                return level ? (v | Comp1Out) : v;
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
            if(offset == CompCsr)
            {
                // COMP1OUT is read-only; keep the rest, including the
                // bit 2 speed selection that comStep() clobbers on every
                // commutation and changeCompInput() then re-applies
                regs[idx] = value & ~Comp1Out;
                // re-evaluate the line: the firmware may have flipped
                // polarity or the input selection under us
                Connections[Comp1Line].Set(PolarityInverted ? !output : output);
                return;
            }
            regs[idx] = value;
        }

        private const long CompCsr = 0x1C;

        private const uint Comp1En = 1u << 0;
        private const uint Comp1Out = 1u << 14;
        private const uint Comp1Pol = 1u << 15;

        private const int Comp1Line = 0;
        private const int Comp2Line = 1;

        private readonly uint[] regs = new uint[0x100];
        private bool output;
    }
}
