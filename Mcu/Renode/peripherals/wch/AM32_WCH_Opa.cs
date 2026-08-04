//
// The CH32V203's BEMF front end: no comparator peripheral, but two
// op-amps used as comparators. OPA->CR routes one phase's BEMF to an
// inverting input and the virtual-neutral network to the non-inverting
// input; the outputs land on real pins - OPA1 on PA3, OPA2 on PA4 -
// which the firmware reads through GPIOA->INDR and edge-detects with
// EXTI lines 3 and 4 (AT_COMP_Init, comparator.c).
//
// The three CR values changeCompInput() writes are matched literally:
//   0x01  OPA1, 1N0=PB11=phase C   -> out PA3
//   0x70  OPA2, 2N1=PA5 =phase A   -> out PA4
//   0x30  OPA2, 2N0=PB10=phase B   -> out PA4
// The output level comes from the motor physics through the bridge
// (CompOutput: neutral above the floating phase = out high, matching
// P=neutral, N=phase). Both pins are driven on every update so a phase
// switch immediately presents the right level on the newly selected
// output.
//
// This block also owns offset 0 of the page: EXTEN_CTR, which
// SetSysClockTo96_HSI read-modify-writes for the PLL HSI predivider -
// stored and read back, not interpreted.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AM32_WCH_Opa : IDoubleWordPeripheral, IKnownSize,
                                IAM32Comparator, INumberedGPIOOutput
    {
        public AM32_WCH_Opa()
        {
            var conns = new Dictionary<int, IGPIO>();
            conns[Opa1Out] = new GPIO(); // -> PA3
            conns[Opa2Out] = new GPIO(); // -> PA4
            Connections = conns;
            Reset();
        }

        public long Size => 0x100;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        // 0=A 1=B 2=C, or -1 while the CR value selects no phase
        public int SensedPhase { get; private set; }

        public bool CompOutput
        {
            get
            {
                return level;
            }
            set
            {
                level = value;
                Drive();
            }
        }

        public void Reset()
        {
            extenCtr = 0;
            cr = 0;
            level = false;
            SensedPhase = -1;
            warned = false;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case ExtenCtr: return extenCtr;
            case Cr: return cr;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case ExtenCtr:
                extenCtr = value;
                return;
            case Cr:
                cr = value;
                switch(value)
                {
                case 0x01: SensedPhase = 2; break;
                case 0x70: SensedPhase = 0; break;
                case 0x30: SensedPhase = 1; break;
                default:
                    SensedPhase = -1;
                    if(!warned)
                    {
                        warned = true;
                        this.Log(LogLevel.Warning,
                                 "unrecognised OPA CR value 0x{0:X}", value);
                    }
                    break;
                }
                Drive();
                return;
            }
        }

        private void Drive()
        {
            // only the selected op-amp carries the comparison; the other
            // output holds low, and its EXTI line is masked by the
            // firmware anyway
            var viaOpa1 = SensedPhase == 2;
            Connections[Opa1Out].Set(viaOpa1 && level);
            Connections[Opa2Out].Set(!viaOpa1 && SensedPhase >= 0 && level);
        }

        private const long ExtenCtr = 0x0;
        private const long Cr = 0x4;

        private const int Opa1Out = 0;
        private const int Opa2Out = 1;

        private uint extenCtr, cr;
        private bool level;
        private bool warned;
    }
}
