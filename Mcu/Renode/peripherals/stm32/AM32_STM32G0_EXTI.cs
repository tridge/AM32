//
// STM32G0 EXTI. Renode's stm32g0.repl declares IRQControllers.STM32F4_EXTI
// here, whose register map is the F4's: IMR at offset 0, one pending
// register, and nothing at all past 0x18. The G0 moved almost everything -
// IMR1 is at 0x80, and rising and falling have separate pending registers
// at 0x0C and 0x10 - so against the stock model every EXTI write from the
// firmware landed somewhere meaningless and no comparator edge ever
// reached the NVIC.
//
// AM32 senses BEMF through this: changeCompInput() arms one edge direction
// at a time on the comparator's line (18 for COMP2, 17 for COMP1 on
// N_VARIANT), and ADC1_COMP_IRQHandler tests RPR1/FPR1 to decide which
// fired. The handler deliberately returns *without* clearing the flag when
// the commutation is too young to be real, so the interrupt must stay
// asserted while a pending bit is set and the mask is on - that re-entry is
// how the firmware waits out the blanking window, and a model that
// self-cleared would break it.
//
// EXTICR is stored but not honoured: Renode wires every GPIO port's pin n
// to line n, so the port selection cannot be applied here. AM32 configures
// no GPIO EXTI line it actually triggers (it unmasks line 15 but arms
// neither edge), so this does not bite today.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.IRQControllers
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32G0_EXTI : IDoubleWordPeripheral, IKnownSize,
                                     INumberedGPIOOutput, IGPIOReceiver,
                                     Miscellaneous.IAM32TriggerNotifier
    {
        // lines 0-31 are configurable; 32 and up are the direct lines,
        // which have no edge selection and pass straight through the mask
        public AM32_STM32G0_EXTI(IMachine machine, int numberOfOutputLines = 34,
                                 int firstDirectLine = 19)
        {
            this.firstDirectLine = firstDirectLine;
            var conns = new Dictionary<int, IGPIO>();
            for(var i = 0; i < numberOfOutputLines; i++)
            {
                conns[i] = new GPIO();
            }
            Connections = conns;
            lines = numberOfOutputLines;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            rtsr = ftsr = imr = emr = rpr = fpr = 0;
            imr2 = emr2 = 0;
            for(var i = 0; i < exticr.Length; i++)
            {
                exticr[i] = 0;
            }
            for(var i = 0; i < lines; i++)
            {
                Connections[i].Unset();
            }
            state = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= lines)
            {
                return;
            }
            if(number >= firstDirectLine)
            {
                // direct line: the peripheral holds the state itself, the
                // mask is all that stands between it and the NVIC
                Connections[number].Set(value && MaskedIn(number));
                return;
            }

            var bit = 1u << number;
            var was = (state & bit) != 0;
            if(was == value)
            {
                return;
            }
            state = value ? (state | bit) : (state & ~bit);

            if(value && (rtsr & bit) != 0)
            {
                rpr |= bit;
            }
            if(!value && (ftsr & bit) != 0)
            {
                fpr |= bit;
            }
            Update(number);
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Rtsr1: return rtsr;
            case Ftsr1: return ftsr;
            case Swier1: return 0;
            case Rpr1: return rpr;
            case Fpr1: return fpr;
            case Imr1: return imr;
            case Emr1: return emr;
            case Imr2: return imr2;
            case Emr2: return emr2;
            default:
                if(offset >= Exticr0 && offset < Exticr0 + 4 * 4)
                {
                    return exticr[(offset - Exticr0) / 4];
                }
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Rtsr1:
            {
                // commit before notifying: the subscriber drives GPIO
                // edges from its callback, and those edges must be
                // evaluated against the trigger state just written,
                // not the stale one
                var changed = rtsr ^ value;
                rtsr = value;
                NotifyTriggerChanges(changed);
                return;
            }
            case Ftsr1:
            {
                var changed = ftsr ^ value;
                ftsr = value;
                NotifyTriggerChanges(changed);
                return;
            }
            case Swier1:
                // software interrupt: raises the rising pending bit
                rpr |= value;
                UpdateAll();
                return;
            case Rpr1:
                // write 1 to clear
                rpr &= ~value;
                UpdateAll();
                return;
            case Fpr1:
                fpr &= ~value;
                UpdateAll();
                return;
            case Imr1:
                imr = value;
                UpdateAll();
                return;
            case Emr1: emr = value; return;
            case Imr2:
                imr2 = value;
                UpdateAll();
                return;
            case Emr2: emr2 = value; return;
            default:
                if(offset >= Exticr0 && offset < Exticr0 + 4 * 4)
                {
                    exticr[(offset - Exticr0) / 4] = value;
                }
                return;
            }
        }

        private bool MaskedIn(int line)
        {
            return line < 32
                ? (imr & (1u << line)) != 0
                : (imr2 & (1u << (line - 32))) != 0;
        }

        private void Update(int line)
        {
            if(line >= firstDirectLine)
            {
                return;
            }
            var bit = 1u << line;
            Connections[line].Set(((rpr | fpr) & bit) != 0 && MaskedIn(line));
        }

        private void UpdateAll()
        {
            var n = firstDirectLine < lines ? firstDirectLine : lines;
            for(var i = 0; i < n; i++)
            {
                Update(i);
            }
        }

        // Fired with the line number for every RTSR1/FTSR1 bit a write
        // changes. The comparator-less G031 gives no other observable
        // signal of which phase changeCompInput() selected: it only ORs
        // and clears the current line's trigger bits, so the register
        // STATE keeps all three phase lines armed while the most recent
        // CHANGE names the current one (each phase alternates edge
        // between visits, so its revisit always flips its bits).
        public event System.Action<int> TriggerChanged;

        private void NotifyTriggerChanges(uint changed)
        {
            var handler = TriggerChanged;
            if(handler == null)
            {
                return;
            }
            for(var i = 0; changed != 0 && i < 32; i++)
            {
                if((changed & (1u << i)) != 0)
                {
                    changed &= ~(1u << i);
                    handler(i);
                }
            }
        }

        private const long Rtsr1 = 0x00;
        private const long Ftsr1 = 0x04;
        private const long Swier1 = 0x08;
        private const long Rpr1 = 0x0C;
        private const long Fpr1 = 0x10;
        private const long Exticr0 = 0x60;
        private const long Imr1 = 0x80;
        private const long Emr1 = 0x84;
        private const long Imr2 = 0x90;
        private const long Emr2 = 0x94;

        private readonly int firstDirectLine;
        private readonly int lines;
        private readonly uint[] exticr = new uint[4];
        private uint rtsr, ftsr, imr, emr, rpr, fpr, imr2, emr2;
        // current input level per line, to turn levels into edges
        private uint state;
    }
}
