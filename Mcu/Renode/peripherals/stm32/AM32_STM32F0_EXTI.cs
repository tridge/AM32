//
// The F0/F4 single-bank EXTI (IMR at 0x00, one pending register at
// 0x14), modelled fully. The stock STM32F4_EXTI almost serves - and
// does serve the other families here - but its software trigger only
// pulses the NVIC line: the SWIER bit never reads back and the pending
// register never shows it. The F051/E230/L431 firmware survives that
// because their EXTI handlers clear the flag unconditionally; the
// F421's EXINT15_4_IRQHandler guards the clear on `intsts &
// EXINT_LINE_15`, which against the stock model always reads 0, so the
// line stayed asserted and the handler re-entered forever - 6.4
// million entries in 0.7 simulated seconds, starving every
// lower-priority interrupt including the 20kHz loop timer, and the ESC
// never armed.
//
// So this model keeps the real semantics: a software trigger or an
// armed GPIO edge sets the pending bit, the bit reads back, writing 1
// clears it (and the SWIER bit with it, as on hardware), and line n's
// output to the NVIC is pending AND unmasked - which also preserves
// the deliberate non-clearing re-entry the comparator handlers rely on
// to wait out the commutation blanking window.
//
// EXTICR does not exist at this address block on the F0 generation
// (it lives in SYSCFG/SCFG), and Renode wires every GPIO port's pin n
// to line n regardless; AM32 arms triggers only for the comparator
// line, so the port selection never matters.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.IRQControllers
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32F0_EXTI : IDoubleWordPeripheral, IKnownSize,
                                     INumberedGPIOOutput, IGPIOReceiver
    {
        public AM32_STM32F0_EXTI(IMachine machine, int numberOfOutputLines = 32)
        {
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
            imr = emr = rtsr = ftsr = swier = pending = 0;
            state = 0;
            for(var i = 0; i < lines; i++)
            {
                Connections[i].Unset();
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= lines)
            {
                return;
            }
            var bit = 1u << number;
            var was = (state & bit) != 0;
            if(was == value)
            {
                return;
            }
            state = value ? (state | bit) : (state & ~bit);

            if((value && (rtsr & bit) != 0) || (!value && (ftsr & bit) != 0))
            {
                pending |= bit;
                Update(number);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Imr: return imr;
            case Emr: return emr;
            case Rtsr: return rtsr;
            case Ftsr: return ftsr;
            case Swier: return swier;
            case Pr: return pending;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Imr:
                imr = value;
                UpdateAll();
                return;
            case Emr:
                emr = value;
                return;
            case Rtsr:
                rtsr = value;
                return;
            case Ftsr:
                ftsr = value;
                return;
            case Swier:
                // writing 1 to a clear bit sets it and pends the line;
                // writing 0, or 1 to an already set bit, does nothing
                pending |= value & ~swier;
                swier |= value;
                UpdateAll();
                return;
            case Pr:
                // write 1 to clear, taking the software trigger bit
                // down with it as on hardware
                pending &= ~value;
                swier &= ~value;
                UpdateAll();
                return;
            }
        }

        private void Update(int line)
        {
            var bit = 1u << line;
            Connections[line].Set((pending & imr & bit) != 0);
        }

        private void UpdateAll()
        {
            for(var i = 0; i < lines; i++)
            {
                Update(i);
            }
        }

        private const long Imr = 0x00;
        private const long Emr = 0x04;
        private const long Rtsr = 0x08;
        private const long Ftsr = 0x0C;
        private const long Swier = 0x10;
        private const long Pr = 0x14;

        private readonly int lines;
        private uint imr, emr, rtsr, ftsr, swier, pending;
        // current input level per line, to turn levels into edges
        private uint state;
    }
}
