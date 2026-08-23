//
// A block of registers that reads back what was written, and nothing
// else. Renode warns on every access to an unimplemented region, and a
// warning that fires thousands of times per second hides the ones worth
// reading - the IWDG taught us that.
//
// Use this only where the firmware writes a peripheral whose behaviour
// the harness does not need to model, and say in the platform why. On
// the G0 that is DMAMUX, whose routing the platform hardwires instead,
// and SYSCFG. Anything the firmware reads back and acts on needs a real
// model, not this.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_RegisterFile : IDoubleWordPeripheral, IKnownSize
    {
        public AM32_RegisterFile(IMachine machine, int size = 0x400,
            bool preserveOnReset = false, long readAsZero = -1)
        {
            this.size = size;
            this.preserveOnReset = preserveOnReset;
            // a status register the firmware polls and write-1-clears:
            // storing the written flags would make a busy bit stick
            // (real hardware treats those bits as read-only or W1C),
            // so the offset always reads idle and drops writes
            this.readAsZero = readAsZero;
            regs = new uint[size / 4];
        }

        public long Size => size;

        public void Reset()
        {
            if(preserveOnReset)
            {
                return;
            }
            for(var i = 0; i < regs.Length; i++)
            {
                regs[i] = 0;
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == readAsZero)
            {
                return 0;
            }
            var i = offset / 4;
            return (i >= 0 && i < regs.Length) ? regs[i] : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == readAsZero)
            {
                return;
            }
            var i = offset / 4;
            if(i >= 0 && i < regs.Length)
            {
                regs[i] = value;
            }
        }

        private readonly int size;
        private readonly bool preserveOnReset;
        private readonly long readAsZero;
        private readonly uint[] regs;
    }
}
