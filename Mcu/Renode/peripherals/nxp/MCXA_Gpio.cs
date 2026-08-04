//
// MCXA153 GPIO. The firmware uses exactly one read path: byte reads of
// PDR[n] at offset 0x60+n (the throttle pin state in getInputPinState()
// and the inverted-dshot idle detection). Pin states arrive as GPIO
// connections from the throttle generator; everything else the SDK
// writes (PDDR, ICR, ...) is stored and read back.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class MCXA_Gpio : IDoubleWordPeripheral, IBytePeripheral,
                             IKnownSize, IGPIOReceiver
    {
        public MCXA_Gpio()
        {
            Reset();
        }

        public long Size => 0x1000;

        public void Reset()
        {
            regs.Clear();
            for(var i = 0; i < pins.Length; i++)
            {
                pins[i] = false;
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= pins.Length)
            {
                return;
            }
            pins[number] = value;
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset >= Pdr && offset < Pdr + 32)
            {
                return pins[offset - Pdr] ? 1u : 0u;
            }
            uint v;
            regs.TryGetValue(offset, out v);
            return v;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            regs[offset] = value;
        }

        public byte ReadByte(long offset)
        {
            if(offset >= Pdr && offset < Pdr + 32)
            {
                return pins[offset - Pdr] ? (byte)1 : (byte)0;
            }
            return 0;
        }

        public void WriteByte(long offset, byte value)
        {
            if(offset >= Pdr && offset < Pdr + 32)
            {
                pins[offset - Pdr] = value != 0;
            }
        }

        private const long Pdr = 0x60;

        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly bool[] pins = new bool[32];
    }
}
