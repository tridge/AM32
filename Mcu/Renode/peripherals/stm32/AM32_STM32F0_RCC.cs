//
// RCC for STM32F0, enough of it for AM32's SystemClock_Config().
//
// The platform Renode ships uses a Python placeholder here. That
// satisfies the HSI and HSI14 ready loops by accident but not LSI, so
// the firmware parks forever in peripherals.c:95. Rather than guess at
// which bits happen to work, mirror every enable bit to its ready bit
// and echo the clock switch, which is all SystemClock_Config() waits
// on:
//
//   CR   HSION  -> HSIRDY,  HSEON -> HSERDY,  PLLON -> PLLRDY
//   CR2  HSI14ON-> HSI14RDY
//   CSR  LSION  -> LSIRDY
//   BDCR LSEON  -> LSERDY
//   CFGR SW     -> SWS
//
// Everything else is plain storage. That matters more than it looks:
// the clock enables in AHBENR/APB1ENR/APB2ENR must read back, and
// APB2RSTR is pulsed on every dshot direction change to reset the
// input capture timer.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AM32_STM32F0_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public long Size => 0x400;

        private const long CR = 0x00;
        private const long CFGR = 0x04;
        private const long APB2RSTR = 0x0C;
        private const long APB1RSTR = 0x10;
        private const long BDCR = 0x20;
        private const long CSR = 0x24;
        private const long AHBRSTR = 0x28;
        private const long CR2 = 0x34;

        private readonly uint[] regs = new uint[0x100];

        public AM32_STM32F0_RCC()
        {
            Reset();
        }

        public void Reset()
        {
            for(var i = 0; i < regs.Length; i++)
            {
                regs[i] = 0;
            }
            // reset state: HSI on and ready, as on silicon
            regs[CR / 4] = (1u << 0) | (1u << 1);
            regs[CR2 / 4] = (1u << 0) | (1u << 1);
        }

        public uint ReadDoubleWord(long offset)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return 0;
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

            switch(offset)
            {
            case CR:
                value = Mirror(value, 0, 1);    // HSION  -> HSIRDY
                value = Mirror(value, 16, 17);  // HSEON  -> HSERDY
                value = Mirror(value, 24, 25);  // PLLON  -> PLLRDY
                break;
            case CR2:
                value = Mirror(value, 0, 1);    // HSI14ON -> HSI14RDY
                break;
            case CSR:
                value = Mirror(value, 0, 1);    // LSION  -> LSIRDY
                break;
            case BDCR:
                value = Mirror(value, 0, 1);    // LSEON  -> LSERDY
                break;
            case CFGR:
                // SWS[3:2] follows SW[1:0]: the firmware spins until the
                // system clock source reads back as PLL. A placeholder
                // returning all-ones fails here, since SWS would read
                // 0b11 and PLL is 0b10.
                value = (value & ~0xCu) | ((value & 0x3u) << 2);
                break;
            case APB2RSTR:
            case APB1RSTR:
            case AHBRSTR:
                // peripheral resets are pulsed set-then-clear; the reset
                // itself is routed by the platform file, not modelled here
                break;
            }

            regs[idx] = value;
        }

        private static uint Mirror(uint value, int enableBit, int readyBit)
        {
            if((value & (1u << enableBit)) != 0)
            {
                return value | (1u << readyBit);
            }
            return value & ~(1u << readyBit);
        }
    }
}
