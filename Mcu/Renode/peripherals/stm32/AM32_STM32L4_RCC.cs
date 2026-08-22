//
// RCC for the STM32L431, enough of it for the firmware's clock setup
// and - load-bearing for the bootloader - the reset cause.
//
// The clock half mirrors each enable bit to its ready bit and echoes
// the clock switch, exactly as the Python placeholder it replaces did:
//
//   CR    MSION->MSIRDY, HSION->HSIRDY, HSEON->HSERDY, PLLON->PLLRDY
//   CFGR  SWS[3:2] mirrors SW[1:0]
//   PLLCFGR stored and read back: sys_can_init() derives PCLK1 from
//     LL_RCC_GetSystemClocksFreq(), which recomputes the PLL output
//     from PLLCFGR - a zero readback makes that 0 MHz and the CAN
//     bitrate switch hangs forever in its default arm
//   BDCR  LSEON->LSERDY
//   CSR   LSION->LSIRDY
//
// CSR also carries the reset cause. checkForSignal() jumps to the
// application when the signal pin reads low unless the reset was a
// software one - and the application's own signal-loss reboot is
// exactly that. Without SFTRSTF the emulated ESC bounces between the
// bootloader and the app forever and no configurator can catch it.
// The CPU resets the machine on a SYSRESETREQ write to AIRCR with no
// cause reaching peripherals, so the write itself is watched, as the
// F0 RCC model does.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AM32_STM32L4_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public long Size => 0x400;

        private const long CR = 0x00;
        private const long CFGR = 0x08;
        private const long PLLCFGR = 0x0C;
        private const long BDCR = 0x90;
        private const long CSR = 0x94;

        private const int SftRstF = 28;
        private const int PinRstF = 26;
        private const int RmvF = 23;
        private const ulong Aircr = 0xE000ED0C;
        private const uint AircrKey = 0x05FA0000;
        private const uint SysResetReq = 1u << 2;

        private readonly uint[] regs = new uint[0x100];
        private bool pendingSoftwareReset;
        private bool softwareReset;
        private bool everReset;

        public AM32_STM32L4_RCC(IMachine machine)
        {
            machine.SystemBus.AddWatchpointHook(
                Aircr, SysbusAccessWidth.DoubleWord, Access.Write,
                (cpu, address, width, value) =>
                {
                    if((value & 0xFFFF0000u) == AircrKey
                       && (value & SysResetReq) != 0)
                    {
                        pendingSoftwareReset = true;
                    }
                });
            Reset();
        }

        public void Reset()
        {
            for(var i = 0; i < regs.Length; i++)
            {
                regs[i] = 0;
            }
            softwareReset = pendingSoftwareReset;
            pendingSoftwareReset = false;
            everReset = true;
        }

        public uint ReadDoubleWord(long offset)
        {
            var idx = offset / 4;
            if(idx < 0 || idx >= regs.Length)
            {
                return 0;
            }
            var v = regs[idx];
            switch(offset)
            {
            case CR:
                return v | ((v & 1u) << 1)             // MSION  -> MSIRDY
                         | (((v >> 8) & 1u) << 10)     // HSION  -> HSIRDY
                         | (((v >> 16) & 1u) << 17)    // HSEON  -> HSERDY
                         | (((v >> 24) & 1u) << 25);   // PLLON  -> PLLRDY
            case CFGR:
                // SWS[3:2] mirrors SW[1:0]
                return (v & 0x3u) | ((v & 0x3u) << 2);
            case BDCR:
                return v | ((v & 1u) << 1);            // LSEON  -> LSERDY
            case CSR:
                v |= (v & 1u) << 1;                    // LSION  -> LSIRDY
                if(softwareReset)
                {
                    v |= 1u << SftRstF;
                }
                return v;
            default:
                return v;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == CSR && (value & (1u << RmvF)) != 0)
            {
                // RMVF clears the cause once the firmware has read it
                softwareReset = false;
            }
            var idx = offset / 4;
            if(idx >= 0 && idx < regs.Length)
            {
                if(offset == CFGR)
                {
                    value &= 0x3u;                     // only SW is stored
                }
                regs[idx] = value;
            }
        }
    }
}
