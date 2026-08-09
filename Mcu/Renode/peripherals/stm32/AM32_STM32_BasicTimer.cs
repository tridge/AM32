// Lightweight STM32 basic timer for AM32's periodic control-loop tick.
//
// TIM6 has no capture/compare channels, but Renode's general STM32_Timer
// creates four capture/compare LimitTimers in addition to its counter.  It
// also updates all four on every overflow.  At AM32's 10 kHz control-loop
// rate that is substantial work for state which cannot be observed.  This
// model implements the basic-timer register subset with one clock entry.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32_BasicTimer : IDoubleWordPeripheral, IKnownSize
    {
        public AM32_STM32_BasicTimer(IMachine machine, ulong frequency,
                                     uint initialLimit = 0xFFFF)
        {
            this.machine = machine;
            this.initialLimit = initialLimit;
            timer = new LimitTimer(machine.ClockSource, frequency, this, "cnt",
                                   limit: initialLimit,
                                   direction: Direction.Ascending,
                                   enabled: false, eventEnabled: true);
            timer.LimitReached += OnLimitReached;
            IRQ = new GPIO();
            registers = new uint[0x100 / 4];
            Reset();
        }

        public void Reset()
        {
            System.Array.Clear(registers, 0, registers.Length);
            registers[AutoReload / 4] = initialLimit;
            timer.Reset();
            timer.Divider = 1;
            timer.Limit = initialLimit;
            timer.Value = 0;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case Control1:
                    return (registers[Control1 / 4] & ~1u)
                        | (timer.Enabled ? 1u : 0u);
                case Counter:
                    if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
                    {
                        cpu.SyncTime();
                    }
                    return (uint)timer.Value;
                case Prescaler:
                    return (uint)timer.Divider - 1;
                case AutoReload:
                    return registers[AutoReload / 4];
                default:
                    var index = offset / 4;
                    return index >= 0 && index < registers.Length
                        ? registers[index] : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var index = offset / 4;
            if(index < 0 || index >= registers.Length)
            {
                return;
            }
            switch(offset)
            {
                case Control1:
                    registers[index] = value;
                    timer.Enabled = (value & CounterEnable) != 0
                        && registers[AutoReload / 4] != 0;
                    return;
                case DmaOrInterruptEnable:
                    registers[index] = value;
                    UpdateInterrupt();
                    return;
                case Status:
                    // STM32 status is rc_w0: writing a zero clears a bit.
                    registers[index] &= value;
                    UpdateInterrupt();
                    return;
                case EventGeneration:
                    if((value & UpdateGeneration) != 0
                       && (registers[Control1 / 4] & UpdateDisable) == 0)
                    {
                        timer.Value = 0;
                        if((registers[Control1 / 4] & UpdateRequestSource) == 0
                           && (registers[DmaOrInterruptEnable / 4]
                               & UpdateInterruptEnable) != 0)
                        {
                            registers[Status / 4] |= UpdateInterruptFlag;
                        }
                    }
                    UpdateInterrupt();
                    return;
                case Counter:
                    timer.Value = value & 0xFFFF;
                    return;
                case Prescaler:
                    registers[index] = value & 0xFFFF;
                    timer.Divider = (value & 0xFFFF) + 1UL;
                    return;
                case AutoReload:
                    registers[index] = value & 0xFFFF;
                    if((value & 0xFFFF) == 0)
                    {
                        timer.Enabled = false;
                    }
                    else if((registers[Control1 / 4]
                             & AutoReloadPreloadEnable) == 0)
                    {
                        timer.Limit = value & 0xFFFF;
                    }
                    return;
                default:
                    registers[index] = value;
                    return;
            }
        }

        public long Size => 0x400;

        public GPIO IRQ { get; }

        private void OnLimitReached()
        {
            if((registers[Control1 / 4] & AutoReloadPreloadEnable) != 0)
            {
                var nextLimit = registers[AutoReload / 4];
                if(nextLimit != 0 && timer.Limit != nextLimit)
                {
                    timer.Limit = nextLimit;
                }
            }
            if((registers[Control1 / 4] & UpdateDisable) == 0)
            {
                registers[Status / 4] |= UpdateInterruptFlag;
                UpdateInterrupt();
            }
        }

        private void UpdateInterrupt()
        {
            IRQ.Set((registers[Status / 4] & UpdateInterruptFlag) != 0
                    && (registers[DmaOrInterruptEnable / 4]
                        & UpdateInterruptEnable) != 0);
        }

        private readonly IMachine machine;
        private readonly uint initialLimit;
        private readonly LimitTimer timer;
        private readonly uint[] registers;

        private const long Control1 = 0x00;
        private const long DmaOrInterruptEnable = 0x0C;
        private const long Status = 0x10;
        private const long EventGeneration = 0x14;
        private const long Counter = 0x24;
        private const long Prescaler = 0x28;
        private const long AutoReload = 0x2C;

        private const uint CounterEnable = 1u << 0;
        private const uint UpdateDisable = 1u << 1;
        private const uint UpdateRequestSource = 1u << 2;
        private const uint AutoReloadPreloadEnable = 1u << 7;
        private const uint UpdateInterruptEnable = 1u << 0;
        private const uint UpdateInterruptFlag = 1u << 0;
        private const uint UpdateGeneration = 1u << 0;
    }
}
