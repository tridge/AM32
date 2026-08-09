// Lightweight STM32 basic timer for counters which AM32 only polls.
//
// TIM2 (the commutation interval clock) and TIM7 (the microsecond utility
// clock) never generate an interrupt in AM32. Renode's general STM32_Timer
// creates five LimitTimers per instance even in that case. Deriving CNT from
// virtual time preserves the observable free-running counter without adding
// scheduled clock entries.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using System;

namespace Antmicro.Renode.Peripherals.Timers
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32_FreeRunningTimer : IDoubleWordPeripheral, IKnownSize
    {
        public AM32_STM32_FreeRunningTimer(IMachine machine, uint frequency,
                                           uint initialLimit = 0xFFFF,
                                           bool synchronize = true)
        {
            this.machine = machine;
            this.frequency = frequency;
            resetLimit = initialLimit;
            this.synchronize = synchronize;
            IRQ = new GPIO();
            registers = new uint[0x100 / 4];
            var pendingTimeMethod = machine.ClockSource.GetType().GetMethod(
                "GetCurrentValueWithPendingTicks", Type.EmptyTypes);
            if(pendingTimeMethod != null && pendingTimeMethod.ReturnType == typeof(ulong))
            {
                currentTicks = (Func<ulong>)Delegate.CreateDelegate(
                    typeof(Func<ulong>), machine.ClockSource, pendingTimeMethod);
            }
            Reset();
        }

        public void Reset()
        {
            System.Array.Clear(registers, 0, registers.Length);
            prescaler = 0;
            UpdateScale();
            autoReload = resetLimit;
            counter = 0;
            enabled = false;
            phaseOriginTicks = NowTicks;
            anchorTimerTicks = 0;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case Control1:
                    return (registers[Control1 / 4] & ~1u) | (enabled ? 1u : 0u);
                case Counter:
                    if(synchronize && machine.SystemBus.TryGetCurrentCPU(out var cpu))
                    {
                        cpu.SyncTime();
                    }
                    return CurrentCounter(NowTicks);
                case Prescaler:
                    return prescaler;
                case AutoReload:
                    return autoReload;
                default:
                    var index = offset / 4;
                    return index >= 0 && index < registers.Length
                        ? registers[index] : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var now = NowTicks;
            switch(offset)
            {
                case Control1:
                    Anchor(now);
                    enabled = (value & 1) != 0;
                    registers[Control1 / 4] = value;
                    return;
                case Counter:
                    counter = Limit(value);
                    anchorTimerTicks = ClockTicks(now);
                    return;
                case Prescaler:
                    Anchor(now);
                    prescaler = value & 0xFFFF;
                    UpdateScale();
                    phaseOriginTicks = now;
                    anchorTimerTicks = 0;
                    registers[Prescaler / 4] = prescaler;
                    return;
                case AutoReload:
                    Anchor(now);
                    autoReload = value;
                    counter = Limit(counter);
                    registers[AutoReload / 4] = value;
                    return;
                case EventGeneration:
                    registers[EventGeneration / 4] = value;
                    if((value & 1) != 0)
                    {
                        counter = 0;
                        anchorTimerTicks = ClockTicks(now);
                    }
                    return;
                default:
                    var index = offset / 4;
                    if(index >= 0 && index < registers.Length)
                    {
                        registers[index] = value;
                    }
                    return;
            }
        }

        public long Size => 0x400;

        public GPIO IRQ { get; }

        private void Anchor(ulong now)
        {
            counter = CurrentCounter(now);
            anchorTimerTicks = ClockTicks(now);
        }

        private uint CurrentCounter(ulong now)
        {
            if(!enabled)
            {
                return counter;
            }
            return Limit((ulong)counter + ClockTicks(now) - anchorTimerTicks);
        }

        private ulong ClockTicks(ulong now)
        {
            // Compute delta * frequency / (prescaler * ticksPerSecond)
            // without allowing the first multiplication to overflow. At
            // 80 MHz that naive product wraps after about 230 seconds.
            var delta = now - phaseOriginTicks;
            if(clockTicksPerCounterTick != 0)
            {
                return delta / clockTicksPerCounterTick;
            }
            var ticksPerSecond = (ulong)TimeInterval.TicksPerSecond;
            var prescalerDivider = (ulong)prescaler + 1;
            var seconds = delta / ticksPerSecond;
            var subsecond = delta % ticksPerSecond;
            var wholeFrequency = frequency / prescalerDivider;
            var frequencyRemainder = frequency % prescalerDivider;
            var secondsRemainder = seconds * frequencyRemainder;
            return seconds * wholeFrequency
                + secondsRemainder / prescalerDivider
                + ((secondsRemainder % prescalerDivider) * ticksPerSecond
                   + subsecond * frequency)
                    / (prescalerDivider * ticksPerSecond);
        }

        private void UpdateScale()
        {
            var prescalerDivider = (ulong)prescaler + 1;
            var ticksPerSecond = (ulong)TimeInterval.TicksPerSecond;
            if(frequency % prescalerDivider == 0)
            {
                var counterFrequency = frequency / prescalerDivider;
                if(counterFrequency != 0 && ticksPerSecond % counterFrequency == 0)
                {
                    clockTicksPerCounterTick = ticksPerSecond / counterFrequency;
                    return;
                }
            }
            clockTicksPerCounterTick = 0;
        }

        private uint Limit(ulong value)
        {
            var period = (ulong)autoReload + 1;
            return (uint)(value % period);
        }

        private ulong NowTicks
        {
            get
            {
                return currentTicks != null ? currentTicks()
                    : machine.ElapsedVirtualTime.TimeElapsed.Ticks;
            }
        }

        private readonly IMachine machine;
        private readonly uint frequency;
        private readonly uint resetLimit;
        private readonly bool synchronize;
        private readonly uint[] registers;
        private ulong phaseOriginTicks;
        private ulong anchorTimerTicks;
        private ulong clockTicksPerCounterTick;
        private uint counter;
        private uint prescaler;
        private uint autoReload;
        private bool enabled;
        private readonly Func<ulong> currentTicks;

        private const long Control1 = 0x00;
        private const long EventGeneration = 0x14;
        private const long Counter = 0x24;
        private const long Prescaler = 0x28;
        private const long AutoReload = 0x2C;
    }
}
