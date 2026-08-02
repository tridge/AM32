//
// Input capture timer that can raise a DMA request.
//
// Renode's Timers.STM32_Timer implements input capture but tags the
// DIER DMA-enable bits, so there is no way for a capture to trigger a
// DMA transfer. AM32 decodes its throttle signal entirely through that
// path: Mcu/f051/Src/IO.c programs CCMR1=0x41 (CC1S=01, IC1F=0100),
// CCER both-edge, DIER.CC1DE, and points DMA1 at CCR1, so every edge
// has to land a captured count in dma_buffer[]. Hence a fresh model -
// Renode's register fields are private, so subclassing is not an
// option.
//
// The counter must wrap at exactly 16 bits: detectInput() in
// Src/signal.c relies on wraparound arithmetic to reject garbage
// deltas, and checkServo() only accepts 200 < smallestnumber < 20000.
//
// Connections:
//   [0] DMA request, wire to the DMA channel the target uses
//   [1] timer IRQ
// It is also an IGPIOReceiver: input 0 is the channel 1 capture pin.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Timers
{
    // The translations are load-bearing, not defensive: the DMA reads
    // CCR1 with PSIZE=16, and without them Renode returns 0 and logs
    // only "Attempted Word read isn't supported", so dma_buffer fills
    // with zeros and detectInput() never locks.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32_CaptureTimer : IDoubleWordPeripheral, IKnownSize,
                                           INumberedGPIOOutput, IGPIOReceiver
    {
        public AM32_STM32_CaptureTimer(IMachine machine, ulong frequency = 48000000)
        {
            this.frequency = frequency;
            var conns = new Dictionary<int, IGPIO>();
            conns[DmaRequestLine] = new GPIO();
            conns[IrqLine] = new GPIO();
            Connections = conns;

            counter = new LimitTimer(machine.ClockSource, frequency, this,
                                     "cnt", MaxCount + 1,
                                     direction: Direction.Ascending,
                                     enabled: false, autoUpdate: true,
                                     eventEnabled: false);
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
            regs[ARR / 4] = MaxCount;
            counter.Enabled = false;
            counter.Divider = 1;
            counter.Value = 0;
            lastPinState = false;
            havePin = false;
            Connections[DmaRequestLine].Unset();
            Connections[IrqLine].Unset();
        }

        // pulsed by RCC APBxRSTR; receiveDshotDma() resets the timer on
        // every direction change and the firmware relies on it
        public void PeripheralReset()
        {
            Reset();
        }

        // debug entry point: inject a capture without going through the
        // pin, to bisect generator-side faults from DMA-side ones
        public void Capture(uint value)
        {
            DoCapture(value);
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == CNT)
            {
                return CurrentCount;
            }
            var idx = offset / 4;
            return (idx >= 0 && idx < regs.Length) ? regs[idx] : 0;
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
            case CR1:
                regs[idx] = value;
                counter.Enabled = (value & CEN) != 0;
                return;
            case PSC:
                regs[idx] = value & MaxCount;
                // takes effect at the next update event, but AM32 always
                // follows a PSC write with EGR.UG, so apply on UG only
                return;
            case EGR:
                if((value & UG) != 0)
                {
                    counter.Divider = (ulong)((regs[PSC / 4] & MaxCount) + 1);
                    counter.Value = 0;
                }
                return;
            case SR:
                // rc_w0: writing 0 to a bit clears it
                regs[idx] &= value;
                UpdateIrq();
                return;
            case CNT:
                counter.Value = value & MaxCount;
                return;
            default:
                regs[idx] = value;
                if(offset == DIER)
                {
                    UpdateIrq();
                }
                return;
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                return;
            }
            var wasSet = havePin && lastPinState;
            havePin = true;
            lastPinState = value;
            if(!Counting)
            {
                return;
            }
            // CCER: CC1P selects falling, CC1NP with CC1P means both
            var ccer = regs[CCER / 4];
            var wantRising = (ccer & CC1P) == 0 || (ccer & CC1NP) != 0;
            var wantFalling = (ccer & CC1P) != 0 || (ccer & CC1NP) != 0;
            var rising = value && !wasSet;
            var falling = !value && wasSet;
            if((rising && wantRising) || (falling && wantFalling))
            {
                DoCapture(CurrentCount);
            }
        }

        private void DoCapture(uint value)
        {
            regs[CCR1 / 4] = value;
            regs[SR / 4] |= CC1IF;
            UpdateIrq();
            if((regs[DIER / 4] & CC1DE) != 0)
            {
                // edge-triggered request; the DMA samples the line
                Connections[DmaRequestLine].Blink();
            }
        }

        private void UpdateIrq()
        {
            var pending = (regs[SR / 4] & regs[DIER / 4] & CC1IE) != 0;
            Connections[IrqLine].Set(pending);
        }

        private bool Counting => (regs[CR1 / 4] & CEN) != 0;
        private uint CurrentCount => (uint)(counter.Value & MaxCount);

        private const long CR1 = 0x00;
        private const long DIER = 0x0C;
        private const long SR = 0x10;
        private const long EGR = 0x14;
        private const long CCER = 0x20;
        private const long CNT = 0x24;
        private const long PSC = 0x28;
        private const long ARR = 0x2C;
        private const long CCR1 = 0x34;

        private const uint CEN = 1u << 0;
        private const uint UG = 1u << 0;
        private const uint CC1IE = 1u << 1;
        private const uint CC1IF = 1u << 1;
        private const uint CC1DE = 1u << 9;
        private const uint CC1P = 1u << 1;
        private const uint CC1NP = 1u << 3;
        private const uint MaxCount = 0xFFFF;

        private const int DmaRequestLine = 0;
        private const int IrqLine = 1;

        private readonly ulong frequency;
        private readonly LimitTimer counter;
        private readonly uint[] regs = new uint[0x100];
        private bool lastPinState;
        private bool havePin;
    }
}
