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
using Antmicro.Renode.Exceptions;
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
                                           INumberedGPIOOutput, IGPIOReceiver,
                                           Miscellaneous.IAM32ReplySource,
                                           Miscellaneous.IAM32DshotFrameSink
    {
        // inputBase/inputPin/inputAf describe the pin the throttle
        // arrives on. Without them a capture happens whatever the pin is
        // configured as, so firmware that never put it in alternate mode
        // or picked the wrong AF still decodes perfectly.
        // channel selects which capture/compare channel the throttle
        // rides on: 1 everywhere except the F031's TIM2_CH3 groups
        public AM32_STM32_CaptureTimer(IMachine machine, ulong frequency = 48000000,
                                       ulong inputBase = 0, int inputPin = 0,
                                       uint inputAf = 0, int channel = 1)
        {
            if(channel < 1 || channel > 4)
            {
                throw new RecoverableException("channel must be 1..4");
            }
            this.channel = channel;
            ccmrOffset = channel <= 2 ? 0x18 : 0x1C;
            ccsShift = 8 * ((channel - 1) & 1);
            ccerShift = 4 * (channel - 1);
            ccrOffset = 0x34 + 4 * (channel - 1);
            this.machine = machine;
            this.inputBase = inputBase;
            this.inputPin = inputPin;
            this.inputAf = inputAf;
            this.frequency = frequency;
            var conns = new Dictionary<int, IGPIO>();
            conns[DmaRequestLine] = new GPIO();
            conns[IrqLine] = new GPIO();
            conns[OutputLine] = new GPIO();
            Connections = conns;

            counter = new LimitTimer(machine.ClockSource, frequency, this,
                                     "cnt", MaxCount + 1,
                                     direction: Direction.Ascending,
                                     enabled: false, autoUpdate: true,
                                     eventEnabled: true);
            counter.LimitReached += OnPeriod;
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
            // CCxS = 01: reset into input capture, so a timer that has
            // not been programmed yet does not drive the shared wire
            regs[ccmrOffset / 4] = 1u << ccsShift;
            counter.Enabled = false;
            counter.Divider = 1;
            counter.Limit = MaxCount + 1;
            counter.Value = 0;
            lastPinState = false;
            havePin = false;
            Connections[DmaRequestLine].Unset();
            Connections[IrqLine].Unset();
            // The shared throttle wire is pulled up, so "not driving" is
            // high, not low. Asserting this line low while the channel is
            // an input would look to the generator like the ESC holding
            // the wire down, and no dshot frame would get through.
            Connections[OutputLine].Set(true);
            replyBits = 0;
            batchedDshotFrame = false;
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

        // Record the counter phase at the first edge. The generator will
        // call CompleteDshotFrame after the 16 bit periods have elapsed;
        // until then no guest-visible capture has occurred.
        public bool BeginDshotFrame(uint frame, ulong bitPeriodNanoseconds)
        {
            if(batchedDshotFrame || !Counting || OutputMode
               || !CaptureEnabled || !PinRouted || !CapturesBothEdges)
            {
                return false;
            }
            batchedDshotFrame = true;
            batchedFrameValue = frame;
            batchedBitPeriodNanoseconds = bitPeriodNanoseconds;
            batchedStartCount = CurrentCount;
            batchedCounterDivider = (regs[PSC / 4] & MaxCount) + 1;
            batchedCounterPeriod = (regs[ARR / 4] & MaxCount) + 1;
            batchDshotReplies = true;
            // Service the longest reply transfer AM32 configures: 23
            // encoded/preamble slots plus 14 padding slots. The firmware
            // selects its padding from the measured shortest edge, not the
            // nominal incoming bitrate; in particular this L431 model can
            // select 14 for BDShot300. If a family selects seven instead,
            // its DMA ignores the final requests after completing at 30.
            batchedReplyPeriods = 37;
            return true;
        }

        public void CompleteDshotFrame()
        {
            if(!batchedDshotFrame)
            {
                return;
            }
            batchedDshotFrame = false;
            // If firmware reconfigured the timer during the frame, behave
            // like edges that arrived while capture was unavailable.
            if(!Counting || OutputMode || !CaptureEnabled)
            {
                return;
            }
            for(var bit = 0; bit < 16; bit++)
            {
                var start = (ulong)bit * batchedBitPeriodNanoseconds;
                var one = (batchedFrameValue & (0x8000u >> bit)) != 0;
                var high = one ? batchedBitPeriodNanoseconds * 3 / 4
                               : batchedBitPeriodNanoseconds * 3 / 8;
                DoCapture(BatchedCountAt(start));
                DoCapture(BatchedCountAt(start + high));
            }
        }

        public void CancelDshotFrame()
        {
            batchedDshotFrame = false;
            batchDshotReplies = false;
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
            if(offset == ccmrOffset)
            {
                regs[idx] = value;
                if(!OutputMode)
                {
                    // the reply is finished; decode what we drove, then
                    // release the wire
                    DecodeReply();
                    ResetReply();
                    Connections[OutputLine].Set(true);
                }
                else
                {
                    ResetReply();
                }
                return;
            }
            switch(offset)
            {
            case CR1:
                regs[idx] = value;
                counter.Enabled = (value & CEN) != 0;
                if(counter.Enabled && OutputMode && batchDshotReplies)
                {
                    // Let guest code run while the reply DMA is active,
                    // then service its periods together at the time the
                    // complete line code would have finished.
                    counter.Limit = ((regs[ARR / 4] & MaxCount) + 1)
                        * batchedReplyPeriods;
                }
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
                    counter.Limit = (regs[ARR / 4] & MaxCount) + 1;
                    counter.Value = 0;
                }
                return;
            case ARR:
                regs[idx] = value & MaxCount;
                // input capture leaves this at 0xFFFF and relies on the
                // 16 bit wrap; the dshot reply sets it to a bit period
                counter.Limit = (value & MaxCount) + 1;
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
            // in output mode the channel drives the wire, it does not
            // listen to it; capturing our own reply would corrupt
            // dma_buffer
            if(!Counting || OutputMode || !CaptureEnabled || !PinRouted)
            {
                return;
            }
            // CCER: CCxP selects falling, CCxNP with CCxP means both
            var ccer = regs[CCER / 4] >> ccerShift;
            var wantRising = (ccer & CC1P) == 0 || (ccer & CC1NP) != 0;
            var wantFalling = (ccer & CC1P) != 0 || (ccer & CC1NP) != 0;
            // the CH32V203's route to both-edge capture: IC1 mapped to
            // TRC (CC1S=11) with the TI1 edge detector as the trigger
            // (SMCR TS=100), CC1P left at rising (Mcu/v203/Src/IO.c).
            // Channel 1 only: TI1F_ED is specifically the channel-1
            // input's edge detector
            if(channel == 1
               && ((regs[ccmrOffset / 4] >> ccsShift) & CC1S) == CC1S
               && (regs[SMCR / 4] & TsMask) == TsTi1Ed)
            {
                wantRising = true;
                wantFalling = true;
            }
            var rising = value && !wasSet;
            var falling = !value && wasSet;
            if((rising && wantRising) || (falling && wantFalling))
            {
                DoCapture(CurrentCount);
            }
        }

        private void DoCapture(uint value)
        {
            regs[ccrOffset / 4] = value;
            regs[SR / 4] |= 1u << channel;
            UpdateIrq();
            if((regs[DIER / 4] & (1u << (8 + channel))) != 0)
            {
                // edge-triggered request; the DMA samples the line
                Connections[DmaRequestLine].Blink();
            }
        }

        private uint BatchedCountAt(ulong nanoseconds)
        {
            var ticks = nanoseconds * frequency
                / (batchedCounterDivider * NanosecondsPerSecond);
            return (uint)((batchedStartCount + ticks) % batchedCounterPeriod);
        }

        // One counter period. In input capture mode this is just the 16
        // bit wrap and nothing happens. In output mode it is one bit of
        // the bidirectional dshot reply: sendDshotDma() puts the timer in
        // PWM mode with ARR as the bit period and points the DMA at CCR1,
        // so each period consumes one gcr[] entry and drives the line.
        //
        // The line is driven at one level per period rather than as a
        // real PWM waveform. AM32 writes gcr[] entries of 0 or 64 against
        // ARR 92, so on hardware a "1" period is a 70% duty pulse, but
        // the GCR line code only carries information in the transitions
        // between periods. Modelling the intra-period edge would add a
        // second timer event per bit for nothing the decode looks at.
        private void OnPeriod()
        {
            if(!OutputMode)
            {
                return;
            }
            if(batchDshotReplies)
            {
                for(var i = 0ul; i < batchedReplyPeriods; i++)
                {
                    OutputOnePeriod();
                }
                counter.Enabled = false;
                return;
            }
            OutputOnePeriod();
        }

        private void OutputOnePeriod()
        {
            var level = (regs[ccrOffset / 4] & MaxCount) != 0;
            if(((regs[CCER / 4] >> ccerShift) & CC1P) != 0)
            {
                level = !level;
            }
            Connections[OutputLine].Set(level && OutputEnabled);
            RecordReplyBit(level && OutputEnabled);
            if((regs[DIER / 4] & (1u << (8 + channel))) != 0)
            {
                // ask the DMA for the next bit
                Connections[DmaRequestLine].Blink();
            }
            regs[SR / 4] |= 1u << channel;
            UpdateIrq();
        }

        // always the reply carrier when present: the STM32 families have
        // exactly one capture timer and it owns the wire
        public bool DecodesReplies => true;

        // The last bidirectional dshot reply this timer actually drove on
        // the wire, GCR decoded back to the 16 bit frame: 12 bits of
        // eRPM-or-EDT payload and a 4 bit CRC. Decoded from the levels
        // that were output, not from the firmware's gcr[] buffer, so it
        // checks the transmit path rather than restating it.
        public uint LastReplyFrame { get; private set; }

        // how many replies have been decoded, so a test can tell "no
        // reply yet" from "a reply of zero"
        public uint ReplyCount { get; private set; }

        // A short history of decoded frames, indexed by reply number, so a
        // client that polls slower than the reply rate still receives
        // every frame rather than every Nth. That matters beyond neatness:
        // extended telemetry interleaves temperature, voltage and current
        // between the eRPM frames, so dropping three replies in four drops
        // three quarters of each telemetry kind.
        public bool TryGetReply(uint index, out uint frame)
        {
            frame = 0;
            if(index >= ReplyCount || ReplyCount - index > (uint)replyRing.Length)
            {
                return false;
            }
            frame = replyRing[index % (uint)replyRing.Length];
            return true;
        }

        // the last reply's levels packed LSB-first, one bit per period,
        // for diagnosing a decode that does not line up
        public ulong ReplyRaw { get; private set; }

        // bit n set if a decoded frame had top nibble n. eRPM and each
        // extended telemetry type carry a different nibble, so a test can
        // tell which kinds of reply went out over a run without having to
        // sample a single frame at exactly the right moment.
        public uint ReplyTypeMask { get; private set; }

        // replies whose 21 periods were not a legal GCR code, and replies
        // that decoded but failed their own CRC
        public uint ReplyGcrErrors { get; private set; }
        public uint ReplyCrcErrors { get; private set; }

        // The line code is 21 bit periods; the 20 GCR bits are the
        // transitions between adjacent periods, then each 5 bit group
        // maps back to a nibble. Same scheme as decode_gcr() in
        // Mcu/SITL/Src/sitl_input.c.
        private void RecordReplyBit(bool level)
        {
            if(replyBits < reply.Length)
            {
                reply[replyBits++] = level;
            }
        }

        // Called when the channel goes back to input capture, i.e. the
        // whole reply has been driven.
        //
        // The transmission opens with buffer_padding idle periods, which
        // are gcr[] entries of 0 driving the line high, so the line code
        // starts at the falling edge out of idle. Finding it that way
        // avoids having to know buffer_padding here, and skips the first
        // recorded period, which is whatever CCR1 held before the DMA
        // supplied the first entry.
        private void DecodeReply()
        {
            var raw = 0ul;
            for(var i = 0; i < replyBits && i < 64; i++)
            {
                if(reply[i])
                {
                    raw |= 1ul << i;
                }
            }
            ReplyRaw = raw;
            var start = -1;
            for(var i = 1; i < replyBits; i++)
            {
                if(!reply[i] && reply[i - 1])
                {
                    start = i;
                    break;
                }
            }
            if(start < 0 || start + ReplyPeriods > replyBits)
            {
                return;
            }
            var gcrnum = 0u;
            for(var j = 1; j < ReplyPeriods; j++)
            {
                gcrnum = (gcrnum << 1)
                    | (uint)((reply[start + j] != reply[start + j - 1]) ? 1 : 0);
            }
            var frame = 0u;
            for(var q = 3; q >= 0; q--)
            {
                var code = (gcrnum >> (q * 5)) & 0x1F;
                var nibble = -1;
                for(var i = 0; i < GcrTable.Length; i++)
                {
                    if(GcrTable[i] == code)
                    {
                        nibble = i;
                        break;
                    }
                }
                if(nibble < 0)
                {
                    // not a legal GCR code; leave the previous frame and
                    // let the count show nothing new arrived
                    ReplyGcrErrors++;
                    return;
                }
                frame = (frame << 4) | (uint)nibble;
            }
            // Src/dshot.c: the low nibble is the inverted xor of the
            // three payload nibbles
            var csum = ~((frame >> 4) ^ (frame >> 8) ^ (frame >> 12)) & 0xF;
            if(csum != (frame & 0xF))
            {
                ReplyCrcErrors++;
            }
            LastReplyFrame = frame;
            ReplyTypeMask |= 1u << (int)((frame >> 12) & 0xF);
            replyRing[ReplyCount % (uint)replyRing.Length] = frame;
            ReplyCount++;
        }

        // restart the reply capture whenever the channel is reconfigured
        private void ResetReply()
        {
            replyBits = 0;
        }

        // Src/dshot.c gcr_encode_table
        private static readonly uint[] GcrTable = {
            0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
            0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
        };

        private const int ReplyPeriods = 21;

        // 64 frames is 16ms of simulated time at 4kHz, far longer than a
        // client's polling interval; a slower one loses the oldest, which
        // TryGetReply reports rather than hides
        private readonly uint[] replyRing = new uint[64];

        // CC1S = 00 means channel 1 is an output; receiveDshotDma() sets
        // it to 01 for input capture
        private bool OutputMode => ((regs[ccmrOffset / 4] >> ccsShift) & CC1S) == 0;

        private bool OutputEnabled => ((regs[CCER / 4] >> ccerShift) & CC1E) != 0;

        // CC1E gates the capture as well as the output: with the channel
        // disabled the pin is not connected to CCR1 at all
        private bool CaptureEnabled => ((regs[CCER / 4] >> ccerShift) & CC1E) != 0;

        private bool CapturesBothEdges
        {
            get
            {
                var ccer = regs[CCER / 4] >> ccerShift;
                if((ccer & CC1NP) != 0)
                {
                    return true;
                }
                return channel == 1
                    && ((regs[ccmrOffset / 4] >> ccsShift) & CC1S) == CC1S
                    && (regs[SMCR / 4] & TsMask) == TsTi1Ed;
            }
        }

        // the pin only reaches the timer in alternate mode with the AF
        // that selects this timer's channel 1. inputBase 0 means the
        // platform did not say, so do not gate on it.
        private bool PinRouted
        {
            get
            {
                if(inputBase == 0)
                {
                    return true;
                }
                var moder = machine.SystemBus.ReadDoubleWord(inputBase);
                if(((moder >> (2 * inputPin)) & 3) != ModeAlternate)
                {
                    return false;
                }
                var afr = machine.SystemBus.ReadDoubleWord(
                    inputBase + (ulong)(inputPin < 8 ? AfrlOffset : AfrhOffset));
                return ((afr >> (4 * (inputPin & 7))) & 0xF) == inputAf;
            }
        }

        private const uint ModeAlternate = 2;
        private const long AfrlOffset = 0x20;
        private const long AfrhOffset = 0x24;

        private void UpdateIrq()
        {
            // the update flag plus this channel's capture flag, each
            // gated by its own DIER enable
            var mask = 1u | (1u << channel);
            var pending = (regs[SR / 4] & regs[DIER / 4] & mask) != 0;
            Connections[IrqLine].Set(pending);
        }

        private bool Counting => (regs[CR1 / 4] & CEN) != 0;
        private uint CurrentCount => (uint)(counter.Value & MaxCount);

        private const long CR1 = 0x00;
        private const long SMCR = 0x08;
        private const long DIER = 0x0C;
        private const long SR = 0x10;
        private const long EGR = 0x14;
        private const long CCMR1 = 0x18;
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
        private const uint CC1E = 1u << 0;
        private const uint CC1P = 1u << 1;
        private const uint CC1NP = 1u << 3;
        private const uint CC1S = 3u << 0;
        private const uint TsMask = 7u << 4;
        private const uint TsTi1Ed = 4u << 4;
        private const uint MaxCount = 0xFFFF;
        private const ulong NanosecondsPerSecond = 1000000000;

        private const int DmaRequestLine = 0;
        private const int IrqLine = 1;
        // drives the shared throttle wire when the ESC is replying
        private const int OutputLine = 2;

        private readonly ulong frequency;
        private readonly int channel;
        private readonly long ccmrOffset;
        private readonly int ccsShift;
        private readonly int ccerShift;
        private readonly long ccrOffset;
        private readonly IMachine machine;
        private readonly ulong inputBase;
        private readonly int inputPin;
        private readonly uint inputAf;
        private readonly LimitTimer counter;
        private readonly uint[] regs = new uint[0x100];
        private bool lastPinState;
        private bool havePin;
        // the whole transmission: buffer_padding idle periods plus the
        // 21 period line code, with room to spare
        private readonly bool[] reply = new bool[64];
        private int replyBits;
        private bool batchedDshotFrame;
        private uint batchedFrameValue;
        private ulong batchedBitPeriodNanoseconds;
        private uint batchedStartCount;
        private ulong batchedCounterDivider;
        private ulong batchedCounterPeriod;
        private bool batchDshotReplies;
        private ulong batchedReplyPeriods = 37;
    }
}
