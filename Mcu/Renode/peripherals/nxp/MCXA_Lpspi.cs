//
// MCXA153 LPSPI. Two very different jobs on this board:
//
// LPSPI0 drives the bidirectional dshot reply: sendDshotDma() flips the
// input pin to SPI data-out, sets TCR.FRAMESZ to 21 bits and writes the
// whole gray-folded gcrnumber to TDR in one go (the NXP branch of
// make_dshot_package() folds the 20 GCR transition bits into 21 wire
// levels with a prefix XOR, where the timer families feed a DMA a level
// per period). The transfer-complete interrupt then puts the pin back
// to capture. So unlike the STM32 targets, where the capture-timer
// model decodes the reply from the levels it drove, here the reply
// frame arrives whole in the TDR write and this model decodes it: undo
// the fold with w ^ (w >> 1), map the four 5-bit GCR groups back to
// nibbles, check the checksum, and publish the same Reply* properties
// the capture timer would, which is where run_renode_tests.py looks.
//
// LPSPI1 clocks the APA102 LED strip: TDR writes with SR.MBF polled
// zero between frames. Frames not 21 bits long are counted but not
// GCR-decoded, which tells the two roles apart without configuration.
//
// TCF is raised after the time the frame really occupies on the wire
// (FRAMESZ bits at the TCR.PRESCALE-derived baud), and that delay is
// load bearing: while armed, transfercomplete() defers the frame
// decode with compute_dshot_flag=1 and the main loop must run
// processDshot() inside the transmission window - the TCF interrupt's
// own transfercomplete() overwrites the flag with 2. An immediate TCF
// tail-chains straight after the DMA interrupt, the received frames
// are never decoded, signaltimeout climbs and the ESC disarms.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.SPI
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class MCXA_Lpspi : IDoubleWordPeripheral, IKnownSize,
                              Miscellaneous.IAM32ReplySource
    {
        // decodesReplies marks the instance on the dshot wire (LPSPI0);
        // the LED strip's LPSPI1 sets it false so the reply lookup can
        // never bind to it, whatever the enumeration order
        public MCXA_Lpspi(IMachine machine, bool decodesReplies = true)
        {
            DecodesReplies = decodesReplies;
            IRQ = new GPIO();
            // one shot per TDR word, at the functional clock; the limit
            // is the frame's length in functional-clock ticks
            shift = new LimitTimer(machine.ClockSource, FunctionalHz, this,
                                   "shift", limit: uint.MaxValue,
                                   direction: Direction.Ascending,
                                   enabled: false, workMode: WorkMode.OneShot,
                                   eventEnabled: true, autoUpdate: false);
            shift.LimitReached += OnTransferDone;
            Reset();
        }

        public long Size => 0x1000;
        public GPIO IRQ { get; }

        public void Reset()
        {
            regs.Clear();
            sr = 0;
            shift.Enabled = false;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Sr:
                // TDF always set (the FIFO never fills), MBF never
                return sr | Tdf;
            case Rdr: return 0;
            default:
                uint v;
                regs.TryGetValue(offset, out v);
                return v;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Sr:
                sr &= ~(value & W1cMask);
                UpdateIrq();
                return;
            case Tdr:
                Transmit(value);
                return;
            default:
                regs[offset] = value;
                return;
            }
        }

        private void Transmit(uint value)
        {
            uint v;
            regs.TryGetValue(Cr, out v);
            if((v & Men) == 0)
            {
                return;
            }
            regs.TryGetValue(Tcr, out v);
            var framesz = (int)(v & 0xFFF) + 1;
            if(framesz == ReplyBits && DecodesReplies)
            {
                DecodeReply(value & ((1u << ReplyBits) - 1));
            }
            else
            {
                LedFrames++;
                LastLedWord = value;
            }
            // the flags follow after the frame's wire time: FRAMESZ bits
            // at functional-clock / 2^(PRESCALE+1) (the doubled divider,
            // Mcu/a153/Src/peripherals.c) - 28us for a dshot600 reply
            var prescale = (int)((v >> 27) & 0x7);
            shift.Enabled = false;
            shift.Value = 0;
            shift.Limit = (ulong)framesz << (prescale + 1);
            shift.Enabled = true;
        }

        private void OnTransferDone()
        {
            sr |= Tcf | Fcf | Wcf;
            UpdateIrq();
        }

        // ---- the decoded reply, same surface as the capture timers ----

        public bool DecodesReplies { get; }
        public uint LastReplyFrame { get; private set; }
        public uint ReplyCount { get; private set; }
        public ulong ReplyRaw { get; private set; }
        public uint ReplyTypeMask { get; private set; }
        public uint ReplyGcrErrors { get; private set; }
        public uint ReplyCrcErrors { get; private set; }

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

        // frames that went to the LED strip, for the GUI
        public uint LedFrames { get; private set; }
        public uint LastLedWord { get; private set; }

        // w holds 21 wire levels MSB first; adjacent-level differences
        // are the 20 GCR bits (the inverse of the firmware's fold)
        private void DecodeReply(uint w)
        {
            ReplyRaw = w;
            var gcrnum = (w ^ (w >> 1)) & 0xFFFFF;
            if(gcrnum == 0)
            {
                // a constant line - the firmware's first send goes out
                // before make_dshot_package() has ever run, with
                // gcrnumber still zero. No transitions means no reply on
                // the wire; the capture-timer decoders skip it silently
                // (no start edge), so this does too.
                return;
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
                    ReplyGcrErrors++;
                    return;
                }
                frame = (frame << 4) | (uint)nibble;
            }
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

        private void UpdateIrq()
        {
            uint ier;
            regs.TryGetValue(Ier, out ier);
            IRQ.Set((sr & ier & (Tcf | Fcf | Wcf)) != 0);
        }

        private const long Cr = 0x10;
        private const long Sr = 0x14;
        private const long Ier = 0x18;
        private const long Tcr = 0x60;
        private const long Tdr = 0x64;
        private const long Rdr = 0x74;

        private const uint Men = 1u << 0;
        private const uint Tdf = 1u << 0;
        private const uint Wcf = 1u << 8;
        private const uint Fcf = 1u << 9;
        private const uint Tcf = 1u << 10;
        private const uint W1cMask = 0x3F00;

        private const int ReplyBits = 21;

        // FRO_12M, both instances (peripherals.c / apa102.c)
        private const long FunctionalHz = 12000000;

        private static readonly uint[] GcrTable = {
            0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
            0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
        };

        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly uint[] replyRing = new uint[64];
        private readonly LimitTimer shift;
        private uint sr;
    }
}
