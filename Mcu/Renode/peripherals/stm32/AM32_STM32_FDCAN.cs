//
// STM32G4 FDCAN, modelled to the depth Src/DroneCAN/sys_can_stm32_CANFD.c
// uses it - classic 8-byte frames only, no FD, no bit timing. Renode has
// no G4 FDCAN model and its bxCAN STMCAN is a different peripheral.
//
// The G4 fixes the message RAM layout (RM0440): per instance, standard
// filters at +0x000, extended at +0x070, RX FIFO0 at +0x0B0, FIFO1 at
// +0x188, the TX FIFO at +0x278, all elements 18 words (72 bytes). The
// RAM itself is ordinary memory at 0x4000A400, declared as a
// MappedMemory in the platform and reached through the system bus here,
// the same way the bridge reads GPIO registers.
//
// What is honoured: the CCCR INIT/CCE/CSR handshake, the RX FIFO0 fill
// and acknowledge counters, the TX FIFO put index and add-request, and
// the IR/IE/ILS/ILE interrupt scheme with the G4's grouped line select.
// RXGFC is stored but not interpreted: AM32 leaves it at reset, where
// non-matching frames go to FIFO0, so everything is delivered there.
// Bit timing (NBTP/DBTP) is stored write-readback; frames cross the hub
// instantly, as Renode CAN frames do.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.CAN
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_STM32_FDCAN : IDoubleWordPeripheral, IKnownSize, ICAN,
                                    INumberedGPIOOutput
    {
        public AM32_STM32_FDCAN(IMachine machine, ulong messageRamBase = 0x4000A400)
        {
            this.machine = machine;
            this.ramBase = messageRamBase;
            var conns = new Dictionary<int, IGPIO>();
            conns[Line0] = new GPIO();
            conns[Line1] = new GPIO();
            Connections = conns;
            Reset();
        }

        public event Action<CANMessageFrame> FrameSent;

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            // CSR/CSA clear, INIT set out of reset as on hardware; the
            // driver's first act is clearing CSR and waiting CSA==0
            lock(sync)
            {
                pending.Clear();
            }
            cccr = CccrInit;
            nbtp = 0;
            ir = 0;
            ie = 0;
            ils = 0;
            ile = 0;
            rxgfc = 0;
            txbc = 0;
            txbtie = 0;
            f0Fill = 0;
            f0Get = 0;
            f0Put = 0;
            txPut = 0;
            UpdateLines();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case CCCR:
                // CSA mirrors CSR: clock-stop is granted the moment it
                // is asked for, and withdrawn the same way
                return (cccr & ~CccrCsa)
                    | ((cccr & CccrCsr) != 0 ? CccrCsa : 0);
            case NBTP: return nbtp;
            case ECR: return 0;
            case IR: return ir;
            case IE: return ie;
            case ILS: return ils;
            case ILE: return ile;
            case RXGFC: return rxgfc;
            case RXF0S:
                return f0Fill | ((uint)f0Get << 8) | ((uint)f0Put << 16)
                    | (f0Fill >= FifoDepth ? RxfsFull : 0);
            case RXF1S: return 0;
            case TXBC: return txbc;
            case TXFQS:
                // never full: transmission is instantaneous, so the free
                // level stays at the FIFO depth and TFQF stays clear
                return FifoDepth | ((uint)txPut << 16);
            case TXBTIE: return txbtie;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case CCCR:
                // INIT and CCE read back as written; entering init
                // empties nothing, as AM32 zeroes the RAM itself
                cccr = value;
                return;
            case NBTP: nbtp = value; return;
            case IR:
                lock(sync)
                {
                    ir &= ~value; // write 1 to clear
                    UpdateLines();
                }
                return;
            case IE: lock(sync) { ie = value; UpdateLines(); } return;
            case ILS: lock(sync) { ils = value; UpdateLines(); } return;
            case ILE: lock(sync) { ile = value; UpdateLines(); } return;
            case RXGFC: rxgfc = value; return;
            case RXF0A:
                lock(sync)
                {
                    if(f0Fill > 0)
                    {
                        f0Get = (int)((value & 3) + 1) % FifoDepth;
                        f0Fill--;
                    }
                    // the freed FIFO admits the next frame off the wire,
                    // raising a fresh RF0N for it
                    if(pending.Count > 0 && f0Fill == 0)
                    {
                        Deliver(pending.Dequeue());
                    }
                }
                return;
            case RXF1A: return;
            case TXBC: txbc = value; return;
            case TXBTIE: txbtie = value; return;
            case TXBAR:
                for(var i = 0; i < FifoDepth; i++)
                {
                    if((value & (1u << i)) != 0)
                    {
                        Transmit(i);
                    }
                }
                return;
            }
        }

        // A frame from the hub: into RX FIFO0's message RAM only when the
        // FIFO is EMPTY, else held. Host-side senders burst in wall time
        // while the emulation runs slower; on a real bus the 1Mbit/s wire
        // spaces frames at least 128us apart and the ISR always wins the
        // race, so every frame gets its own RF0N. Stacking a burst into
        // the FIFO would strand all but the first: the driver clears the
        // write-one-to-clear RF0N and reads one element per interrupt,
        // and a frame already in the FIFO raises no new edge.
        public void OnFrameReceived(CANMessageFrame message)
        {
            if((cccr & CccrInit) != 0 || message.Data.Length > 8)
            {
                return;
            }
            lock(sync)
            {
                if(f0Fill > 0 || pending.Count > 0)
                {
                    if(pending.Count < PendingLimit)
                    {
                        pending.Enqueue(message);
                    }
                    else
                    {
                        ir |= IrRf0l;
                        UpdateLines();
                    }
                    return;
                }
                Deliver(message);
            }
        }

        private void Deliver(CANMessageFrame message)
        {
            var addr = ramBase + RxFifo0Offset + (ulong)(f0Put * ElementBytes);
            var r0 = message.ExtendedFormat
                ? ((message.Id & 0x1FFFFFFFu) | ElemXtd)
                : ((message.Id & 0x7FFu) << 18);
            if(message.RemoteFrame)
            {
                r0 |= ElemRtr;
            }
            var bus = machine.SystemBus;
            bus.WriteDoubleWord(addr, r0);
            bus.WriteDoubleWord(addr + 4, (uint)message.Data.Length << 16);
            uint w0 = 0, w1 = 0;
            for(var i = 0; i < message.Data.Length; i++)
            {
                if(i < 4)
                {
                    w0 |= (uint)message.Data[i] << (8 * i);
                }
                else
                {
                    w1 |= (uint)message.Data[i] << (8 * (i - 4));
                }
            }
            bus.WriteDoubleWord(addr + 8, w0);
            bus.WriteDoubleWord(addr + 12, w1);
            f0Put = (f0Put + 1) % FifoDepth;
            f0Fill++;
            ir |= IrRf0n;
            if(f0Fill >= FifoDepth)
            {
                ir |= IrRf0f;
            }
            UpdateLines();
        }

        private void Transmit(int index)
        {
            var addr = ramBase + TxFifoOffset + (ulong)(index * ElementBytes);
            var bus = machine.SystemBus;
            var t0 = bus.ReadDoubleWord(addr);
            var t1 = bus.ReadDoubleWord(addr + 4);
            var dlc = (int)((t1 >> 16) & 0xF);
            if(dlc > 8)
            {
                dlc = 8; // classic frames only
            }
            var data = new byte[dlc];
            var w0 = bus.ReadDoubleWord(addr + 8);
            var w1 = bus.ReadDoubleWord(addr + 12);
            for(var i = 0; i < dlc; i++)
            {
                data[i] = (byte)((i < 4 ? w0 >> (8 * i) : w1 >> (8 * (i - 4))) & 0xFF);
            }
            var extended = (t0 & ElemXtd) != 0;
            var frame = new CANMessageFrame(
                extended ? (t0 & 0x1FFFFFFFu) : ((t0 >> 18) & 0x7FFu), data,
                extendedFormat: extended,
                remoteFrame: (t0 & ElemRtr) != 0);
            txPut = (txPut + 1) % FifoDepth;
            var handler = FrameSent;
            if(handler != null)
            {
                handler(frame);
            }
            lock(sync)
            {
                ir |= IrTc;
                UpdateLines();
            }
        }

        // The G4 routes each IR bit's group to line 0 or 1 through ILS;
        // AM32 puts SMSG (TC) and PERR (BO) on line 1 and leaves the RX
        // FIFO groups on line 0.
        private void UpdateLines()
        {
            uint line0 = 0, line1 = 0;
            var pending = ir & ie;
            for(var bit = 0; bit < IrGroup.Length; bit++)
            {
                if((pending & (1u << bit)) == 0)
                {
                    continue;
                }
                if(((ils >> IrGroup[bit]) & 1) != 0)
                {
                    line1 |= 1;
                }
                else
                {
                    line0 |= 1;
                }
            }
            Connections[Line0].Set(line0 != 0 && (ile & 1) != 0);
            Connections[Line1].Set(line1 != 0 && (ile & 2) != 0);
        }

        // interrupt group of each FDCAN_IR bit, per RM0440: RXFIFO0,
        // RXFIFO1, SMSG, TFERR, MISC, BERR, PERR
        private static readonly int[] IrGroup = {
            0, 0, 0,        // RF0N RF0F RF0L
            1, 1, 1,        // RF1N RF1F RF1L
            2, 2, 2,        // HPM TC TCF
            3, 3, 3, 3,     // TFE TEFN TEFF TEFL
            4, 4, 4, 4,     // TSW MRAF TOO ELO
            5, 5,           // EP EW
            6, 6, 6, 6, 6,  // BO WDI PEA PED ARA
        };

        private const long CCCR = 0x018;
        private const long NBTP = 0x01C;
        private const long ECR = 0x040;
        private const long IR = 0x050;
        private const long IE = 0x054;
        private const long ILS = 0x058;
        private const long ILE = 0x05C;
        private const long RXGFC = 0x080;
        private const long RXF0S = 0x090;
        private const long RXF0A = 0x094;
        private const long RXF1S = 0x098;
        private const long RXF1A = 0x09C;
        private const long TXBC = 0x0C0;
        private const long TXFQS = 0x0C4;
        private const long TXBAR = 0x0CC;
        private const long TXBTIE = 0x0DC;

        private const uint CccrInit = 1u << 0;
        private const uint CccrCsa = 1u << 3;
        private const uint CccrCsr = 1u << 4;

        private const uint IrRf0n = 1u << 0;
        private const uint IrRf0f = 1u << 1;
        private const uint IrRf0l = 1u << 2;
        private const uint IrTc = 1u << 7;

        private const uint RxfsFull = 1u << 24;

        private const uint ElemRtr = 1u << 29;
        private const uint ElemXtd = 1u << 30;

        private const int FifoDepth = 3;
        private const int ElementBytes = 72;
        // ~1.3s of a saturated 1Mbit/s wire; past this the bus is
        // genuinely oversubscribed and dropping is honest
        private const int PendingLimit = 512;
        private const ulong RxFifo0Offset = 0x0B0;
        private const ulong TxFifoOffset = 0x278;

        private const int Line0 = 0;
        private const int Line1 = 1;

        private readonly IMachine machine;
        private readonly ulong ramBase;
        // one lock for the IR/FIFO/queue state: the socket thread
        // delivers while the emulation thread acknowledges and clears
        private readonly object sync = new object();
        private readonly Queue<CANMessageFrame> pending = new Queue<CANMessageFrame>();
        private uint cccr, nbtp, ir, ie, ils, ile, rxgfc, txbc, txbtie;
        private uint f0Fill;
        private int f0Get, f0Put, txPut;
    }
}
