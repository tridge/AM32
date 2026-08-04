//
// MCXA153 eDMA. Channel n's register page sits at base + 0x1000*(n+1);
// each page holds the channel controls (CH_CSR/CH_ES/CH_INT/CH_MUX) and
// a TCD. AM32 uses three channels: CH0 carries the CTIMER0 captures
// into dma_buffer (request source 31, CTIMER0 MATCH0), CH1 drains the
// ADC result FIFO (source 51), CH2 feeds LPUART1 TX (source 24).
//
// One request executes one minor loop (NBYTES bytes); CITER counts
// minor loops down to the major-loop completion, which applies
// SLAST/DLAST_SGA, reloads CITER from BITER, raises the channel
// interrupt when INTMAJOR is set and drops ERQ when DREQ is set.
//
// TCD_ATTR.SMOD is load-bearing: the capture channel reads with a
// 3-bit source-address modulo, so the two source words alternate
// between CTIMER0's CR[1] and CR[2] inside an 8-byte window.
//
// Requests arrive as GPIO inputs whose NUMBER is the request source id,
// matching what the peripherals' repl wiring sends; the mux is honoured
// by matching CH_MUX.SRC.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.DMA
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord)]
    public class MCXA_Edma : IDoubleWordPeripheral, IWordPeripheral,
                             IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public MCXA_Edma(IMachine machine, int numberOfChannels = 3)
        {
            this.machine = machine;
            channels = new Channel[numberOfChannels];
            var conns = new Dictionary<int, IGPIO>();
            for(var i = 0; i < numberOfChannels; i++)
            {
                channels[i] = new Channel();
                conns[i] = new GPIO();
            }
            Connections = conns;
            Reset();
        }

        public long Size => 0x1000 * (channels.Length + 1);
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            foreach(var c in channels)
            {
                c.Reset();
            }
            foreach(var kv in Connections)
            {
                kv.Value.Unset();
            }
        }

        // a peripheral raised a request; the GPIO input number is the
        // eDMA request source id
        public void OnGPIO(int number, bool value)
        {
            if(!value)
            {
                return;
            }
            for(var i = 0; i < channels.Length; i++)
            {
                var c = channels[i];
                if((c.Csr & Erq) != 0 && (c.Mux & 0x7F) == (uint)number)
                {
                    MinorLoop(i, c);
                }
            }
        }

        private void MinorLoop(int index, Channel c)
        {
            var ssize = 1 << (int)((c.Attr >> 8) & 0x7);
            var dsize = 1 << (int)(c.Attr & 0x7);
            var smod = (int)((c.Attr >> 11) & 0x1F);
            var bytes = c.Nbytes & 0x3FFFFFFF;
            var soff = (short)c.Soff;
            var doff = (short)c.Doff;

            var moved = 0u;
            while(moved < bytes)
            {
                var data = ReadWidth(c.Saddr, ssize);
                WriteWidth(c.Daddr, data, dsize);
                c.Saddr = Advance(c.Saddr, soff, smod);
                c.Daddr = (uint)(c.Daddr + doff);
                moved += (uint)Math.Max(ssize, dsize);
            }

            var citer = c.Citer & 0x7FFF;
            citer = citer > 0 ? (uint)(citer - 1) : 0;
            c.Citer = (c.Citer & ~0x7FFFu) | citer;
            if(citer == 0)
            {
                // major loop complete
                c.Saddr = (uint)(c.Saddr + (int)c.Slast);
                c.Daddr = (uint)(c.Daddr + (int)c.Dlast);
                c.Citer = c.Biter;
                c.Csr |= Done;
                if((c.TcdCsr & Dreq) != 0)
                {
                    c.Csr &= ~Erq;
                }
                if((c.TcdCsr & IntMajor) != 0)
                {
                    c.Int |= 1;
                    Connections[index].Set(true);
                }
            }
        }

        private static uint Advance(uint addr, short off, int smod)
        {
            if(smod == 0)
            {
                return (uint)(addr + off);
            }
            var mask = (1u << smod) - 1;
            return (addr & ~mask) | ((uint)(addr + off) & mask);
        }

        private uint ReadWidth(ulong addr, int width)
        {
            switch(width)
            {
            case 1: return machine.SystemBus.ReadByte(addr);
            case 2: return machine.SystemBus.ReadWord(addr);
            default: return machine.SystemBus.ReadDoubleWord(addr);
            }
        }

        private void WriteWidth(ulong addr, uint value, int width)
        {
            switch(width)
            {
            case 1: machine.SystemBus.WriteByte(addr, (byte)value); return;
            case 2: machine.SystemBus.WriteWord(addr, (ushort)value); return;
            default: machine.SystemBus.WriteDoubleWord(addr, value); return;
            }
        }

        private bool Decode(long offset, out int channel, out long reg)
        {
            channel = (int)(offset >> 12) - 1;
            reg = offset & 0xFFF;
            return channel >= 0 && channel < channels.Length;
        }

        public uint ReadDoubleWord(long offset)
        {
            int ch;
            long reg;
            if(!Decode(offset, out ch, out reg))
            {
                return 0; // the management page: never used
            }
            var c = channels[ch];
            switch(reg)
            {
            case ChCsr: return c.Csr;
            case ChEs: return c.Es;
            case ChInt: return c.Int;
            case ChMux: return c.Mux;
            case TcdSaddr: return c.Saddr;
            case TcdSoff: return c.Soff | ((uint)c.Attr << 16);
            case TcdNbytes: return c.Nbytes;
            case TcdSlast: return c.Slast;
            case TcdDaddr: return c.Daddr;
            case TcdDoff: return c.Doff | (c.Citer << 16);
            case TcdDlast: return c.Dlast;
            case TcdCsrOff: return c.TcdCsr | (c.Biter << 16);
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            int ch;
            long reg;
            if(!Decode(offset, out ch, out reg))
            {
                return;
            }
            var c = channels[ch];
            switch(reg)
            {
            case ChCsr:
                // DONE is write-1-to-clear; ERQ is a plain bit
                if((value & Done) != 0)
                {
                    c.Csr &= ~Done;
                }
                c.Csr = (c.Csr & Done) | (value & ~Done);
                return;
            case ChEs:
                c.Es &= ~value;
                return;
            case ChInt:
                c.Int &= ~value;
                if(c.Int == 0)
                {
                    Connections[ch].Set(false);
                }
                return;
            case ChMux: c.Mux = value; return;
            case TcdSaddr: c.Saddr = value; return;
            case TcdSoff:
                c.Soff = (ushort)value;
                c.Attr = (ushort)(value >> 16);
                return;
            case TcdNbytes: c.Nbytes = value; return;
            case TcdSlast: c.Slast = value; return;
            case TcdDaddr: c.Daddr = value; return;
            case TcdDoff:
                c.Doff = (ushort)value;
                c.Citer = (ushort)(value >> 16);
                return;
            case TcdDlast: c.Dlast = value; return;
            case TcdCsrOff:
                c.TcdCsr = (ushort)value;
                c.Biter = (ushort)(value >> 16);
                return;
            default: return;
            }
        }

        public ushort ReadWord(long offset)
        {
            int ch;
            long reg;
            if(!Decode(offset, out ch, out reg))
            {
                return 0;
            }
            var c = channels[ch];
            switch(reg)
            {
            case TcdSoff: return (ushort)c.Soff;
            case TcdAttr: return (ushort)c.Attr;
            case TcdDoff: return (ushort)c.Doff;
            case TcdCiter: return (ushort)c.Citer;
            case TcdCsrOff: return (ushort)c.TcdCsr;
            case TcdBiter: return (ushort)c.Biter;
            default: return (ushort)ReadDoubleWord(offset & ~3L);
            }
        }

        public void WriteWord(long offset, ushort value)
        {
            int ch;
            long reg;
            if(!Decode(offset, out ch, out reg))
            {
                return;
            }
            var c = channels[ch];
            switch(reg)
            {
            case TcdSoff: c.Soff = value; return;
            case TcdAttr: c.Attr = value; return;
            case TcdDoff: c.Doff = value; return;
            case TcdCiter: c.Citer = value; return;
            case TcdCsrOff: c.TcdCsr = value; return;
            case TcdBiter: c.Biter = value; return;
            default: return;
            }
        }

        private class Channel
        {
            public void Reset()
            {
                Csr = Es = Int = Mux = 0;
                Saddr = Nbytes = Slast = Daddr = Dlast = 0;
                Soff = Attr = Doff = Citer = TcdCsr = Biter = 0;
            }

            public uint Csr, Es, Int, Mux;
            public uint Saddr, Nbytes, Slast, Daddr, Dlast;
            public uint Soff, Attr, Doff, Citer, TcdCsr, Biter;
        }

        private const long ChCsr = 0x00;
        private const long ChEs = 0x04;
        private const long ChInt = 0x08;
        private const long ChMux = 0x14;
        private const long TcdSaddr = 0x20;
        private const long TcdSoff = 0x24;
        private const long TcdAttr = 0x26;
        private const long TcdNbytes = 0x28;
        private const long TcdSlast = 0x2C;
        private const long TcdDaddr = 0x30;
        private const long TcdDoff = 0x34;
        private const long TcdCiter = 0x36;
        private const long TcdDlast = 0x38;
        private const long TcdCsrOff = 0x3C;
        private const long TcdBiter = 0x3E;

        private const uint Erq = 1u << 0;
        private const uint Done = 1u << 30;
        private const ushort IntMajor = 1 << 1;
        private const ushort Dreq = 1 << 3;

        private readonly IMachine machine;
        private readonly Channel[] channels;
    }
}
