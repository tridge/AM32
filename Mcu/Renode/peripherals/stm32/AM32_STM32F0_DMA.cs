//
// DMA1 for STM32F0, driven by peripheral requests.
//
// Renode's stock F0 platform ships a Python stub here that throws as
// soon as the firmware programs it. Its STM32G0DMA is closer, but
// mishandles the case AM32 actually uses: the dshot/servo capture path
// sets PSIZE=16 with MSIZE=32 (CCR=0x98b) to widen each 16-bit CCR1
// capture into a uint32_t dma_buffer[] slot, and repeating the 16-bit
// read rather than zero-extending it fills the buffer with values like
// 0x12341234, so detectInput() never locks.
//
// A request arrives as a GPIO pulse on the channel's input line, wired
// from the capture timer. Only peripheral-to-memory is request driven;
// memory-to-peripheral is what dshot telemetry output uses and is
// handled the same way, just with the direction reversed.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.DMA
{
    public class AM32_STM32F0_DMA : IDoubleWordPeripheral, IKnownSize,
                                    INumberedGPIOOutput, IGPIOReceiver
    {
        public AM32_STM32F0_DMA(IMachine machine, int numberOfChannels = 7)
        {
            this.machine = machine;
            this.channelCount = numberOfChannels;
            channels = new Channel[numberOfChannels];
            var conns = new Dictionary<int, IGPIO>();
            for(var i = 0; i < numberOfChannels; i++)
            {
                channels[i] = new Channel();
                conns[i] = new GPIO();
            }
            Connections = conns;
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            isr = 0;
            for(var i = 0; i < channelCount; i++)
            {
                channels[i] = new Channel();
                Connections[i].Unset();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == ISR)
            {
                return isr;
            }
            if(offset == IFCR)
            {
                return 0;
            }
            var ch = ChannelFromOffset(offset);
            if(ch < 0)
            {
                return 0;
            }
            var c = channels[ch];
            switch((offset - ChannelBase) % ChannelStride)
            {
            case 0x00: return c.Ccr;
            case 0x04: return c.Remaining; // counts down, as on hardware
            case 0x08: return c.Cpar;
            case 0x0C: return c.Cmar;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == IFCR)
            {
                // Write 1 to clear, PER BIT: only the global CGIF clears
                // the other three. Clearing the whole nibble on any bit
                // was a live bug on the CH32V203, the one family whose
                // capture handler enables the half-transfer interrupt:
                // when HT and TC accumulated across a masked window (the
                // arming tune), the HT-branch's clear wiped the pending
                // TC too, the transfer-complete path never re-armed the
                // DMA and dshot input went permanently deaf on arming.
                for(var i = 0; i < channelCount; i++)
                {
                    var nib = (value >> (4 * i)) & 0xF;
                    if(nib == 0)
                    {
                        continue;
                    }
                    if((nib & GIF) != 0)
                    {
                        nib = 0xF;
                    }
                    isr &= ~(nib << (4 * i));
                    // the line stays asserted while an event flag is
                    // still set, so a flag cleared separately is still
                    // serviced by a re-dispatch; once the last event
                    // flag goes, GIF goes with it (RM0008: an
                    // individual clear also clears GIF when no other
                    // flag remains)
                    if((isr & ((TCIF | HTIF | TEIF) << (4 * i))) == 0)
                    {
                        isr &= ~(GIF << (4 * i));
                        Connections[i].Unset();
                    }
                }
                return;
            }
            var ch = ChannelFromOffset(offset);
            if(ch < 0)
            {
                return;
            }
            var c = channels[ch];
            switch((offset - ChannelBase) % ChannelStride)
            {
            case 0x00:
                c.Ccr = value;
                if((value & EN) != 0)
                {
                    // reloaded on enable, so a re-armed transfer starts fresh
                    c.Remaining = c.Cndtr;
                    c.Position = 0;
                }
                break;
            case 0x04:
                c.Cndtr = value & 0xFFFF;
                c.Remaining = c.Cndtr;
                c.Position = 0;
                break;
            case 0x08: c.Cpar = value; break;
            case 0x0C: c.Cmar = value; break;
            }
        }

        // a peripheral raised a request on this channel
        public void OnGPIO(int number, bool value)
        {
            if(!value || number < 0 || number >= channelCount)
            {
                return;
            }
            ServiceRequest(number);
        }

        private void ServiceRequest(int ch)
        {
            var c = channels[ch];
            if((c.Ccr & EN) == 0 || c.Remaining == 0)
            {
                return;
            }
            var psize = SizeInBytes((c.Ccr >> 8) & 3);
            var msize = SizeInBytes((c.Ccr >> 10) & 3);
            var toMemory = (c.Ccr & DIR) == 0;
            var pAddr = (ulong)(c.Cpar + (((c.Ccr & PINC) != 0) ? c.Position * (uint)psize : 0));
            var mAddr = (ulong)(c.Cmar + (((c.Ccr & MINC) != 0) ? c.Position * (uint)msize : 0));

            if(toMemory)
            {
                // read the peripheral at its own width, then widen. This
                // is the bit the stock model gets wrong: it repeats the
                // narrow read to fill the wider slot.
                var data = ReadWidth(pAddr, psize);
                WriteWidth(mAddr, data, msize);
            }
            else
            {
                var data = ReadWidth(mAddr, msize);
                WriteWidth(pAddr, data, psize);
            }

            c.Position++;
            c.Remaining--;

            if(c.Remaining == c.Cndtr / 2 && (c.Ccr & HTIE) != 0)
            {
                SetFlag(ch, HTIF);
            }
            if(c.Remaining == 0)
            {
                if((c.Ccr & CIRC) != 0)
                {
                    c.Remaining = c.Cndtr;
                    c.Position = 0;
                }
                if((c.Ccr & TCIE) != 0)
                {
                    SetFlag(ch, TCIF);
                }
            }
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

        private void WriteWidth(ulong addr, uint data, int width)
        {
            switch(width)
            {
            case 1: machine.SystemBus.WriteByte(addr, (byte)data); break;
            case 2: machine.SystemBus.WriteWord(addr, (ushort)data); break;
            default: machine.SystemBus.WriteDoubleWord(addr, data); break;
            }
        }

        private void SetFlag(int ch, uint flag)
        {
            isr |= (flag | GIF) << (4 * ch);
            Connections[ch].Set();
        }

        private static int SizeInBytes(uint enc)
        {
            return enc == 0 ? 1 : (enc == 1 ? 2 : 4);
        }

        private int ChannelFromOffset(long offset)
        {
            if(offset < ChannelBase)
            {
                return -1;
            }
            var ch = (int)((offset - ChannelBase) / ChannelStride);
            return ch < channelCount ? ch : -1;
        }

        private class Channel
        {
            public uint Ccr;
            public uint Cndtr;
            public uint Cpar;
            public uint Cmar;
            public uint Remaining;
            public uint Position;
        }

        private const long ISR = 0x00;
        private const long IFCR = 0x04;
        private const long ChannelBase = 0x08;
        private const long ChannelStride = 0x14;

        private const uint EN = 1u << 0;
        private const uint TCIE = 1u << 1;
        private const uint HTIE = 1u << 2;
        private const uint DIR = 1u << 4;
        private const uint CIRC = 1u << 5;
        private const uint PINC = 1u << 6;
        private const uint MINC = 1u << 7;

        private const uint GIF = 1u << 0;
        private const uint TCIF = 1u << 1;
        private const uint HTIF = 1u << 2;
        private const uint TEIF = 1u << 3;

        private readonly IMachine machine;
        private readonly int channelCount;
        private readonly Channel[] channels;
        private uint isr;
    }
}
