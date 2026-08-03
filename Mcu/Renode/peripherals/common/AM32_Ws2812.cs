//
// WS2812 LED strip decoder. AM32 bit-bangs the strip from plain GPIO
// writes timed by busy-polling the utility timer (Mcu/g431/Src/WS2812.c):
// a one is a long high then short low, a zero the reverse, 24 bits per
// LED in GRB order, and a long idle latches the frame.
//
// This listens to the data pin and reverses that: it times each high
// pulse against the virtual clock, classifies it by ThresholdNs, and
// commits a colour every 24 bits. A gap longer than LatchNs resets to
// LED zero, so multi-LED strips decode per position.
//
// The registers are a debug window; the GUI gets the colour through
// the guilink's device-info reply.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AM32_Ws2812 : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver
    {
        public AM32_Ws2812(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public long Size => 0x100;

        // the colour survives a firmware reset on purpose: real LEDs
        // hold their last frame until told otherwise
        public void Reset()
        {
        }

        // 0x00 LED0 as 0xRRGGBB, 0x04 frames decoded, 0x08 LEDs seen
        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00: return leds[0];
            case 0x04: return frames;
            case 0x08: return count;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        // high time above this is a one; the G4 driver's nominal times
        // are 250ns and 500ns
        public uint ThresholdNs { get; set; } = 375;

        // idle longer than this latches the frame and resets to LED 0
        public uint LatchNs { get; set; } = 50000;

        // LED colours as 0xRRGGBB, by strip position
        public uint Led(int index)
        {
            return index >= 0 && index < leds.Length ? leds[index] : 0;
        }

        public uint Color => leds[0];
        public uint Frames => frames;
        public uint Count => count;

        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                return;
            }
            var now = machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds * 1000.0;
            if(value)
            {
                if(now - lastEdge > LatchNs)
                {
                    // frame gap: whatever was accumulated is done
                    bits = 0;
                    index = 0;
                }
                else if(bits > 0)
                {
                    lows[bits - 1] = now - lastEdge;
                }
                rose = now;
            }
            else if(rose > 0)
            {
                if(bits < highs.Length)
                {
                    highs[bits] = now - rose;
                    lows[bits] = 0;
                }
                if(++bits == 24)
                {
                    Commit();
                }
            }
            lastEdge = now;
        }

        private void Commit()
        {
            // Classify each bit by its high time - except the first,
            // whose rising timestamp arrives early (virtual time is
            // attributed at translation-block granularity, and the code
            // before the first edge shares its block), inflating the
            // measured pulse. Its LOW period is bounded by mid-stream
            // edges on both sides, so read it inverted from that.
            uint grb = 0;
            for(var i = 0; i < 24; i++)
            {
                bool one;
                if(i == 0 && lows[0] > 0)
                {
                    one = lows[0] < ThresholdNs;
                }
                else
                {
                    one = highs[i] > ThresholdNs;
                }
                grb = (grb << 1) | (one ? 1u : 0u);
            }
            // wire order is green, red, blue
            var g = (grb >> 16) & 0xFF;
            var r = (grb >> 8) & 0xFF;
            var b = grb & 0xFF;
            if(index < leds.Length)
            {
                leds[index] = (r << 16) | (g << 8) | b;
                if((uint)(index + 1) > count)
                {
                    count = (uint)(index + 1);
                }
            }
            index++;
            frames++;
            bits = 0;
        }

        private readonly IMachine machine;
        private readonly uint[] leds = new uint[8];
        private readonly double[] highs = new double[24];
        private readonly double[] lows = new double[24];
        private double rose;
        private double lastEdge;
        private int bits;
        private int index;
        private uint frames;
        private uint count;
    }
}
