//
// MCXA153 LPADC. The firmware arms a command chain once - CMD1 current,
// CMD2 voltage, CMD3 internal temperature with LOOP=1 so it runs twice -
// and software-triggers it from the 20kHz loop; every result lands in
// the FIFO and, with FCTRL.FWMARK=0 and DE.FWMDE0 set, raises a DMA
// request that eDMA channel 1 answers by reading RESFIFO into
// ADCDataDMA[]. Nothing reads the FIFO by hand.
//
// Calibration always reads as done: STAT.CAL_RDY and GCC[0].RDY are
// hard-wired, and GCC's GAIN_CAL of 0 makes calibADC()'s gain
// adjustment exactly 1.
//
// Sample values come from the motor model through the bridge, inverting
// main.c's NXP conversions (16-bit results against 3.3V). Temperature
// is the two-point bandgap formula in Mcu/a153/Src/ADC.c: V2 is pinned
// at 40000 counts and V1 solved from
//   T = 738 * 10.06*(V2-V1) / (V2 + 10.06*(V2-V1)) - 287.5
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Antmicro.Renode.Peripherals.Analog
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class MCXA_Lpadc : IDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput
    {
        public MCXA_Lpadc(IMachine machine, int voltageChannel, int currentChannel,
                          int temperatureChannel, int voltageDivider,
                          int millivoltPerAmp, int currentOffsetMv)
        {
            this.machine = machine;
            this.voltageChannel = voltageChannel;
            this.currentChannel = currentChannel;
            this.temperatureChannel = temperatureChannel;
            this.voltageDivider = voltageDivider;
            this.millivoltPerAmp = millivoltPerAmp;
            this.currentOffsetMv = currentOffsetMv;
            var conns = new Dictionary<int, IGPIO>();
            conns[0] = new GPIO();
            Connections = conns;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        // total conversions, for sanity checks from the monitor
        public ulong Conversions { get; private set; }

        public void Reset()
        {
            regs.Clear();
            fifo.Clear();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Stat:
                return CalRdy;
            case Gcc0:
                // RDY with GAIN_CAL 0: gain adjustment of exactly 1
                return GccRdy;
            case Resfifo:
                if(fifo.Count == 0)
                {
                    return 0;
                }
                return fifo.Dequeue();
            default:
                uint v;
                regs.TryGetValue(offset, out v);
                return v;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == Swtrig)
            {
                if((value & 1) != 0)
                {
                    RunChain();
                }
                return;
            }
            regs[offset] = value;
        }

        private uint Reg(long offset)
        {
            uint v;
            regs.TryGetValue(offset, out v);
            return v;
        }

        // execute the command chain from TCTRL[0].TCMD; each command
        // yields LOOP+1 results, each raising one FIFO-watermark DMA
        // request (FWMARK is 0)
        private void RunChain()
        {
            if((Reg(Ctrl) & AdcEn) == 0)
            {
                return;
            }
            var cmd = (int)((Reg(Tctrl0) >> 24) & 0xF);
            var guard = 0;
            while(cmd != 0 && guard++ < 16)
            {
                var cmdl = Reg(Cmd1L + (cmd - 1) * 8);
                var cmdh = Reg(Cmd1L + (cmd - 1) * 8 + 4);
                var channel = (int)(cmdl & 0x1F);
                var count = (int)((cmdh >> 16) & 0xF) + 1;
                Emit(channel, count);
                cmd = (int)((cmdh >> 24) & 0x7);
            }
        }

        private void Emit(int channel, int count)
        {
            for(var i = 0; i < count; i++)
            {
                fifo.Enqueue(Sample(channel, i));
                Conversions++;
                if((Reg(De) & Fwmde0) != 0)
                {
                    Connections[0].Blink();
                }
            }
        }

        // raw 16-bit counts for a channel, inverting main.c's NXP branch
        private uint Sample(int channel, int index)
        {
            double volts = 0, amps = 0, degrees = 25;
            var b = Bridge;
            if(b != null)
            {
                volts = b.BusVoltage;
                amps = b.BusCurrent;
                degrees = b.TemperatureC;
            }

            if(channel == temperatureChannel)
            {
                // second result is the reference conversion V2
                if(index != 0)
                {
                    return TempV2;
                }
                // solve computeTemperature() for V1 given V2
                var num = degrees + 287.5;
                var d = num * TempV2 / (738.0 - num);
                return Clamp(TempV2 - d / 10.06);
            }
            double pinMv;
            if(channel == voltageChannel)
            {
                // battery_voltage(10mV) = raw * 3300/65535 * divider / 100
                pinMv = volts * 1000.0 * 10.0 / voltageDivider;
            }
            else if(channel == currentChannel)
            {
                // actual_current(10mA) =
                //   ((raw*3300/65535) - CURRENT_OFFSET) * 100 / MILLIVOLT_PER_AMP
                pinMv = amps * millivoltPerAmp + currentOffsetMv;
            }
            else
            {
                return 0;
            }
            if(pinMv < 0)
            {
                pinMv = 0;
            }
            return Clamp(pinMv * 65535.0 / 3300.0);
        }

        private static uint Clamp(double counts)
        {
            return (uint)Math.Max(0.0, Math.Min(65535.0, Math.Round(counts)));
        }

        private AM32_F051_Bridge Bridge
        {
            get
            {
                if(bridge == null)
                {
                    bridge = machine.GetPeripheralsOfType<AM32_F051_Bridge>()
                                    .FirstOrDefault();
                }
                return bridge;
            }
        }

        private const long Ctrl = 0x10;
        private const long Stat = 0x14;
        private const long De = 0x1C;
        private const long Swtrig = 0x34;
        private const long Tctrl0 = 0xA0;
        private const long Gcc0 = 0xF0;
        private const long Cmd1L = 0x100;
        private const long Resfifo = 0x300;

        private const uint AdcEn = 1u << 0;
        private const uint Fwmde0 = 1u << 0;
        private const uint CalRdy = 1u << 10;
        private const uint GccRdy = 1u << 24;
        private const uint TempV2 = 40000;

        private readonly IMachine machine;
        private readonly int voltageChannel, currentChannel, temperatureChannel;
        private readonly int voltageDivider, millivoltPerAmp, currentOffsetMv;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly Queue<uint> fifo = new Queue<uint>();
        private AM32_F051_Bridge bridge;
    }
}
