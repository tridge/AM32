//
// CH32V203 ADC, driven from the motor physics like the STM32 families'
// AM32_STM32F0_ADC but with the F1-generation register file the WCH
// part keeps: RSQR1..3 rank sequencer, RDATAR at 0x4C, and the CAL /
// RSTCAL self-clearing calibration bits ADCInit() spins on.
//
// A software start (CTLR2 SWSTART with ADON set) converts the whole
// regular sequence in rank order and raises one DMA request per
// conversion - AM32 reads conversions only through DMA1 channel 1 into
// ADCDataDMA[] (Mcu/v203/Src/ADC.c). AIRBOT-class targets sequence
// CH1 (PA1, voltage), CH6 (PA6, current), CH16 (temperature).
//
// Temperature has no TS_CAL page on this part: TempSensor_Volt_To_
// Temper() reads a factory point from 0x1FFFF720 (low half reference
// millivolts, high half reference degrees) and applies a fixed
// -4.3 mV/C slope, so this model inverts exactly that against the same
// word, which the family .resc seeds. getConvertedDegrees() divides by
// 4096, not 4095, hence the scaling below. The slope is a parameter in
// tenths of a millivolt per degree because the Artery sensor runs the
// other way: the F415's fixed formula is 1344mV at 25C RISING 4.2mV/C,
// where the WCH and GD parts fall 4.3mV/C.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
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
    public class AM32_WCH_Adc : IDoubleWordPeripheral, IKnownSize,
                                INumberedGPIOOutput
    {
        public AM32_WCH_Adc(IMachine machine, int voltageChannel,
                            int currentChannel, int voltageDivider,
                            int millivoltPerAmp, int currentOffsetMv,
                            int temperatureChannel = 16,
                            ulong tempCalWord = 0x1FFFF720,
                            int tempSlopeTenthsMvPerC = -43,
                            int ntcChannel = -1, uint ntcCounts = 3104)
        {
            this.tempSlopeTenthsMvPerC = tempSlopeTenthsMvPerC;
            this.machine = machine;
            this.voltageChannel = voltageChannel;
            this.currentChannel = currentChannel;
            this.voltageDivider = voltageDivider;
            this.millivoltPerAmp = millivoltPerAmp;
            this.currentOffsetMv = currentOffsetMv;
            this.temperatureChannel = temperatureChannel;
            this.tempCalWord = tempCalWord;
            this.ntcChannel = ntcChannel;
            this.ntcCounts = ntcCounts;
            if(voltageDivider <= 0)
            {
                throw new RecoverableException("voltageDivider must be positive");
            }
            var conns = new Dictionary<int, IGPIO>();
            conns[DmaRequestLine] = new GPIO();
            Connections = conns;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }
        public ulong Conversions { get; private set; }

        public void Reset()
        {
            statr = 0;
            ctlr1 = 0;
            ctlr2 = 0;
            samptr1 = 0;
            samptr2 = 0;
            rdatar = 0;
            for(var i = 0; i < rsqr.Length; i++)
            {
                rsqr[i] = 0;
            }
            Conversions = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Statr: return statr;
            case Ctlr1: return ctlr1;
            // CAL and RSTCAL never read back set: the calibration
            // completes instantly and ADCInit() spins on both
            case Ctlr2: return ctlr2;
            case Samptr1: return samptr1;
            case Samptr2: return samptr2;
            case Rsqr1: return rsqr[0];
            case Rsqr2: return rsqr[1];
            case Rsqr3: return rsqr[2];
            case Rdatar:
                statr &= ~Eoc;
                return rdatar;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            // status flags are write-zero-to-clear (the SPL writes the
            // complement mask), so writing back anything else must not
            // set them
            case Statr: statr &= value; return;
            case Ctlr1: ctlr1 = value; return;
            case Ctlr2:
                // CAL and RSTCAL complete instantly, and SWSTART
                // self-clears when conversion begins - retaining it
                // would let a later unrelated read-modify-write start
                // another sequence
                ctlr2 = value & ~(Cal | Rstcal | Swstart);
                if((value & Swstart) != 0 && (value & Adon) != 0)
                {
                    Convert();
                }
                return;
            case Samptr1: samptr1 = value; return;
            case Samptr2: samptr2 = value; return;
            case Rsqr1: rsqr[0] = value; return;
            case Rsqr2: rsqr[1] = value; return;
            case Rsqr3: rsqr[2] = value; return;
            }
        }

        private void Convert()
        {
            // RSQR1[23:20] is ranks-1; rank r reads a 5-bit channel from
            // RSQR3 (ranks 1-6), RSQR2 (7-12) or RSQR1 (13-16)
            var ranks = (int)((rsqr[0] >> 20) & 0xF) + 1;
            for(var rank = 1; rank <= ranks && rank <= 16; rank++)
            {
                int reg, shift;
                if(rank <= 6)
                {
                    reg = 2;
                    shift = 5 * (rank - 1);
                }
                else if(rank <= 12)
                {
                    reg = 1;
                    shift = 5 * (rank - 7);
                }
                else
                {
                    reg = 0;
                    shift = 5 * (rank - 13);
                }
                var ch = (int)((rsqr[reg] >> shift) & 0x1F);
                rdatar = Sample(ch);
                statr |= Eoc;
                Conversions++;
                if((ctlr2 & Dma) != 0)
                {
                    Connections[DmaRequestLine].Blink();
                }
            }
        }

        private uint Sample(int channel)
        {
            double volts = 0, amps = 0, degrees = 0;
            var b = Bridge;
            if(b != null)
            {
                volts = b.BusVoltage;
                amps = b.BusCurrent;
                degrees = b.TemperatureC;
            }

            double pinMv;
            if(channel == voltageChannel)
            {
                // battery_voltage(10mV) = raw * 3300 / 4095 * divider / 100
                pinMv = volts * 1000.0 * 10.0 / voltageDivider;
            }
            else if(channel == currentChannel)
            {
                pinMv = amps * millivoltPerAmp + currentOffsetMv;
            }
            else if(channel == temperatureChannel)
            {
                return TemperatureCounts(degrees);
            }
            else if(channel == ntcChannel)
            {
                // An external NTC divider, decoded by the firmware
                // through its per-board NTC_table (Inc/ntc_tables.h),
                // which this model does not carry - so a fixed count is
                // returned rather than the physics temperature. Left at
                // 0 the table's first entry reads as 400C and the
                // thermal clamp cuts the duty to nothing before the
                // motor can start. The default 3104 decodes to the 38C
                // physics ambient on the table most F421 boards share.
                return ntcCounts;
            }
            else
            {
                return 0;
            }
            if(pinMv < 0)
            {
                pinMv = 0;
            }
            return Clamp(pinMv * 4095.0 / 3300.0);
        }

        private uint TemperatureCounts(double degrees)
        {
            // the linear sensor around the seeded reference point:
            // TempSensor_Volt_To_Temper's T = refT - (mv - refMv)*10/43
            // inverted at the default -4.3mV/C, the F415's rising
            // formula at +4.2, each fed raw*3300/4096 by the firmware
            var word = machine.SystemBus.ReadDoubleWord(tempCalWord);
            var refMv = (int)(word & 0xFFFF);
            var refT = (int)((word >> 16) & 0xFFFF);
            var mv = refMv + (degrees - refT) * tempSlopeTenthsMvPerC / 10.0;
            return Clamp(mv * 4096.0 / 3300.0);
        }

        private static uint Clamp(double counts)
        {
            return (uint)Math.Max(0.0, Math.Min(4095.0, Math.Round(counts)));
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

        private const long Statr = 0x00;
        private const long Ctlr1 = 0x04;
        private const long Ctlr2 = 0x08;
        private const long Samptr1 = 0x0C;
        private const long Samptr2 = 0x10;
        private const long Rsqr1 = 0x2C;
        private const long Rsqr2 = 0x30;
        private const long Rsqr3 = 0x34;
        private const long Rdatar = 0x4C;

        private const uint Eoc = 1u << 1;
        private const uint Adon = 1u << 0;
        private const uint Cal = 1u << 2;
        private const uint Rstcal = 1u << 3;
        private const uint Dma = 1u << 8;
        private const uint Swstart = 1u << 22;

        private const int DmaRequestLine = 0;

        private readonly IMachine machine;
        private readonly int voltageChannel;
        private readonly int currentChannel;
        private readonly int voltageDivider;
        private readonly int millivoltPerAmp;
        private readonly int currentOffsetMv;
        private readonly int temperatureChannel;
        private readonly ulong tempCalWord;
        private readonly int tempSlopeTenthsMvPerC;
        private readonly int ntcChannel;
        private readonly uint ntcCounts;

        private AM32_F051_Bridge bridge;
        private uint statr, ctlr1, ctlr2, samptr1, samptr2, rdatar;
        private readonly uint[] rsqr = new uint[3];
    }
}
