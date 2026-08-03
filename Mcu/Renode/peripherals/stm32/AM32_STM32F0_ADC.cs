//
// STM32F0 ADC, driven from the motor physics.
//
// Renode's stock Analog.STM32F0_ADC gets the calibrate and enable
// handshake right, which is all the boot needed, but it has no DMA
// output at all - no GPIO connections, nothing to request a transfer.
// AM32 reads its conversions exclusively through DMA1 channel 1 into
// ADCDataDMA[] (Mcu/f051/Src/ADC.c), so against the stock model that
// buffer stays zero and the firmware sees no battery voltage or
// current. Worse, the platform also asked the stock model to self
// trigger at 1kHz, which AM32 does not use - it starts conversions in
// software from the 1kHz loop - so sequences piled up and logged
// "Issued a start event before the last sequence finished" forever.
//
// This model converts on ADSTART, walks the channels selected in
// CHSELR in ascending order as SCANDIR=0 requires, and raises a DMA
// request per conversion, so the firmware's real DMA path runs.
//
// Sample values come from Mcu/SITL/sim/motor.c through the bridge, and
// are converted to raw counts by inverting main.c's arithmetic - the
// same inversion Mcu/SITL/Src/ADC.c does for the SITL, kept in step
// with it deliberately.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
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
    public class AM32_STM32F0_ADC : IDoubleWordPeripheral, IKnownSize,
                                    INumberedGPIOOutput
    {
        // channels and scaling are per target, out of Inc/targets.h:
        // VOLTAGE_ADC_CHANNEL, CURRENT_ADC_CHANNEL,
        // TARGET_VOLTAGE_DIVIDER, MILLIVOLT_PER_AMP, CURRENT_OFFSET.
        //
        // The temperature sensor sits on a different channel, with
        // different factory calibration, on each family: IN16 calibrated
        // at 110C on the F051, IN12 at 130C on the G0, IN17 at 130C on
        // the L4.
        //
        // sqrSequencer selects the L4's ADCv3 regular sequence: ranked
        // 5-bit channel fields in SQR1..SQR4 with the length in
        // SQR1[3:0], instead of the F0/G0 CHSELR in either of its two
        // modes. The rest of the register interface AM32 touches - the
        // ADCAL and ADEN/ADRDY handshake, CFGR DMAEN at bit 0, DR at
        // 0x40, common CCR at 0x308 - matches bit for bit, which is why
        // this is a mode rather than a separate model. The L4-only
        // DEEPPWD/ADVREGEN writes fall through WriteControl harmlessly
        // and nothing polls them.
        public AM32_STM32F0_ADC(IMachine machine, int voltageChannel,
                                int currentChannel, int voltageDivider,
                                int millivoltPerAmp, int currentOffsetMv,
                                int temperatureChannel = 16,
                                ulong tsCal1 = 0x1FFFF7B8,
                                ulong tsCal2 = 0x1FFFF7C2,
                                int tsCal2Temp = 110,
                                int tsCalVrefMv = 3300,
                                bool sqrSequencer = false)
        {
            this.machine = machine;
            this.voltageChannel = voltageChannel;
            this.currentChannel = currentChannel;
            this.voltageDivider = voltageDivider;
            this.millivoltPerAmp = millivoltPerAmp;
            this.currentOffsetMv = currentOffsetMv;
            this.temperatureChannel = temperatureChannel;
            this.tsCal1 = tsCal1;
            this.tsCal2 = tsCal2;
            this.tsCal2Temp = tsCal2Temp;
            this.tsCalVrefMv = tsCalVrefMv;
            this.sqrSequencer = sqrSequencer;
            if(tsCalVrefMv <= 0)
            {
                throw new RecoverableException("tsCalVrefMv must be positive");
            }
            if(voltageDivider <= 0)
            {
                throw new RecoverableException("voltageDivider must be positive");
            }

            var conns = new Dictionary<int, IGPIO>();
            conns[DmaRequestLine] = new GPIO();
            conns[IrqLine] = new GPIO();
            Connections = conns;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            isr = 0;
            ier = 0;
            cr = 0;
            cfgr1 = 0;
            cfgr2 = 0;
            chselr = 0;
            ccr = 0;
            dr = 0;
            smpr1 = 0;
            smpr2 = 0;
            for(var i = 0; i < sqr.Length; i++)
            {
                sqr[i] = 0;
            }
            Conversions = 0;
        }

        public ulong Conversions { get; private set; }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case ISR: return isr;
            case IER: return ier;
            // ADCAL and ADSTART self clear here: the calibration and
            // enable loops in Mcu/f051/Src/ADC.c poll them
            case CR: return cr;
            case CFGR1: return cfgr1;
            case CFGR2: return cfgr2;
            case CHSELR: return chselr;
            case CCR: return ccr;
            // stored, not interpreted beyond Convert(): the LL sequencer
            // and sampling-time helpers read-modify-write these, so a
            // zero readback would lose the fields written before
            case SMPR1: return smpr1;
            case SMPR2: return smpr2;
            case SQR1: return sqr[0];
            case SQR2: return sqr[1];
            case SQR3: return sqr[2];
            case SQR4: return sqr[3];
            case DR:
                isr &= ~EOC;
                return dr;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case ISR:
                isr &= ~value; // write 1 to clear
                return;
            case IER: ier = value; return;
            case CR: WriteControl(value); return;
            case CFGR1: cfgr1 = value; return;
            case CFGR2: cfgr2 = value; return;
            case CHSELR: chselr = value; return;
            case CCR: ccr = value; return;
            case SMPR1: smpr1 = value; return;
            case SMPR2: smpr2 = value; return;
            case SQR1: sqr[0] = value; return;
            case SQR2: sqr[1] = value; return;
            case SQR3: sqr[2] = value; return;
            case SQR4: sqr[3] = value; return;
            }
        }

        private void WriteControl(uint value)
        {
            if((value & ADCAL) != 0)
            {
                // completes immediately; the firmware spins on it
                cr &= ~ADCAL;
                return;
            }
            if((value & ADDIS) != 0)
            {
                cr &= ~ADEN;
                isr &= ~ADRDY;
                return;
            }
            if((value & ADEN) != 0)
            {
                cr |= ADEN;
                isr |= ADRDY;
            }
            if((value & ADSTART) != 0 && (cr & ADEN) != 0)
            {
                Convert();
            }
        }

        // One regular sequence, in the order ADC_DMA_Callback() assumes
        // when it indexes ADCDataDMA[]. There are two ways to express
        // that order and the firmware uses a different one per family:
        // the F051 leaves CHSELR a channel bitmap scanned in ascending
        // order (SCANDIR=0), while the G0 sets CFGR1.CHSELRMOD to make
        // it a list of 4-bit channel numbers, one per rank, terminated
        // by 0xF. Reading a configured sequence as a bitmap picks the
        // wrong channels entirely, so this follows the mode bit.
        private void Convert()
        {
            var any = false;
            if(sqrSequencer)
            {
                // ADCv3: SQR1[3:0] is ranks-1, then 5-bit channel fields
                // at a 6-bit stride - SQ1..SQ4 in SQR1 from bit 6,
                // SQ5..SQ9 in SQR2 from bit 0, and so on
                var ranks = (int)(sqr[0] & 0xF) + 1;
                for(var rank = 1; rank <= ranks && rank <= 16; rank++)
                {
                    int reg, shift;
                    if(rank <= 4)
                    {
                        reg = 0;
                        shift = 6 * rank;
                    }
                    else
                    {
                        reg = (rank - 5) / 5 + 1;
                        shift = 6 * ((rank - 5) % 5);
                    }
                    var ch = (int)((sqr[reg] >> shift) & 0x1F);
                    any = true;
                    ConvertOne(ch);
                }
            }
            else if((cfgr1 & CHSELRMOD) != 0)
            {
                for(var rank = 0; rank < 8; rank++)
                {
                    var ch = (int)((chselr >> (4 * rank)) & 0xF);
                    if(ch == 0xF)
                    {
                        break; // end of sequence marker
                    }
                    any = true;
                    ConvertOne(ch);
                }
            }
            else
            {
                for(var ch = 0; ch < 19; ch++)
                {
                    if((chselr & (1u << ch)) == 0)
                    {
                        continue;
                    }
                    any = true;
                    ConvertOne(ch);
                }
            }
            if(any)
            {
                isr |= EOS;
                if((ier & (EOC | EOS)) != 0)
                {
                    Connections[IrqLine].Blink();
                }
            }
        }

        private void ConvertOne(int ch)
        {
            dr = Sample(ch);
            isr |= EOC;
            Conversions++;
            if((cfgr1 & DMAEN) != 0)
            {
                // request one transfer; the DMA reads DR back
                Connections[DmaRequestLine].Blink();
            }
        }

        // raw counts for a channel, inverting main.c's conversions
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
                // actual_current(10mA) =
                //     ((raw*3300/41) - CURRENT_OFFSET*100) / MILLIVOLT_PER_AMP
                pinMv = amps * millivoltPerAmp + currentOffsetMv;
            }
            else if(channel == temperatureChannel)
            {
                return TemperatureCounts(degrees);
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

        // inverse of __LL_ADC_CALC_TEMPERATURE against the factory
        // calibration halfwords, which the harness seeds
        private uint TemperatureCounts(double degrees)
        {
            var cal1 = machine.SystemBus.ReadWord(tsCal1);
            var cal2 = machine.SystemBus.ReadWord(tsCal2);
            if(cal1 == cal2)
            {
                return cal1; // uncalibrated; avoid inventing a slope
            }
            var counts = cal1 + (degrees - 30.0) * (cal2 - cal1)
                                / (tsCal2Temp - 30.0);
            // __LL_ADC_CALC_TEMPERATURE scales the reading by
            // VDDA / TEMPSENSOR_CAL_VREFANALOG before comparing it
            // against the calibration points, and main.c passes 3300 for
            // VDDA on both families. The F0 calibrates at 3.3V so that is
            // a no-op there, but the G0 calibrates at 3.0V, and without
            // the inverse here a real 25C came back as 57.5C.
            return Clamp(counts * tsCalVrefMv / 3300.0);
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

        private const long ISR = 0x00;
        private const long IER = 0x04;
        private const long CR = 0x08;
        private const long CFGR1 = 0x0C;
        private const long CFGR2 = 0x10;
        private const long CHSELR = 0x28;
        private const long DR = 0x40;
        private const long CCR = 0x308;
        // ADCv3 (L4) regular sequence and sampling time registers
        private const long SMPR1 = 0x14;
        private const long SMPR2 = 0x18;
        private const long SQR1 = 0x30;
        private const long SQR2 = 0x34;
        private const long SQR3 = 0x38;
        private const long SQR4 = 0x3C;

        private const uint ADRDY = 1u << 0;
        private const uint EOC = 1u << 2;
        private const uint EOS = 1u << 3;

        private const uint ADEN = 1u << 0;
        private const uint ADDIS = 1u << 1;
        private const uint ADSTART = 1u << 2;
        private const uint ADCAL = 1u << 31;

        private const uint DMAEN = 1u << 0;

        // CHSELR holds a rank list rather than a channel bitmap
        private const uint CHSELRMOD = 1u << 21;

        private const int DmaRequestLine = 0;
        private const int IrqLine = 1;

        private readonly IMachine machine;
        private readonly int temperatureChannel;
        private readonly ulong tsCal1;
        private readonly ulong tsCal2;
        private readonly int tsCal2Temp;
        private readonly int tsCalVrefMv;
        private readonly int voltageChannel;
        private readonly int currentChannel;
        private readonly int voltageDivider;
        private readonly int millivoltPerAmp;
        private readonly int currentOffsetMv;

        private readonly bool sqrSequencer;

        private AM32_F051_Bridge bridge;
        private uint isr, ier, cr, cfgr1, cfgr2, chselr, ccr, dr;
        private uint smpr1, smpr2;
        private readonly uint[] sqr = new uint[4];
    }
}
