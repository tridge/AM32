//
// MCXA153 FlexPWM0: three submodules driving the three phases, with the
// commutation done entirely in registers - phaseouts.c writes only
// DTSRCSEL (per-submodule PWM23 source: 0 = PWM, 1 = inverted PWM,
// 2 = the constant 1 preloaded into SWCOUT) and MASK (per-channel
// force-off), then strobes SM0's CTRL2.FORCE, which fans out to SM1/SM2
// through their FORCE_SEL=master. PWM_A is the LOW-side FET, PWM_B the
// HIGH side.
//
// This model keeps the registers and answers the bridge's
// IAM32PwmSource queries by decoding MASK+DTSRCSEL exactly the way the
// hardware would drive the pins:
//   both channels masked                -> phase floating
//   sel23=2 (constant), A unmasked     -> low side held on
//   sel23=1 (inverted PWM), A+B live   -> complementary PWM
//   sel23=1, A masked, B live          -> PWM without complementary
//   sel23=1, B masked, A live          -> proportional brake
//
// VAL registers are buffered behind MCTRL.LDOK; the FORCE strobe is
// what latches a commutation's MASK/DTSRCSEL writes, as on hardware.
// All registers are 16-bit.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Timers
{
    [AllowedTranslations(AllowedTranslation.ByteToWord | AllowedTranslation.DoubleWordToWord)]
    public class MCXA_FlexPwm : IWordPeripheral, IKnownSize, IAM32PwmSource
    {
        public MCXA_FlexPwm(ulong ipbusHz = 192000000)
        {
            this.ipbusHz = ipbusHz;
            Reset();
        }

        public long Size => 0x200;

        public void Reset()
        {
            for(var i = 0; i < 3; i++)
            {
                sm[i] = new Submodule();
            }
            outen = 0;
            mask = 0;
            maskActive = 0;
            swcout = 0;
            dtsrcsel = 0;
            dtsrcselActive = 0;
            mctrl = 0;
        }

        // ---- IAM32PwmSource ----

        public bool Running => (mctrl & RunMask) != 0;

        public uint Arr => sm[0].Val1Active;

        public uint DeadTimeNs
        {
            get
            {
                // DTCNT0 in IPBUS ticks
                var ticks = sm[0].Dtcnt0 & 0x7FF;
                return (uint)(ticks * 1000000000UL / ipbusHz);
            }
        }

        public uint TickPs
        {
            get
            {
                var prsc = (int)((sm[0].Ctrl >> 4) & 0x7);
                return (uint)((1000000000000UL << prsc) / ipbusHz);
            }
        }

        public uint PhaseDuty(int phase)
        {
            return phase >= 0 && phase < 3 ? sm[phase].Val3Active : 0;
        }

        public int PhaseState(int phase)
        {
            if(phase < 0 || phase > 2 || !Running)
            {
                return 0;
            }
            var maskA = (maskActive & (1u << (8 + phase))) != 0;
            var maskB = (maskActive & (1u << (4 + phase))) != 0;
            var sel = (dtsrcselActive >> (2 + 4 * phase)) & 0x3;
            // OUTEN gates everything; the firmware sets 0x770 once
            if((outen & (1u << (8 + phase))) == 0)
            {
                maskA = true;
            }
            if((outen & (1u << (4 + phase))) == 0)
            {
                maskB = true;
            }
            if(maskA && maskB)
            {
                return 0;
            }
            if(sel == 2)
            {
                // SWCOUT's constant 1 through the deadtime generator:
                // low side on (the firmware preloads all SMxOUT23 bits)
                return maskA ? 0 : 1;
            }
            if(sel == 1)
            {
                if(!maskA && !maskB)
                {
                    return 2;
                }
                if(maskA)
                {
                    return 3;
                }
                return 4;
            }
            // sel 0: the un-inverted PWM source; the firmware only
            // leaves it on the floating phase, both channels masked
            return 0;
        }

        // ---- register file ----

        public ushort ReadWord(long offset)
        {
            if(offset < 0x120)
            {
                var index = (int)(offset / 0x60);
                var reg = offset % 0x60;
                if(index > 2)
                {
                    return 0;
                }
                return sm[index].Read(reg);
            }
            switch(offset)
            {
            case Outen: return (ushort)outen;
            case MaskReg: return (ushort)mask;
            case Swcout: return (ushort)swcout;
            case Dtsrcsel: return (ushort)dtsrcsel;
            case Mctrl:
                // LDOK reads back until the reload consumes it; the
                // firmware only checks RUN, so report both as written
                return (ushort)mctrl;
            default: return 0;
            }
        }

        public void WriteWord(long offset, ushort value)
        {
            if(offset < 0x120)
            {
                var index = (int)(offset / 0x60);
                var reg = offset % 0x60;
                if(index > 2)
                {
                    return;
                }
                var force = sm[index].Write(reg, value);
                if(force && index == 0)
                {
                    // SM0's FORCE fans out (SM1/SM2 use master select):
                    // latch the commutation state. VAL registers stay
                    // buffered - with FORCEN's force-init disabled a
                    // FORCE is an output event, not a load point; only
                    // LDOK moves them (flexpwm.c leaves LDMOD 0)
                    maskActive = mask;
                    dtsrcselActive = dtsrcsel;
                }
                return;
            }
            switch(offset)
            {
            case Outen: outen = value; return;
            case MaskReg: mask = value; return;
            case Swcout: swcout = value; return;
            case Dtsrcsel: dtsrcsel = value; return;
            case Mctrl:
                // CLDOK (bits 7:4) clears LDOK bits; LDOK (3:0) loads
                // the buffered registers; RUN (10:8) starts the
                // submodules
                var cldok = (value >> 4) & 0xF;
                var ldok = value & 0xF;
                mctrl = (uint)(value & ~0xF0);
                if(ldok != 0)
                {
                    foreach(var s in sm)
                    {
                        s.Load();
                    }
                }
                if(cldok != 0)
                {
                    mctrl &= (uint)~cldok;
                }
                return;
            default: return;
            }
        }

        private class Submodule
        {
            public ushort Read(long reg)
            {
                switch(reg)
                {
                case Ctrl2: return ctrl2;
                case CtrlReg: return Ctrl;
                case Init: return init;
                case Val1: return val1;
                case Val3: return val3;
                case Dtcnt0Reg: return (ushort)Dtcnt0;
                case Dtcnt1: return dtcnt1;
                default:
                    ushort v;
                    other.TryGetValue(reg, out v);
                    return v;
                }
            }

            // returns true when the write strobed FORCE
            public bool Write(long reg, ushort value)
            {
                switch(reg)
                {
                case Ctrl2:
                    ctrl2 = (ushort)(value & ~Force);
                    return (value & Force) != 0;
                case CtrlReg: Ctrl = value; return false;
                case Init: init = value; return false;
                case Val1: val1 = value; return false;
                case Val3: val3 = value; return false;
                case Dtcnt0Reg: Dtcnt0 = value; return false;
                case Dtcnt1: dtcnt1 = value; return false;
                default: other[reg] = value; return false;
                }
            }

            public void Load()
            {
                Val1Active = val1;
                Val3Active = val3;
            }

            public ushort Ctrl;
            public uint Dtcnt0;
            public uint Val1Active;
            public uint Val3Active;

            private ushort ctrl2, init, val1, val3, dtcnt1;
            private readonly Dictionary<long, ushort> other = new Dictionary<long, ushort>();

            private const long Init = 0x02;
            private const long Ctrl2 = 0x04;
            private const long CtrlReg = 0x06;
            private const long Val1 = 0x0E;
            private const long Val3 = 0x16;
            private const long Dtcnt0Reg = 0x30;
            private const long Dtcnt1 = 0x32;
            private const ushort Force = 1 << 6;
        }

        private const long Outen = 0x180;
        private const long MaskReg = 0x182;
        private const long Swcout = 0x184;
        private const long Dtsrcsel = 0x186;
        private const long Mctrl = 0x188;

        private const uint RunMask = 0x7u << 8;

        private readonly ulong ipbusHz;
        private readonly Submodule[] sm = new Submodule[3];
        private uint outen, mask, maskActive, swcout, dtsrcsel, dtsrcselActive, mctrl;
    }
}
