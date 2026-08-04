//
// MCXA153 CTIMER: the LPC-lineage timer. AM32 uses three of them -
// CTIMER0 as the dshot/servo capture timer, CTIMER1 as the commutation
// timer, CTIMER2 as the free-running interval counter.
//
// What the firmware relies on (Mcu/a153/Src/timers.c, IO.c,
// mcxa153_it.c):
//  - TCR CEN/CRST, PR prescale, TC readable AND writable
//  - MR[0..3] matches: MR0I/MR0R/MR1I in MCR; IR write-1-to-clear
//  - capture channels CR[1] (rising) and CR[2] (falling) fed from ONE
//    pin (INPUTMUX routes both to it), CCR per-channel edge enables
//  - CTCR ENCC+SELCC: clear TC on a chosen capture edge - the capture
//    values form the per-edge sawtooth doDshotCorrection() undoes
//  - a MATCH0 event on every TC clear (MR[0]=0), which is the eDMA
//    request that carries CR[1]/CR[2] into dma_buffer
//
// Connections: input 0 = the capture pin. Output 0 = IRQ, output 1 =
// the MATCH0 DMA request (edge), consumed by the eDMA's request mux.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using System;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class MCXA_Ctimer : IDoubleWordPeripheral, IKnownSize,
                               INumberedGPIOOutput, IGPIOReceiver
    {
        public MCXA_Ctimer(IMachine machine, ulong frequency)
        {
            this.frequency = frequency;
            var conns = new Dictionary<int, IGPIO>();
            conns[IrqLine] = new GPIO();
            conns[Match0DmaLine] = new GPIO();
            Connections = conns;
            counter = new LimitTimer(machine.ClockSource, frequency, this, "tc",
                                     limit: uint.MaxValue,
                                     direction: Direction.Ascending,
                                     enabled: false, workMode: WorkMode.Periodic,
                                     eventEnabled: true, autoUpdate: true);
            counter.LimitReached += OnLimit;
            // MR1 with MR1I but not MR1R: an interrupt while TC runs on,
            // which a periodic limit cannot model without wrapping the
            // count and wrecking the captures - so it gets its own
            // one-shot, rearmed at every point TC restarts from zero
            match1 = new LimitTimer(machine.ClockSource, frequency, this, "mr1",
                                    limit: uint.MaxValue,
                                    direction: Direction.Ascending,
                                    enabled: false, workMode: WorkMode.OneShot,
                                    eventEnabled: true, autoUpdate: false);
            match1.LimitReached += OnMatch1;
            Reset();
        }

        public long Size => 0x1000;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        public void Reset()
        {
            ir = 0;
            tcr = 0;
            pr = 0;
            mcr = 0;
            ccr = 0;
            ctcr = 0;
            for(var i = 0; i < 4; i++)
            {
                mr[i] = 0;
                cr[i] = 0;
            }
            counter.Enabled = false;
            counter.Divider = 1;
            counter.Limit = uint.MaxValue;
            counter.Value = 0;
            match1.Enabled = false;
            match1.Divider = 1;
            lastPin = false;
            havePin = false;
            Connections[IrqLine].Unset();
        }

        // the capture pin, from the throttle generator
        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                return;
            }
            var was = havePin && lastPin;
            havePin = true;
            lastPin = value;
            if((tcr & Cen) == 0)
            {
                return;
            }
            var rising = value && !was;
            var falling = !value && was;
            // CCR: CAP1RE b3, CAP1FE b4, CAP2RE b6, CAP2FE b7 (CAPnRE
            // at 3n, CAPnFE at 3n+1). Both channels watch the same pin.
            var tc = (uint)counter.Value;
            var clear = false;
            if(rising && (ccr & Cap1Re) != 0)
            {
                cr[1] = tc;
                clear |= ClearsOn(2); // SELCC 2 = CAP1 rising
            }
            if(falling && (ccr & Cap1Fe) != 0)
            {
                cr[1] = tc;
                clear |= ClearsOn(3);
            }
            if(rising && (ccr & Cap2Re) != 0)
            {
                cr[2] = tc;
                clear |= ClearsOn(4);
            }
            if(falling && (ccr & Cap2Fe) != 0)
            {
                cr[2] = tc;
                clear |= ClearsOn(5);
            }
            if(clear)
            {
                counter.Value = 0;
                RearmMatch1();
                // MR[0] = 0 matches on the cleared counter: the match0
                // DMA request that triggers the capture transfer
                if(mr[0] == 0)
                {
                    Connections[Match0DmaLine].Blink();
                }
            }
        }

        // one MR1 interrupt per restart-from-zero; TC runs on past it
        private void RearmMatch1()
        {
            match1.Enabled = false;
            if((mcr & Mr1I) == 0 || mr[1] == 0 || (tcr & Cen) == 0)
            {
                return;
            }
            match1.Value = 0;
            match1.Limit = mr[1];
            match1.Enabled = true;
        }

        private void OnMatch1()
        {
            ir |= Mr1Int;
            UpdateIrq();
        }

        private bool ClearsOn(int selcc)
        {
            return (ctcr & Encc) != 0 && ((ctcr >> 5) & 0x7) == (uint)selcc;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Ir: return ir;
            case Tcr: return tcr;
            case Tc: return (uint)counter.Value;
            case Pr: return pr;
            case Mcr: return mcr;
            case Ccr: return ccr;
            case Ctcr: return ctcr;
            default:
                if(offset >= Mr0 && offset < Mr0 + 16)
                {
                    return mr[(offset - Mr0) / 4];
                }
                if(offset >= Cr0 && offset < Cr0 + 16)
                {
                    return cr[(offset - Cr0) / 4];
                }
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Ir:
                ir &= ~value; // write 1 to clear
                UpdateIrq();
                return;
            case Tcr:
                if((value & Crst) != 0)
                {
                    counter.Value = 0;
                }
                var wasOn = (tcr & Cen) != 0;
                tcr = value & Cen;
                counter.Enabled = (tcr & Cen) != 0;
                if(counter.Enabled && !wasOn)
                {
                    RearmMatch1();
                }
                else if(!counter.Enabled)
                {
                    match1.Enabled = false;
                }
                return;
            case Tc:
                counter.Value = value;
                if(value == 0)
                {
                    RearmMatch1();
                }
                return;
            case Pr:
                pr = value;
                counter.Divider = (ulong)pr + 1;
                match1.Divider = (ulong)pr + 1;
                return;
            case Mcr:
                mcr = value;
                ApplyMatchLimit();
                RearmMatch1();
                UpdateIrq();
                return;
            case Ccr: ccr = value; return;
            case Ctcr: ctcr = value; return;
            default:
                if(offset >= Mr0 && offset < Mr0 + 16)
                {
                    var index = (offset - Mr0) / 4;
                    mr[index] = value;
                    ApplyMatchLimit();
                    if(index == 1)
                    {
                        RearmMatch1();
                    }
                }
                return;
            }
        }

        // MR0 with MR0R resets on match - the COM timer's period - so it
        // maps onto the periodic limit. Free runs to the 32-bit wrap
        // otherwise. MR1's interrupt-without-reset is match1's job.
        private void ApplyMatchLimit()
        {
            if((mcr & Mr0R) != 0 && mr[0] != 0)
            {
                counter.Limit = mr[0];
                limitIsMatch0 = true;
            }
            else
            {
                counter.Limit = uint.MaxValue;
                limitIsMatch0 = false;
            }
        }

        private void OnLimit()
        {
            if(limitIsMatch0 && (mcr & Mr0I) != 0)
            {
                ir |= Mr0Int;
                UpdateIrq();
            }
        }

        private void UpdateIrq()
        {
            var pending = ((ir & Mr0Int) != 0 && (mcr & Mr0I) != 0)
                || ((ir & Mr1Int) != 0 && (mcr & Mr1I) != 0);
            Connections[IrqLine].Set(pending);
        }

        private const long Ir = 0x00;
        private const long Tcr = 0x04;
        private const long Tc = 0x08;
        private const long Pr = 0x0C;
        private const long Mcr = 0x14;
        private const long Mr0 = 0x18;
        private const long Ccr = 0x28;
        private const long Cr0 = 0x2C;
        private const long Ctcr = 0x70;

        private const uint Cen = 1u << 0;
        private const uint Crst = 1u << 1;
        private const uint Mr0I = 1u << 0;
        private const uint Mr0R = 1u << 1;
        private const uint Mr1I = 1u << 3;
        private const uint Mr0Int = 1u << 0;
        private const uint Mr1Int = 1u << 1;
        private const uint Cap1Re = 1u << 3;
        private const uint Cap1Fe = 1u << 4;
        private const uint Cap2Re = 1u << 6;
        private const uint Cap2Fe = 1u << 7;
        private const uint Encc = 1u << 4;

        private const int IrqLine = 0;
        private const int Match0DmaLine = 1;

        private readonly ulong frequency;
        private readonly LimitTimer counter;
        private readonly LimitTimer match1;
        private readonly uint[] mr = new uint[4];
        private readonly uint[] cr = new uint[4];
        private uint ir, tcr, pr, mcr, ccr, ctcr;
        private bool lastPin, havePin;
        private bool limitIsMatch0;
    }
}
