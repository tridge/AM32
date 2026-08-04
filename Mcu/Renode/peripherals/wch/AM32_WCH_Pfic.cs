//
// WCH PFIC (Programmable Fast Interrupt Controller) for the CH32V203,
// plus the QingKe V4B trap plumbing Renode's generic RiscV32 lacks.
//
// Three WCH-isms have to be emulated together here, because the stock
// CPU knows none of them:
//
//  1. Vectoring. The firmware writes mtvec = _vector_base|3, WCH's
//     "table of handler addresses" mode. Renode coerces that to its
//     mode 1 (CLINT vectored), where an interrupt lands at
//     base + 4*cause and an exception at base - both of which are data
//     words inside the WCH vector table, not code. This model hooks
//     both landing addresses and redirects PC to the handler read from
//     the table. Only the machine-external interrupt (cause 11) is ever
//     raised, so the interrupt landing is the single address base+0x2c.
//
//  2. Source selection. Because everything funnels through MEIP, the
//     CPU cannot know which WCH interrupt fired. This model keeps the
//     PFIC enable/pending/priority state, picks the winner at dispatch
//     (lowest IPRIOR value, then lowest interrupt number) and indexes
//     the vector table with it.
//
//  3. HPE. Every AM32 handler is WCH-Interrupt-fast: the compiler
//     saves no caller-saved registers and relies on the hardware
//     prologue/epilogue. The dispatch hook saves ra, t0-t2, a0-a7 and
//     t3-t6; a pre-opcode hook on mret (0x30200073) restores them.
//     An opcode hook rather than an address hook deliberately: address
//     hooks miss translation blocks entered through the indirect-jump
//     fast path (a ret landing on the mret is exactly that), while
//     opcode hooks are embedded at translation time and always fire.
//     The startup's own mret into main() pops nothing because the
//     frame stack is empty.
//
// An interrupt becomes active at dispatch and stops requesting until
// its mret, like the real PFIC's IACTR - without this the level-held
// EXTI lines would re-trap the moment a handler re-enables interrupts.
// Preemption follows the V4B's nested mode, which the startup enables
// with INTSYSCR.INESTEN: bit 7 of the priority byte is the preemption
// class and bits 6:5 only order simultaneous requests, so EXTI/TIM3 at
// 0x00/0x20 preempt SysTick/DMA handlers at 0xC0/0xE0 but never each
// other. In that mode the hardware also keeps mstatus.MIE set on
// interrupt entry - the dispatch here restores it after the generic
// trap cleared it - so a higher class cuts in immediately and the
// eligibility gating is what keeps equals and lowers out until mret.
// VTF registers are accepted and ignored: SetVTFIRQ() points them at
// the same symbols the vector table already holds, so table dispatch
// is behaviour-identical.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using System;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.IRQControllers
{
    public class AM32_WCH_Pfic : IDoubleWordPeripheral, IBytePeripheral,
                                 IKnownSize, IIRQController
    {
        public AM32_WCH_Pfic(IMachine machine, RiscV32 cpu)
        {
            this.machine = machine;
            this.cpu = cpu;
            Reset();
        }

        public long Size => 0x1000;

        public void Reset()
        {
            lock(sync)
            {
                Array.Clear(enable, 0, enable.Length);
                Array.Clear(pendingLevel, 0, pendingLevel.Length);
                Array.Clear(pendingSoft, 0, pendingSoft.Length);
                Array.Clear(active, 0, active.Length);
                Array.Clear(priority, 0, priority.Length);
                frames.Clear();
                // hooks survive a reset on purpose: the CPU keeps them,
                // and installing a second copy would double-dispatch.
                // mie does NOT survive - the CPU reset clears it - so
                // Evaluate() re-establishes MEIE on the next delivery
                meieEnsured = false;
                resetRequested = false;
            }
        }

        // interrupt inputs, wired in the platform as "-> pfic@N" with N
        // the WCH interrupt number (the vector table index)
        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= MaxIrq)
            {
                this.Log(LogLevel.Error, "interrupt number {0} out of range", number);
                return;
            }
            lock(sync)
            {
                var w = number >> 5;
                var bit = 1u << (number & 31);
                if(value)
                {
                    pendingLevel[w] |= bit;
                }
                else
                {
                    pendingLevel[w] &= ~bit;
                }
                Evaluate();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            lock(sync)
            {
                if(offset >= Isr && offset < Isr + 0x20)
                {
                    return enable[(offset - Isr) / 4];
                }
                if(offset >= Ipr && offset < Ipr + 0x20)
                {
                    var w = (offset - Ipr) / 4;
                    return pendingLevel[w] | pendingSoft[w];
                }
                if(offset >= Iactr && offset < Iactr + 0x20)
                {
                    return active[(offset - Iactr) / 4];
                }
                if(offset >= Iprior && offset < Iprior + 0x100)
                {
                    var i = (int)(offset - Iprior);
                    return (uint)(priority[i] | priority[i + 1] << 8
                        | priority[i + 2] << 16 | priority[i + 3] << 24);
                }
                switch(offset)
                {
                case Ithresdr: return ithresdr;
                case Cfgr: return 0;
                case Sctlr: return sctlr;
                default: return 0;
                }
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            lock(sync)
            {
                if(offset >= Ienr && offset < Ienr + 0x20)
                {
                    enable[(offset - Ienr) / 4] |= value;
                    Evaluate();
                    return;
                }
                if(offset >= Irer && offset < Irer + 0x20)
                {
                    enable[(offset - Irer) / 4] &= ~value;
                    Evaluate();
                    return;
                }
                if(offset >= Ipsr && offset < Ipsr + 0x20)
                {
                    pendingSoft[(offset - Ipsr) / 4] |= value;
                    Evaluate();
                    return;
                }
                if(offset >= Iprr && offset < Iprr + 0x20)
                {
                    pendingSoft[(offset - Iprr) / 4] &= ~value;
                    Evaluate();
                    return;
                }
                if(offset >= Iprior && offset < Iprior + 0x100)
                {
                    var i = (int)(offset - Iprior);
                    priority[i] = (byte)value;
                    priority[i + 1] = (byte)(value >> 8);
                    priority[i + 2] = (byte)(value >> 16);
                    priority[i + 3] = (byte)(value >> 24);
                    return;
                }
                switch(offset)
                {
                case Ithresdr: ithresdr = value; return;
                case Cfgr:
                    if((value & 0xFFFF0000) == Key3 && (value & 0x80) != 0)
                    {
                        // NVIC_SystemReset(): the signal-loss paths in
                        // main.c depend on actually restarting. The
                        // family .resc's reset macro reloads the ELF,
                        // which is what points the PC back at the entry.
                        // Debounced: Renode applies the reset at a safe
                        // point, not instantly like the real register,
                        // and the firmware keeps calling until it lands
                        // - our own Reset() re-arms the request.
                        if(!resetRequested)
                        {
                            resetRequested = true;
                            this.Log(LogLevel.Info, "system reset requested");
                            machine.RequestReset();
                        }
                    }
                    return;
                case Sctlr: sctlr = value; return;
                default: return; // VTFIDR/VTFADDR and anything else: accepted, unused
                }
            }
        }

        public byte ReadByte(long offset)
        {
            lock(sync)
            {
                if(offset >= Iprior && offset < Iprior + 0x100)
                {
                    return priority[offset - Iprior];
                }
                return 0;
            }
        }

        public void WriteByte(long offset, byte value)
        {
            lock(sync)
            {
                if(offset >= Iprior && offset < Iprior + 0x100)
                {
                    priority[offset - Iprior] = value;
                }
                // VTFIDR bytes land here too; accepted, unused
            }
        }

        public ulong Dispatches { get; private set; }
        public int FrameDepth => frames.Count;

        // per-interrupt dispatch totals, for bring-up diagnostics
        public string DispatchStats
        {
            get
            {
                lock(sync)
                {
                    var parts = new List<string>();
                    foreach(var kv in dispatchCounts)
                    {
                        parts.Add(string.Format("{0}:{1}", kv.Key, kv.Value));
                    }
                    return string.Join(" ", parts);
                }
            }
        }

        // The V4B with nesting enabled (INTSYSCR.INESTEN, which the
        // startup sets) splits the priority byte: bit 7 is the
        // preemption level, bits 6:5 only order simultaneous requests.
        // A source may preempt only a strictly lower class - EXTI/TIM3
        // at 0x00/0x20 preempt SysTick/DMA handlers at 0xC0/0xE0, never
        // each other and never the other way around.
        private int PreemptLevel(int irq)
        {
            return priority[irq] >> 7;
        }

        // preemption level of the innermost active handler, or worse
        // than any real level when none is active. Must be called with
        // sync held.
        private int ActivePreemptLevel()
        {
            var level = int.MaxValue;
            foreach(var f in frames)
            {
                var l = PreemptLevel(f.Irq);
                if(l < level)
                {
                    level = l;
                }
            }
            return level;
        }

        // must be called with sync held
        private void Evaluate()
        {
            var ceiling = ActivePreemptLevel();
            var any = false;
            for(var w = 0; w < Words && !any; w++)
            {
                var elig = (pendingLevel[w] | pendingSoft[w]) & enable[w] & ~active[w];
                while(elig != 0)
                {
                    var b = TrailingZeros(elig);
                    elig &= elig - 1;
                    if(PreemptLevel((w << 5) + b) < ceiling)
                    {
                        any = true;
                        break;
                    }
                }
            }
            if(any && !hooksInstalled)
            {
                InstallHooks();
            }
            if(any && hooksInstalled && !meieEnsured)
            {
                // re-established after a machine reset cleared mie; the
                // hooks themselves survive on the CPU
                cpu.MIE = (ulong)cpu.MIE | (1ul << MachineExternalInterrupt);
                meieEnsured = true;
            }
            cpu.OnGPIO(MachineExternalInterrupt, any);
        }

        private void InstallHooks()
        {
            // mtvec is written early in handle_reset, long before the
            // firmware can enable any interrupt, so it is valid by the
            // first Evaluate() that finds an eligible source
            vectorBase = (ulong)cpu.MTVEC & ~3ul;
            if(vectorBase == 0)
            {
                this.Log(LogLevel.Error, "interrupt enabled before mtvec was set");
                return;
            }
            cpu.AddHook(vectorBase + 4 * MachineExternalInterrupt,
                        (_, __) => DispatchInterrupt());
            cpu.AddHook(vectorBase, (_, __) => DispatchException());
            cpu.AddPreOpcodeExecutionHook(0xFFFFFFFFul, MretOpcode,
                                          (_, __) => RestoreFrame());
            cpu.EnablePreOpcodeExecutionHooks(1);
            // the firmware never touches the mie CSR - real WCH
            // interrupt gating is PFIC enables plus mstatus.MIE - but
            // Renode's CPU requires MEIE for MEIP delivery
            cpu.MIE = (ulong)cpu.MIE | (1ul << MachineExternalInterrupt);
            meieEnsured = true;
            hooksInstalled = true;
        }

        // Abandon a trap that found nothing to dispatch: return to the
        // interrupted code as an mret would, restoring the MIE the trap
        // entry cleared from MPIE - without that the CPU would run with
        // interrupts silently off until the next __enable_irq().
        private void AbandonTrap()
        {
            var mstatus = (ulong)cpu.MSTATUS;
            if((mstatus & Mpie) != 0)
            {
                cpu.MSTATUS = mstatus | Mie;
            }
            cpu.PC = cpu.MEPC;
        }

        private void DispatchInterrupt()
        {
            lock(sync)
            {
                var ceiling = ActivePreemptLevel();
                var irq = -1;
                var best = int.MaxValue;
                for(var w = 0; w < Words; w++)
                {
                    var elig = (pendingLevel[w] | pendingSoft[w]) & enable[w] & ~active[w];
                    while(elig != 0)
                    {
                        var b = TrailingZeros(elig);
                        elig &= elig - 1;
                        var n = (w << 5) + b;
                        if(PreemptLevel(n) >= ceiling)
                        {
                            continue;
                        }
                        // full byte then number: subpriority orders the
                        // candidates that may all be dispatched
                        var score = (priority[n] << 8) | n;
                        if(score < best)
                        {
                            best = score;
                            irq = n;
                        }
                    }
                }
                if(irq < 0)
                {
                    // raced with the source dropping
                    AbandonTrap();
                    return;
                }
                var handler = machine.SystemBus.ReadDoubleWord(vectorBase + 4 * (ulong)irq);
                if(handler == 0)
                {
                    this.Log(LogLevel.Error,
                             "interrupt {0} has no vector table entry", irq);
                    AbandonTrap();
                    return;
                }
                var frame = new ulong[HpeRegs.Length];
                for(var i = 0; i < HpeRegs.Length; i++)
                {
                    frame[i] = cpu.GetRegister(HpeRegs[i]).RawValue;
                }
                // the hardware prologue shadows the trap CSRs per
                // nesting level too: a nested trap overwrites the
                // single architectural mepc/mstatus, and without this
                // the outer handler's mret would consume the inner
                // trap's values and return into the wrong code
                frames.Add(new Frame
                {
                    Irq = irq,
                    Regs = frame,
                    Mepc = (ulong)cpu.MEPC,
                    Mstatus = (ulong)cpu.MSTATUS,
                    Mcause = (ulong)cpu.MCAUSE,
                });
                active[irq >> 5] |= 1u << (irq & 31);
                pendingSoft[irq >> 5] &= ~(1u << (irq & 31));
                Dispatches++;
                ulong dc;
                dispatchCounts.TryGetValue(irq, out dc);
                dispatchCounts[irq] = dc + 1;
                // real vectored dispatch reports the selected source in
                // mcause, not the generic MEIP the CPU delivered
                cpu.MCAUSE = 0x80000000ul | (ulong)irq;
                // with nesting enabled the V4B keeps mstatus.MIE set on
                // interrupt entry; a strictly higher preemption class
                // can cut in immediately, and the Evaluate() gating
                // above is what keeps equals and lowers out
                cpu.MSTATUS = (ulong)cpu.MSTATUS | Mie;
                Evaluate();
                cpu.PC = handler;
            }
        }

        private void DispatchException()
        {
            var mcause = (ulong)cpu.MCAUSE;
            this.Log(LogLevel.Error,
                     "exception mcause=0x{0:X} mepc=0x{1:X} mtval=0x{2:X}; "
                     + "redirecting to HardFault_Handler",
                     mcause, (ulong)cpu.MEPC, (ulong)cpu.MTVAL);
            var handler = machine.SystemBus.ReadDoubleWord(vectorBase + 4 * HardFaultEntry);
            if(handler != 0)
            {
                cpu.PC = handler;
            }
        }

        private void RestoreFrame()
        {
            lock(sync)
            {
                if(frames.Count == 0)
                {
                    return; // the startup's mret into main()
                }
                var f = frames[frames.Count - 1];
                frames.RemoveAt(frames.Count - 1);
                for(var i = 0; i < HpeRegs.Length; i++)
                {
                    cpu.SetRegister(HpeRegs[i], f.Regs[i]);
                }
                // before the mret executes, so it consumes this frame's
                // mepc and mstatus.MPIE rather than a nested trap's
                cpu.MEPC = f.Mepc;
                cpu.MSTATUS = f.Mstatus;
                cpu.MCAUSE = f.Mcause;
                active[f.Irq >> 5] &= ~(1u << (f.Irq & 31));
                Evaluate();
            }
        }

        private static int TrailingZeros(uint v)
        {
            var n = 0;
            while((v & 1) == 0)
            {
                v >>= 1;
                n++;
            }
            return n;
        }

        private struct Frame
        {
            public int Irq;
            public ulong[] Regs;
            public ulong Mepc;
            public ulong Mstatus;
            public ulong Mcause;
        }

        // ra, t0-t2, a0-a7, t3-t6: what the hardware prologue saves
        private static readonly int[] HpeRegs =
            { 1, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 28, 29, 30, 31 };

        private const long Isr = 0x000;
        private const long Ipr = 0x020;
        private const long Ithresdr = 0x040;
        private const long Cfgr = 0x048;
        private const long Ienr = 0x100;
        private const long Irer = 0x180;
        private const long Ipsr = 0x200;
        private const long Iprr = 0x280;
        private const long Iactr = 0x300;
        private const long Iprior = 0x400;
        private const long Sctlr = 0xD10;

        private const uint Key3 = 0xBEEF0000;
        private const ulong Mie = 1ul << 3;
        private const ulong Mpie = 1ul << 7;
        private const int Words = 8;
        private const int MaxIrq = 256;
        private const int MachineExternalInterrupt = 11;
        private const ulong HardFaultEntry = 3;
        private const ulong MretOpcode = 0x30200073;

        private readonly IMachine machine;
        private readonly RiscV32 cpu;
        private readonly object sync = new object();
        private readonly uint[] enable = new uint[Words];
        private readonly uint[] pendingLevel = new uint[Words];
        private readonly uint[] pendingSoft = new uint[Words];
        private readonly uint[] active = new uint[Words];
        private readonly byte[] priority = new byte[256];
        private readonly List<Frame> frames = new List<Frame>();
        private readonly Dictionary<int, ulong> dispatchCounts = new Dictionary<int, ulong>();
        private uint ithresdr, sctlr;
        private ulong vectorBase;
        private bool hooksInstalled;
        private bool meieEnsured;
        private bool resetRequested;
    }
}
