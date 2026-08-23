//
// BEMF sensing for the comparator-less MCUs: the F031 and G031 dies
// have no COMP peripheral, so their boards put external comparator
// chips on three GPIO pins and the firmware watches them through EXTI.
//
// Which phase is being sensed is not written to any peripheral - the
// firmware keeps it in globals - but it is observable: changeCompInput()
// arms exactly one of the three phase lines in RTSR/FTSR (assigning on
// the F031, read-modify-writing one line's bit on the G031, either way
// leaving one phase line armed). SensedPhase reads the trigger
// registers back through the EXTI peripheral object and reports the
// phase whose line is armed. The register offsets are constructor
// parameters because the two EXTI generations differ: F0 keeps
// RTSR/FTSR at 0x08/0x0C, G0 at 0x00/0x04.
//
// The F031's changeCompInput() ASSIGNS the trigger registers, leaving
// exactly one phase line armed, so the state scan above suffices. The
// G031's only ORs and clears the current line's bits, leaving all
// three lines armed - there the phase is named by which line's bits a
// write CHANGED (each phase alternates edge between visits), which the
// G0 EXTI model reports through its TriggerChanged event.
//
// The output level drives the phase's real GPIO pin (wired in the
// generated platform), so both the polled IDR reads in getBemfState()
// and the EXTI edge path are the firmware's own. The level is the
// INVERSE of the internal-comparator families' convention: their
// firmware polls !getCompOutputLevel() and arms the falling trigger for
// a "rising" crossing, while the F031/G031 read the pin directly and
// arm the rising trigger - the external comparator's output has the
// opposite polarity to the internal COMP's.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AM32_ExtiBemf : IDoubleWordPeripheral, IKnownSize,
                                 IAM32Comparator, INumberedGPIOOutput
    {
        // inverted flips the driven level for boards whose external
        // comparator has the opposite polarity (INVERTED_EXTI targets,
        // whose firmware also flips its edge bookkeeping)
        public AM32_ExtiBemf(IMachine machine, ulong extiBase,
                             long rtsrOffset, long ftsrOffset,
                             int phaseALine, int phaseBLine, int phaseCLine,
                             bool inverted = false,
                             long prOffset = -1, long pr2Offset = -1)
        {
            this.prOffset = prOffset;
            this.pr2Offset = pr2Offset;
            this.inverted = inverted;
            this.machine = machine;
            this.extiBase = extiBase;
            this.rtsrOffset = rtsrOffset;
            this.ftsrOffset = ftsrOffset;
            lines = new[] { phaseALine, phaseBLine, phaseCLine };
            foreach(var l in lines)
            {
                if(l < 0 || l > 31)
                {
                    throw new RecoverableException("phase EXTI line out of range");
                }
            }
            var conns = new Dictionary<int, IGPIO>();
            for(var i = 0; i < 3; i++)
            {
                conns[i] = new GPIO();
            }
            Connections = conns;
            Reset();
        }

        public long Size => 0x100;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; private set; }

        // 0=A 1=B 2=C: the phase whose EXTI line was selected last. If
        // the trigger-register state leaves exactly one phase line armed
        // (the F031's assignment-style writes), that names it; otherwise
        // the last TriggerChanged event does (the G031's read-modify
        // writes). Before the first changeCompInput() this reports
        // phase C, the same phase the shim starts on.
        public int SensedPhase
        {
            get
            {
                var e = Exti;
                if(e == null)
                {
                    return -1;
                }
                var armed = e.ReadDoubleWord(rtsrOffset) | e.ReadDoubleWord(ftsrOffset);
                var phase = -1;
                var count = 0;
                for(var i = 0; i < 3; i++)
                {
                    if((armed & (1u << lines[i])) != 0)
                    {
                        phase = i;
                        count++;
                    }
                }
                if(!selectionSeen)
                {
                    // Until the firmware has picked a phase, any pending
                    // on a phase line is a modeling artifact: the pins
                    // idle at a fixed level, so no real edge can have
                    // happened - but a guest-initiated reset can leave
                    // one latched across the EXTI re-init, and the
                    // F031's interruptRoutine() cannot clear it while
                    // its commutation globals are still zero, which
                    // storms the interrupt forever. A real external
                    // comparator sits quietly on its rail; power-up
                    // leaves nothing pending. Sweep them, as the bridge
                    // polls this getter every physics tick.
                    var mask = 0u;
                    for(var i = 0; i < 3; i++)
                    {
                        mask |= 1u << lines[i];
                    }
                    foreach(var off in new[] { prOffset, pr2Offset })
                    {
                        if(off < 0)
                        {
                            continue;
                        }
                        var pend = e.ReadDoubleWord(off) & mask;
                        if(pend != 0)
                        {
                            e.WriteDoubleWord(off, pend);
                        }
                    }
                }
                if(count == 1)
                {
                    if(selectionSeen)
                    {
                        lastPhase = phase;
                    }
                    else
                    {
                        // The FIRST selection must be a stable state, not
                        // a snapshot of the boot-time EXTI init, which
                        // arms the three lines one write apart: a poll
                        // landing between those writes sees exactly one
                        // line armed and would start driving that phase
                        // pin against its own armed trigger - an
                        // interrupt storm interruptRoutine() cannot
                        // clear while its commutation globals are still
                        // zero. A real changeCompInput() state persists
                        // for a whole commutation step, so requiring it
                        // to hold for 100us of virtual time separates
                        // the two. Once selected, track changes
                        // instantly, as commutation timing needs.
                        var now = machine.ElapsedVirtualTime.TimeElapsed
                            .TotalMicroseconds;
                        if(pendingPhase != phase)
                        {
                            pendingPhase = phase;
                            pendingSinceUs = now;
                        }
                        else if(now - pendingSinceUs >= SelectionStableUs)
                        {
                            lastPhase = phase;
                            selectionSeen = true;
                        }
                    }
                }
                else
                {
                    pendingPhase = -1;
                }
                return lastPhase;
            }
        }

        public bool CompOutput
        {
            get
            {
                return level;
            }
            set
            {
                level = value;
                Drive();
            }
        }

        public void Reset()
        {
            level = false;
            lastPhase = 2;
            selectionSeen = false;
            pendingPhase = -1;
            pendingSinceUs = 0;
            // settle the pins at their idle-high level NOW, before the
            // firmware arms the rising triggers: driving them later
            // would make a boot-time rising edge on every phase line,
            // and the F031's interruptRoutine cannot clear that storm
            // while its current_GPIO_PIN global is still zero
            for(var i = 0; i < 3; i++)
            {
                Connections[i].Set(true);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            // debug window: the sensed phase and the driven level
            switch(offset)
            {
            case 0x0: return (uint)SensedPhase;
            case 0x4: return level ? 1u : 0u;
            case 0x8: return selectionSeen ? 1u : 0u;
            case 0xC: return (uint)(pendingPhase + 1);
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        private void Drive()
        {
            // Hold everything idle until the firmware has selected a
            // phase (the first changeCompInput()). Before that the
            // interruptRoutine globals are still zero and its filter
            // loop returns without masking, so a single parked-rotor
            // chatter edge would storm forever - real external
            // comparators sit quietly on a rail until there is signal,
            // their hysteresis eating the noise our physics models.
            var sensed = selectionSeen ? lastPhase : -1;
            // inverse of the internal-COMP families' level (see the
            // header), or the level itself on an INVERTED_EXTI board.
            // Only the sensed phase's pin carries the comparison, the
            // other two idle high (comparator outputs are open-drain
            // pulled up on these boards).
            var driven = inverted ? level : !level;
            for(var i = 0; i < 3; i++)
            {
                Connections[i].Set(i != sensed || driven);
            }
        }

        private IDoubleWordPeripheral Exti
        {
            get
            {
                if(exti == null)
                {
                    exti = machine.SystemBus.WhatPeripheralIsAt(extiBase)
                        as IDoubleWordPeripheral;
                    if(exti == null)
                    {
                        this.Log(LogLevel.Error, "no EXTI at 0x{0:X}", extiBase);
                        return null;
                    }
                    var notifier = exti as IAM32TriggerNotifier;
                    if(notifier != null)
                    {
                        notifier.TriggerChanged += OnTriggerChanged;
                    }
                }
                return exti;
            }
        }

        private void OnTriggerChanged(int line)
        {
            for(var i = 0; i < 3; i++)
            {
                if(lines[i] == line)
                {
                    selectionSeen = true;
                    if(lastPhase != i)
                    {
                        lastPhase = i;
                    }
                    Drive();
                    return;
                }
            }
        }

        private const double SelectionStableUs = 100;

        private int pendingPhase = -1;
        private double pendingSinceUs;
        private readonly long prOffset;
        private readonly long pr2Offset;

        private readonly IMachine machine;
        private readonly ulong extiBase;
        private readonly long rtsrOffset;
        private readonly long ftsrOffset;
        private readonly int[] lines;
        private IDoubleWordPeripheral exti;
        private readonly bool inverted;
        private bool level;
        private bool selectionSeen;
        private int lastPhase = 2;
    }
}
