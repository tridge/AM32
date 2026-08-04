//
// The stock STM32F4_EXTI with one fix: a software interrupt latches the
// pending register.
//
// Renode's model sets the output line on a SWIER write but never sets
// the corresponding PR bit, so a handler that READS the pending
// register to decide what to service sees nothing, clears nothing, and
// the level-held NVIC line re-enters it forever. The F415 is the first
// family here that trips this: EXINT15_10_IRQHandler checks
// EXINT->intsts before clearing (Mcu/f415/Src/at32f415_it.c), where the
// L431's equivalent clears line 15 unconditionally and never notices.
// On real hardware a SWIER write sets the PR bit exactly as a
// triggered edge does, and clearing PR clears the SWIER bit with it -
// which is what this model does.
//
// Everything else is the stock model verbatim (Renode 1.16.1), because
// its other behaviors are load bearing: not clearing a pending line on
// its own is what lets the comparator handlers wait out the
// commutation blanking window by immediate re-entry.
//
using System.Collections.Generic;
using System.Collections.ObjectModel;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.IRQControllers
{
    public class AM32_STM32_Exti : BasicDoubleWordPeripheral, IKnownSize, IIRQController, INumberedGPIOOutput
    {
        public AM32_STM32_Exti(IMachine machine, int numberOfOutputLines = 14, int firstDirectLine = DefaultFirstDirectLine) : base(machine)
        {
            var innerConnections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < numberOfOutputLines; ++i)
            {
                innerConnections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(innerConnections);

            core = new STM32_EXTICore(this, BitHelper.CalculateQuadWordMask(firstDirectLine, 0), treatOutOfRangeLinesAsDirect: true, allowMaskingDirectLines: false);

            numberOfLinesMask = BitHelper.CalculateQuadWordMask((int)NumberOfLines, 0);

            DefineRegisters();
            Reset();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number >= NumberOfLines)
            {
                this.Log(LogLevel.Error, "GPIO number {0} is out of range [0; {1})", number, NumberOfLines);
                return;
            }
            var lineNumber = (byte)number;

            if(core.CanSetInterruptValue(lineNumber, value, out var isLineConfigurable))
            {
                value = isLineConfigurable ? true : value;
                core.UpdatePendingValue(lineNumber, value);
                Connections[number].Set(value);
            }
        }

        public override void Reset()
        {
            base.Reset();
            foreach(var gpio in Connections)
            {
                gpio.Value.Unset();
            }
        }

        public long Size => 0x400;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long NumberOfLines => Connections.Count;

        protected const int DefaultFirstDirectLine = 23;

        private void DefineRegisters()
        {
            Registers.InterruptMask.Define(this)
                .WithValueField(0, 32, out core.InterruptMask, name: "IMR");

            Registers.EventMask.Define(this)
                .WithValueField(0, 32, name: "EMR");

            Registers.RisingTriggerSelection.Define(this)
                .WithValueField(0, 32, out core.RisingEdgeMask, name: "RTSR");

            Registers.FallingTriggerSelection.Define(this)
                .WithValueField(0, 32, out core.FallingEdgeMask, name: "FTSR");

            // reads back the software-set bits still pending; clearing
            // PR clears them, as on hardware
            Registers.SoftwareInterruptEvent.Define(this)
                .WithValueField(0, 32, name: "SWIER",
                    valueProviderCallback: _ => softwareInterrupt,
                    writeCallback: (_, value) =>
                    {
                        value &= numberOfLinesMask & core.InterruptMask.Value;
                        softwareInterrupt |= value;
                        // THE FIX: latch the pending bit, so a handler
                        // that inspects PR sees the line it must clear
                        core.PendingInterrupts.Value |= value;
                        BitHelper.ForeachActiveBit(value, x => Connections[x].Set());
                    });

            Registers.PendingRegister.Define(this)
                .WithValueField(0, 32, out core.PendingInterrupts, FieldMode.Read | FieldMode.WriteOneToClear, name: "PR",
                    writeCallback: (_, value) =>
                    {
                        softwareInterrupt &= ~value;
                        value &= numberOfLinesMask;
                        BitHelper.ForeachActiveBit(value, x => Connections[x].Unset());
                    });
        }

        private ulong softwareInterrupt;

        private readonly ulong numberOfLinesMask;
        private readonly STM32_EXTICore core;

        private enum Registers
        {
            InterruptMask = 0x0,
            EventMask = 0x4,
            RisingTriggerSelection = 0x8,
            FallingTriggerSelection = 0xC,
            SoftwareInterruptEvent = 0x10,
            PendingRegister = 0x14
        }
    }
}
