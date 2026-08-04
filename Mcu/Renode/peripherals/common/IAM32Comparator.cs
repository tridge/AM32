//
// What the motor bridge needs from a comparator, whichever family it is.
// The F0 has one COMP block sharing a page with SYSCFG; the G0 has two
// separate comparators and a different CSR layout. The bridge only cares
// which phase is being watched and what level to drive.
//
// This is a file of its own because "include @*.cs" compiles each file
// separately, so a type must be included before anything referencing it.
//
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // derives from IPeripheral so the bridge can find it with
    // GetPeripheralsOfType, which is constrained to peripherals
    public interface IAM32Comparator : IPeripheral
    {
        // 0=A 1=B 2=C, or -1 when the selected input is not a phase pin
        int SensedPhase { get; }

        // set by the motor model: true when the virtual neutral is above
        // the floating phase
        bool CompOutput { get; set; }
    }

    // Implemented by an EXTI model that can report which line's
    // rising/falling trigger a write changed. Declared here rather than
    // with the G0 EXTI so AM32_ExtiBemf can name the type on families
    // whose scripts never compile that model (the F031's stock F4 EXTI
    // gives the same information through its register state instead).
    public interface IAM32TriggerNotifier
    {
        event System.Action<int> TriggerChanged;
    }
}
