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

    // Implemented by a PWM peripheral that knows each phase's bridge
    // state directly - the NXP FlexPWM, whose commutation is pure
    // MASK/DTSRCSEL register state with no GPIO mode changes for the
    // bridge to decode. When one of these is present the bridge takes
    // everything from it instead of the GPIO+timer decode.
    public interface IAM32PwmSource : IPeripheral
    {
        // SITL_PHASE_*: 0 float, 1 low, 2 pwm, 3 pwm without
        // complementary, 4 proportional brake
        int PhaseState(int phase);

        // the active (post-LDOK) compare value for the phase
        uint PhaseDuty(int phase);

        uint Arr { get; }
        uint DeadTimeNs { get; }
        uint TickPs { get; }
        bool Running { get; }
    }

    // Implemented by whatever peripheral decodes the ESC's bidirectional
    // dshot replies - the capture timer on the STM32 families, the LPSPI
    // on the NXP - so the guilink can stream them without knowing which.
    // DecodesReplies distinguishes an instance that really carries the
    // reply wire from a sibling of the same class that does not (the
    // A153's LED SPI), so the lookup does not depend on enumeration
    // order.
    public interface IAM32ReplySource : IPeripheral
    {
        bool DecodesReplies { get; }
        uint ReplyCount { get; }
        bool TryGetReply(uint index, out uint frame);
    }

    // Optional fast path between the bench-side DShot generator and a
    // capture timer. It batches the host scheduling of one wire frame, but
    // the sink must still produce the same captured timestamps and DMA
    // requests that the individual pin edges would have produced.
    public interface IAM32DshotFrameSink : IPeripheral
    {
        bool BeginDshotFrame(uint frame, ulong bitPeriodNanoseconds);
        void CompleteDshotFrame();
        void CancelDshotFrame();
    }

    // Optional edge recorder fed by the motor bridge at its existing
    // physics batch boundary. This exposes state changes without adding a
    // sampling timer to the emulation.
    public interface IAM32LogicAnalyzer : IPeripheral
    {
        void ObserveBridge(int phaseA, int phaseB, int phaseC,
                           int sensedPhase, bool comparator);
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
