# Renode emulation of AM32 targets

Runs a real, unmodified AM32 firmware ELF on an emulated MCU, so that
the per-MCU register code under `Mcu/<target>/Src/` actually executes.
The SITL replaces all of that with its own fakes, which is why it could
not catch the PR #361 bug where two targets' `phaseouts.c` tested the
wrong variable, compiled cleanly and did nothing.

## Status

The firmware boots, detects a servo throttle signal through real capture
and DMA, arms, and **spins a motor closed loop on BEMF** sensed through
the real comparator. The firmware's own measured `commutation_interval`
agrees with the rpm the physics reports, so the timing it derives and
the motor it derives it from check out independently.

**Two MCU families are supported, F051 and G071**, covering all 52 F051
and 54 G071 targets with no per-target file to write - the platform is
generated from `Inc/targets.h` on demand. What differs between the
families is a table in `gen_target.py`, not a second code path.

Targets are selected by asking the preprocessor which MCU each one
resolves to, not by their names. Most are named after the MCU, but
`STELLAR_G071_V1` ends in its board revision, and an earlier suffix match
silently dropped it from `--list` and therefore from every sweep driven
by it. A target that is never listed is never tested, and nothing
complains.

The G071 was not a copy of the F051. Three things had to be modelled
before it would run, and each is a real difference rather than a gap in
the port:

- **the EXTI is a different peripheral.** Renode's own `stm32g0.repl`
  declares an `STM32F4_EXTI` here with a comment admitting the registers
  do not match. `IMR1` is at 0x80 and rising and falling have separate
  pending registers, so no comparator edge ever reached the NVIC.
- **the ADC uses the fully configurable sequencer.** `CFGR1.CHSELRMOD`
  makes `CHSELR` a list of four bit channel numbers per rank rather than
  a channel bitmap. Read one as the other and the wrong channels convert
  - the firmware saw no battery voltage at all and would not start.
- **the comparators are separate and differently laid out**: two of them
  at 0x40010200, output at CSR bit 30 rather than 14, and a four bit
  `INMSEL` at [7:4] rather than three at [6:4]. `N_VARIANT` targets move
  between COMP1 and COMP2 per commutation step.

### One target does not spin

`DT160_64K_G071` arms but cannot start the test motor. It is not a gap
in the emulation: it differs from `DT120_64K_G071` in **exactly one
preprocessed macro**, `DEAD_TIME` 210 against 120, and everything else -
comparator map, pin map, capture timer, DMA channel, ADC scaling - is
identical. Diff the two with `gen_target.py` and see.

210 is the only `DEAD_TIME` in either family past 127, where the
`BDTR.DTG` encoding stops being linear. Per RM0444, `DTG[7:5]=110` means
`(32 + DTG[4:0]) x 8` ticks, so 210 asks for **400 ticks = 6.25us** at
64 MHz, not the 3.28us a linear reading would suggest. With `ARR` 2665
(24 kHz) the firmware's startup duty of 400 ticks is exactly the dead
time, so the phase never drives at all. Whether that is intended for a
160 A ESC is a firmware question, not an emulator one.

### Input protocols

Servo and DShot both work, as real pin edges through the firmware's own
capture and DMA path rather than by writing values into `dma_buffer`.
That is the point: `detectInput()`, `checkDshot()` and
`computeDshotDMA()` are the auto-detection and decode most likely to
carry an MCU porting bug, and poking the buffer would bypass exactly the
code under test.

    python3 Mcu/Renode/run_renode_tests.py --target FD6288_F051 --dshot 600

DShot 300 and 600 both arm and spin. DShot 632 lands on the same
internal throttle scale as a 1300us servo pulse, so the three are
directly comparable, and they agree: 2934 rpm on servo, 2934 on
dshot300, 2935 on dshot600.

**DShot150 is not offered, because AM32 cannot detect it.**
`checkDshot()` classifies on the smallest gap between consecutive edges,
and accepts 1-3 or 4-8 counts at the detection prescaler of
`CPU_FREQUENCY_MHZ / 6` - 187.5ns per count at 48MHz.

Which gap is smallest depends on the frame. Detection happens at zero
throttle, where the frame is all zero bits, so the shortest interval is
a zero's high time, 0.375 of a bit period; once ones appear it becomes a
one's low time, 0.25 of a bit period. Either way dshot150 is out of
range:

| | 0.375T (all-zero frame) | 0.25T (frame with ones) |
|---|---|---|
| dshot600 | 625ns = 3.3 counts | 417ns = 2.2 counts |
| dshot300 | 1250ns = 6.7 counts | 833ns = 4.4 counts |
| dshot150 | 2500ns = **13.3 counts** | 1667ns = 8.9 counts |

Measured rather than predicted: driving dshot150 leaves
`smallestnumber` at 13, `dshot` at 0 and `inputSet` at 0, and the ESC
never arms.

That is worth stating precisely because it is a case of the two
harnesses agreeing. The SITL, which replaces every peripheral with a
fake, reached the same conclusion (`Mcu/SITL/sitl_gui.py`) - and this
harness, which runs the real capture and DMA registers, reproduces it
from the timing rather than from the same shared code. Two independent
routes to the same limitation is much better evidence than either alone.

The generator emits frames at 4kHz. Nothing in the firmware requires
that - a minimal inter-frame gap gives about 19kHz and decodes just as
cleanly - but every edge is a timer event, so the realistic rate costs
nearly five times less to simulate for no loss of coverage.

### Bidirectional DShot

    python3 Mcu/Renode/run_renode_tests.py --target FD6288_F051 --bdshot

Works, and spins at 2936 rpm - the same as servo and plain dshot.

Two things had to exist for it. The capture timer gained an
output-compare mode, because `sendDshotDma()` reuses **the same timer and
the same pin** as a PWM output with the DMA feeding `CCR1`, one `gcr[]`
entry per bit period. And the throttle generator became the arbiter of a
shared half duplex wire: Renode GPIO has no contention, so the generator
drives its own frame and otherwise passes the ESC's level through to the
pin, which is what a flight controller releasing the line looks like.

The wire is **pulled up**, so "not driving" is high, not low. Getting
that wrong is not subtle in its effect but is easy to miss in its cause:
a released line reading low looks exactly like the ESC holding the wire
down, and bdshot detection goes from working to no captures at all.

The reply is driven as one level per bit period rather than as a real
PWM waveform. AM32 writes `gcr[]` entries of 0 or 64 against `ARR` 92, so
on hardware a set period is a 70% duty pulse, but the GCR line code
carries information only in the transitions between periods.

`dshot_badcounts` settles at exactly 100 and then stops rising. That is
correct rather than a defect: bidirectional frames carry an inverted CRC,
and the firmware only switches to that interpretation after seeing the
line idle high 100 times, so the first hundred frames genuinely fail CRC
before the mode is recognised.

### Decoding the reply, and extended telemetry

    python3 Mcu/Renode/run_renode_tests.py --target FD6288_F051 --edt

The capture timer GCR-decodes the reply back out of the levels it drove
on the wire, rather than reading the firmware's `gcr[]` buffer, so the
assertion covers the transmit path instead of restating it. The line
code starts at the falling edge out of idle: the transmission opens with
`buffer_padding` idle-high periods, and the first recorded period is
whatever `CCR1` held before the DMA supplied an entry.

That closes the loop on the whole chain. A spin decodes to frame
`0x76D3`, whose payload is shift 3 and mantissa 365 - a 2920us
electrical period, which at 14 poles is 2935 rpm, the speed the physics
says the motor is turning. The firmware sensed the emulated motor
through the comparators and reported it correctly, and 12295 replies in
a run carry no GCR or CRC errors.

`--edt` additionally sends dshot command 13 while armed and stopped,
which enables extended telemetry. Frames then interleave one in two with
eRPM, and each kind carries its own top nibble, so a run shows which
went out: the type mask goes from `0xAA81` without EDT to `0xEAD5` with
it, and the four added bits are exactly temperature, voltage, current
and the EDT-init frame.

Calibration and sweep work stays in the SITL, on speed grounds - see
below.

## Running

Renode is not vendored; install it into the gitignored `tools/` tree the
same way the ARM toolchain is installed. Then, for any F051 or G071
target:

    python3 Mcu/Renode/gen_target.py FD6288_F051 --run

That generates the platform, picks the ELF out of `obj/`, generates a
matching eeprom, wires up the motor physics and drops you in the Renode
monitor with an ESC that will actually spin:

    (monitor) emulation RunFor "2.5"
    (monitor) sysbus WriteDoubleWord 0x50000000 1300   # 1300us throttle
    (monitor) emulation RunFor "1.5"
    (monitor) python "print monitor.Machine['sysbus.bridge'].Rpm"

`--exec CMD` runs monitor commands instead of sitting at the prompt,
`--model` picks the motor (and tunes the eeprom to it), `--elf` and
`--eeprom` override what it picks, `--list` shows what can be emulated.

### Driving it, and watching it

The throttle generator is a peripheral with monitor properties, so the
input is changed live:

    (monitor) throttle PulseUs 1300      # pulse width, microseconds
    (monitor) throttle FrameUs 20000     # frame period, default 50Hz

`--run` also loads `status()` into the monitor, with the firmware symbol
table for that ELF baked in:

    (monitor) python "status()"
    armed=1 running=1 inputSet=1 input=632 adjusted_input=632
    duty_cycle=611 commutation_interval=975 zero_crosses=2835 step=5
    bemf_timeout_happened=0 desync_happened=0
    battery_voltage=1227 actual_current=71 converted_degrees=38
    rpm=2933.7 theta=1.82 ia=0.00 ib=2.76 ic=-2.76

The first two lines are firmware state read out of SRAM at each symbol's
own width; the last is motor truth from the physics. For a time series,
`sample()` between `RunFor` steps and `save('run.csv')` at the end, then
plot the CSV.

`battery_voltage` (10mV), `actual_current` (10mA) and
`converted_degrees` are what the firmware makes of the ADC, and they now
mean something: the ADC model is fed from the same physics, so 1227
against a bus at 12.32V is the firmware's own arithmetic checking out
end to end, through a real DMA transfer into `ADCDataDMA[]`. Current
reads a little below the instantaneous value because the firmware runs
it through a moving average (`Src/main.c:816`); voltage and temperature,
being steady, match exactly.

**There is still no equivalent of the SITL GUI** - no live graphing,
only `status()` and a CSV to plot elsewhere.

### Debugging with gdb

    python3 Mcu/Renode/gen_target.py ARK_4IN1_F051 --gdb

Starts Renode with a gdb stub, then opens an xterm running the ARM gdb
from `tools/`, attached to the emulated core with the matching ELF. The
machine is halted at reset when gdb connects, so you get control before
any firmware instruction runs; `continue` in gdb starts it.

    (gdb) break tenKhzRoutine
    (gdb) continue
    Breakpoint 1, tenKhzRoutine () at Src/main.c:1348
    (gdb) bt
    #0  tenKhzRoutine () at Src/main.c:1348
    #1  0x0800617a in TIM6_DAC_IRQHandler () at Mcu/f051/Src/stm32f0xx_it.c:213
    #2  <signal handler called>
    #3  0x0800325a in main () at Src/main.c:1911

Backtraces cross the interrupt boundary, because the NVIC is emulated
rather than faked.

The ELF is checked for `.debug_info` before either window opens - the
AM32 makefile builds `-g3`, so it should always be there, and failing
early beats discovering it from a gdb prompt. **It also builds `-O3`**,
so expect inlined frames and locals reported as optimised out.

`--gdb-port` moves the stub off 3333, `--gdb-bin` picks a different gdb,
`--no-xterm` prints the launcher path to run yourself, and
`--no-skip-delays` turns off the `delayMillis` hook, which is worth
doing before single-stepping since that hook rewrites PC to return early
from the busy-wait delays.

### Log noise

Renode warns on every access to an unimplemented region, and two of
those warnings turned out to be pointing at real defects rather than
being noise to silence. Both are fixed; `logLevel 3` in the test harness
had been hiding them.

`IWDG` was an unimplemented tag, and the AM32 main loop kicks the
watchdog constantly, so it produced thousands of warnings per simulated
second. It now has a small model, which also counts the kicks:

    (monitor) python "print monitor.Machine['sysbus.iwdg'].Kicks"

The counter is accepted but never enforced: a watchdog that actually
fired would reset the CPU every time you paused at a breakpoint.

`adc: Issued a start event before the last sequence finished` was the
second, and it meant the ADC path did not work at all. The platform had
the stock model self-triggering at 1kHz, a hardware trigger AM32 does
not use - it starts conversions in software - and the stock model has no
DMA output, so conversions were never drained and `ADCDataDMA[]` stayed
zero. Replaced by a model that scans `CHSELR` and raises a DMA request
per conversion, with the external trigger dropped.

The eeprom is generated rather than optional. Renode zero-fills unbacked
memory where erased flash reads 0xFF, so without one the firmware takes
the settings migration path - and Renode's own diagnostic for a missing
one is `LoadBinary ... Parameters did not match the signature`, which
says nothing about the real problem.

The test runner takes the same `--target` and picks the matching ELF out
of `obj/`:

    python3 Mcu/Renode/run_renode_tests.py --target ARK_4IN1_F051

Paths must be absolute if you drive Renode directly: the harness runs
from a scratch directory, as the SITL suite does.

## What is here

    gen_target.py                   builds a platform for any F051 or G071
                                    target out of Inc/targets.h
    platforms/stm32f051_base.repl   MCU-common. Vendored from Renode's
    platforms/stm32g071_base.repl   platforms/cpus/stm32f0.repl and
                                    stm32g0.repl (Antmicro, MIT - header
                                    retained) and edited
    peripherals/stm32/              our peripheral models, GPL-3, loaded at
                                    runtime with `include @...cs`; no Renode
                                    rebuild needed
    peripherals/common/             family-neutral models: the motor bridge,
                                    the throttle generator, the comparator
                                    interface both families implement
    scripts/am32_f051.resc          one per family, shared by every generated
    scripts/am32_g071.resc          target script of that family

### Adding a target

Nothing to add. `gen_target.py` reads the target out of `Inc/targets.h`
and writes the `.repl` and `.resc` into `obj/renode/`; the test harness
generates into its own scratch directory. If a target builds, it should
emulate.

**`targets.h` is not parsed - it is preprocessed.** It is nested
`#ifdef` several levels deep with `#ifndef` fallbacks at the end, so a
parser here would drift from what the compiler actually sees. Instead
the real preprocessor runs over a stub that defines the target and the
resolved macros are read back with `-dM`. That costs ~66ms, which is
what makes generating on demand better than checking 52 platform files
into the tree and letting them rot.

Generating rather than hand-writing also removes a class of silent
error, because several things vary per target and **none of them fail
loudly when wrong**:

- the **comparator map**. The `INMSEL` field selects which pin the
  inverting input watches, but which pin is phase A, B or C differs per
  target - six permutations across the F051 range. Wrong, and the
  firmware commutates against the wrong phase and merely runs badly.
- the **bridge pin map**. Most targets put phase A on PA10/PB1, but a
  third of them rotate the phases across the same six pins.
- the **capture timer and DMA channel**: TIM15 or TIM3 on the F051,
  TIM3 or TIM16 on the G071.
- the **bridge topology**. Most targets drive a high and a low side per
  phase, but `USE_INVERTED_LOW` ones turn the low FET on by writing BRR,
  and `PWM_ENABLE_BRIDGE` ones have a gate driver with one PWM and one
  enable pin and no low side at all. Getting this wrong is the quietest
  failure of the lot: `CRTEENSY_HILARIESC_F051` spun and passed its
  tests for a while with its static low phase modelled as floating.
- the **eeprom address**, 0x0801F800 on the 128k parts against
  0x0800F800 elsewhere.

### How a generated overlay fits the base

It `using`s the base and adds what differs. It may only **add**:
redeclaring a name the base already has fails with "Variable 'x' was
already declared". That rule decides what the base can contain, because
TIM3 and TIM15 swap roles between hardware groups - one captures the
throttle while the other stays general purpose. Neither can have a
default in the base, so the base declares neither and the overlay
declares both. The comparator and the bridge are absent from the base
for the same reason: their maps are required constructor arguments, and
a default would be silently wrong for the targets that differ.

The target name cannot be turned into a platform path inside a `.resc`:
the monitor expands a path variable at the start of a path but leaves a
second one later in the same path as a literal, with no error. Hence a
generated `.resc` per target that sets `$platform` outright.

### Why the platform file is vendored rather than included

Renode's `stm32f0.repl` cannot be used as-is, and cannot be extended
either: a `using` of it makes its peripheral names undeclarable, so
replacing the RCC means owning the whole file. Changes made:

- **Memories added.** The stock file declares *none* — the per-part
  files (`stm32f042`, `stm32f072`) add them. F051 is 32K flash / 8K SRAM,
  plus a flash alias at 0 and the factory calibration page at
  `0x1FFFF000` (the 1 kHz ADC path reads temperature calibration
  halfwords at `0x1FFFF7B8`/`0x1FFFF7C2`; an unbacked pair divides by
  zero).
- **Timers 10 MHz -> 48 MHz.** The stock file is a generic F0. Left
  alone, every `delayMicros()` runs ~4.8x too long.
- **TIM6 update interrupt wired to `nvic@17`.** The stock file leaves
  TIM6 unconnected, so `tenKhzRoutine()` would never run and the ESC
  could never arm.
- **RCC replaced.** The stock RCC is a Python "flip-flop" stub that
  alternates each read. That satisfies the HSI and HSI14 ready loops by
  luck, but it returns 0 unconditionally for `CSR`, so the firmware
  parks forever in the LSI ready loop at `Mcu/f051/Src/peripherals.c:95`.
- **Python DMA stub removed.** It throws (`sysbus` undefined in its
  script context) as soon as the firmware programs it. A real model is
  needed for the throttle input path.
- **TIM1 and the SYSCFG/COMP page replaced.** The stock `STM32_Timer` has
  no MOE, no complementary outputs, no dead time and no preload
  shadowing; the COMP page was a bare tag, so `COMP1OUT` read back as
  nothing and BEMF sensing could not work at all.
- **Stock alternate-function connection blocks for `timer1` and the
  capture timer deleted.** A later connection block *replaces* an
  earlier one, so leaving them silently overrode the DMA and NVIC wiring
  in the peripheral declarations. This cost real debugging time:
  captures happened and went nowhere.

`stm32g0.repl` is vendored for the same reason but needed far less. It
already has real memories, DMA and a flash controller, and its RCC stub
is good enough that the firmware reaches `main()` unaided - the F051 hung
in its clock spin loops. The G0 edits are the 10 MHz to 64 MHz timer
correction, the same two alternate-function block deletions, the EXTI
and comparator replacements described above, a calibration page, and
register-file stubs for SYSCFG and DMAMUX.

That last one is a deliberate deviation worth stating: the platform
hardwires DMA routing rather than modelling DMAMUX, so **the firmware's
DMAMUX configuration is not checked**. If a target ever routes a request
to the wrong channel, this harness will not notice.

Both made-up peripherals - the motor bridge and the throttle generator -
move to 0x60000000 on the G0, because 0x50000000 is unmapped on the F051
but is where the G0 puts GPIO.

## Speed, and where it goes

Measured on this tree, not estimated, with the `delayMillis` skip hook
active. Wall seconds per simulated second, and the resulting motor state
after 1.5 s of spinning:

| `batchUs` | boot+arm | spinning | rpm | `commutation_interval` |
|---|---|---|---|---|
| 2  | 15.0x | 19.1x | 2418 | 1181 |
| 5  | 9.7x  | 11.1x | 2362 | 1210 |
| **10** | **6.3x** | **9.6x** | **2437** | **1170** |
| 20 | 5.9x  | 8.1x  | 2441 | 1168 |

The rpm column above predates seeding MOTOR_KV from the model, so the
absolute values are ~20% low against what the test reports now. The
comparison between rows is unaffected: every row ran the same eeprom.

For reference the MCU emulation alone, with no physics, is 4.8x booting
and 6.8x throttled; the host SITL runs ~1.4x *faster* than real time.

The physics coupling cost is **per-batch overhead, not integration**.
Every row above runs the same number of physics sub-steps per simulated
second - `batchUs` changes only how often Renode samples the registers
and crosses the P/Invoke boundary - yet the cost more than halves from 2
to 10. The integration itself is nearly free.

`batchUs: 10` is the default: it costs 0.8% in rpm against the 2us
reference for a 2x speedup. The spread across the table is real sampling
error and not noise - two runs at the same setting are bit-identical
(same rpm, same zero-cross count, same commutation interval), so the
differences are attributable to batch size alone. Every setting spins
cleanly: `bemf_timeout_happened` and `desync_happened` are both 0
throughout.

A stationary, undriven motor is skipped entirely rather than integrated,
which is most of boot. That helps but does not eliminate the boot cost,
because the batch tick itself - four register reads and a P/Invoke - is
what is expensive, not the work it decides to skip.

Calibration work still belongs in the SITL - a 60s chirp here is minutes
- but a spin is entirely practical.

## Checked against the SITL

Same model, same eeprom, same 1300us throttle, same 4.0s of simulated
time: the emulated F051 settles at 2934 rpm against the host SITL's
3125, 6.1% apart. **Dead time accounts for most of that**, the SITL
target running 500ns against FD6288's 937ns (DEAD_TIME 45 at 48MHz).

The evidence is a genuinely controlled pair, which fell out of running
every F051 target against the same model. `DIATONE_F051` and
`MAMBA_F40PRO_F051` differ in exactly one macro - diff their
preprocessed output and `DEAD_TIME` is the only line:

| target | DEAD_TIME | dead time | rpm |
|---|---|---|---|
| DIATONE_F051 | 45 | 937 ns | 2934 |
| MAMBA_F40PRO_F051 | 20 | 417 ns | 3115 |

520ns of dead time is worth 6.2% of rpm here, and the SITL sits 437ns
below FD6288, so dead time is the right size to be the dominant term
rather than a rounding error.

**Do not read that as a slope.** Grouping all 52 F051 targets by
`DEAD_TIME` shows a staircase, not a line - the firmware quantises what
it does with it:

| DEAD_TIME | rpm |
|---|---|
| 14, 20 | 3115 |
| 25, 30, 40, 45 | 2934 |
| 50, 60, 70 | 2854 |
| 80 | 2707 |
| 100 | 2592 |

The DIATONE/MAMBA pair straddles the 20-to-25 step, which is why it
reads as a large effect. Two targets four counts apart inside one tread
would have shown nothing at all. This also settles a `USE_INVERTED_LOW`
question cleanly: `CRTEENSY_HILARIESC_F051` (DEAD_TIME 40) landing on
exactly the same 2935 as FD6288 (45) once the inversion was modelled is
the expected result, not a suspicious coincidence, because 40 and 45
share a tread.

**A warning about how not to measure this.** An earlier version of this
file claimed the opposite, that dead time was worth 0.004%, on the
strength of comparing FD6288 (DEAD_TIME 45) against ARK_4IN1
(DEAD_TIME 25) and finding 2934.2252 against 2934.3298. That pair is
confounded: ARK also sets `TARGET_MIN_BEMF_COUNTS 3` where FD6288 has
2, and the two effects are each worth about 6% in opposite directions,
so they cancel almost exactly. Two targets are only a controlled
experiment if you diff every macro, not the one you are thinking about.

Two independent implementations - one substituting every peripheral, one
executing the real register code - agreeing to within a known hardware
difference is the evidence that the register path reproduces the
calibrated model rather than quietly diverging from it.

## Physics: borrowed, not forked

`Mcu/SITL/sim/motor.c` is compiled unmodified into `libam32sim.so`
(`Mcu/Renode/sim/`) and reached over P/Invoke. It has been refit against
real hardware more than once, so a C# transliteration would diverge on
the next recalibration and leave two models with no ground truth.
`am32sim_shim.c` supplies what the SITL's fake peripherals would have -
and here every one of those inputs comes from a register the real
`Mcu/f051/Src` code wrote.

## Bring-up findings

Recorded because they cost time to discover:

- The **ADC did not need a model.** `Analog.STM32F0_ADC` implements the
  `ADCAL` self-clear and `ADRDY`, so both spin loops in
  `Mcu/f051/Src/ADC.c` pass unmodified.
- **EXTI `SWIER` is supported** by the stock EXTI model, so the deferred
  dshot decode path needs no work.
- The **default eeprom sets `input_type = DSHOT_IN`** (`default_settings[]`
  byte 46 is `0x01`). With `dshot` already set, `detectInput()` only ever
  calls `checkDshot()`, never `checkServo()` — so a servo test signal is
  silently ignored. Any servo-based test must override
  `INPUT_SIGNAL_TYPE` to 0 (auto) or 2.
- Custom C# peripherals load fine on the **mono** portable build; the
  dotnet variant is not required.
