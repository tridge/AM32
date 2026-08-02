# Renode emulation of AM32 targets

Runs a real, unmodified AM32 firmware ELF on an emulated MCU, so that
the per-MCU register code under `Mcu/<target>/Src/` actually executes.
The SITL replaces all of that with its own fakes, which is why it could
not catch the PR #361 bug where two targets' `phaseouts.c` tested the
wrong variable, compiled cleanly and did nothing.

## Status

The F051 firmware boots, detects a servo throttle signal through real
capture and DMA, arms, and **spins a motor closed loop on BEMF** sensed
through the real comparator. The firmware's own measured
`commutation_interval` agrees with the rpm the physics reports, so the
timing it derives and the motor it derives it from check out
independently.

**Any of the 52 F051 targets works**, with no per-target file to write -
the platform is generated from `Inc/targets.h` on demand. They cover 8
distinct hardware configurations, differing in capture timer, DMA
channel, throttle pin, comparator map and bridge pin map. Five have been
run end to end, one per configuration reachable with a built ELF:

| target | capture | throttle | comparator A/B/C | phase A high |
|---|---|---|---|---|
| `FD6288_F051` | TIM15 + ch5 | PA2 | PA5 / PA4 / PA0 | PA10 |
| `ARK_4IN1_F051` | TIM3 + ch4 | PB4 | PA0 / PA4 / PA5 | PA10 |
| `RAZOR32_F051` | TIM15 + ch5 | PA2 | PA4 / PA5 / PA0 | PA9 |
| `DIATONE_F051` | TIM3 + ch4 | PB4 | PA5 / PA0 / PA4 | PA10 |
| `PB054_F051` | TIM3 + ch4 | PB4 | PA0 / PA5 / PA4 | PA10 |

All five arm and spin, and none needed a change to any peripheral model.

Calibration and sweep work stays in the SITL, on speed grounds - see
below.

## Running

Renode is not vendored; install it into the gitignored `tools/` tree the
same way the ARM toolchain is installed, then:

Renode is not vendored; install it into the gitignored `tools/` tree the
same way the ARM toolchain is installed. Then, for any F051 target:

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
    duty_cycle=611 commutation_interval=977 zero_crosses=1809 step=5
    bemf_timeout_happened=0 desync_happened=0
    rpm=2933.9 theta=5.33 ia=0.00 ib=3.12 ic=-3.12

The first line is firmware state read out of SRAM at each symbol's own
width; the second is motor truth from the physics. For a time series,
`sample()` between `RunFor` steps and `save('run.csv')` at the end, then
plot the CSV.

**There is no equivalent of the SITL GUI**, and one quantity it graphs
is not available here at all: the emulated ADC is Renode's stock model
and is *not* fed from the physics, so the voltage and current the
firmware believes it sees are meaningless in this harness. `ia`/`ib`/`ic`
above are what the motor is really drawing, not what the firmware
measures. Wiring an ADC model to the physics, as `Mcu/SITL/Src/ADC.c`
does for the SITL, is the missing piece.

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

Renode warns on every access to an unimplemented region. `IWDG` used to
be one, and the AM32 main loop kicks the watchdog constantly, so it
produced thousands of warnings per simulated second and buried anything
worth reading - `logLevel 3` in the test harness had been hiding it. It
now has a small model instead, which also counts the kicks:

    (monitor) python "print monitor.Machine['sysbus.iwdg'].Kicks"

The counter is accepted but never enforced: a watchdog that actually
fired would reset the CPU every time you paused at a breakpoint.

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

    gen_target.py                   builds a platform for any F051 target out
                                    of Inc/targets.h
    platforms/stm32f051_base.repl   MCU-common. Vendored from Renode's
                                    platforms/cpus/stm32f0.repl (Antmicro, MIT -
                                    header retained) and edited
    peripherals/stm32/              our peripheral models, GPL-3, loaded at
                                    runtime with `include @...cs`; no Renode
                                    rebuild needed
    scripts/am32_f051.resc          shared by every generated target script

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
error, because three things vary per target and **none of them fail
loudly when wrong**:

- the **comparator map**. `COMP->CSR[6:4]` selects PA4, PA5 or PA0, but
  which is phase A, B or C differs per target - six permutations across
  the F051 range. Wrong, and the firmware commutates against the wrong
  phase and merely runs badly.
- the **bridge pin map**. Most targets put phase A on PA10/PB1, but a
  third of them rotate the phases across the same six pins.
- the **capture timer and DMA channel**, TIM15 + channel 5 or TIM3 +
  channel 4.

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
3125, 6.1% apart. **That gap is currently unexplained.**

It was previously attributed here to dead time, the SITL target running
500ns against this one's 937ns (DEAD_TIME 45 at 48MHz). Adding the ARK
target refuted that, because it is the controlled experiment: same
model, same eeprom, same throttle, same settled duty (ARR 1999, CCR3
611), differing only in DEAD_TIME - 25 rather than 45, so 520ns rather
than 937ns, which is nearly the SITL's figure.

| target | DTG | dead time | rpm |
|---|---|---|---|
| FD6288_F051 | 45 | 937 ns | 2934.2252 |
| ARK_4IN1_F051 | 25 | 520 ns | 2934.3298 |

417ns of dead time is worth 0.1 rpm, 0.004%. Closing the gap would take
1500x that. The model's dead window opens both fets and lets the body
diode conduct, so with continuous current the phase stays clamped to a
rail and the applied volt-seconds barely move - physically reasonable,
and it means dead time is simply not a lever on steady-state rpm here.

So the 6.1% is still to be accounted for. It is not in the physics,
which is the same compiled object in both. Candidates not yet tested:
zero-cross detection timing through the comparator RC filters, the
`batchUs` register sampling interval, and differences in how the two
harnesses derive the PWM phase.

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
