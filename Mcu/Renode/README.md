# Renode emulation of AM32 targets

Runs a real, unmodified AM32 firmware ELF on an emulated MCU, so that
the per-MCU register code under `Mcu/<target>/Src/` actually executes.
The SITL replaces all of that with its own fakes, which is why it could
not catch the PR #361 bug where two targets' `phaseouts.c` tested the
wrong variable, compiled cleanly and did nothing.

**Scope: a boot/register-level harness, not a motor simulator.** See
"Why no motor" below — this is a measured decision, not an omission.

## Status

Bring-up. The F051 firmware boots to its main loop. No throttle input,
no comparator, no physics yet.

## Running

Renode is not vendored; install it into the gitignored `tools/` tree the
same way the ARM toolchain is installed, then:

    renode --disable-xwt --console \
      -e "\$repo=@/path/to/repo; \$elf=@obj/AM32_FD6288_F051_2.20.elf; include @Mcu/Renode/scripts/am32_f051.resc"

Paths must be absolute: the harness runs from a scratch directory, as
the SITL suite does.

## What is here

    platforms/stm32f051.repl   vendored from Renode's platforms/cpus/stm32f0.repl
                               (Antmicro, MIT - header retained) and edited
    peripherals/stm32/         our peripheral models, GPL-3, loaded at runtime
                               with `include @...cs`; no Renode rebuild needed
    scripts/am32_f051.resc     entry point

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

## Speed, and what it means for the motor

Measured on this tree, not estimated, with the `delayMillis` skip hook
active:

| phase | cost |
|---|---|
| boot and startup tune | 4.2x slower than real time |
| armed, zero throttle | 7.5x |
| armed, throttled, stepping for BEMF | 9.2x |
| host SITL, same machine | ~1.4x **faster** than real time |

The stepping figure understates a real spin: stuck-rotor protection has
already latched and zeroed `input`, so the bridge is not being driven
continuously. Expect worse once the comparator closes the loop.

That rules out calibration work here — a 60 s chirp would take about 9
minutes and the full suite days — but it does not rule out a motor.
Spinning one is worth it for what the SITL structurally cannot reach:
the real `phaseouts.c` and `comparator.c` register code driving real
physics. Calibration and sweeps stay in the SITL.

The physics will be **DllImported from `Mcu/SITL/sim/motor.c`, not
ported to C#**. `motor.c` was substantially rewritten recently, so a
fork would diverge on the next recalibration and leave two models with
no ground truth. Its physics core needs only `sitl_phase_mode[3]` and
the gate states; nearly every `extern` reference to a firmware global
sits in the logging functions, which can be stubbed or fed from
emulated SRAM.

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
