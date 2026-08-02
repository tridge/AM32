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

Calibration and sweep work stays in the SITL, on speed grounds - see
below.

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
- **TIM1 and the SYSCFG/COMP page replaced.** The stock `STM32_Timer` has
  no MOE, no complementary outputs, no dead time and no preload
  shadowing; the COMP page was a bare tag, so `COMP1OUT` read back as
  nothing and BEMF sensing could not work at all.
- **Stock alternate-function connection blocks for `timer1` and
  `timer15` deleted.** A later connection block *replaces* an earlier
  one, so leaving them silently overrode the DMA and NVIC wiring in the
  peripheral declarations. This cost real debugging time: captures
  happened and went nowhere.

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
