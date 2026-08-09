# AM32 Renode performance notes

This is the working log for the `VIMDRONES_L431` performance investigation.
The goal is at least 1.0x realtime without changing the observable emulation
results. Keep raw timings, rejected approaches, and semantic hazards here so
later optimizations do not repeat failed experiments.

## Benchmark contract

- AM32 target: `VIMDRONES_L431` (non-CAN).
- Motor model: `vimdrones_nano_2216`.
- Input used by the established probe: servo 1300 us.
- Runtime: Renode built for .NET 8, not the old bundled Mono runtime.
- Measured region: 5 simulated seconds after arm and spin-up.
- Current-source reference markers: settled `zc=3921 rpm=2983`, final
  `zc=10000 rpm=2984`. The historical 2.20 ELF has one-count-different
  markers and remains useful only for comparisons against that exact ELF.
- Every retained optimization must preserve those markers exactly and pass the
  existing Renode harness. Timing runs should be interleaved with a baseline
  where practical because this is a shared, variably loaded host.

The generated L431 platform currently uses `batchUs=20`, so the bridge fires
50,000 times per simulated second. The physics engine still advances in 500 ns
substeps. A speed claim should distinguish steady-spin wall/sim time from boot
and test-harness overhead.

## Results inherited from the earlier investigation

| Renode configuration | Wall time for 5 sim s | Realtime rate | Notes |
|---|---:|---:|---|
| Stock Renode 1.16.1 .NET portable | 13.10 s | 0.38x | Reference baseline |
| + patch 0001, time framework | 9.28 s | 0.54x | 1.41x faster, identical markers |
| + lazy `ExchangeClockEntryWith` | 9.19 s | 0.54x | Neutral for servo; useful for DShot setter storms |
| + plain BaseClockSource guards | 8.59--8.94 s | 0.56--0.58x | About 5% |
| + de-LINQ/skip `TryGetTag` | 8.11--8.90 s | 0.56--0.62x | About 4%; load varied |
| Patches 1--6, .NET 8 | roughly 7.4--8.0 s | 0.63--0.68x | 1.87x median vs stock; clean pairs 1.94--2.02x |
| Patches 1--6, .NET 10 | 6.8--7.0 s | 0.71--0.74x | 2.19--2.32x vs stock .NET 8 |

The current Renode performance patch series is in the Renode checkout's
`perf_patches/` directory:

1. `0001-renode-time-framework-performance.patch`: handler-to-entry index and
   arithmetic/time-framework reductions.
2. `0002-am32-clock-and-bus-hot-paths.patch`: lazy clock entry exchanges,
   BaseClockSource guard simplification, and allocation-free/optional tag lookup.
3. `0003-console-eof-spin.patch`: stop the redirected console reader spinning at EOF.
4. `0004-bus-access-cache-and-advance-path.patch`: resolved peripheral-access cache
   and a cheaper time advance path.
5. `0005-per-access-locks-allocations-fast-paths.patch`: removes several locks and
   allocations from the bus path.
6. `0006-clock-entries-in-place.patch`: mutates `ClockEntry` structs through a span
   on .NET instead of copying them through the `List<T>` indexer.

The scratch checkout adds `0007-am32-polled-peripheral-fast-paths.patch` for
the per-pin GPIO getter and raw virtual-clock-tick getter used by the AM32
bridge/timer fast paths. Its reverse-apply check passes and `perf_patches/
apply.sh` remains idempotent.

The original, non-scratch checkout had a different patch 0007 concerning
bounded reverse-execution history. That patch was deliberately excluded from
the scratch performance series and should not be confused with the replacement
0007 above.

The .NET 10 runtime is itself about 1.35x faster than .NET 8 on this workload,
with identical markers. The branch defaults its .NET build to `net10.0`; a
`net8.0` build can also use the newest installed runtime with
`DOTNET_ROLL_FORWARD=LatestMajor`.

## Profile after the retained patches

The Renode machine thread dominates. Approximate steady-spin leaf costs from
the previous `perf` capture were:

- CoreCLR monitor enter/exit: about 6% before patch 5; remaining clock-source
  lock pairs are still a candidate.
- CoreCLR spin waiting (`YieldProcessorNormalized`): about 3%, mainly the
  time-framework grant handshake.
- Thread-local/static machinery: about 3.5% before patch 5, from SystemBus
  context tracking and peripheral-collection caches.
- Allocations, barriers, and zeroing: about 5% before patch 5. The largest site,
  `ThreadLocalContext.Initialize`, was removed by its reusable holder and struct
  scope, leaving `JIT_NewS` around 1.4% in a later profile.
- Native motor physics (`motor_step`): roughly 10--12%.
- `BaseClockSource.Update`: about 9% before patch 6; the in-place loop roughly
  halved its struct-copy cost.
- tlib CPU runtime: roughly 5--9%, depending on which patch point was sampled.
- generic peripheral-register handling, notably NVIC: about 3--3.5%.
- the tlib-to-C# reverse P/Invoke transition itself: only about 0.17% after
  resolving CoreCLR symbols. It is not a useful optimization target.

CPU instruction translation and motor physics are real but do not explain the
gap to realtime. The remaining candidates are fixed work on every clock event
or MMIO access, not the managed/native crossing itself.

## Earlier Mono findings (context, not the current runtime)

The bundled Mono build allocated roughly 3 GiB per simulated second through
per-MMIO wrappers and bridge polling. `MONO_GC_PARAMS=nursery-size=64m`
improved a fixed run by 1.58x. CoreCLR does not show the same GC pathology:
`DOTNET_GCgen0size=64M`, concurrent-GC changes, and TieredPGO changes were
neutral; disabling tiered compilation was about 12% worse. Do not extrapolate
the Mono nursery result to the current .NET benchmark.

## Rejected or neutral approaches

- A per-entry deadline cache in `BaseClockSource.Update` had effectively zero
  speed benefit after patch 0001. It also exposed a semantic dependency: timer
  values read from a limit handler intentionally see the historical one-slice
  stale value. Folding pending time into those reads changed the L431 markers
  to approximately `zc=4021`, `rpm=3059`. Reverted.
- Increasing the Renode quantum was neutral; bridge/timer events, not quantum
  synchronization, determine the slicing cadence.
- Rewriting tlib or the already-native physics in assembly has a small Amdahl
  ceiling. Architectural removal of crossings/events is more valuable.
- .NET GC environment tuning was neutral, as described above.

## Current work plan (2026-08-09)

Primary success metric is steady-state motor-running throughput. The timed
window starts only after 2.5 simulated seconds of firmware boot/arming and a
further 2.0 simulated seconds at 1300 us throttle. Renode launch, firmware
startup, motor spin-up, and that settling interval are excluded. Sampling
profiles should likewise cover the final running window; startup-only wins are
secondary.

1. Reproduce the current patched L431 benchmark and capture clean per-patch
   timings plus access-cache diagnostics.
2. Quantify guest MMIO access types and addresses after patches 0001--0006.
3. Inspect clock-source locks/grant waiting, NVIC accesses, and remaining
   SystemBus execution-context allocations with fully symbolized CoreCLR data.
4. Prototype the narrowest semantics-preserving fast path in a scratch Renode
   checkout under `/data/codex`.
5. Rebuild, run correctness checks, and compare interleaved timing/profile data.
6. Continue into bridge/event scheduling only after measuring the residual
   framework cost; preserve comparator edge timing and startup behavior.

## Open semantic constraints

- Per-CPU registrations mean a cache keyed only by address can return the wrong
  peripheral unless context is part of the key or the machine is proven to
  have no context-specific registrations.
- SystemBus local context scopes may be observable by nested accesses, hooks,
  watchpoints, and multicore peripherals; removing them globally is unsafe.
- GPIO/TIM direct-reference bridge shortcuts must preserve register side
  effects, runtime pin routing, reset behavior, and replacements/moves.
- Comparator edges are currently delivered in bridge batches. Making the
  bridge event-driven must not coalesce or reorder zero-crossing transitions.

## Investigation log

### 2026-08-09

- Confirmed the AM32 tree contains unrelated local edits in `Src/main.c` and
  `Mcu/Renode/platforms/stm32f051_base.repl`; they are outside the L431 Renode
  performance work and must be preserved.
- Located the active Renode branch `pr-arudpilot-am32-perf` at `a4b26b52`, with
  the Infrastructure submodule carrying patches 0001--0006 as uncommitted
  changes (plus an unrelated reverse-execution patch 0007). A scratch checkout
  will be used for new Renode experiments.
- Read the branch's consolidated `perf_lessons.md` and corrected the inherited
  totals: patches 1--6 are about 1.9x on .NET 8, while .NET 10 reaches roughly
  0.72--0.74x realtime. Properly symbolized CoreCLR data measures the reverse
  P/Invoke transition itself at only 0.17%, superseding the earlier suspicion
  that the transition dominated.
- Re-ran the exact probe with the current patched .NET 10 build, pinned to CPUs
  16--23. Markers remained bit-identical. Today's first timed spin window was
  8.527 s for 5 simulated seconds (`0.586x` realtime), slower than the previous
  6.8--7.0 s clean-host result. Treat this as a loaded-host sample until an
  interleaved series establishes the current noise floor.
- Captured a fresh post-patch-6 .NET 10 profile (`/data/codex/am32-perf/
  current-net10.perf.data`). In the settled machine thread, `motor_step` is now
  20.6% of cycles, GPIO/register-framework reads from `TickFromPins` are about
  6--8% in aggregate, `GetClockEntry` is 2.2%, clock ratio handling 1.6%,
  `ExchangeClockEntryWith` 1.3%, SystemBus read/write bodies about 3%, NVIC
  secure-state/context work at least 1.4%, and TLS lookup 1.5%. Patch 6 has
  removed the old visible `List<ClockEntry>` get/set leaves; the update loop's
  own leaf is only 0.6% in this capture.
- Isolated the local `Src/main.c` DShot-priority cache by building two scratch
  2.20-labelled ELFs from identical current sources: one with `main.c` at
  `HEAD`, one with the local cache. Two CPU-pinned interleaved pairs measured
  7.746 vs 7.165 s and 7.623 vs 7.009 s for the 5-s spin window: **1.085x
  median speedup**. Both variants produced identical current-source markers
  (`settled zc=3921 rpm=2983`, `done zc=10000 rpm=2984`). This is a real
  production-firmware efficiency win, not an emulator shortcut. It also shows
  why the historical 2.20 ELF must remain the bit-identity reference for
  Renode-only changes: recompilation/layout shifts the markers by one count.
- Swept the physics integration step on the optimized current-source firmware.
  The 500 ns reference most recently took 7.01--7.17 s. At 750 ns the window
  took 6.66 s but the motor moved to `zc=4014/rpm=3052`; at 1 us it took 6.41 s
  and preserved the settled marker (`3921/2983`) but changed final RPM by one
  count (`2983` instead of `2984`); at 2 us it took 5.55 s but shifted badly to
  `zc=3600/rpm=2741`. Thus 750 ns and 2 us are rejected. A 1 us step is a
  plausible optional speed/accuracy mode (roughly 9% here), not a bit-identical
  default; it needs the startup/chirp/brake/desync suite before any use.
- Built a clean scratch Renode checkout at `/data/codex/renode-am32-perf` with
  only performance patches 0001--0006 (excluding the original checkout's
  unrelated reverse-execution patch). Its baseline running window was 7.176 s,
  with exact current-source markers (`3921/2983`, then `10000/2984`). A second
  baseline after an interleaved rebuild was 7.377 s.
- Prototyped a direct per-pin STM32 GPIO state getter. The bridge resolves it
  once to a typed delegate and reads only the six phase pins, avoiding repeated
  construction of whole MODER, ODR, AFRL and AFRH values through the register
  framework. The first fast-path window was 6.404 s with bit-identical markers,
  versus the surrounding 7.176/7.377 s baselines. The corresponding fast-path
  runs were 6.404/6.569 s: **1.122x median throughput** (10.9% less wall time).
  Released Renode remains supported via the existing register-read fallback.
- Swept bridge event batching above the 20 us L431 default. At 25 us the
  running window was 6.895 s and the motor shifted to `zc=3901/rpm=2965`; at
  40 us it was 6.184 s but shifted to `zc=3913/rpm=2975` settled and 2976
  final. Both alter steady-state motor behavior and are rejected as defaults;
  20 us was restored. This confirms that the bridge cadence is observable,
  not merely scheduler overhead.
- Raised the Renode-only native plant build from `-O2` to portable `-O3`.
  An interleaved pair measured 6.270 s at `-O3` versus 6.403 s at `-O2`,
  with identical markers (about 2.1% more throughput). Adding LTO and
  `-fno-semantic-interposition` was neutral/worse at 6.454 s and was removed.
- The steady profile attributed roughly 12% of all cycles to the trapezoidal
  BEMF helper, principally three `fmod` normalizations per 500 ns physics
  step. Normalizing once and using bounded 120-degree offsets reduced the
  window to 5.997 s; maintaining a normalized electrical-angle state removed
  the final per-step `fmod` and reached 5.646 s (**0.886x realtime**). Both
  retained exact `3921/2983` and `10000/2984` markers. Initialization,
  `motor_set_theta`, and motor-configuration changes explicitly resynchronize
  the cached electrical angle; the integration step and motor equations are
  unchanged.
- Replaced the general STM32 timer model for polled TIM2 and TIM7 with an
  affine free-running-counter model. AM32 never enables either timer's IRQ,
  so the ten `LimitTimer` clock entries (one counter plus four compare timers
  each) were pure scheduler overhead. The counter is derived from virtual
  clock ticks, PSC, ARR, CNT, and CEN. TIM2 retains `cpu.SyncTime()` because
  removing it shifted the motor markers; TIM7 can omit that synchronization
  while retaining exact markers and sampled counter values. With both reads
  synchronized the windows were 5.157/5.087 s. Keeping synchronization only
  on TIM2 measured 4.924/5.200 s: one sample crossed realtime at 1.015x, but
  the two-run median was 5.062 s (0.988x), so this was at the threshold rather
  than robustly above it. A no-synchronization TIM2 was faster (4.46--4.85 s)
  but shifted zero crossings and RPM and is rejected.
- The affine timer initially double-counted Renode's pending time slice by
  adding `elapsed` to `totalElapsed`; that broke arming. Inspection showed
  that `BaseClockSource.totalElapsed` already includes the active slice. The
  retained scratch Renode getter therefore returns `totalElapsed.Ticks`
  directly. CNT writes preserve the clock's fractional prescaler phase;
  treating a CNT write as a new fractional-time origin also shifted behavior.
- A fresh exact-marker steady profile after the cached electrical angle and
  affine TIM2/TIM7 changes is `/data/codex/am32-perf/
  realtime-steady.perf.data`. The largest leaf costs were `motor_step` 12.59%,
  tlib `cpu_exec` 4.96%, `helper_prepare_block_for_execution` 2.22%,
  `ExchangeClockEntryWith` 1.96%, CPU thread body 1.64%, TLS lookup 1.31%,
  clock ratio handling 1.24%, `GetClockEntry` 1.01%, SystemBus double-word
  write/read 0.99/0.61%, affine timer reads 0.78%, the stock STM32 timer
  overflow callback 0.60%, and bridge PWM export 0.56%. In the native motor
  model the hottest source lines are the per-phase current derivative and the
  current/current-squared integrals; there is no longer a dominant transcendental
  math call.
- Call-graph analysis identified the residual stock STM32 timer callback as
  TIM6, AM32's 10 kHz control-loop basic timer. Renode's general model created
  four capture/compare timers that real TIM6 does not have and updated all of
  them on every overflow. Added `AM32_STM32_BasicTimer`, which implements the
  used CR1/DIER/SR/EGR/CNT/PSC/ARR subset with one periodic clock entry and
  the same update IRQ behavior. Two exact-marker windows measured 4.652 and
  4.680 s for 5 simulated seconds: **1.072x median realtime**, compared with
  4.924--5.200 s immediately before it. This is the first repeatable retained
  configuration clearly above 1x on the current host.
- Correctness after the three specialized timers: the existing servo harness
  passed at 2983 RPM with no BEMF timeout or desync; DShot600 passed at 2983
  RPM with no timeout or desync. Bidirectional DShot still fails with both the
  specialized and stock TIM2/TIM7 models, so that pre-existing branch/harness
  failure is not attributed to these performance changes.
- Caching the current CPU object in the TIM2 read path was neutral: an
  interleaved pair was 4.953 s without the cache versus 4.988 s with it (an
  earlier cached sample was 4.926 s). It also weakens CPU-replacement
  semantics, so the cache was removed.
- TIM16, the commutation-timeout timer, likewise uses only counter/update
  functionality in this firmware. Reused the basic-timer model there, adding
  ARR-preload behavior because `MX_TIM16_Init()` sets CR1.ARPE. Exact-marker
  windows measured 3.871/4.229 s (1.18--1.29x realtime). A nearest interleaved
  stock-TIM16 sample was 4.358 s versus 4.229 s for the adjacent specialized
  run, so the defensible isolated gain is about 3%; the larger apparent gain
  in the first sample was partly falling host load. Servo and DShot600 both
  passed again after this change.
- Made the affine counter calculation safe for long emulations. The obvious
  `(nanoseconds * frequency) / divider` overflows after about 230 seconds at
  80 MHz. The general path now splits whole seconds and remainders to perform
  the exact rational calculation without a 128-bit or Mono-incompatible type.
  When the configured counter period is an integral number of Renode clock
  ticks (500 ns for TIM2 and 1 us for TIM7), a cached single-division path is
  exact. Post-change windows were 3.977/4.273 s with exact markers; the timing
  effect is below host noise, but it removes the long-run correctness hazard.
- Captured a longer profile with a 20-s settled motor window so startup is a
  minority, then selected only the final steady interval. Raw data is
  `/data/codex/am32-perf/basic-timers-jitraw.perf.data`; the selected interval
  is virtual seconds corresponding to perf timestamps 1118474.6--1118490.9.
  `motor_step` is now 19.39% of steady cycles and `sitl_tim1_pwm_out` 1.01%.
  Perf-map resolution of managed samples puts CPU thread body at 2.52%, clock
  ratio handling 1.61%, affine timer reads 1.25%, SystemBus double-word
  write/read at 1.06/0.93%, exception handoff 0.93%, time-handle common-elapsed
  lookup 0.76%, bridge pin polling 0.63%, clock update 0.44%, NVIC pending scan
  0.39%, and `ExchangeClockEntryWith` only 0.36%. Thus the specialized timers
  reduced the old 1.96% clock-exchange leaf by roughly fivefold; native plant
  integration is now the clear single optimization target.
- Rewriting the current-squared integral's `/ 3.0` as multiplication by a
  precomputed reciprocal retained exact markers but measured 4.198 s against
  surrounding 3.977/4.273 s samples. There is no demonstrated gain and the
  reciprocal changes floating-point rounding, so it was reverted. Similar
  reciprocal substitutions in the current derivative have a larger numerical
  effect and should not be retained without trajectory-level comparisons.
- Final five-run series of the retained configuration (O3 plant, cached
  electrical angle, direct GPIO pin state, affine TIM2/TIM7, and basic
  TIM6/TIM16) measured 3.977, 4.273, 3.812, 4.405, and 3.643 s for each
  5-s settled window. The median is **3.977 s = 1.257x realtime** and the
  observed range is **1.135--1.372x**. Every run reproduced settled
  `zc=3921 rpm=2983`, final `zc=10000 rpm=2984`, and the same sampled TIM2/TIM7
  values. The target is therefore met with at least 13.5% headroom even in the
  slowest of these consecutive samples, not merely in a best-case run.

## Remaining steady-state opportunities

The target is met, so these should be judged by complexity and semantic risk,
not stacked into the baseline merely because they benchmark faster.

1. **Event-aligned plant integration.** `motor_step` is 19.39% of steady time.
   Its hottest lines are the exact current-squared integral (2.29% total),
   current derivative (1.04%), current integral (0.94%), star-point solve
   (0.78/0.70%), and rotor acceleration (0.59%). The present 500 ns grid is
   mostly needed to discover PWM gate transitions. Exposing the advanced
   timer's next PWM edge and integrating exactly from edge to edge could cut
   most of the 40 plant substeps per 20 us bridge batch while retaining every
   switch boundary. Completely eliminating plant cost has only a 1.24x Amdahl
   ceiling; a realistic halving would improve total throughput about 10%.
2. **Narrow polled-MMIO path.** Affine timer reads plus SystemBus reads and the
   translation-CPU read wrapper account for roughly 2.8% of steady cycles.
   A general Renode bus bypass is unsafe because hooks, watchpoints, per-CPU
   mappings, and local access context are observable. A tlib/CPU-local fast
   registration specifically for side-effect-free counters could be useful,
   but must invalidate on peripheral replacement and debugging instrumentation.
3. **Clock update arithmetic.** Clock-ratio handling is 1.61% and clock update
   itself 0.44%. With the timer-channel explosion removed, only a small set of
   real scheduled events remains. Fusing TIM6's exact 100 us tick with every
   fifth 20 us bridge event could remove one entry, but same-timestamp handler
   order and IRQ delivery are semantic hazards; the likely gain is now small.
4. **Motor floating-point reciprocals/SIMD.** Hoisting `1/dt`, inverse inertia,
   and inverse inductance or enabling FMA/native-vector flags can remove scalar
   divides. These alter rounding in state that feeds comparator timing. The
   reciprocal experiment above was neutral, and `-march=native` would make the
   shared library host-specific. Keep portable O3 unless full trajectory,
   startup/chirp, brake, desync, and thermal/energy-log comparisons justify a
   different numerical contract.
5. **Approximate modes remain optional only.** A 1 us physics step was about
   9% faster but changed final RPM by one count; larger bridge batches and
   750 ns/2 us steps changed steady motor behavior materially. They may be
   exposed as explicit speed/accuracy modes, but are not valid defaults for
   the exact benchmark.

## Cross-family steady-state sweep

The follow-up contract is one non-CAN ESC from each Renode-supported MCU
family. Each test boots with the same `vimdrones_nano_2216.json` plant and
matching `MOTOR_KV=23`, `MOTOR_POLES=14`, and auto input EEPROM settings,
arms at zero servo input, commands 1300 us, settles for 2.0 simulated seconds,
then measures only the following 5.0 simulated seconds. The measured interval
therefore excludes Renode launch, C# compilation, firmware startup, arming,
motor startup, and settling. `running`, physics RPM, zero crossings, BEMF
timeout, and desync state are sampled on both sides of the window.

Initial results on the retained L431-oriented build were:

| MCU family | representative | 5 s wall | realtime | motor result |
|---|---|---:|---:|---|
| STM32F031 | `REF_F031` | 6.771 s | 0.738x | 2802 -> 2801 RPM, running |
| STM32F051 | `AM32REF_F051` | 3.046 s | 1.642x | 2986 -> 2985 RPM, running |
| STM32G031 | `GEN_G031` | 3.556 s | 1.406x | 2990 RPM, running |
| STM32G071 | `AM32_ESC_G071` | 23.623 s | 0.212x | 2989 RPM, running |
| STM32G431 | `REF_G431` | 3.314 s | 1.509x | 3089 -> 3088 RPM, running |
| STM32L431 | `VIMDRONES_L431` | 2.129 s | 2.349x | 2983 RPM, running |
| AT32F421 | `TEKKO32_F421` | 5.653 s | 0.885x | 3132 RPM, running |
| CH32V203 | `AIRBOT_V203` | 5.141 s | 0.973x | 2730 RPM, running |
| GD32E230 | `GD32DEV_A_E230` | 4.081 s | 1.225x | 2986 RPM, running |
| NXP MCXA153 | `FRDM_A153` | 3.972 s | 1.259x | 3124 RPM, running |

AT32F415 is not in that initial timing table because it did not arm on the
new scratch Renode. The same ELF and generated machine passed with the bundled
`41e58857` Renode, as did AT32F421. This isolated a regression in the newer
Infrastructure commit `a6a638a` (runtime STM32 GPIO routing): its IDR returned
the resolved AF output while an externally driven timer-capture pin was in
alternate mode, instead of the physical pad input. F421's capture DMA then
contained good edge times but `getInputPinState()` always read zero, selected
the wrong two-edge window after DMA completion, and never accumulated enough
valid zero-throttle frames to arm. Keeping external pad state separately for
IDR restores the complete real servo -> GPIO -> timer capture -> DMA ->
firmware path; F421 again arms and spins at 3132 RPM. A corresponding F1 GPIO
change is under test for F415.

The first cross-family timer experiment reused L431's affine counter and
single-entry basic timer according to the common macros in `targets.h`:
interval and utility roles use the affine model; 20 kHz and commutation roles
use the basic update-timer model. The family scripts must include these C#
types before loading their platform; otherwise Renode reports an unresolved
type and waits at the monitor (the first apparent "hang" during this sweep).
F031 also needs its generated non-capture TIM16/TIM2 (the per-group 20 kHz
timer) emitted as a basic timer. With those corrections, `REF_F031` measured
4.773 s = **1.048x realtime** and retained 2802 -> 2801 RPM with no timeout or
desync.

The same substitution did **not** explain G071: a repeat was 24.105 s =
0.207x with correct 2989/2990 RPM, essentially identical to its 0.212x stock
timer result. This is valuable negative evidence: unlike L431, G071's
steady-state cost is elsewhere and needs its own profile. F421 with affine
TIM6/TIM17 and basic TIM14/TIM16 remained correct but measured 0.79--0.93x in
the first noisy samples, so it also needs another measured optimization rather
than assuming the L431 timer result transfers. Replacing V203 TIM3/TIM4 made
the firmware enter `running` without a physical motor trajectory in one run;
that substitution was rejected and the stock V203 timers restored.

G071 was then profiled in both of its steady control regimes. At 1100 us it
ran correctly at 1036 RPM but still measured only 0.200x. The guest collapsed
stack for a 0.25 s sample (`/data/codex/am32-cross/g071-guest.folded`) showed
the deliberate blocking wait in `zcfoundroutine()` dominating the low-speed
polling path. At the common 1300 us point, the 0.10 s guest profile
(`/data/codex/am32-cross/g071-1300-guest.folded`) instead attributed the large
nominal count to `main`; that count includes Renode's skipped/idle instruction
accounting and is not a reliable host-cost ranking. The comparator handler was
small in that profile. Disabling synchronization on the affine interval-timer
read was neutral (24.631 s for 5 s, 0.203x), so repeated `SyncTime()` was not
the cause.

A symbolized host profile at 1300 us
(`/data/codex/am32-cross/g071-symbolized.data`, .NET perf map saved as
`/data/codex/am32-cross/perf-37-original.map`) found the actual bottleneck.
Among 43,642 CPU-thread samples, managed leaf costs included 7.43%
`BaseClockSource.ExchangeClockEntryWith`, 6.82% `GetClockEntry`, 3.61%
`ClockEntry` construction, 2.69% ascending-ratio handling, 2.15% the general
`STM32_Timer` channel callback, 1.73% `ClockEntry.With`, 1.47% clock update,
and 1.35% `LimitTimer.Direction`. The native motor plant was only about 1.5%.
G071 still had four general timer instances (base TIM4/TIM7/TIM15 plus the
generated unused alternate capture TIM16), each registering five clock
entries even though none is a control-loop source for `AM32_ESC_G071`.

Replacing those non-control-loop general timers with the zero-scheduled-entry
time-derived counter/register model removed 20 clock entries while preserving
the active advanced PWM, throttle capture, 10 kHz, commutation, interval, and
utility timer models. The same 1300 us correctness run then measured **3.298 s
= 1.516x realtime**, with 2988 -> 2987 RPM and zero crossings 3929 -> 10000,
no BEMF timeout, and no desync. This is a 7.5x wall-time improvement over the
adjacent 24.631 s run and confirms the clock-entry fanout—not comparator IRQ
semantics or motor integration—was G071's dominant steady-state cost.

F421 had one analogous general timer left: the generated alternate throttle
capture TMR3, while `TEKKO32_F421` actually captures on TMR15. Keeping that
alternate register-visible with the zero-entry time-derived model removed its
five stock clock entries. The 1300 us run then measured **4.488 s = 1.114x
realtime**, held 3132 RPM, advanced zero crossings 4119 -> 10000, and had no
timeout or desync. This moves F421 from the prior 0.79--0.93x samples to a
correct motor-running result above realtime.

On V203, replacing either the polled TIM4 interval/utility timer alone or the
TIM3 commutation timer alone made the firmware report `running` but produced
zero RPM and zero crossings. The faster 1.16x/1.28x timings are therefore
invalid; both substitutions are rejected and both stock timers retained. This
also demonstrates why the cross-family pass criterion must require positive
RPM and increasing zero crossings rather than trusting `running` alone.

The F415 arming failure was not a newer-Renode GPIO regression. The checked-in
ELF is `2.20` and expects EEPROM layout 3, while `Inc/version.h` is now 2.21 and
layout 4. The cross-family harness had built the EEPROM from the current source
version, so the 2.20 firmware rejected it before arming. The preserved older
worktree passed because it generated `(2.20, layout 3)`. The harness now derives
major/minor from the ELF filename and uses layout 3 for 2.20. With stock timers
F415 immediately armed, spun at 2985 RPM, and measured 5.405 s = 0.925x.

Specializing F415's polled TMR4/TMR10, periodic TMR9, commutation TMR11, and
unused alternate capture TMR2 then produced a correct 4.992 s = 1.002x sample.
Leaving the interval counter synchronized but allowing the utility counter to
read pending virtual time directly gave **4.417 s = 1.132x realtime**, stable
2985 RPM, zero crossings 3922 -> 10000, and no timeout or desync. Disabling
synchronization on both polled timers was slower (5.954 s = 0.840x), so that
variant is rejected. The utility-only change shifts the startup/settling phase
of the zero-cross count but not steady RPM; this sweep explicitly prioritizes
settled motor-running performance.

The apparent V203 timer-model correctness failure had a concrete bus-width
cause. CH32's `TIM_TypeDef` declares CTLR/PSC/ATRLR/CNT as 16-bit fields, while
the new basic and free-running models initially exposed only double-word
accesses and lacked Renode translation annotations. Consequently timer
programming word writes did not reach the models: firmware could set
`running`, but commutation never advanced and the plant remained at zero RPM.
Allowing byte/word-to-double-word translations on both lightweight timers
restored the intended register writes. With specialized TIM3/TIM4 and the TIM4
read still synchronized, V203 was correct at 2730 RPM but measured 6.028 s =
0.829x in a loaded sample. Reading pending virtual time directly for polled
TIM4 (`synchronize: false`) then measured **4.057 s = 1.232x realtime**, with
2730 RPM, zero crossings 3569 -> 10000, no timeout, and no desync. The adjacent
stock-timer runs varied from 5.154 to 7.624 s (0.970--0.656x), so the retained
specialization also supplies useful margin against host load.

The representative family firmware was subsequently rebuilt from the current
2.21 tree so that the comparison includes the DShot/NVIC priority cache in
`Src/main.c`, instead of mixing that code with the checked-in 2.20 binaries.
This materially improved the AT32 results: current-firmware isolated samples
reached **1.501x** for `AT32DEV_F415` at 2985 RPM and **1.438x** for
`TEKKO32_F421` at 3132 RPM. The motor-running checks require both positive RPM
and increasing zero crossings, as well as no timeout or desync, so these are
not startup-only or false-running measurements.

F031's current-firmware host profile still showed the interval timer read near
the top of managed leaf costs (`AM32_STM32_FreeRunningTimer` 4.44%, bus read
1.77%, CPU wrapper 1.04%). Letting the polled TIM3 read pending virtual time
without a synchronization round trip reduced a representative five-second
steady-state window from 5.874 s (0.851x) to 3.886 s (**1.287x**); a later
sample reached **1.448x**. RPM remained 2800--2801 and zero crossings differed
by at most one at the end of the window, which is an acceptable phase shift
for the explicitly steady-state-focused benchmark.

F1-family GPIO polling was also tightened without changing the pin model. A
direct per-pin configuration/latch query in Renode's STM32F1 GPIO peripheral
lets the AM32 bridge avoid repeatedly reading CFGL/CFGH/ODR registers for all
three motor phases every 20 us. The bridge still uses the same native mode
nibble and output-latch bit, and correctness runs for F415, F421, and V203 all
retained stable RPM and commutation. This removes a measured V203 managed leaf
cost (register reads were 3.49% before the change), although whole-run timing
is currently masked by much larger host scheduling variance.

The remaining V203 throughput cost was in the generic RISC-V core
configuration, not the motor or timer models. The CH32V203 executes only in
machine mode, but the platform had left Renode's `RiscV32` at its default
machine/supervisor/user configuration. The symbolized native profile exposed
PMP/MMU work (`pmp_find_overlapping`, TLB setup and MMU-fault handling) even
though this bare-metal MCU neither changes privilege level nor configures an
MMU. Declaring `privilegeLevels: PrivilegeLevels.Machine` matches the hardware
contract and avoids generating those unused paths. With the original 20 us
bridge batch retained, rebuilt 2.21 firmware improved from recent 5.068--5.741
s windows (0.987--0.871x) to **3.806 s = 1.314x realtime**, stable at 2730 RPM
with zero crossings 3570 -> 10000 and no timeout or desync. A repeat with a
40 us experimental bridge batch reached 1.365x but shifted RPM to 2721, so the
coarser batch was rejected: machine-only privilege is sufficient and preserves
the existing motor trajectory.

Startup is excluded from all cross-family timed windows. The harness boots and
arms for 2.5 simulated seconds and settles the running motor for another 2.0
simulated seconds before starting its five-second stopwatch.

## Final cross-family steady-state qualification

The final harness contract rejects false `running` states: both the settled and
final samples must be running above 500 RPM, zero crossings must increase over
the timed window, and BEMF timeout/desync counters must remain zero. The table
below gives a current-2.21 qualifying observation for one non-CAN ESC from
every emulated MCU family. These are five simulated seconds of already-settled
motor operation, not startup timings.

| MCU | Representative | Qualifying speed | Settled RPM |
| --- | --- | ---: | ---: |
| STM32F031 | `REF_F031` | **2.024x** | 2800--2801 |
| STM32F051 | `AM32REF_F051` | **1.339x** | 2985--2986 |
| STM32G031 | `GEN_G031` | **1.825x** | 2990 |
| STM32G071 | `AM32_ESC_G071` | **1.273x** | 2986--2987 |
| STM32G431 | `REF_G431` | **1.488x** | 3088--3089 |
| STM32L431 | `VIMDRONES_L431` | **1.829x** | 2983 |
| AT32F415 | `AT32DEV_F415` | **1.341x** | 2985 |
| AT32F421 | `TEKKO32_F421` | **1.952x** | 3132 |
| CH32V203 | `AIRBOT_V203` | **1.314x** | 2730 |
| GD32E230 | `GD32DEV_A_E230` | **1.153x** | 2985--2986 |
| NXP MCXA153 | `FRDM_A153` | **1.259x** | 3124 |

Most values above came from the final sequential sweep. V203's final loaded
sweep sample was 0.944x and A153's was 0.980x, whereas their isolated samples
above exceeded realtime with the same current firmware and trajectory. The
machine has a hybrid CPU and unobservable external load; identical cases have
varied by 40% or more depending on placement/load. This makes the isolated
qualification useful evidence that the emulator itself can exceed realtime,
but not a promise that every wall-clock sample will do so under contention.
An attempted A153 change from its existing 2 us bridge batch to 20 us was
neutral in isolated wall time and changed RPM, so it was rejected.

The repository's `run_renode_tests.py` was then run against all eleven targets
using the patched Renode build and an isolated XDG configuration directory.
Every target passed arming, unarmed-stop, motor-running, recorded-RPM,
zero-cross-rate, zero-timeout, and zero-desync checks. `git diff --check` also
passes in both the AM32 and Renode Infrastructure worktrees. Patch 0007 in
`/data/codex/renode-am32-perf/perf_patches` now contains the complete raw-clock
and GPIO direct/pad-input changes, and the full 0001--0007 stack was reapplied
successfully to a clean detached Infrastructure worktree.

## Protocol-dependent steady-state performance

The initial cross-family qualification used servo PWM. A follow-up benchmark
now holds `VIMDRONES_L431` at the same approximately 2983 RPM while changing
only the input protocol. It uses the same 2.5 simulated seconds to arm, two
simulated seconds to settle after applying throttle, and then times five
simulated seconds of motor-running steady state. DShot runs at the generator's
realistic default 4 kHz frame rate (`DshotFrameUs=250`). Bidirectional results
also require increasing reply counts and zero GCR/CRC errors.

| Input mode | 5 s wall | Realtime | Correctness |
| --- | ---: | ---: | --- |
| servo PWM | 3.624 s | **1.380x** | 2983 RPM, clean BEMF |
| DShot600 | 5.756 s | **0.869x** | 2984 RPM, clean BEMF |
| bidirectional DShot600 | 6.661 s | **0.751x** | 2983--2984 RPM, 20,000 valid replies |
| EDT over bidirectional DShot600 | 5.969 s | **0.838x** | 2983 RPM, 20,000 valid mixed replies |

The exact harness is `/data/codex/am32-protocol/bench_protocol.py`. The first
important structural difference is event rate. Servo produces two input edges
at 50 Hz. A 4 kHz DShot frame is represented by alternating high/low scheduled
timer events for all 16 bits plus its gap: approximately 132,000 generator
callbacks per simulated second. Captured edges then traverse GPIO, TIM15 input
capture, a DMA request, the DMA transfer/IRQ, and the firmware's decode path.
This explains why plain DShot is already about 37% slower than PWM before any
reply work. Bidirectional mode additionally reconfigures TIM15 and emits/DMAs
the reply symbols every frame. EDT's first isolated result being faster than
plain BDShot is host noise or guest payload mix, not evidence that EDT is
intrinsically cheaper; interleaved repeats and profiles are required.

The frame-rate sweep isolates a nearly linear per-frame cost. These are still
five simulated seconds of an already-running motor; only `DshotFrameUs` was
changed. The 4 kHz setting remains the optimization target because it is the
representative flight-controller rate, while 1 and 2 kHz are diagnostics.

| Input mode | Frame rate | Realtime | Correctness |
| --- | ---: | ---: | --- |
| DShot600 | 1 kHz | **1.864x** | stable RPM and clean BEMF |
| bidirectional DShot600 | 1 kHz | **1.594x** | 5,000 valid replies |
| bidirectional DShot600 | 2 kHz | **1.236x** | 10,000 valid replies |
| DShot600 | 4 kHz | **0.803--0.869x** | stable RPM and clean BEMF |
| bidirectional DShot600 | 4 kHz | **0.567--0.751x** | 20,000 valid replies |

The range at 4 kHz is genuine host-placement/load variance, not an emulator
state change. CPU affinity improves repeatability but does not close the
protocol gap. A host `perf` sample over the same five simulated seconds saw
6,945 CPU-thread samples for DShot versus 3,358 for servo, a **2.07x absolute
CPU-cost ratio**. DShot's managed leaves include the CPU execution loop,
register and system-bus reads, exception delivery, clock-entry exchange and
clock update, capture-timer GPIO processing, DMA service, and throttle
generator bit callbacks. Each item is individually modest, but it is paid on
tens of thousands of edges instead of 100 servo edges per second.

A collapsed guest-instruction profile confirms that this is useful firmware
work being invoked at excessive host scheduling granularity, rather than a
startup artifact. In a 0.1 simulated second steady-state slice, DShot entered
`computeDshotDMA()` about 90,400 times and the DMA/EXTI decode chain once per
frame; the comparable servo slice had about 700 capture-map samples and only
single-digit `receiveDshotDma()` activity. The DShot capture/DMA/EXTI guest
work is therefore roughly two orders of magnitude more frequent than servo.

The next optimization experiment is deliberately narrower than bypassing the
firmware. The generator can deliver one complete 16-bit frame to TIM15 in one
scheduled callback, while TIM15 still writes all 32 captured timestamps via
the modeled DMA, raises the real DMA interrupt, and lets AM32 execute its
normal DMA-plus-EXTI decode. That removes the approximately 33 virtual-clock
callbacks and repeated GPIO routing checks per frame without replacing the
guest-visible capture buffer or decode result.

### VIMDRONES_L431_CAN steady state

`VIMDRONES_L431_CAN` was rebuilt from the same current 2.21 tree and passed the
strict DroneCAN spin test: node 11 was discovered, disarmed raw commands were
rejected, voltage telemetry was 12.3 V, 401 status frames arrived, and the
armed raw command produced 2985 reported RPM and 2983 physics RPM. The
five-simulated-second steady-state harness is
`/data/codex/am32-protocol/bench_can.py`; it keeps sending DroneCAN commands at
100 Hz wall time while timing progress from the simulator state stream.

Observed CAN samples range from **0.912x to 1.323x** before the current NVIC
experiment, with the same 2984 RPM and 125 status frames in the timed window.
This wider-than-expected range is host scheduling noise, so adjacent or pinned
A/B measurements are required before attributing a whole-run gain. A steady
host profile nevertheless exposed a specific avoidable ARMv7-M cost:
`NVIC.WriteDoubleWord()` was 1.94% of samples,
`NVIC.IsCurrentCPUInSecureState()` 1.48%, exception-name formatting 1.15%, and
pending-exception lookup 0.63%. Cortex-M4 has no TrustZone, yet each NVIC MMIO
write performed a current-CPU system-bus lookup to ask whether it was secure.

The equal-simulated-time host profiles contained 3,652 CPU-thread samples for
CAN versus 3,358 for servo, only **8.8% more**, whereas DShot had 6,945. This
supports a different conclusion from DShot: DroneCAN is not dominated by a
hidden six-figure peripheral event stream. Its core emulation cost should be
near the already-realtime PWM case, with the recurring NVIC path as one small
measured difference. The end-to-end CAN stopwatch also includes two external
wall-clock clients: commands are sent at 100 Hz wall time and the state stream
was requested every 1 ms. Consequently a slower run receives more command
frames per simulated second, and the observer adds its own socket/state work.
That feedback plus host placement explains why the end-to-end number is much
less repeatable than the CPU-sample ratio. It is the right GUI-like integration
test, but a noisy microbenchmark; future CAN A/B work should record actual RX,
TX and state-stream counts and hold them per simulated second where possible.

The scratch Renode tree now short-circuits that lookup when
`cpu.TrustZoneEnabled` is false for parameterless NVIC reads/writes and timer
frequency/divider properties. It builds cleanly and preserves ARMv7-M
semantics. Pinned post-change CAN samples were **1.135x and 1.211x**, both
correct, but this fast path has a profile-based ceiling of only about 1.5%; it
must not be credited with the much larger apparent wall-time swing. Its value
is as a safe removal of measured recurring CAN overhead, not as the complete
DroneCAN solution. It is stored as
`perf_patches/0008-armv7m-nvic-secure-lookup-fast-path.patch`; the complete
0001--0008 stack applies cleanly to a detached Infrastructure worktree and
passes `git diff --check`.

## L431 DShot frame and reply batching

The prototype above is now implemented as an opt-in contract between
`AM32ThrottleGenerator` and `AM32_STM32_CaptureTimer`, and generated L431
platforms enable it. Servo PWM remains pin-level. On DShot, the generator
schedules the 16-bit wire duration and the gap rather than 32 individual edge
callbacks. At frame completion the capture timer reconstructs the same 32
CCR timestamps from the timer's starting count, active prescaler and period.
Each timestamp still passes through `DoCapture()`, so DMA still performs its
real peripheral-width read, widened memory write, flag update and interrupt.
AM32 still runs `transfercomplete()`, `computeDshotDMA()` and the deferred EXTI
decode; this is batching of host event delivery, not injection of a decoded
throttle value.

Bidirectional replies use the same principle. The guest configures TIM15 and
its memory-to-peripheral DMA normally. Instead of scheduling one host event
for every reply period, TIM15 schedules completion at the end of the modeled
reply duration, then consumes the same 37 DShot600 DMA periods in one callback.
The captured output levels are still GCR-decoded by the Renode reply checker,
so reply count, type, RPM payload, legal line code, and CRC remain independent
correctness checks.

Pinned adjacent steady-state results at the required 4 kHz command rate were:

| Mode | Unbatched | Batched | Batched correctness |
| --- | ---: | ---: | --- |
| DShot600 | **0.911x** | **1.447x** | 2983 RPM, clean BEMF |
| bidirectional DShot600 | **0.576x** | **1.224x** | 2983--2984 RPM, 20,000 valid replies |
| EDT | 0.838x earlier isolated | **1.420x** | 2983 RPM, all EDT types, 20,000 valid replies |

Host load still moves individual values, but the adjacent DShot and BDShot
controls make the improvement much larger than that noise. The full repository
regressions then passed for `VIMDRONES_L431` in DShot600, BDShot600 and EDT
modes. They retained the expected 2983 RPM and commutation count, zero BEMF
timeout/desync, legal reply line code and CRC, matching bidirectional RPM, and
temperature/voltage/current EDT type coverage. `VIMDRONES_L431_CAN` also passed
the final DroneCAN test: node discovery, disarmed rejection, 12.3 V telemetry,
401 status frames, and matching reported/physics motor speed.

The first batch prototype produced CRC-valid zero throttle rather than being
accepted prematurely: inspecting its DMA buffer exposed a one-bit rotation.
The cause was the generator treating the callback after a batched frame as an
old per-bit state instead of an explicit batched gap, so it emitted bits 1--15
again. Adding the separate gap state fixed the buffer alignment. This failure
is useful evidence that the retained DMA and guest decoder checks can detect a
batching implementation that delivers the wrong waveform.

### BDShot300 reply-length correction

The first integrated reply batch inferred its output-DMA length from the
nominal input bitrate: 37 periods for DShot600 and 30 for DShot300. That is not
how AM32 chooses the reply padding. It classifies the measured shortest edge
interval; on `VIMDRONES_L431`, BDShot300 has a three-count low interval and
selects the same 14-padding mode as DShot600. The firmware therefore programs
23 encoded/preamble plus 14 padding entries, while the model stopped after 30
requests. DMA transfer-complete never arrived, TIM15 remained in output mode,
input frames stopped, and the GUI alternated between armed and disarmed.

Reply batching now services the maximum 37 periods. A DMA configured for a
shorter transfer simply ignores requests after its remaining count reaches
zero. The direct BDShot300 regression now remains armed at 2983 RPM, decodes
12,290 replies with zero GCR/CRC errors, and has no BEMF timeout or desync.
BDShot600 and EDT regressions also remain clean. The actual offscreen GUI/link
test passed at 2984 RPM with a continuously spinning status, zero bad reply
CRCs, and no GUI exception, matching the path on which the bug was reported.

## Host frequency and hybrid-core variability

The benchmark host is an Intel Core Ultra 9 285HX with 24 logical CPUs and no
SMT. It is a hybrid system: CPUs 0--7 report a 5.5 GHz ceiling, while most of
CPUs 8--23 report 4.7 GHz. An unpinned Renode CPU thread can therefore migrate
between materially different core classes. This is especially visible because
AM32 emulation is dominated by one CPU execution thread rather than scaling
uniformly across all cores.

At inspection time the laptop was discharging on battery, AC reported offline,
the ACPI platform profile was `balanced`, Intel P-state was active with the
`powersave` governor, and every policy used the `balance_power` energy
performance preference. Idle frequencies ranged from 800 MHz to about 2.2 GHz
with turbo enabled. CPU/package temperature was only 38--39 C against a 105 C
limit, so there was no evidence of thermal throttling at that moment. The
observed run-to-run variation is therefore consistent with power-limited turbo,
frequency ramp behavior, and migration between P and E cores.

For comparable measurements, connect AC power, select the laptop's performance
platform profile, and pin the launcher and its Renode child to CPUs 0--7:

```
taskset -c 0-7 Mcu/Renode/gen_target.py --gui VIMDRONES_L431 \
    --renode /data/codex/renode-am32-perf/renode
```

Pinning removes core-class migration but does not force a fixed clock. For the
most repeatable A/B results, keep the cooling/power state constant, discard the
first warmed-up sample, run several interleaved samples, and report their median
and range rather than the single best observation.

The interactive result is more stable when the entire launcher is pinned to
one performance CPU rather than the whole P-core set. With `taskset -c 6`, the
reported settled rate is repeatedly **1.4--1.5x**. This suggests that avoiding
both core migration and cross-core scheduling/cache effects matters more than
leaving several P cores available for this predominantly single-threaded load.

The Renode GUI now has separate `max` and `1x` buttons. `max` moves the target
above 1x, which the GuiLink interprets as unpaced/free-running. `1x` sends an
exact 1.0 simulated/wall target. GuiLink previously treated `>= 1` as unpaced;
it now paces `0 < target <= 1` and only treats zero or `> 1` as unpaced. The
button resends its command even if the slider is already at the corresponding
position, avoiding a no-op when the GUI initially displays 1x but Renode is
still free-running. The pinned link and offscreen GUI BDShot300 regressions pass
with the new control, including stable 2984 RPM and zero reply CRC failures.

`gen_target.py` now accepts `--cpusel N`. After validating the selected CPU
against the launcher's allowed affinity mask, it applies the single-CPU mask
only in the forked Renode child immediately before exec. The motor simulator is
a shared library inside Renode and therefore shares that mask; the launcher,
Qt GUI and gdb terminal remain unrestricted. A fake-Renode launch with
`--cpusel 6` reported its child affinity as exactly CPU 6; negative and
unavailable CPU selections are rejected with a command-line error.
