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

**Eleven MCU families are supported: F051, F031, G071, G031, L431,
G431, the RISC-V CH32V203, the GigaDevice GD32E230, the NXP MCXA153
and the Artery AT32F415 and AT32F421**, covering the 52 F051, 3 F031,
54 G071, 1 G031, 18 L431, 6 G431, 1 V203, 8 E230, 1 A153, 7 F415 and
100 F421 targets with no per-target file to write - the platform is
generated from
`Inc/targets.h` on demand. What differs between the families is a
table in `gen_target.py`, not a second code path. The `_CAN` targets
run their DroneCAN firmware on an emulated CAN peripheral - the
L431's and F415's bxCAN, the G431's FDCAN (see below); they are
recognised by the value the preprocessor gives `DRONECAN_SUPPORT`,
not by their names.
The SEQURE_G431 pair currently fails its spin assertions to a known
low-speed startup fidelity gap described below.

Pass `--bootloader-elf` to load an application and bootloader together
and start the MCU from the bootloader, including after an emulated MCU
reset. With no option the existing direct-to-application boot remains
the default. For example, using a prebuilt bootloader from a neighbouring
AM32-bootloader tree:

    Mcu/Renode/gen_target.py AM32REF_F051 --run \
        --bootloader-elf ../AM32-bootloader.2/obj/AM32_F051_BOOTLOADER_PA2_V18.elf

Targets are classified by asking the preprocessor which MCU each one
resolves to, not by their names - a cheap substring prefilter narrows
the candidate list first, but the family decision is the
preprocessor's. Most are named after the MCU, but `STELLAR_G071_V1`
ends in its board revision, and an earlier suffix match silently
dropped it from `--list` and therefore from every sweep driven by it. A
target that is never listed is never tested, and nothing complains.

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

The L431 is the first Cortex-M4F here and the first family with no
stock Renode platform to vendor - `stm32l431_base.repl` is written from
RM0394 and the SVD. Much of it is an F051 reunion: GPIO back at
0x48000000, the EXTI with the F0/F4 single-bank layout at the same
address (so the stock `STM32F4_EXTI` serves, non-clearing re-entry
behaviour included) and the comparators back on EXTI lines 21 and 22.
What was genuinely new:

- **the inverting input select is two fields.** `INMSEL` is three bits
  at [6:4] (bit 7 is part of `INPSEL` here, which is why the G0 model's
  four bit read cannot serve) plus `INMESEL` at [26:25], and the LL
  driver's IO2..IO5 all collide on `INMSEL` 7. The phase map is keyed
  on the full `(comparator, INMSEL, INMESEL)` triple in
  `AM32_STM32L4_Comp`.
- **the ADC is ADCv3 with the SQR rank sequencer**: ranked five bit
  channel fields in SQR1..SQR4 instead of `CHSELR` in either of its two
  modes, and ADC1 moves to 0x50040000. Everything else AM32 touches
  matches the F0/G0 model bit for bit, so this is a `sqrSequencer` mode
  on the shared model, not a third one. The calibrate/enable handshake
  gains `DEEPPWD`/`ADVREGEN` writes, which nothing polls.
- **TIM1 routes to the phase pins on AF1** where the F0 and G0 use AF2
  (`timerAf` on the bridge), and the throttle pin PA2 is TIM15_CH1 on
  **AF14**, past the 8 alternate functions the M0 families have.
- **DMA request routing is the CSELR register**, which falls past the
  DMA model's channel block and is ignored - routing stays hardwired in
  the platform, the same deliberate deviation as the G0's DMAMUX stub.
- **`PerformanceInMips` is 48, not the other families' one instruction
  per cycle.** The L431 runs from 4-wait-state flash with no cache, so
  under one instruction per cycle is the honest number - and the number
  is load-bearing. At 80 MIPS the dshot DMA re-arm in the capture ISR
  fits inside the first bit's 1.25us high time, so a capture window
  that locked one edge late misses the frame's first edge again on
  every re-arm, measures ~250us frametimes for exactly the eight
  windows `computeDshotDMA()` averages into its accept bounds, and the
  ESC then arms deaf with every real 51us frame silently discarded. At
  48 MIPS the re-arm overruns the first bit, the window drifts into the
  inter-frame gap and alignment becomes the only stable state, as on
  hardware.

### DroneCAN over the emulated bxCAN

The `_CAN` L431 targets run their real DroneCAN stack - libcanard's
bxCAN driver against Renode's stock `CAN.STMCAN`, which turns out to
model everything the driver needs: the `MSR.INAK` init handshake, the
TX mailboxes with `TSR` bookkeeping, the RX FIFOs with `RFOM` release,
and the acceptance filters (AM32 leaves filter 0 in accept-all mask
mode). The frames leave the machine through a `CANHub` into
`AM32_CanMcast`, a bridge speaking the ArduPilot multicast CAN scheme:
UDP datagrams on `239.65.82.<bus>:57732` with the 10-byte header and
CRC16-CCITT that `Src/DroneCAN/sys_can_SITL.c` and libcanard's mcast
driver use. Anything on `mcast:<bus>` sees the emulated ESC -
**dronecan_gui_tool**, the SITL GUI's DroneCAN panel,
`sitl_can_test.py` - and can arm it, throttle it, fetch and set its
parameters and read its `esc.Status` telemetry.

    python3 Mcu/Renode/gen_target.py VIMDRONES_L431_CAN --run
    # ... then: dronecan_gui_tool mcast:0

`--can-bus N` moves it to another bus number (-1 disconnects);
`--can-node` sets the node id written into the generated eeprom,
because an anonymous node sends nothing until a DNA allocator answers
and a bare bench run has none. The launcher's eeprom also sets input
type 5 (dronecan only), as a real CAN ESC would be configured - and
not only for realism: with the input type left at auto, the throttle
generator's self-started zero-servo signal and the CAN input both
write `newinput`, and the flapping input keeps resetting the arming
counter so the ESC never arms. A real bench would show the same fight
if a servo lead were left plugged in.

`run_renode_tests.py --target VIMDRONES_L431_CAN --can` covers the
path end to end with the GUI's own `CanPanel`: the node appears on the
bus with the right id, refuses to spin unarmed, arms and spins at the
recorded speed on `RawCommand`, and the rpm its `esc.Status` telemetry
reports matches the physics to 5%. The scripted servo and dshot modes
also run on `_CAN` targets (their test eeprom keeps input type 0), so
the CAN builds are held to the same spin figures as everything else.

Two things surfaced bringing this up. `sys_can_init()` derives the CAN
bitrate from `LL_RCC_GetSystemClocksFreq()`, which recomputes the PLL
output from `PLLCFGR` - against the stub's original zero readback that
made PCLK1 come out 0 MHz and the driver hang in its bitrate switch
(the same zero readback had been quietly leaving `LL_USART_Init()`
unconfigured, so storing PLLCFGR fixed latent telemetry breakage too).
And the bridge deliberately keeps its sockets across a firmware reset:
`RestartNode` and the signal-loss reboot both go through
`NVIC_SystemReset()`, and the node has to come back on the same bus as
it would on real wire.

### The G431

The fourth family, 160MHz off HSI or HSE (the SEQURE pair), base
written from RM0440 and the SVD like the L431's. Much of it is a
recombination: the G0's comparator layout (four-bit INMSEL at [7:4])
on the F051's EXTI lines 21/22, the L4's ADCv3 SQR sequencer with ADC1
back at 0x50000000, DMAMUX stubbed exactly as on the G0. What was
genuinely new:

- **every G431 group splits the phases across COMP1 and COMP2** with
  `PHASE_x_COMP_NUMBER` but without `N_VARIANT`, so the generator now
  keys the split on the macro that actually says so.
- **the SEQURE converts on both ADC instances** (`USE_ADC_1_2`):
  temperature and the NTC on ADC1, voltage and current on ADC2 with
  its own DMA channel - two model instances with `<base, +0x100>`
  registrations so ADC1 stops short of ADC2, and a register-file stub
  for the common page.
- **phase pins reach GPIOF** (PF0 low sides on two groups) and the
  SEQURE's TIM1_CH3N sits on PB15 at **AF4** where every other low
  side is AF6 - the bridge gained a fourth port and a second accepted
  AF (`timerAfAlt`).
- **DMA1_Channel2's NVIC line is deliberately not wired** even though
  the ADC transfers on that channel: the firmware enables the IRQ but
  ships no handler, so a delivered interrupt would land in
  Default_Handler's infinite loop. The ADC callback is polled from the
  1kHz loop.
- **the FDCAN is a new model** (`AM32_STM32_FDCAN`): the CCCR
  INIT/CCE/CSR handshake, the G4's fixed message RAM layout reached
  through the system bus, RX FIFO0 with real fill/get/put counters,
  and the grouped ILS interrupt-line select. Host-side senders burst
  in wall time while the emulation runs slower, so frames the 3-deep
  FIFO cannot hold yet wait in a queue that stands in for the 1Mbit/s
  wire's own serialization - without it a fast sender loses 97% of its
  commands. Classic 8-byte frames only; FD frames are rejected at the
  mcast bridge.

`PerformanceInMips` is 64, for the same dshot-alignment reason as the
L431's 48.

**Known gap: the SEQURE_G431 startup.** SEQURE defines
`NO_POLLING_START`, handing commutation to comparator edges after two
zero crossings, and in the emulation the motor locks into a stable
low-speed rocking resonance: each mechanical oscillation produces
exactly one clean comparator edge at the true crossing, so commutation
perfectly follows the rock instead of leading it, and the rpm
oscillates around zero at any throttle. The other four G431 targets
start in polled mode and pass everything. Circuit analysis (and a
negative sub-microsecond edge-delivery experiment) puts the missing
physics in the comparator front end: the model applies no divider gain
- so the low-speed BEMF towers over the modelled noise band where the
real board's divider brings them within reach - and its identical
phase/neutral gains cancel PWM common-mode exactly, where real
resistor mismatch feeds it through as differential transients. Both
need values measured from the real board; until then the SEQURE pair
has no recorded spin figures.

The L431 also exposed a harness bug the other families could not hit:
the firmware ELF was picked by globbing `AM32_<target>_*.elf`, and for
a target with a `_CAN` sibling the sibling sorts last - so the servo
suite quietly loaded `AM32_VIMDRONES_L431_CAN_2.20.elf`, whose vector
table lives at 0x08004000, and the CPU halted at reset. `find_elf()`
now matches the version suffix exactly.

Known L431 fidelity gaps, both harmless to the suite: USART1 TX DMA
telemetry is not modelled (the stock USART raises no TX DMA request;
nothing in the tests enables interval telemetry), and the `APB2RSTR`
reset pulse `receiveDshotDma()` sends TIM15 every direction change is
ignored by the RCC stub, as it is on the other families.

### The CH32V203

The fifth family is not an STM32 and not an ARM: WCH's CH32V203, a
96MHz QingKe V4B RISC-V core. The peripheral generation is a faithful
STM32F1, which is why most of the machine is reused models - the F0
DMA (F1 layout is identical), the F4 EXTI (same single-bank layout,
same load-bearing non-clearing re-entry), the shared advanced and
capture timer models, our IWDG, and Renode's stock `STM32F1GPIOPort`.
The interrupt controller is another matter. Renode's generic `RiscV32`
boots the ELF once three WCH-isms are stubbed (`am32_v203.resc`:
custom CSRs 0xbc0 and 0x804, and 0x800, an mstatus alias that
`__enable_irq()` writes), but WCH's PFIC and "fast interrupt"
machinery had to be emulated in `AM32_WCH_Pfic`:

- **WCH vectoring is a table of handler addresses** (mtvec mode 3),
  which Renode coerces to its CLINT-vectored mode 1 - so every
  interrupt lands on a *data word inside the vector table*. All
  sources funnel through the machine-external interrupt, so the model
  hooks that one landing address, picks the winning PFIC source
  (lowest priority byte, then lowest number), and redirects PC to the
  handler read from the real table. Exceptions land at the table base
  and are redirected to `HardFault_Handler` after logging.
- **every AM32 handler is `WCH-Interrupt-fast`**: the compiler saves
  no registers and relies on the hardware prologue/epilogue, which
  also shadows mepc/mstatus per nesting level. The model saves the 16
  caller-saved registers plus the trap CSRs at dispatch and restores
  them from a pre-opcode hook on `mret` (0x30200073). An opcode hook
  rather than an address hook is deliberate: Renode address hooks miss
  translation blocks entered through the indirect-jump fast path - a
  `ret` landing on the mret is exactly that - while opcode hooks are
  embedded at translation time and always fire. Skipping the CSR
  shadowing was good for the subtlest bug of the port: with one
  architectural mepc, the EXTI trap taken inside the 20kHz handler
  overwrote the outer return address, and the outer `mret` jumped into
  a data variable with another context's registers.
- **there is no comparator - two op-amps do BEMF duty.**
  `changeCompInput()` routes one phase per step through `OPA->CR`, the
  outputs land on real pins (PA3/PA4) read back through `GPIOA->INDR`
  and edge-detected by EXTI lines 3 and 4. `AM32_WCH_Opa` maps the
  three CR values to phases and drives the actual GPIO pins, so both
  the INDR reads and the EXTI path are the firmware's own.
- **the core SysTick is WCH's own** (64-bit CNT/CMP at 0xE000F000,
  SR cleared by writing zero), the 20kHz loop timer here -
  `AM32_WCH_SysTick`.
- **the ADC is F1-generation**: RSQR1..3 rank sequencer, RDATAR, and
  self-clearing CAL/RSTCAL bits the init spins on (`AM32_WCH_Adc`).
  Temperature has no TS_CAL page; the factory point at 0x1FFFF720
  with a fixed -4.3mV/C slope is seeded and inverted instead.
- **dshot capture reaches both edges differently**: IC1 mapped to TRC
  (CC1S=11) with the TI1 edge detector as trigger (SMCR TS=100)
  instead of the STM32's CCER both-edge polarity - a mode the capture
  timer model now recognises. Miss it and servo works while every
  dshot flavour arms deaf.
- the bridge grew an **`f1Gpio` mode**: phase drive decoded from
  CFGLR/CFGHR nibbles (output mode with the CNF alternate bit) and
  ODR at 0x0C, with no per-pin AF number to check - F1 routing is the
  AFIO remap, which the firmware always programs and the model
  deliberately trusts.
- being the first family to enable the DMA **half-transfer interrupt**
  (the capture handler needs HT for its servo polarity flip), it
  exposed a live bug in the shared DMA model: IFCR cleared the whole
  channel nibble on any bit write where real hardware clears per bit.
  The arming tune runs with interrupts masked, HT and TC accumulated
  across it, and the handler's HT-only clear wiped the pending TC -
  after which the transfer-complete path never re-armed the DMA and
  dshot input went permanently deaf the moment the ESC armed. One
  model fix cured three symptoms: dshot300 arm-then-timeout, a
  bidirectional reply frozen at a spinup-era period, and a degraded
  bdshot spin speed.

Two Renode-level gaps are papered over in the PFIC model rather than
the platform: the CPU requires `mie.MEIE` for external-interrupt
delivery, which the firmware (correctly, for WCH) never sets, so the
model sets it on hook install and re-establishes it after a reset
clears it; and `NVIC_SystemReset` via the PFIC CFGR key write
requests a machine reset, with the family script's reset macro
reloading the ELF - a RISC-V CPU reset does not re-read an entry
point the way a Cortex-M re-reads its vector table, and without the
reload the signal-loss reboot would leave the firmware running past
the call with half-cleared state.

### The F031 and G031 - no comparator at all

Two more STM32s, and the first pair whose dies have **no analog
comparator**: their boards put external comparator chips on three GPIO
pins and the firmware watches them through EXTI, polling the pin level
directly during startup and taking edges once running. The phase being
sensed is never written to any peripheral - the firmware keeps it in
globals - but it is observable, and `AM32_ExtiBemf` recovers it two
different ways:

- **the F031's `changeCompInput()` assigns RTSR/FTSR**, leaving
  exactly one phase line armed, so reading the trigger registers back
  names the phase;
- **the G031's only ORs and clears the current line's bits**, leaving
  all three lines armed forever - there the phase is named by which
  line's bits a write *changed* (each phase alternates edge between
  visits, so its revisit always flips its bits), which the G0 EXTI
  model reports through a `TriggerChanged` event.

The model then drives the real GPIO pins, so both the polled IDR reads
and the EXTI edge path are the firmware's own. Two bring-up findings
worth keeping:

- **a parked-rotor chatter edge before the first commutation is
  unrecoverable on these MCUs**: `interruptRoutine()`'s filter loop
  reads through a still-NULL port pointer and returns before masking,
  so one early edge storms forever. Real external comparators sit
  quietly on a rail until there is signal - their hysteresis eats the
  noise our physics models - so the model holds the pins idle until
  the firmware has actually selected a phase.
- **`FVT_LITTLEBEE_50_F031` defines `INVERTED_EXTI`**: its comparator
  has the opposite output polarity, the firmware flips its edge
  bookkeeping, and without the matching `inverted` flag on the model
  the motor "spins" backwards at -543 rpm with a desync a second.
- **known gap: a reversed-direction start on the G031** would miss its
  first phase selection - the initial trigger state already matches
  what `changeCompInput()` writes, so no change event fires until the
  second commutation. The suite always spins forward; a reversed
  eeprom would need the model to derive the initial phase another way.

The rest is family bookkeeping. The F031 rotates the timer roles (TIM3
interval, TIM16/TIM2 trading capture and 20kHz loop per group) and its
A/B groups capture the throttle on **TIM2 channel 3** - the capture
timer model grew a channel parameter for it. Its DMA requests are
remapped onto a five-channel controller through SYSCFG bits the
platform hardwires, its SystemClock_Config polls the flash-latency
readback (a register file where the F051 gets away with a tag), and it
has 4K of SRAM. The firmware's EXTI handlers also write the pending
register with the line *number* rather than a mask - a real quirk that
works because `maskPhaseInterrupts()` does the actual clearing. The
G031 is a G071 minus the comparators, TIM6 and TIM15 (TIM16 is the
20kHz loop timer), with 8K of SRAM and its dshot deferral software
interrupt on EXTI line 3 rather than 15. Its phase-A low-side FET pin
(PB14) shares a line number with the phase-A BEMF input (PC14), and
since EXTICR is stored but not honoured the generated overlay
disconnects PB14 from the EXTI.

### The GD32E230

GigaDevice's Cortex-M23, and a faithful clone of the STM32F0
generation: GPIO, the timer block including CCHP=BDTR, EXTI, the
five-channel DMA, the FWDGT watchdog and the single CMP - bit-identical
to the F051's COMP1 half at the same address, with the same
input-select encodings and the same EXTI line 21 - all run against the
existing STM32 models unchanged, under GD names (TIMERn is ST's
TIM(n+1)). The two real divergences:

- **the ADC is the F1-generation register file** (RSQ rank sequencer,
  RDATA at 0x4C, CLB/RSTCLB self-clearing calibration) - the same
  layout the CH32V203 has, so `AM32_WCH_Adc` serves. The GD part has
  no factory calibration page; the firmware applies fixed datasheet
  constants (1.43V at 25C, -4.3mV/C), which the .resc seeds as the
  model's reference word.
- **`GD32DEV_B_E230` is refused at generation**: that board wires a
  phase's high side to the *complementary* TIMER0 output and
  compensates with an inverted PWM mode on that one channel, a pairing
  the bridge cannot represent. The other seven buildable E230 targets
  use the straight mapping. (`CM_MINI_E230` exists in targets.h but
  has no Makefile rule, so there is no firmware to run; and
  `SKYSTARS_SL40_E230`'s `USE_RGB_LED` is three discrete GPIO outputs,
  which nothing decodes, so its LED is not modelled.)

`PA6_VOLTAGE` on `GD32DEV_A_E230` does not change the ADC channel
setup - it swaps which DMA slot the firmware reads as voltage versus
current, so the generator swaps which channel the model scales as
which.

### The NXP MCXA153

Nothing on this part descends from an STM32, so every peripheral model
under `peripherals/nxp/` is new: eDMA (TCD engine, with the 3-bit
source-address modulo that makes the capture channel alternate between
CTIMER0's two capture registers inside one 8-byte window), the
LPC-lineage CTIMER, the LPTMR 20kHz loop timer, the FlexPWM, the LPCMP
pair, the LPADC and the LPSPI. Four structural differences from every
other family:

- **the bridge reads the PWM block directly instead of decoding
  pins.** A153 commutation never touches a GPIO mode register: the
  phase states live entirely in the FlexPWM's MASK/DTSRCSEL/OUTEN
  registers, latched by SM0's CTRL2.FORCE strobe. The FlexPWM model
  decodes them into the SITL's five phase states and hands them to the
  bridge through `IAM32PwmSource`; when one of those is present the
  bridge skips the whole GPIO+timer decode.
- **the comparators are two separate peripherals with separate NVIC
  lines.** The firmware commutates by enabling exactly one of them and
  retargeting its minus-input mux, so the phase key is (unit, MSEL) -
  A and C share CMP0 on different inputs, B is CMP1. A small
  `MCXA_LpcmpMux` hub owns the phase map and presents the pair to the
  bridge as one `IAM32Comparator`.
- **the bidirectional dshot reply leaves through the SPI.**
  `sendDshotDma()` gray-folds the whole 20-bit GCR into 21 wire levels
  and writes them to LPSPI0's TDR in one go, where the timer families
  feed a DMA one level per period. The LPSPI model undoes the fold
  (`w ^ (w >> 1)`) and carries the same `Reply*` decode surface the
  capture timers do, now behind `IAM32ReplySource` so the guilink and
  the test runner find whichever the platform has.
- **the flash driver is masked boot ROM.** `save_flash_nolib()` calls
  through a function-pointer tree at 0x03003FE0; there is no
  memory-mapped flash controller to model, so the generator compiles a
  small thumb blob (`sim/mcxa_rom_api.c`) with the firmware's own
  toolchain, loads it into the ROM window and writes the API tree and
  driver table with the symbol addresses read back from the ELF.

One modelling lesson cost a debugging session: CTIMER0's MR1 is an
interrupt-without-reset match (the 10us idle-line timeout), and
modelling it as the counter's periodic limit wrapped TC every 50 ticks
- which corrupted every capture into the 0-50 range and, worse, fired
the 10us handler continuously, whose pin sampling then falsely
triggered the firmware's inverted-dshot autodetect. It is now a
one-shot rearmed at every point TC restarts from zero while the
counter free-runs.

The A153 also handed the harness its first firmware find on this
family: **bidirectional dshot from power-on cannot arm** (servo and
plain dshot both arm and spin). Three counters race: `zero_input_count`
passes the arming threshold within ~30 frames, the A153-only
inverted-dshot autodetect in `CTIMER0_IRQHandler` then flips the
capture polarity at ~100 frames and resets it to zero, and
`dshot_telemetry` latches a few frames later - after which
`transfercomplete()` returns down the telemetry branch before the
`zero_input_count++` block ever runs again. The 1s arming gate needs
more than 30, reads a frozen handful forever, and clears `inputSet`;
re-detection succeeds, but with `dshot_telemetry` still latched the
counter can never rebuild, so the cycle repeats. `BDSHOT_NOARM` in
`run_renode_tests.py` documents it; the bdshot run still asserts the
reply plumbing, and the `--link` run drives plain dshot instead so the
spin assertion stays.

### The AT32F415

Artery's 144MHz Cortex-M4 (no FPU), and after the CH32V203 the second
F1-generation reunion: CFGLR/CFGHR GPIO at the F1 addresses, F1
layouts for the timers, DMA, EXINT, WDT and bxCAN, and the
rank-sequenced F1 ADC. Most of the machine is therefore the V203's
model set - the F0 DMA, `STM32F1GPIOPort`, `AM32_WCH_Adc`, the
bridge's `f1Gpio` mode, the shared timer models and our IWDG - with
both APB buses at /2 and the F1 timer doubler putting every timer at
the full 144MHz. The capture timer differs per hardware group: AT_D
boards ride TMR3_CH1 on PB4, AT_H boards TMR2_**CH3** on PA2 - the
second user of the capture model's channel parameter after the F031.
The Artery-isms:

- **the comparator is a block of its own** at 0x40002400, not a SYSCFG
  tenant: nearly the F051's COMP1 half-register (enable bit 0, INMSEL
  at [6:4], output at bit 14), whole-assigned with `PHASE_x_COMP`
  constants whose stray bit 30 lands in the unused CMP2 half. It gates
  EXTI line 19, which has its **own NVIC vector** (`CMP1_IRQn` 70)
  rather than sharing an EXINT one. `AM32_AT32_Cmp` models it, with
  the output bit position a constructor parameter because the F421's
  otherwise-similar block reads back at bit 30 - the model is shared
  with that port.
- **the stock EXTI model broke the deferred-dshot path**, the first
  family to notice: Renode's `STM32F4_EXTI` sets the output line on a
  SWIER write but never latches the pending bit. The L431 gets away
  with it because its handler clears line 15 unconditionally; the
  F415's `EXINT15_10_IRQHandler` *reads* `intsts` to decide what to
  service, saw nothing, cleared nothing, and the level-held NVIC line
  re-entered it forever - the 20kHz loop (a lower priority) starved
  and the ESC never armed. `AM32_STM32_Exti` is the stock model with
  the software interrupt latching PR, as hardware does.
- **DMA request routing is a flexible mux**, not fixed F1 wiring:
  `SRC_SEL0/1` at 0xA0/0xA4 assign 8-bit request IDs per channel
  (ADC1 to channel 1, UART1 TX to 4, the capture timer to 6). The
  offsets fall past the model's seven-channel register block and are
  ignored; the platform hardwires the routing, the same deliberate
  deviation as the G0's DMAMUX and the L4's CSELR.
- **the temperature sensor slope runs the other way.** No factory
  calibration page; `getConvertedDegrees()` applies fixed constants
  amounting to 1344mV at 25C **rising** 4.2mV/C, where the WCH and GD
  sensors fall 4.3mV/C - so `AM32_WCH_Adc` grew a slope parameter,
  and the family .resc seeds the reference word 1mV high to cancel
  the firmware's fixed-point truncation. The .resc also seeds the
  flash-size halfword at 0x1FFFF7E0 (128), which `sector_size()` in
  `Mcu/f415/Src/eeprom.c` reads to pick 1K over 2K sectors.
- **the ADC channel macros are dead code**: `Mcu/f415/Src/ADC.c`
  hardwires rank 1 = CH3 read back as voltage and rank 2 = CH6 as
  current, whatever `VOLTAGE_ADC_CHANNEL` says, so the generator uses
  the fixed channels as it does on the V203.
- **the eRPM reply is 25/24 high on real hardware too**: the firmware
  sums commutation intervals assuming 0.5us ticks, but its interval
  timer runs at 144MHz/75 = 1.92MHz. The bidirectional-reply check
  in `run_renode_tests.py` expects the scale (`REPLY_RPM_SCALE`)
  instead of failing the honest emulation of it.
- **CRM wants real readback**: `system_core_clock_update()`,
  `usart_init()` and `sys_can_init()` all recompute clocks from the
  stored CFG multiplier and PLL register, so the stub keeps what was
  written and mirrors the enable-to-ready and SCLKSEL-to-SCLKSTS
  bits, like the other families' RCC stubs.
- **the `_CAN` targets are bxCAN**, bit-for-bit the STM32's at the
  STM32 address with the same four vectors, so the stock `STMCAN` and
  the existing mcast bridge serve exactly as on the L431. The 16K CAN
  bootloader puts the app base at 0x08004000, and DroneCAN's
  bootloader flag lives in ERTC backup registers, a write-readback
  stub. `TBS_12S_F415` (no CAN) shares the 128K parts' relocated
  eeprom at 0x0801F800.

`DAKEFPV_35A_F415` is build-disabled upstream (no comparator group in
targets.h), so generation refuses it with a comparator error and it
never appears in sweeps. `PerformanceInMips` is 96 - 144MHz from
4-wait-state flash, pinned by the same dshot capture-window alignment
reasoning as the L431's 48.

### The AT32F421

Artery's 120MHz Cortex-M4 (no FPU) remake of the STM32F0 generation,
and the biggest family in the tree: 105 buildable targets, of which
100 generate and pass. Almost everything AM32 touches is
register-compatible under AT names (TMRn is ST's TIMn at the same
instance numbers and addresses, WDT is the IWDG, EXINT is the F0
EXTI), so the machine is mostly reused models: GPIO, the advanced and
capture timers, the F0 DMA, our IWDG. The ADC is the F1-generation
register file again (OSQ rank sequencer, ODT at 0x4C, self-clearing
ADCAL/ADCALINIT), so `AM32_WCH_Adc` serves as on the V203 and E230,
and USART1 is the F1 register file, so the stock `STM32_UART` stands
in for the F0 families' `STM32F7_USART`. What was genuinely new:

- **the comparator is Artery's own single-register CMP**
  (`AM32_AT32_Cmp`): one `ctrlsts` word at offset 0x1C of the SCFG
  page - the F051's address, but not its layout. The inverting-input
  select at [6:4] keeps the F051 encodings and adds 7 = PA2 (the
  `AT_245` polling-mode groups); the output is CMPVALUE at **bit 30**,
  where the F415 sibling keeps its at bit 14, so the bit position is a
  constructor parameter. `comStep()` whole-assigns the register with
  `PHASE_x_COMP` constants that carry bit 30 *set* - writing the
  read-only output bit, which the model masks as hardware does - and
  `changeCompInput()` read-modify-writes the speed field at runtime,
  keyed on `average_interval`, so the model stores everything it does
  not interpret. EXTI line 21, `ADC1_CMP_IRQn`, as on the F051.
- **the stock EXTI's software trigger broke arming, invisibly.** The
  F421 rides `STM32F4_EXTI`'s register layout exactly, but its
  `EXINT15_4_IRQHandler` guards the flag clear on reading the pending
  bit back - `if (EXINT->intsts & EXINT_LINE_15)` - where every other
  family's handler clears unconditionally. Against the stock model a
  `SWIER` trigger raises the NVIC line but the pending register never
  reads back set, so the guard never passed, the line was never
  cleared, and the handler re-entered forever: 6.4 million entries in
  0.7 simulated seconds, starving the 20kHz loop timer (priority 3
  behind the deferral's 2) so `armed_timeout_count` never advanced and
  the ESC sat unarmed with every subsystem apparently healthy.
  `AM32_STM32_Exti` - the model the F415 introduced for its own
  flavour of the same starvation - replaces it here too with the real
  semantics: SWIER sets the pending bit, the bit reads back, clearing
  PR clears SWIER. The deliberate non-clearing re-entry the comparator
  handler uses to wait out the blanking window is preserved.
- **the NTC targets are the first whose firmware actually decodes an
  external thermistor**: under `USE_NTC` the ARTERY branch of the 1kHz
  loop calls `getNTCDegrees()` on a fifth ADC rank instead of the
  internal sensor (the G431 SEQURE also converts an NTC, but its
  reading is unused). An unmapped channel reads 0, which the board's
  `NTC_table` decodes as **400C**, and the thermal clamp then holds
  `duty_cycle_maximum` at 1 - the motor armed and never started, with
  the "off" temperature limit of 255 sailing straight past 400. The
  ADC model's `ntcChannel` returns a fixed count that decodes to the
  38C physics ambient on the table most F421 boards share; the model
  does not carry the per-board tables, so the NTC reading does not
  track the physics temperature.
- **the internal temperature slope runs the other way.** No factory
  calibration page, as on the GD - but Artery's fixed-constant formula
  `(12800 - raw*33000/4096)/-42 + 25` has a *positive* 4.2mV/C slope
  where the WCH/GD sensors fall with temperature. The shared ADC
  model's fixed negative-slope inversion can only be made exact at the
  reference point itself, so the .resc seeds 38C at 1337mV - chosen so
  the firmware's truncating integer math lands exactly 38 - and
  excursions from there would read mirrored. Exact until the physics
  models heating.
- **five targets are refused at generation**: `AIRBEE_F421`,
  `SP8_AIO_F421`, `NEUTRON_1_2S_AIO_F421`, `TBS_MINI_F421` and
  `JMITEST_1_2S_F421` define `USE_INVERTED_HIGH` (active-low high-side
  gates), which the bridge deliberately refuses rather than model
  untested; the generator now says so before the machine loads.

The rest is family bookkeeping: the AT_B group captures the throttle
on TMR3_CH1 (PB4, mux 1), the AT_C/E/F groups on TMR15_CH1 (PA2, mux
0 - `UN_TIM_Init()` sets no mux there because the reset value already
routes it), both on the shared `DMA1_Channel5_4` line; the CRM is the
same enable-to-ready Python stub as the GD's RCU with the PLL/MISC
writes stored; and flash programming is F1-style direct stores that
land in `MappedMemory` past a register-file FLASH controller whose
busy flag always reads clear.

### One target is skipped

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
time, so the phase never drives at all.

Upstream confirms the intent: the target was for **slot car ESCs, which
only run at 100% throttle**, where "at low throttle dead time will
exceed duty cycle except at very low pwm frequencies". So it is not a
firmware defect either - the sweep was asserting a partial throttle spin
the target was never meant to do.

Testing it at full throttle does not rescue it, which is worth knowing
before anyone tries: 2000us and 1300us give **byte identical** rpm and
rotor angle, because AM32 always begins with a fixed low duty startup
ramp and never reaches `running`, so the commanded throttle is never
applied. The failure happens before throttle matters.

`run_renode_tests.py` therefore skips it by name, with the reason in
`UNSPINNABLE`, so a red sweep still means something.

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
same way the ARM toolchain is installed. Then, for any F051, G071 or
non-CAN L431 target:

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

### Live logic analyser

`--sigrok` starts a TCP server for libsigrok's `renode-la` driver and
starts the emulation:

    python3 Mcu/Renode/gen_target.py VIMDRONES_L431 --sigrok

Connect a frontend to it:

    pulseview -d renode-la:conn=tcp/127.0.0.1/4242

The capture is continuous: the stream follows simulated time from the
frontend's start until it stops, so it behaves like a bench analyser
rather than a one-shot buffer. `--sigrok-port` and
`--sigrok-sample-rate` change the defaults; the rate the frontend
selects is the one the stream uses, and the launcher's value is only
what the server advertises.

The channel names, the device name and the sample rate all travel in
the protocol's greeting, so the frontend labels the traces by itself:

| channel | signal | source and resolution |
|---|---|---|
| `input` | input wire | exact servo, DShot and bidirectional-reply edges |
| `ws2812` | WS2812 data | exact GPIO edges on targets with a strip; low otherwise |
| `A.mode0..2` | phase A mode bits | bridge state at the physics batch boundary |
| `B.mode0..2` | phase B mode bits | bridge state at the physics batch boundary |
| `C.mode0..2` | phase C mode bits | bridge state at the physics batch boundary |
| `comparator` | comparator output | physics result at the batch boundary |
| `sense0..1` | sensed phase bits | 0=A, 1=B, 2=C |

The three-bit phase value is the motor model's existing state: 0 float,
1 low, 2 complementary PWM, 3 PWM without complementary drive and 4
proportional brake. It is deliberately not a synthetic PWM carrier, and
the two exact-edge channels are the only ones that would line up with a
bench analyser on the same wire. Comparator transitions are delivered by
the bridge at its 20us batch boundary; multiple physics transitions
inside one batch share that timestamp and cannot appear as
nanosecond-separated edges.

There is no sample-rate timer in the emulator. The analyser records only
timestamped changes and expands them into fixed-rate samples on a
millisecond flush tick, so 10 MHz does not add ten million emulation
events per simulated second. It does produce 20MB of samples per
simulated second at that rate, though, so a long session is cheaper at a
lower rate.

A headless capture uses the same connection. Keep its stdin open, as
`sigrok-cli --continuous` stops as soon as stdin reports a keypress,
and an empty pipe counts as one:

    tail -f /dev/null | sigrok-cli -d renode-la:conn=tcp/127.0.0.1/4242 \
        -c samplerate=10M --continuous -O bits

Nothing drives the throttle with `--sigrok` alone, so the ESC arms at
zero and every channel sits flat. Combine it with `--link` or `--gui`,
or drive the wire some other way, to see a running motor.

### Driving it from the SITL GUI

    python3 Mcu/Renode/gen_target.py FD6288_F051 --gui

On a hybrid host, `--cpusel N` pins Renode and its in-process motor simulator
to one host CPU. The launcher, Qt GUI and gdb remain available to the normal
scheduler, avoiding contention with the emulation thread. For example, the
current benchmark host is most consistent with Renode on CPU 6:

    python3 Mcu/Renode/gen_target.py VIMDRONES_L431 --gui --cpusel 6 \
        --renode /data/codex/renode-am32-perf/renode

That starts the emulator with `guilink` serving the SITL's own two UDP
protocols and opens `Mcu/SITL/sitl_gui.py --backend renode` on them, so
the throttle slider, the BDShot telemetry readout, the rpm and current
graphs, the motor view, the stuck rotor slider and the parameter editor
all drive an emulated ESC. Nothing in the GUI knows which backend it is
talking to; both ends speak the same wire format.

`--link` does the same without opening a GUI, for driving from a script
with `Mcu/SITL/sitl_dshot.py` or `sitl_gui_backend.py`.

On a `_CAN` target, `--gui` also passes `--renode-can` so the GUI's
DroneCAN panel comes up live against the mcast bridge: enable CAN
there and the ESC arms and throttles over the bus rather than the
dshot wire, with telemetry and the parameter editor going through
DroneCAN exactly as against the SITL. The panel's rate control is
wall clock, like everything else the GUI sends.

**The input is setpoints, not a wire recording, and that is forced by the
clock.** Renode runs far below real time, so the GUI's 50Hz stream of
servo frames - 50Hz of *wall* clock - is a 5.5Hz signal as the firmware
experiences it, well under what `detectInput()` needs, and nothing would
ever arm. So an incoming packet sets the throttle rather than becoming
one frame on the wire, and `AM32ThrottleGenerator` synthesises correctly
timed frames in virtual time. What the firmware decodes is still real pin
edges through the real capture and DMA path; what is lost is driving
malformed or oddly rated signals from the GUI, which stays a SITL job.

The frame rate follows from that too: the GUI's rate control is wall
clock and means nothing here, so the wire rate is `--gui-dshot-us`
(default 250, ie. 4kHz). It is worth less than it looks. Every edge is a
timer event, so a quarter of the frames is a quarter of the edges, but
measured over the same simulated interval on `FD6288_F051` it buys about
a quarter of the wall clock, not three quarters:

| `--gui-dshot-us` | wall seconds for the same 4s of simulated time | rpm |
|---|---|---|
| 250 (4kHz) | 61 | 2937 |
| 1000 (1kHz) | 45 | 2934 |

The rest of the cost is the physics and the commutation, which the wire
rate does not touch - and the rpm is the same either way, so the trade is
telemetry resolution against a quarter of the wait.

**AM32 latches the input protocol it detected**, and `detectInput()` only
ever re-checks that one afterwards, so switching between servo and dshot
once the firmware has decided needs a reboot - exactly as it would on the
bench. So does writing eeprom settings, which are read at boot. Under the
SITL that is the process panel; here it is the **Restart ESC** button,
which resets the machine.

Two consequences of the slow clock are worth knowing before they look
like bugs:

- **arming takes tens of wall seconds.** The firmware wants about 1.5
  simulated seconds of zero throttle, and simulated seconds are
  expensive. Leave the throttle at zero and wait.
- **the wire is silent until the GUI speaks.** The generator otherwise
  self-starts in servo mode, and the firmware would detect servo during
  the seconds before the GUI attaches - after which `detectInput()` only
  ever calls `checkServo()` again, so a dshot stream arriving later is
  never looked at and the ESC never arms. With a client attached the
  client owns the wire, which is also what a bench ESC with no flight
  controller plugged in looks like.

The GUI also shows what only the emulator can answer, because none of it
is on the wire and all of it matters most when the wire has gone quiet:

- **which firmware is running**, read out of emulated flash at the
  `filename` symbol - a fixed 30 byte string in its own section, which is
  where a configurator reads it from too. It is what is actually loaded,
  not what the ELF path claims.
- **where the core is**: the program counter, whether it is in the
  application or below `0x08001000` in the bootloader region, and whether
  it has halted at all. A halted core used to be indistinguishable from
  an ESC that would not arm; see the bring-up findings.
- **the emulation speed it is achieving.** The speedup slider works here
  too: the link sleeps the emulation thread to hold simulated over wall
  time at targets through exactly 1x. The `1x` button requests real-time
  pacing, while `max` disables pacing and lets the emulator run flat out.
  The label shows the rate actually reached.
- **how far through arming it is.** `tenKhzRoutine()` counts
  `armed_timeout_count` up at `LOOP_FREQUENCY_HZ` while the input reads
  zero and wants a full second of it, so the counter is a progress bar.
  On a backend this slow that is the difference between waiting and
  debugging.

Everything else the GUI offers against the SITL is served: the physics
state stream behind the graphs and the motor view, the eeprom behind the
parameter editor, the motor model picker, and the stuck rotor slider.
What the emulator cannot serve is disabled in the UI rather than left
silently dead - DroneCAN on targets without a CAN peripheral, synthesized
tone audio (which depends on the SITL's fake output timer), and the SITL
process launcher (`gen_target.py` owns the emulator). Motor audio is served
from the same native physics model as the graphs and motor view.

Pausing is safe, which is what makes this worth having alongside gdb: the
physics is frozen with everything else, no host time leaks in, and the
rotor resumes exactly where it stopped. Break in `tenKhzRoutine`, look at
the motor, continue.

#### Testing it

    python3 Mcu/Renode/run_renode_tests.py --target FD6288_F051 --link
    python3 Mcu/Renode/run_renode_tests.py --target FD6288_F051 --gui
    python3 Mcu/Renode/run_renode_tests.py --target VIMDRONES_L431_CAN --can

`--link` drives the target through the udp ports with the GUI's own
backend classes; `--gui` runs the real `sitl_gui.py` under Qt's offscreen
platform and scripts it through its control port, which is the only way
to cover the UI a person actually uses. `--can` arms and throttles a
`_CAN` target over DroneCAN through the mcast bridge with the GUI's
`CanPanel`, on bus 7 by default so a live SITL or GUI on the same
machine is not disturbed.

Both are paced by **simulated** time read out of the state stream rather
than by the wall clock, so they hold the throttle for the same emulated
interval as the scripted tests however slowly the host runs;
`--link-seconds` is only a backstop. That is what makes the result
comparable: over the ten target dev subset, every target reaches its
recorded rpm through the link to within 2 rpm, F051 and G071 alike,
including the inverted-low-side and PA11/PA12-remap cases.

They cost about a minute a target rather than seconds, because a real
client has to sit on the other end. That is the reason the scripted tests
still write the throttle straight into the generator's registers and are
what a sweep runs.

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

## Running the SITL test scripts on real firmware

The scripted tests in `Mcu/SITL/tests/*.scr` (see the language reference
in `Mcu/SITL/run_test.py`) run against an emulated target's real
firmware with:

    python3 Mcu/Renode/run_test.py VIMDRONES_L431 Mcu/SITL/tests/test_spin_stop.scr

Same script language, same reports (timeline, variable change log, CSV,
PNG, interactive HTML) - the SITL runner's machinery wrapped around an
emulated MCU, so one script exercises both the portable C and each
family's register-level code. The firmware ELF is found in `obj/` (or
built) and rebuilt firmware matters: the eeprom layout and the tested
behaviour come from the code, so run `make <TARGET>` after pulling
firmware changes.

The wire protocols are AM32_GuiLink's, shared with `sitl_gui.py`; the
firmware variable watch uses the SITL's state-port packets (cmd 8) with
ELF-resolved addresses in the entries, since the emulator has no symbol
table. What necessarily differs from the SITL, and can need script
care:

- an `eepromBuffer.` write lands in the settings flash AND the
  firmware's in-RAM eepromBuffer, which is what a runtime DroneCAN
  parameter write does - the change is live, no reboot. Derived
  values that only `loadEEpromSettings()` computes at boot still
  need a scripted `reset`, exactly as on the bench
- `reset` is a machine reset and the virtual clock keeps running: the
  report time axis is continuous with no epoch stitching
- SITL-only exports (`sitl_tone_active`) read as constant 0, so the
  idiomatic "wait out the beeps" line passes immediately; the beeps
  cost virtual time here and are covered by `wait armed == 1`
- `speedup` through 1 paces the emulation; 0 or greater than 1 free-runs, with
  exactly 1 meaning real-time pacing

## What is here

    gen_target.py                   builds a platform for any F051, G071,
                                    L431 or G431 target out of Inc/targets.h
    platforms/stm32f051_base.repl   MCU-common. Vendored from Renode's
    platforms/stm32g071_base.repl   platforms/cpus/stm32f0.repl and
                                    stm32g0.repl (Antmicro, MIT - header
                                    retained) and edited
    platforms/stm32l431_base.repl   MCU-common, written from RM0394/RM0440
    platforms/stm32g431_base.repl   and the SVDs - Renode ships no stm32l4
                                    or stm32g4 platform
    peripherals/stm32/              our peripheral models, GPL-3, loaded at
                                    runtime with `include @...cs`; no Renode
                                    rebuild needed
    peripherals/common/             family-neutral models: the motor bridge,
                                    the throttle generator, the guilink
                                    server that serves the SITL wire
                                    protocols, the mcast CAN bridge, the
                                    comparator interface every family
                                    implements
    scripts/am32_f051.resc          one per family, shared by every generated
    scripts/am32_g071.resc          target script of that family
    scripts/am32_l431.resc
    scripts/am32_g431.resc

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
  TIM3 or TIM16 on the G071, always TIM15 on the L431 and G431.
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

`batchUs: 20` is the default: it costs ~1.7% in steady rpm against the
recorded batchUs 10 values (2.5% against the 2us reference) for another
1.2x once the GC fix above is in. That was validated by running the
scripted spin on all 105 recorded targets - everything passes, no BEMF
timeouts, no desyncs, worst rpm shift +1.84% - and the recorded values
in `data/expected_spin.json` were re-harvested at the new default, so
the tests hold targets to the current configuration exactly. The spread
across the table is real sampling error and not noise - two runs at the
same setting are bit-identical (same rpm, same zero-cross count, same
commutation interval), so the differences are attributable to batch
size alone.

A stationary, undriven motor is skipped entirely rather than integrated,
which is most of boot. That helps but does not eliminate the boot cost,
because the batch tick itself - four register reads and a P/Invoke - is
what is expensive, not the work it decides to skip.

The single biggest cost is mono's garbage collector, not the emulation:
Renode allocates on every emulated bus access - about 3GB per simulated
second at spin - and the bundled mono's ~4MB nursery then collects ~800
times a simulated second, each collection stopping every thread by
signal. The launchers therefore run Renode with
`MONO_GC_PARAMS=nursery-size=64m` (an explicit setting in the
environment wins): measured 1.58x wall time on the spin test, results
bit-identical. For scale, executing the translated Cortex-M0 code is
only 2-3% of the run; after the GC fix the remaining time is dominated
by Renode's C# time framework and generic bus dispatch. The table above
was measured before this fix.

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

- **A firmware self-reset used to kill the emulated CPU**, and the
  emulation carried on looking healthy afterwards. `cpu
  VectorTableOffset` is set once when the script loads, because there is
  no bootloader and the CPU has to be pointed at the app's table; Renode
  restores its default table at 0 on reset, where nothing is mapped but
  an empty 4K bootloader region, so SP and PC both read zero and the CPU
  halted. Everything else kept running - virtual time advanced (faster
  than usual, with no instructions to execute), the physics ticked, the
  state stream flowed - so from the outside it looked like an ESC that
  simply refused to arm. The four second scripted tests never saw it;
  any interactive session hits it within a minute, because AM32 reboots
  itself on signal loss. Fixed with a `macro reset` in each family
  script, which is exactly what Renode's own "No action for reset -
  macro `<machine>`.reset is not registered" was pointing at.
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
