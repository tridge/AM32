#!/usr/bin/env python3
'''
Builds a Renode platform (.repl) and script (.resc) for an AM32 target
straight out of Inc/targets.h, so a new target needs no hand-written
emulator description.

targets.h is not parsed. It is nested #ifdef several levels deep and has
#ifndef fallbacks at the end, so a parser here would drift from what the
compiler actually sees. Instead the real preprocessor is run over a stub
that defines the target, and the resolved macros are read back with
-dM. That takes about 50ms, which is why generating on demand is
practical rather than checking 52 platform files into the tree.

usage:
    gen_target.py TARGET [--outdir DIR]      write the pair, print both paths
    gen_target.py TARGET --run               generate, then launch renode
    gen_target.py TARGET --run --bootloader-elf ELF
                                             boot through a bootloader ELF
    gen_target.py TARGET --run --exec CMD    ... and script it
    gen_target.py TARGET --gui               ... driven by Mcu/SITL/sitl_gui.py
    gen_target.py TARGET --gui --cpusel N    ... pinned to host CPU N
    gen_target.py TARGET --sigrok            ... live logic analyser on TCP
    gen_target.py --list                     targets this can emulate

F051, F031, G071, G031, L431, G431, V203, E230, A153, F415 and F421
targets work, _CAN variants included - the L431's and F415's bxCAN and
the G431's FDCAN are all modelled. Anything else exits 77, as the test
harness does for a skip.
'''

import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))

# every macro the platform description needs. Read from the preprocessor
# rather than assumed, including the PHASE_*_COMP fallbacks targets.h
# applies at the bottom of the file.
WANTED = [
    'MCU_F051', 'MCU_G071', 'MCU_L431', 'MCU_G431', 'MCU_CH32V203',
    'MCU_F031', 'MCU_G031', 'MCU_GDE23', 'MCU_A153', 'MCU_AT415',
    'MCU_AT421',
    # the NXP LPCMP pair: which unit and which minus input each phase is on
    'PHASE_A_COMP_UNIT', 'PHASE_A_COMP_INP',
    'PHASE_B_COMP_UNIT', 'PHASE_B_COMP_INP',
    'PHASE_C_COMP_UNIT', 'PHASE_C_COMP_INP',
    'TEMP_ADC_CHANNEL',
    # the comparator-less F031/G031: external comparators on GPIO pins
    'PHASE_A_EXTI_PIN', 'PHASE_A_EXTI_PORT', 'PHASE_A_EXTI_LINE',
    'PHASE_B_EXTI_PIN', 'PHASE_B_EXTI_PORT', 'PHASE_B_EXTI_LINE',
    'PHASE_C_EXTI_PIN', 'PHASE_C_EXTI_PORT', 'PHASE_C_EXTI_LINE',
    'INVERTED_EXTI', 'IC_TIMER_CHANNEL',
    # the G431 SEQURE splits conversion across both ADC instances, and
    # its NTC rides in ADC1's sequence
    'USE_ADC_1_2', 'NTC_ADC_CHANNEL',
    # per-phase low-side alternate functions: the G4 puts TIM1_CH3N on
    # PB15 at AF4 where the other low sides are AF6
    'AF_A_LOW', 'AF_B_LOW', 'AF_C_LOW',
    # WS2812 LED strip, bit-banged on a GPIOB pin
    'USE_LED_STRIP', 'WS2812_PIN',
    # always defined by targets.h - 0 for a plain target, 1 for a _CAN
    # one - so it is the value that says whether CAN support is built in
    'DRONECAN_SUPPORT',
    # V203 configuration variants the models do not cover: PA2 as the
    # comparator instead of the OPA outputs, swapped fixed ADC channels,
    # and ADC throttle input
    'USE_PA2_AS_COMP', 'PA6_VOLTAGE', 'USE_ADC_INPUT',
    'IC_TIMER_REGISTER', 'INPUT_DMA_CHANNEL', 'INPUT_PIN', 'INPUT_PIN_PORT',
    'DEAD_TIME', 'FILE_NAME', 'EEPROM_START_ADD',
    # tenKhzRoutine()'s rate: armed_timeout_count counts to it, so it is
    # what turns that counter into "seconds held at zero throttle"
    'LOOP_FREQUENCY_HZ',
    'PHASE_A_COMP', 'PHASE_B_COMP', 'PHASE_C_COMP',
    # N_VARIANT targets put the three phases across both G0 comparators
    'N_VARIANT', 'MAIN_COMP',
    'PHASE_A_COMP_NUMBER', 'PHASE_B_COMP_NUMBER', 'PHASE_C_COMP_NUMBER',
    # a gate driver with one PWM and one enable per phase, instead of
    # separate high and low side pins
    'PWM_ENABLE_BRIDGE',
    'USE_INVERTED_LOW', 'USE_INVERTED_HIGH',
    'PHASE_A_GPIO_HIGH', 'PHASE_A_GPIO_PORT_HIGH',
    'PHASE_A_GPIO_LOW', 'PHASE_A_GPIO_PORT_LOW',
    'PHASE_B_GPIO_HIGH', 'PHASE_B_GPIO_PORT_HIGH',
    'PHASE_B_GPIO_LOW', 'PHASE_B_GPIO_PORT_LOW',
    'PHASE_C_GPIO_HIGH', 'PHASE_C_GPIO_PORT_HIGH',
    'PHASE_C_GPIO_LOW', 'PHASE_C_GPIO_PORT_LOW',
    'PHASE_A_GPIO_PWM', 'PHASE_A_GPIO_PORT_PWM',
    'PHASE_A_GPIO_ENABLE', 'PHASE_A_GPIO_PORT_ENABLE',
    'PHASE_B_GPIO_PWM', 'PHASE_B_GPIO_PORT_PWM',
    'PHASE_B_GPIO_ENABLE', 'PHASE_B_GPIO_PORT_ENABLE',
    'PHASE_C_GPIO_PWM', 'PHASE_C_GPIO_PORT_PWM',
    'PHASE_C_GPIO_ENABLE', 'PHASE_C_GPIO_PORT_ENABLE',
    'VOLTAGE_ADC_CHANNEL', 'CURRENT_ADC_CHANNEL', 'TARGET_VOLTAGE_DIVIDER',
    'MILLIVOLT_PER_AMP', 'CURRENT_OFFSET',
]

# The comparator's inverting-input selection, as (INMSEL, INMESEL)
# pairs, per family. The F0 names its choices after the pin; the G0 and
# L4 use the LL driver's IOn names. Only the L4 has a second field:
# INMESEL at CSR[26:25] extends the three-bit INMSEL, and four of its
# five choices collide on INMSEL 7, so the pair is what identifies the
# input. The F0 and G0 keep INMESEL 0.
INMSEL = {
    'f051': {'COMP_PA4': (4, 0), 'COMP_PA5': (5, 0), 'COMP_PA0': (6, 0)},
    'g071': {'LL_COMP_INPUT_MINUS_IO1': (6, 0),
             'LL_COMP_INPUT_MINUS_IO2': (7, 0),
             'LL_COMP_INPUT_MINUS_IO3': (8, 0)},
    'l431': {'LL_COMP_INPUT_MINUS_IO1': (6, 0),
             'LL_COMP_INPUT_MINUS_IO2': (7, 0),
             'LL_COMP_INPUT_MINUS_IO3': (7, 1),
             'LL_COMP_INPUT_MINUS_IO4': (7, 2),
             'LL_COMP_INPUT_MINUS_IO5': (7, 3)},
    # the G4 keeps the G0's four-bit INMSEL at [7:4] (no INMESEL)
    'g431': {'LL_COMP_INPUT_MINUS_IO1': (6, 0),
             'LL_COMP_INPUT_MINUS_IO2': (7, 0)},
    # the E230 writes whole CMP_CS words; CMPMSEL[6:4] carries the same
    # encodings as the F051's COMP1 (4=PA4, 5=PA5, 6=PA0)
    'e230': {'0x61': (6, 0), '0x41': (4, 0), '0x51': (5, 0)},
    # the F415 writes whole CTRLSTS1 words too, with the F051's INMSEL
    # encodings at [6:4]; the set bit 30 lands in the unused CMP2 half
    'f415': {'0x400000E5': (6, 0), '0x400000C5': (4, 0),
             '0x400000D5': (5, 0)},
    # so does the F421; CMPINVSEL[6:4] keeps the F051 encodings and
    # adds 7=PA2 (the AT_245 polling-mode groups)
    'f421': {'0x400000E5': (6, 0), '0x400000C5': (4, 0),
             '0x400000D5': (5, 0), '0x400000F5': (7, 0)},
}

# The raw ADC count an external NTC reads back, chosen so
# getNTCDegrees() decodes the physics default 38C on the target's own
# table in Inc/ntc_tables.h - the tables are per-target, and the
# SKYSTARS boards use a different curve where the common count would
# read 59C. The count is fixed rather than physics-tracking; the .repl
# comment at the emission site says so.
NTC_COUNTS_DEFAULT = 3104
NTC_COUNTS = {
    'SKYSTARS_F60_F421': 3536,
    'SKYSTARS_F80_F421': 3536,
}

# the capture timer, as (address, nvic line), per family
CAPTURE_TIMER = {
    'f051': {'TIM3': (0x40000400, 16), 'TIM15': (0x40014000, 20)},
    'g071': {'TIM3': (0x40000400, 16), 'TIM16': (0x40014400, 21)},
    'l431': {'TIM15': (0x40014000, 24)},
    'g431': {'TIM15': (0x40014000, 24)},
    'v203': {'TIM2': (0x40000000, 44)},
    # groups A/B capture on TIM2_CH3, group C on TIM16_CH1
    'f031': {'TIM2': (0x40000000, 15), 'TIM16': (0x40014400, 21)},
    'g031': {'TIM3': (0x40000400, 16)},
    # TIMER2 is ST's TIM3; the throttle rides its channel 0 on PB4
    'e230': {'TIMER2': (0x40000400, 16)},
    # group AT_D captures on TMR3_CH1 (PB4), group AT_H on TMR2_CH3
    # (PA2); the nvic lines are the timers' own global vectors
    'f415': {'TMR3': (0x40000400, 29), 'TMR2': (0x40000000, 28)},
    # AT_B captures on TMR3_CH1 (PB4), AT_C/E/F on TMR15_CH1 (PA2)
    'f421': {'TMR3': (0x40000400, 16), 'TMR15': (0x40014000, 20)},
}

# stock declaration and alternate-function map for whichever timers are
# NOT the capture timer, lifted from Renode's own stm32f0/stm32g0 repl
STOCK_TIMER = {
    'f051': {
        'TIM3': ('timer3', 0x40000400, 16, [
            '    0 -> gpioPortA#06@1 | gpioPortB#04@1 | gpioPortC#06@1',
            '    1 -> gpioPortA#07@1 | gpioPortB#05@1 | gpioPortC#07@1',
            '    2 -> gpioPortB#00@1 | gpioPortC#08@1',
            '    3 -> gpioPortB#01@1 | gpioPortC#09@1',
        ]),
        'TIM15': ('timer15', 0x40014000, 20, [
            '    0 -> gpioPortA#01@5 | gpioPortA#02@0 | gpioPortB#14@1 | gpioPortB#15@3',
            '    1 -> gpioPortA#03@0 | gpioPortB#15@1',
        ]),
    },
    'g071': {
        'TIM3': ('timer3', 0x40000400, 16, [
            '    0 -> gpioPortA#06@1 | gpioPortB#04@1 | gpioPortC#06@1',
            '    1 -> gpioPortA#07@1 | gpioPortB#05@1 | gpioPortC#07@1',
            '    2 -> gpioPortB#00@1',
            '    3 -> gpioPortB#01@1',
        ]),
        'TIM16': ('timer16', 0x40014400, 21, [
            '    0 -> gpioPortA#06@5 | gpioPortB#06@2 | gpioPortB#08@2 | gpioPortD#00@2',
        ]),
    },
    # TIM15 is the capture timer on every L431 group, so this entry is
    # never declared as a stock timer; it exists for capture_input_af()
    # to resolve the throttle pin's alternate function (PA2 is AF14)
    'l431': {
        'TIM15': ('timer15', 0x40014000, 24, [
            '    0 -> gpioPortA#02@14 | gpioPortB#14@14',
        ]),
    },
    # likewise on the G431, where TIM15_CH1 on PA2 is AF9
    'g431': {
        'TIM15': ('timer15', 0x40014000, 24, [
            '    0 -> gpioPortA#02@9 | gpioPortB#14@1',
        ]),
    },
    # F1-generation pin muxing has no per-pin AF numbers, so there is
    # no AF map to hold: TIM3 and TIM4 are declared by the base repl and
    # capture_input_af() finding nothing leaves the capture pin ungated
    'v203': {},
    # TIM2 and TIM16 trade the capture and 20kHz-loop roles per group;
    # the F031 A/B groups capture on TIM2's CHANNEL 3 (PA2 at AF2)
    'f031': {
        'TIM2': ('timer2', 0x40000000, 15, [
            '    0 -> gpioPortA#00@2 | gpioPortA#05@2 | gpioPortA#15@2',
            '    1 -> gpioPortA#01@2 | gpioPortB#03@2',
            '    2 -> gpioPortA#02@2 | gpioPortB#10@2',
            '    3 -> gpioPortA#03@2 | gpioPortB#11@2',
        ]),
        'TIM16': ('timer16', 0x40014400, 21, [
            '    0 -> gpioPortA#06@5 | gpioPortB#08@2',
        ]),
    },
    # TIMER2_CH0 on PB4 is GD AF1, the F0 TIM3_CH1 routing
    'e230': {
        'TIMER2': ('timer2', 0x40000400, 16, [
            '    0 -> gpioPortA#06@1 | gpioPortB#04@1 | gpioPortC#06@1',
        ]),
    },
    # TMR2 and TMR3 trade the capture role per F415 group; F1-generation
    # muxing, so as on the V203 there is no AF map to hold and
    # capture_input_af() finding nothing leaves the capture pin ungated
    # (the firmware keeps it in INPUT mode while capturing anyway)
    'f415': {
        'TMR2': ('timer2', 0x40000000, 28, []),
        'TMR3': ('timer3', 0x40000400, 29, []),
    },
    # TMR3_CH1 on PB4 is AT MUX_1, the F0 TIM3_CH1 routing; TMR15_CH1
    # on PA2 is MUX_0 - UN_TIM_Init() sets no mux for it, and the reset
    # AFR value 0 is already TMR15_CH1, so the capture gate is AF 0
    'f421': {
        'TMR3': ('timer3', 0x40000400, 16, [
            '    0 -> gpioPortA#06@1 | gpioPortB#04@1',
        ]),
        'TMR15': ('timer15', 0x40014000, 20, [
            '    0 -> gpioPortA#02@0 | gpioPortB#14@1',
        ]),
    },
    # TIM3 captures on every G031 group; TIM16 is the 20kHz loop timer
    'g031': {
        'TIM3': ('timer3', 0x40000400, 16, [
            '    0 -> gpioPortA#06@1 | gpioPortB#04@1 | gpioPortC#06@1',
            '    1 -> gpioPortA#07@1 | gpioPortB#05@1 | gpioPortC#07@1',
            '    2 -> gpioPortB#00@1',
            '    3 -> gpioPortB#01@1',
        ]),
        # no port D on this die, so its stock pin is dropped
        'TIM16': ('timer16', 0x40014400, 21, [
            '    0 -> gpioPortA#06@5 | gpioPortB#06@2 | gpioPortB#08@2',
        ]),
    },
}

# Everything that differs between the MCU families, in one place, so
# another family is a table entry plus a base .repl rather than a new
# code path.
#   dma_irq        nvic line the capture DMA channel raises
#   adc_dma        DMA channel index the ADC transfers on (0 based)
#   adc_base       where ADC1 decodes; the L4 moves it to 0x50040000
#   adc_sqr        the ADCv3 SQR-rank sequencer instead of CHSELR
#   timer_af       alternate function that routes TIM1 to the phase pins
#   extra_dma_irqs further (channel, nvic) wiring beyond the capture
#                  channel - the L4 has per-channel DMA interrupts and
#                  services the ADC transfer from one
FAMILY = {
    'f051': {
        'macro': 'MCU_F051',
        'timer_hz': 48000000,
        'gpio_a': 0x48000000,
        'throttle': 0x50000000,
        'bridge': 0x50000400,
        'guilink': 0x50000800,
        'dma_irq': 11,
        'adc_dma': 0,
        'adc_irq': 'nvicInput12@2',
        'adc_base': 0x40012400,
        'adc_sqr': False,
        # temperature sensor channel, its factory calibration pair, the
        # temperature the second point was taken at, and the Vref+ the
        # calibration was done with
        'temp_channel': 16,
        'ts_cal': (0x1FFFF7B8, 0x1FFFF7C2, 110, 3300),
        # no PA11/PA12 phase remap on this family
        'syscfg': 0,
        'timer_af': 2,
        'extra_dma_irqs': [],
    },
    'g071': {
        'macro': 'MCU_G071',
        'timer_hz': 64000000,
        # the G0 puts GPIO where the F0 has spare address space, so the
        # two made-up peripherals move out of the way
        'gpio_a': 0x50000000,
        # SYSCFG_CFGR1, whose bits 3 and 4 remap PA11/PA12
        'syscfg': 0x40010000,
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        'dma_irq': 9,
        'adc_dma': 1,
        'adc_irq': 'nvicInput12@2',
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 12,
        'ts_cal': (0x1FFF75A8, 0x1FFF75CA, 130, 3000),
        'timer_af': 2,
        'extra_dma_irqs': [],
    },
    'g431': {
        'macro': 'MCU_G431',
        'timer_hz': 160000000,
        'gpio_a': 0x48000000,
        # no PA11/PA12 phase remap on this family
        'syscfg': 0,
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        # DMA1_Channel1_IRQn: the capture is on channel 1 in every G4
        # group, routed by DMAMUX (stubbed; routing hardwired here)
        'dma_irq': 11,
        # ADC1 transfers on DMA1 channel 2
        'adc_dma': 1,
        # ADC1_2_IRQn; the firmware never unmasks it
        'adc_irq': 'nvic@18',
        'adc_base': 0x50000000,
        'adc_sqr': True,
        'temp_channel': 16,
        # second calibration point at 110C on the G4, not the G0/L4's 130
        'ts_cal': (0x1FFF75A8, 0x1FFF75CA, 110, 3000),
        # TIM1 routes to the phase pins on AF6 here
        'timer_af': 6,
        # two G4 groups put phase A's low side on PF0
        'gpio_f': 0x48001400,
        # DMA1_Channel2_IRQHandler does not exist in the firmware (the
        # ADC callback is polled from the 1kHz loop); wiring the line
        # would send a spurious interrupt into Default_Handler's
        # infinite loop, so deliberately none
        'extra_dma_irqs': [],
        # the platform's CAN peripheral, for the hub wiring
        'can_name': 'fdcan1',
    },
    'l431': {
        'macro': 'MCU_L431',
        'timer_hz': 80000000,
        'gpio_a': 0x48000000,
        # no PA11/PA12 phase remap on this family
        'syscfg': 0,
        # GPIO is back at the F0's address, but 0x60000000 is provably
        # free on every family so far - keep the G0's placement rather
        # than reason about the AHB2 gap below ADC1 at 0x50040000
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        # DMA1_Channel5_IRQn: the capture is on channel 5 in every L431
        # hardware group
        'dma_irq': 15,
        'adc_dma': 0,
        # ADC1_IRQn; the firmware never unmasked it, wired for honesty
        'adc_irq': 'nvic@18',
        'adc_base': 0x50040000,
        'adc_sqr': True,
        'temp_channel': 17,
        'ts_cal': (0x1FFF75A8, 0x1FFF75CA, 130, 3000),
        # TIM1 is AF1 on the L4 where the F0 and G0 use AF2
        'timer_af': 1,
        # DMA1_Channel1_IRQn: ADC_DMA_Callback() runs from this ISR as
        # well as from the 1kHz loop
        'extra_dma_irqs': [(0, 11)],
        # the platform's CAN peripheral, for the hub wiring
        'can_name': 'can1',
    },
    'v203': {
        'macro': 'MCU_CH32V203',
        # APB timers tick at 48MHz under the 96MHz core clock; all the
        # firmware's prescaler arithmetic assumes 48
        'timer_hz': 48000000,
        # F1-generation addressing throughout
        'gpio_a': 0x40010800,
        'syscfg': 0,
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        # DMA1_Channel5_IRQn as a PFIC interrupt number: the capture is
        # on channel 5 (TIM2_CH1's fixed F1 routing)
        'dma_irq': 31,
        # the ADC transfers on DMA1 channel 1
        'adc_dma': 0,
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 16,
        # ADCInit() hardwires the sequence instead of taking channels
        # from targets.h: rank 1 is CH1 (PA1, voltage), rank 2 is CH6
        # (PA6, current), rank 3 the internal temperature sensor
        'fixed_voltage_channel': 1,
        'fixed_current_channel': 6,
        # unused by the bridge's F1 mode, which reads CFG nibbles
        'timer_af': 0,
        # DMA1_Channel1 (ADC) and DMA1_Channel7 (telemetry TX) both have
        # real handlers in ch32v20x_it.c
        'extra_dma_irqs': [(0, 27), (6, 33)],
        # comparator duty is done by the OPA block, fixed per family and
        # declared in the base platform; there are no PHASE_x_COMP macros
        'opa_comp': True,
        # F1-style CFGLR/CFGHR GPIO decode in the bridge
        'f1_gpio': True,
        # the F1-generation rank-sequenced ADC model
        'wch_adc': True,
        'base_repl': 'ch32v203_base.repl',
    },
    'f031': {
        'macro': 'MCU_F031',
        'timer_hz': 48000000,
        'gpio_a': 0x48000000,
        'syscfg': 0,
        'throttle': 0x50000000,
        'bridge': 0x50000400,
        'guilink': 0x50000800,
        # the F0 fixed DMA interrupt map: the capture channel differs
        # per group (channel 1 for TIM2_CH3, channel 3 for TIM16_CH1
        # via the SYSCFG remap), so the line is looked up per channel
        'dma_irq': 9,
        'dma_irq_map': {0: 9, 1: 10, 2: 10, 3: 11, 4: 11},
        # the ADC transfers on channel 2 via the SYSCFG remap; its
        # interrupts stay masked (the 1kHz loop polls the buffer)
        'adc_dma': 1,
        'adc_irq': 'nvic@12',
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 16,
        'ts_cal': (0x1FFFF7B8, 0x1FFFF7C2, 110, 3300),
        'timer_af': 2,
        'extra_dma_irqs': [],
        # no comparator on the die: external comparators on GPIO pins,
        # phase derived from the armed EXTI trigger (F0 EXTI layout)
        'exti_bemf': True,
        'exti_base': 0x40010400,
        'exti_rtsr': 0x08,
        'exti_ftsr': 0x0C,
    },
    'g031': {
        'macro': 'MCU_G031',
        'timer_hz': 64000000,
        'gpio_a': 0x50000000,
        'syscfg': 0,
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        # DMA1_Channel1_IRQn: the capture is on channel 1
        'dma_irq': 9,
        # ADC on channel 2, serviced from DMA1_Channel2_3_IRQHandler
        'adc_dma': 1,
        'adc_irq': 'nvic@12',
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 12,
        'ts_cal': (0x1FFF75A8, 0x1FFF75CA, 130, 3000),
        'timer_af': 2,
        # ADC on channel 2 and telemetry TX on channel 3, sharing the
        # DMA1_Channel2_3 line (the USART model raises no TX requests
        # today, but the wiring is the honest one)
        'extra_dma_irqs': [(1, 10), (2, 10)],
        # no comparator on the die (G0 EXTI layout: RTSR1/FTSR1 at 0/4)
        'exti_bemf': True,
        'exti_base': 0x40021800,
        'exti_rtsr': 0x00,
        'exti_ftsr': 0x04,
    },
    'f415': {
        'macro': 'MCU_AT415',
        # 144MHz AHB with both APBs at /2; the F1 timer doubler puts
        # every timer back at 144MHz
        'timer_hz': 144000000,
        # F1-generation addressing throughout, as on the V203
        'gpio_a': 0x40010800,
        'syscfg': 0,
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        # the capture arrives on channel 6 via the Artery flexible
        # request mux (stubbed; routing hardwired) -> DMA1_Channel6_IRQn
        'dma_irq': 16,
        # the ADC transfers on channel 1
        'adc_dma': 0,
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 16,
        # ADC_Init() hardwires the sequence instead of taking channels
        # from targets.h: rank 1 is CH3 (PA3), which ADC_DMA_Callback()
        # reads as the voltage, rank 2 CH6 (PA6) as the current, rank 3
        # the internal temperature sensor. The per-target channel macros
        # are dead code on this family.
        'fixed_voltage_channel': 3,
        'fixed_current_channel': 6,
        # no factory calibration: getConvertedDegrees() applies fixed
        # constants, seeded as a reference word for the F1-generation
        # ADC model - with the slope RISING 4.2mV/C, unlike the WCH/GD
        'temp_slope_tenths': 42,
        # unused by the bridge's F1 mode, which reads CFG nibbles
        'timer_af': 0,
        # DMA1_Channel1 (ADC, ADC_DMA_Callback runs from its ISR) and
        # DMA1_Channel4 (telemetry TX) both have real handlers in
        # at32f415_it.c
        'extra_dma_irqs': [(0, 11), (3, 14)],
        # F1-style CFGLR/CFGHR GPIO decode in the bridge
        'f1_gpio': True,
        # the F1-generation rank-sequenced ADC model
        'wch_adc': True,
        'base_repl': 'at32f415_base.repl',
        # bxCAN, bit-for-bit the STM32's, for the hub wiring
        'can_name': 'can1',
    },
    'e230': {
        'macro': 'MCU_GDE23',
        # 72MHz core with all buses at /1, so the timers tick at 72MHz
        'timer_hz': 72000000,
        'gpio_a': 0x48000000,
        'syscfg': 0,
        'throttle': 0x50000000,
        'bridge': 0x50000400,
        'guilink': 0x50000800,
        # the classic F0 DMA interrupt map; the capture arrives on GD
        # channel 3 (the ST channel-4 slot) -> DMA_Channel3_4_IRQn
        'dma_irq': 11,
        'dma_irq_map': {0: 9, 1: 10, 2: 10, 3: 11, 4: 11},
        # the ADC transfers on GD channel 0 with its interrupts masked
        # (the 1kHz loop polls the buffer and re-triggers)
        'adc_dma': 0,
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 16,
        # no factory calibration on this part: the fixed-constant
        # formula (1430mV at 25C, -4.3mV/C) is seeded as a reference
        # word for the F1-generation ADC model, WCH-style
        'timer_af': 2,
        'extra_dma_irqs': [],
        # the F1-generation (RSQ-sequenced) ADC model
        'wch_adc': True,
        'base_repl': 'gd32e230_base.repl',
    },
    'a153': {
        # Nothing on this part is STM32-shaped: FlexPWM commutation read
        # directly by the bridge (IAM32PwmSource), CTIMER capture, LPCMP
        # pair, LPADC command chain, and the bidirectional dshot reply
        # leaves through LPSPI0 - so config() and the emitters take a
        # dedicated path and most keys here have no meaning
        'macro': 'MCU_A153',
        'throttle': 0x60000000,
        'bridge': 0x60000400,
        'guilink': 0x60000800,
        'base_repl': 'mcxa153_base.repl',
        'nxp': True,
    },
    'f421': {
        'macro': 'MCU_AT421',
        # 120MHz core with every bus at /1, so the timers tick at 120MHz
        'timer_hz': 120000000,
        'gpio_a': 0x48000000,
        'syscfg': 0,
        'throttle': 0x50000000,
        'bridge': 0x50000400,
        'guilink': 0x50000800,
        # the classic F0 DMA interrupt map; the capture arrives on
        # channel 4 (AT_B) or 5 (AT_C/E/F) -> DMA1_Channel5_4_IRQn
        'dma_irq': 11,
        'dma_irq_map': {0: 9, 1: 10, 2: 10, 3: 11, 4: 11},
        # the ADC transfers on channel 1; DMA1_Channel1_IRQHandler
        # exists (it calls ADC_DMA_Callback) but ADC_Init() leaves its
        # NVIC enable commented out, so the wire below stays masked
        'adc_dma': 0,
        'adc_base': 0x40012400,
        'adc_sqr': False,
        'temp_channel': 16,
        # no factory calibration on this part: getConvertedDegrees()'s
        # fixed constants are inverted through a seeded reference word
        # for the F1-generation ADC model, WCH-style (see the .resc).
        # The sensor voltage RISES with temperature at 4.2mV/C, the same
        # relationship as the F415's
        'temp_slope_tenths': 42,
        'timer_af': 2,
        'extra_dma_irqs': [(0, 9)],
        # the F1-generation (rank-sequenced) ADC model
        'wch_adc': True,
        'base_repl': 'at32f421_base.repl',
    },
}


# EEprom_t.buffer in Inc/eeprom.h - the settings block the firmware
# reads at boot, and what the GUI parameter editor fetches and writes
EEPROM_SIZE = 192

# where the application is linked, above the bootloader region. Also
# what the .resc points the reset vector at, since no bootloader is
# loaded. A PC below this is executing in the bootloader region. The
# DroneCAN targets link above a 16K bootloader instead of the plain 4K
# one; per-target the value is cfg['app_base'].
APP_BASE = 0x08001000
APP_BASE_CAN = 0x08004000

# the MCXA153's boot-ROM windows: the API tree, the driver table it
# points to, and where the emulation's flash blob is linked
A153_ROM_TREE = 0x03003FE0
A153_ROM_TABLE = 0x03003FA0
A153_ROM_CODE = 0x03004000


class Unsupported(Exception):
    pass


def macros(target, nm='arm-none-eabi-gcc'):
    '''resolved macros for a target, as the compiler sees them'''
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, 'target_probe.c')
        with open(src, 'w') as f:
            f.write('#define %s\n#include "targets.h"\n' % target)
        try:
            out = subprocess.check_output(
                [nm, '-E', '-dM', '-I', os.path.join(REPO, 'Inc'), src],
                stderr=subprocess.PIPE).decode()
        except OSError:
            raise Unsupported('%s not usable; it is needed to read targets.h' % nm)
        except subprocess.CalledProcessError as e:
            err = e.stderr.decode()
            # targets.h ends with an #error for an unrecognised target
            if 'Missing defines for target' in err:
                raise Unsupported('%s is not a target in Inc/targets.h' % target)
            raise Unsupported('preprocessing %s failed: %s'
                              % (target, ' '.join(err.split())[:200]))
    found = {}
    for line in out.splitlines():
        p = line.split(None, 2)
        if len(p) >= 2 and p[0] == '#define' and p[1] in WANTED:
            found[p[1]] = p[2].strip() if len(p) > 2 else ''
    return found


def suffix_number(macro, prefix, what):
    '''LL_ADC_CHANNEL_6 (or the SPL's ADC_CHANNEL_6) -> 6. The macro
       expands to a bitfield expression rather than a plain number, so
       the name is what carries it.'''
    for pfx in (prefix, prefix.replace('LL_', '', 1)):
        if macro and macro.startswith(pfx):
            try:
                return int(macro[len(pfx):])
            except ValueError:
                break
    raise Unsupported('cannot read %s from %s' % (what, macro))


def pin_name(port, pin):
    '''GPIOA + LL_GPIO_PIN_10 (or the SPL's GPIO_Pin_10, or the
       Artery GPIO_PINS_10) -> "PA10"'''
    if port.startswith('GPIO'):
        for prefix in ('LL_GPIO_PIN_', 'GPIO_Pin_', 'GPIO_PINS_',
                       'GPIO_PIN_'):
            if pin.startswith(prefix):
                return 'P%s%s' % (port[4:], pin[len(prefix):])
    raise Unsupported('cannot read pin %s %s' % (port, pin))


def pin_number(pin):
    '''LL_GPIO_PIN_2, GPIO_Pin_2 or GPIO_PINS_2 -> 2'''
    for prefix in ('LL_GPIO_PIN_', 'GPIO_Pin_', 'GPIO_PINS_',
                   'GPIO_PIN_'):
        if pin.startswith(prefix):
            try:
                return int(pin[len(prefix):])
            except ValueError:
                break
    raise Unsupported('cannot read a pin number from %s' % pin)


def capture_input_af(family, timer, port, pin, channel=0):
    '''alternate function that routes <timer>_CH<channel+1> to P<port><pin>.

       Parsed out of STOCK_TIMER's channel 0 line rather than restated:
       those "gpioPortA#02@0" entries are already the per pin AF map, and
       the table holds an entry for whichever timer is the capture timer
       on this target. Returns None if the pin is not a CH1 option, which
       leaves the gate off rather than guessing.'''
    spec = STOCK_TIMER.get(family, {}).get(timer)
    if spec is None or channel >= len(spec[3]):
        return None
    want = 'gpioPort%s#%02d@' % (port, pin)
    for entry in spec[3][channel].split('->', 1)[1].split('|'):
        entry = entry.strip()
        if entry.startswith(want):
            return int(entry[len(want):])
    return None


def config(target, nm='arm-none-eabi-gcc'):
    '''everything the .repl needs, or Unsupported with the reason'''
    m = macros(target, nm)
    if 'FILE_NAME' not in m:
        raise Unsupported('%s is not a target in Inc/targets.h' % target)

    family = None
    for fam, spec in FAMILY.items():
        if spec['macro'] in m:
            family = fam
            break
    if family is None:
        raise Unsupported('%s is not an F051, F031, G071, G031, L431, '
                          'G431, V203, E230, A153, F415 or F421 target; '
                          'those are the '
                          'only AM32 MCU families with a Renode platform '
                          'base so far' % target)

    # DRONECAN_SUPPORT is always defined - 0 on a plain target, 1 on a
    # _CAN one - so the value is the test, not the name or definedness.
    # The L431's bxCAN and the G431's FDCAN are modelled.
    dronecan = m.get('DRONECAN_SUPPORT', '0').strip() not in ('', '0')

    if family == 'a153':
        return a153_config(target, m, dronecan)

    if FAMILY[family].get('opa_comp'):
        # the OPA phase mapping and the fixed ADC channel order are
        # hardwired in the models; a target selecting one of these
        # variants would launch and silently misbehave
        for macro in ('USE_PA2_AS_COMP', 'PA6_VOLTAGE', 'USE_ADC_INPUT'):
            if macro in m:
                raise Unsupported('%s: %s is not modelled on the %s'
                                  % (target, macro, family))
    if dronecan and 'can_name' not in FAMILY[family]:
        raise Unsupported('%s: CAN is not modelled on the %s'
                          % (target, family))

    timer = m.get('IC_TIMER_REGISTER')
    if timer not in CAPTURE_TIMER[family]:
        raise Unsupported('capture timer %s is not modelled on the %s'
                          % (timer, family))
    # which capture/compare channel the throttle rides on: the F031's
    # A/B groups use TIM2_CH3, everything else channel 1 (the non-LL
    # spellings - the V203's "(1-1)", the E230's TIMER_CH_0 - are all
    # channel-1 hardware)
    icch = m.get('IC_TIMER_CHANNEL', '')
    if icch.startswith('LL_TIM_CHANNEL_CH'):
        try:
            capture_channel = int(icch[len('LL_TIM_CHANNEL_CH'):])
        except ValueError:
            raise Unsupported('cannot read IC_TIMER_CHANNEL from %r' % icch)
    elif icch.startswith('TMR_SELECT_CHANNEL_'):
        # the Artery spelling: group AT_H rides TMR2's channel 3
        try:
            capture_channel = int(icch[len('TMR_SELECT_CHANNEL_'):])
        except ValueError:
            raise Unsupported('cannot read IC_TIMER_CHANNEL from %r' % icch)
    elif icch in ('', '(1-1)', 'TIMER_CH_0'):
        # the non-LL spellings in the tree today all mean channel 1;
        # anything new must be classified rather than guessed at
        capture_channel = 1
    else:
        raise Unsupported('capture channel spelling %r is not recognised'
                          % icch)
    chan = m.get('INPUT_DMA_CHANNEL', '')
    if chan.startswith('LL_DMA_CHANNEL_'):
        dma_channel = int(chan[len('LL_DMA_CHANNEL_'):]) - 1
    elif chan.startswith('DMA1_CHANNEL'):
        # the Artery spelling
        dma_channel = int(chan[len('DMA1_CHANNEL'):]) - 1
    elif chan.startswith('DMA1_Channel'):
        # the SPL spelling the WCH targets use
        dma_channel = int(chan[len('DMA1_Channel'):]) - 1
    elif chan.startswith('DMA1_CHANNEL'):
        # the Artery spelling
        dma_channel = int(chan[len('DMA1_CHANNEL'):]) - 1
    elif chan.startswith('DMA_CH'):
        # GD numbers its channels from 0, matching the model's index
        dma_channel = int(chan[len('DMA_CH'):])
    else:
        raise Unsupported('cannot read DMA channel %s' % chan)

    bemf = None
    if FAMILY[family].get('exti_bemf'):
        # no comparator on the die: external comparator chips drive
        # GPIO pins, named by the PHASE_x_EXTI_* macros
        bemf = {}
        for ph in 'ABC':
            port = m.get('PHASE_%s_EXTI_PORT' % ph)
            pin = m.get('PHASE_%s_EXTI_PIN' % ph)
            line = m.get('PHASE_%s_EXTI_LINE' % ph)
            if not port or not pin or line is None:
                raise Unsupported('phase %s has no EXTI pin defined' % ph)
            try:
                bemf[ph] = (pin_name(port, pin), int(line))
            except ValueError:
                raise Unsupported('cannot read PHASE_%s_EXTI_LINE from %r'
                                  % (ph, line))
        # a board whose external comparator has the opposite polarity;
        # the firmware flips its edge bookkeeping and the model must
        # flip the driven level to match
        inverted_exti = 'INVERTED_EXTI' in m
    if FAMILY[family].get('opa_comp') or bemf is not None:
        # no comparator macros to read on these families
        comps = {}
    else:
        inmsel = INMSEL[family]
        comps = {}
        for ph in 'ABC':
            c = m.get('PHASE_%s_COMP' % ph)
            if c not in inmsel:
                raise Unsupported('comparator input %s for phase %s is not '
                                  'one of %s' % (c, ph,
                                                 '/'.join(sorted(inmsel))))
            comps[ph] = inmsel[c]

    # Which comparator senses each phase. The G0 N_VARIANT targets and
    # every G431 group split the phases across COMP1 and COMP2, saying
    # so with PHASE_x_COMP_NUMBER; everything else keeps all three on
    # MAIN_COMP, and the F051 has only COMP1. Keying on the macro rather
    # than on N_VARIANT is what lets the G431 groups through.
    if FAMILY[family].get('opa_comp') or bemf is not None:
        main = 0
        comp_of = {}
    else:
        main = 1 if family in ('f051', 'e230', 'f415',
                               'f421') else comp_number(
            m.get('MAIN_COMP', 'COMP2'))
        comp_of = {}
        for ph in 'ABC':
            num = m.get('PHASE_%s_COMP_NUMBER' % ph)
            comp_of[ph] = comp_number(num) if num else main

    if family == 'e230':
        if 'USE_ADC_INPUT' in m:
            raise Unsupported('%s: USE_ADC_INPUT is not modelled on the '
                              'e230' % target)
    if family == 'f415':
        # PA6_VOLTAGE would swap the hardwired ADC rank meanings in
        # Mcu/f415/Src/ADC.c; no current F415 target sets either
        for macro in ('PA6_VOLTAGE', 'USE_ADC_INPUT'):
            if macro in m:
                raise Unsupported('%s: %s is not modelled on the f415'
                                  % (target, macro))

    if 'USE_INVERTED_HIGH' in m:
        # active-low high sides (five F421 targets today); the bridge
        # refuses the flag rather than model it untested, so refuse at
        # generation with the reason instead of at machine load
        raise Unsupported('%s: USE_INVERTED_HIGH is not modelled by the '
                          'bridge' % target)

    # a PWM_ENABLE_BRIDGE target names its pins PWM and ENABLE rather
    # than HIGH and LOW; the bridge takes them in the same two slots
    enable_bridge = 'PWM_ENABLE_BRIDGE' in m
    sides = ('PWM', 'ENABLE') if enable_bridge else ('HIGH', 'LOW')
    pins = {}
    for ph in 'ABC':
        for slot, side in zip(('HIGH', 'LOW'), sides):
            port = m.get('PHASE_%s_GPIO_PORT_%s' % (ph, side))
            pin = m.get('PHASE_%s_GPIO_%s' % (ph, side))
            if port is None or pin is None:
                raise Unsupported('phase %s has no %s pin defined' % (ph, side))
            pins[ph + slot] = pin_name(port, pin)

    if family == 'e230':
        for ph in 'ABC':
            if pins[ph + 'HIGH'] not in ('PA8', 'PA9', 'PA10'):
                # the GD_B group puts a phase's high side on the
                # complementary pin and compensates with an inverted PWM
                # mode on that one channel - a pairing the bridge cannot
                # represent
                raise Unsupported('%s: high side %s is a complementary '
                                  'output; the swapped pair is not '
                                  'modelled' % (target, pins[ph + 'HIGH']))

    def number(name, default):
        try:
            return int(m.get(name, default))
        except ValueError:
            raise Unsupported('%s is not a number: %s' % (name, m.get(name)))

    eeprom = m.get('EEPROM_START_ADD', '')
    # the macro is a cast expression, e.g. "(uint32_t)0x0800F800"
    eeprom = eeprom.split(')')[-1].strip()
    try:
        eeprom_addr = int(eeprom, 0)
    except ValueError:
        raise Unsupported('cannot read EEPROM_START_ADD from %r'
                          % m.get('EEPROM_START_ADD'))

    # three G431 targets have no current shunt at all; -1 means no
    # channel ever matches in the ADC model and the reading stays 0
    cur = m.get('CURRENT_ADC_CHANNEL')
    ntc = m.get('NTC_ADC_CHANNEL')
    volt = m.get('VOLTAGE_ADC_CHANNEL')
    if family == 'e230' and 'PA6_VOLTAGE' in m:
        # rank order stays (voltage-macro, current-macro, temp), but
        # ADC_DMA_Callback() reads rank 1 as the voltage - so the
        # channel the model must scale as voltage is the current
        # macro's, and vice versa
        m = dict(m)
        m['VOLTAGE_ADC_CHANNEL'], m['CURRENT_ADC_CHANNEL'] = (
            m.get('CURRENT_ADC_CHANNEL'), m.get('VOLTAGE_ADC_CHANNEL'))
        volt = m.get('VOLTAGE_ADC_CHANNEL')
        cur = m.get('CURRENT_ADC_CHANNEL')
    if 'fixed_voltage_channel' in FAMILY[family]:
        # the WCH and Artery ADC drivers hardwire their sequence instead
        # of reading channel macros from targets.h - the F415 targets
        # define VOLTAGE/CURRENT_ADC_CHANNEL, but Mcu/f415/Src/ADC.c
        # never looks at them
        voltage_channel = FAMILY[family]['fixed_voltage_channel']
        current_channel = FAMILY[family]['fixed_current_channel']
    else:
        voltage_channel = suffix_number(volt, 'LL_ADC_CHANNEL_',
                                        'voltage channel')
        current_channel = suffix_number(cur, 'LL_ADC_CHANNEL_',
                                        'current channel') if cur else -1
    return {
        'target': target,
        'family': family,
        'name': m.get('FILE_NAME', target).strip('"').strip(),
        'voltage_channel': voltage_channel,
        'current_channel': current_channel,
        'ntc_channel': suffix_number(ntc, 'LL_ADC_CHANNEL_',
                                     'ntc channel') if ntc else -1,
        'ntc_counts': NTC_COUNTS.get(target, NTC_COUNTS_DEFAULT),
        'adc12': 'USE_ADC_1_2' in m,
        'voltage_divider': number('TARGET_VOLTAGE_DIVIDER', 110),
        'millivolt_per_amp': number('MILLIVOLT_PER_AMP', 20),
        'current_offset': number('CURRENT_OFFSET', 0),
        'timer': timer,
        'timer_addr': CAPTURE_TIMER[family][timer][0],
        'timer_irq': CAPTURE_TIMER[family][timer][1],
        # reference manual counts channels from 1, Renode from 0
        'dma_channel': dma_channel,
        'throttle_pin': pin_name(m['INPUT_PIN_PORT'], m['INPUT_PIN']),
        'input_base': FAMILY[family]['gpio_a']
                      + 0x400 * (ord(m['INPUT_PIN_PORT'][4:]) - ord('A')),
        'input_pin': pin_number(m['INPUT_PIN']),
        'input_af': capture_input_af(
            family, timer, m['INPUT_PIN_PORT'][4:],
            pin_number(m['INPUT_PIN']), capture_channel - 1),
        'capture_channel': capture_channel,
        'bemf': bemf,
        'inverted_exti': FAMILY[family].get('exti_bemf', False)
                         and 'INVERTED_EXTI' in m,
        'dead_time': m.get('DEAD_TIME', '?'),
        'loop_hz': number('LOOP_FREQUENCY_HZ', 20000),
        'eeprom_addr': eeprom_addr,
        'dronecan': dronecan,
        'app_base': APP_BASE_CAN if dronecan else APP_BASE,
        'comps': comps,
        'comp_of': comp_of,
        'main_comp': main,
        'enable_bridge': enable_bridge,
        'inverted_low': 'USE_INVERTED_LOW' in m,
        'inverted_high': 'USE_INVERTED_HIGH' in m,
        'pins': pins,
        # per-phase low-side AF overrides (AF_x_LOW): the G4 SEQURE puts
        # TIM1_CH3N on PB15 at AF4 where everything else is AF6
        'low_af': {ph: suffix_number(m['AF_%s_LOW' % ph], 'LL_GPIO_AF_',
                                     'low-side AF')
                   if m.get('AF_%s_LOW' % ph) else None
                   for ph in 'ABC'},
        # WS2812.c hardwires the strip to GPIOB; only the pin varies
        'ws2812_pin': pin_number(m['WS2812_PIN'])
                      if 'USE_LED_STRIP' in m and m.get('WS2812_PIN')
                      else None,
    }


def inmsel_name(pair):
    '''(7, 2) -> "7.2", (5, 0) -> "5": comment-friendly input code'''
    return '%d.%d' % pair if pair[1] else '%d' % pair[0]


def comp_number(macro):
    '''COMP2 -> 2'''
    if macro not in ('COMP1', 'COMP2'):
        raise Unsupported('comparator %r is not COMP1 or COMP2' % macro)
    return int(macro[4:])


def a153_config(target, m, dronecan):
    '''the NXP MCXA153: none of the STM32 macro shapes apply, so this
       replaces the body of config() for the family'''
    if dronecan:
        raise Unsupported('%s: CAN is not modelled on the a153' % target)
    if 'USE_ADC_INPUT' in m:
        raise Unsupported('%s: USE_ADC_INPUT is not modelled on the a153'
                          % target)

    def number(name):
        try:
            return int(m[name], 0)
        except (KeyError, ValueError):
            raise Unsupported('%s is not a number: %r' % (name, m.get(name)))

    # which LPCMP unit and which minus input each phase is on
    comps = {}
    for ph in 'ABC':
        unit = m.get('PHASE_%s_COMP_UNIT' % ph, '')
        if unit not in ('CMP0', 'CMP1'):
            raise Unsupported('phase %s comparator unit %r is not CMP0 or '
                              'CMP1' % (ph, unit))
        comps[ph] = (int(unit[3:]), number('PHASE_%s_COMP_INP' % ph))

    port = m.get('INPUT_PIN_PORT', '')
    if not port.startswith('PORT'):
        raise Unsupported('cannot read INPUT_PIN_PORT from %r' % port)

    eeprom = m.get('EEPROM_START_ADD', '').split(')')[-1].strip()
    try:
        eeprom_addr = int(eeprom, 0)
    except ValueError:
        raise Unsupported('cannot read EEPROM_START_ADD from %r'
                          % m.get('EEPROM_START_ADD'))

    return {
        'target': target,
        'family': 'a153',
        'name': m.get('FILE_NAME', target).strip('"').strip(),
        'voltage_channel': number('VOLTAGE_ADC_CHANNEL'),
        'current_channel': number('CURRENT_ADC_CHANNEL'),
        'temp_channel': number('TEMP_ADC_CHANNEL'),
        'voltage_divider': number('TARGET_VOLTAGE_DIVIDER'),
        'millivolt_per_amp': number('MILLIVOLT_PER_AMP'),
        'current_offset': number('CURRENT_OFFSET'),
        'comps': comps,
        'input_gpio': int(port[4:]),
        'input_pin': number('INPUT_PIN'),
        'dead_time': m.get('DEAD_TIME', '?'),
        'loop_hz': int(m.get('LOOP_FREQUENCY_HZ', 20000)),
        'eeprom_addr': eeprom_addr,
        'dronecan': False,
        'app_base': 0x4000,
    }


def a153_platform(cfg, sigrok=False):
    '''the FRDM_A153 overlay over mcxa153_base.repl'''
    return '\n'.join([
        '// GENERATED by Mcu/Renode/gen_target.py from Inc/targets.h -',
        '// edit that, or the generator, not this file.',
        '//',
        '// target %s (%s), DEAD_TIME %s' % (cfg['target'], cfg['name'],
                                             cfg['dead_time']),
        '// throttle in on GPIO%d pin %d, captured by CTIMER0 into eDMA'
        % (cfg['input_gpio'], cfg['input_pin']),
        '// channel 0; comparators: A=CMP%d/IN%d B=CMP%d/IN%d C=CMP%d/IN%d'
        % (cfg['comps']['A'] + cfg['comps']['B'] + cfg['comps']['C']),
        '',
        'using "%s"' % os.path.join(HERE, 'platforms', 'mcxa153_base.repl'),
        '',
        'cmpmux: Miscellaneous.MCXA_LpcmpMux @ sysbus 0x%08X'
        % (FAMILY['a153']['guilink'] + 0x800),
        '    phaseAComp: %d' % cfg['comps']['A'][0],
        '    phaseAMsel: %d' % cfg['comps']['A'][1],
        '    phaseBComp: %d' % cfg['comps']['B'][0],
        '    phaseBMsel: %d' % cfg['comps']['B'][1],
        '    phaseCComp: %d' % cfg['comps']['C'][0],
        '    phaseCMsel: %d' % cfg['comps']['C'][1],
        '',
        'cmp0: Miscellaneous.MCXA_Lpcmp @ sysbus 0x400B1000',
        '    mux: cmpmux',
        '    index: 0',
        '    IRQ -> nvic@64',
        '',
        'cmp1: Miscellaneous.MCXA_Lpcmp @ sysbus 0x400B2000',
        '    mux: cmpmux',
        '    index: 1',
        '    IRQ -> nvic@65',
        '',
        'adc0: Analog.MCXA_Lpadc @ sysbus 0x400AF000',
        '    voltageChannel: %d' % cfg['voltage_channel'],
        '    currentChannel: %d' % cfg['current_channel'],
        '    temperatureChannel: %d' % cfg['temp_channel'],
        '    voltageDivider: %d' % cfg['voltage_divider'],
        '    millivoltPerAmp: %d' % cfg['millivolt_per_amp'],
        '    currentOffsetMv: %d' % cfg['current_offset'],
        '    0 -> edma@51',
        '',
        'throttle: Miscellaneous.AM32ThrottleGenerator @ sysbus 0x%08X'
        % FAMILY['a153']['throttle'],
        '    0 -> ctimer0@0 | gpio%d@%d%s' % (
            cfg['input_gpio'], cfg['input_pin'],
            ' | sigrok@0' if sigrok else ''),
        '',
        '// the ESC end of the shared wire, for the bootloader\'s',
        '// bit banged serial reply (see the generic families above)',
        'gpio%d:' % cfg['input_gpio'],
        '    %d -> throttle@0' % cfg['input_pin'],
        '',
        '// the phase pin arguments are never used on this family: the',
        '// FlexPWM is an IAM32PwmSource, which replaces the GPIO+timer',
        '// decode',
        'bridge: Miscellaneous.AM32_F051_Bridge @ sysbus 0x%08X'
        % FAMILY['a153']['bridge'],
        '    phaseAHigh: "PA8"',
        '    phaseALow: "PB13"',
        '    phaseBHigh: "PA9"',
        '    phaseBLow: "PB14"',
        '    phaseCHigh: "PA10"',
        '    phaseCLow: "PB15"',
        '',
        'guilink: Miscellaneous.AM32_GuiLink @ sysbus 0x%08X'
        % FAMILY['a153']['guilink'],
        '    eepromAddress: 0x%08X' % cfg['eeprom_addr'],
        '    eepromSize: %d' % EEPROM_SIZE,
        '',
    ] + (sigrok_block(FAMILY['a153']) if sigrok else []))


def a153_rom_api(outdir, nm):
    '''Compile the ROM flash-driver blob (sim/mcxa_rom_api.c) with the
       firmware's own toolchain, link it at the ROM code base, and return
       (bin_path, monitor commands writing the API tree and driver table
       with the symbol addresses read back from the ELF). The table
       layout is flash_driver_interface_t in Mcu/a153/Inc/
       mcxa153_rom_api.h.'''
    src = os.path.join(HERE, 'sim', 'mcxa_rom_api.c')
    elf = os.path.join(outdir, 'mcxa_rom_api.elf')
    binp = os.path.join(outdir, 'mcxa_rom_api.bin')
    objcopy = nm.replace('gcc', 'objcopy')
    nm_tool = nm.replace('gcc', 'nm')
    try:
        subprocess.check_call(
            [nm, '-mcpu=cortex-m33', '-mthumb', '-Os', '-ffreestanding',
             '-nostdlib', '-Wl,-Ttext=0x%08X' % A153_ROM_CODE,
             '-Wl,--entry=rom_flash_init', '-o', elf, src])
        subprocess.check_call([objcopy, '-O', 'binary', elf, binp])
        out = subprocess.check_output([nm_tool, elf]).decode()
    except (OSError, subprocess.CalledProcessError) as e:
        raise Unsupported('cannot build the a153 ROM flash blob: %s' % e)
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] in 'Tt':
            # thumb entry points: bit 0 set
            syms[parts[2]] = int(parts[0], 16) | 1
    table = ['rom_flash_init', 'rom_flash_erase_sector', 'rom_flash_ok',
             'rom_flash_program_page', 'rom_flash_ok', 'rom_flash_ok',
             'rom_flash_ok', 'rom_flash_ok', 'rom_flash_get_property',
             'rom_flash_ok', 'rom_flash_ok', 'rom_flash_ok',
             'rom_flash_read']
    missing = [s for s in set(table) if s not in syms]
    if missing:
        raise Unsupported('a153 ROM blob lacks symbols: %s'
                          % ', '.join(missing))
    lines = [
        '',
        '# the masked-ROM flash driver: a compiled blob, its API tree at',
        '# 0x03003FE0 and the driver table the tree points to',
        'sysbus LoadBinary @%s 0x%08X' % (binp, A153_ROM_CODE),
        'sysbus WriteDoubleWord 0x%08X 0x0' % A153_ROM_TREE,
        'sysbus WriteDoubleWord 0x%08X 0x%08X' % (A153_ROM_TREE + 4,
                                                  A153_ROM_TABLE),
        'sysbus WriteDoubleWord 0x%08X 0x0' % (A153_ROM_TREE + 8),
    ]
    for i, name in enumerate(table):
        lines.append('sysbus WriteDoubleWord 0x%08X 0x%08X'
                     % (A153_ROM_TABLE + 4 * i, syms[name]))
    lines.append('sysbus WriteDoubleWord 0x%08X 0x00010000'
                 % (A153_ROM_TABLE + 4 * len(table)))
    lines.append('')
    return binp, lines


def bemf_block(cfg, spec):
    '''the comparator-less families: an EXTI-watching BEMF block whose
       outputs drive the real phase pins. EXTICR is stored but not
       honoured by the EXTI models - every port's pin n reaches line n -
       so any phase POWER pin sharing a line number with a BEMF input on
       another port is disconnected from the EXTI by restating its
       port's pin map (the ws2812 trick).'''
    lines = {ph: cfg['bemf'][ph][1] for ph in 'ABC'}
    L = [
        '// No comparator on this die: external comparator chips drive GPIO',
        '// pins, watched through EXTI. The model derives the sensed phase',
        '// from which line has a trigger armed, and drives the real pins so',
        '// both the polled IDR reads and the EXTI edges are the firmware\'s.',
        'bemf: Miscellaneous.AM32_ExtiBemf @ sysbus 0x%08X'
        % (spec['guilink'] + 0xC00),
        '    extiBase: 0x%08X' % spec['exti_base'],
        '    rtsrOffset: 0x%02X' % spec['exti_rtsr'],
        '    ftsrOffset: 0x%02X' % spec['exti_ftsr'],
    ] + [
        '    phase%sLine: %d' % (ph, lines[ph]) for ph in 'ABC'
    ] + ([
        '    inverted: true',
    ] if cfg.get('inverted_exti') else []) + [
        '    %d -> gpioPort%s@%s' % (i, cfg['bemf'][ph][0][1],
                                     cfg['bemf'][ph][0][2:])
        for i, ph in enumerate('ABC')
    ]
    # Disconnect every OTHER port's pin from each BEMF line: EXTICR is
    # stored but not honoured, so PF6 toggling as a gate-driver pin or
    # PB6 as telemetry TX would otherwise alias the PA6 BEMF line. The
    # BEMF port keeps its pin; every other declared port loses that pin
    # number. (An LED-strip overlay would restate a port again and undo
    # this; no current comparator-less target has one.)
    bemf_pins = {(cfg['bemf'][ph][0][1], int(cfg['bemf'][ph][0][2:]))
                 for ph in 'ABC'}
    ports = ['A', 'B', 'C', 'F']
    drop = {}
    for ph in 'ABC':
        line = cfg['bemf'][ph][1]
        for port in ports:
            if (port, line) not in bemf_pins:
                drop.setdefault(port, set()).add(line)
    for port in sorted(drop):
        L += [
            '',
            '// line%s %s belong%s to a BEMF input on another port; EXTICR is'
            % ('s' if len(drop[port]) > 1 else '',
               ', '.join(str(n) for n in sorted(drop[port])),
               '' if len(drop[port]) > 1 else 's'),
            '// not honoured, so this port\'s pin%s disconnect%s from the EXTI'
            % (('s', '') if len(drop[port]) > 1 else ('', 's')),
            'gpioPort%s:' % port,
        ] + [
            '    %d -> exti@%d' % (n, n) for n in range(16)
            if n not in drop[port]
        ]
    return L


def comp_block(cfg):
    '''the comparator declaration, which is the biggest family split'''
    if cfg.get('bemf'):
        return bemf_block(cfg, FAMILY[cfg['family']])
    if FAMILY[cfg['family']].get('opa_comp'):
        return [
            '// no comparator peripheral on this family: the OPA block in the',
            '// base platform routes the phases, selected by OPA->CR writes',
            '// from changeCompInput(), with its outputs on PA3/PA4 feeding',
            '// EXTI lines 3 and 4 through the GPIO port wiring.',
        ]
    if cfg['family'] == 'f415':
        return [
            '// Artery CMP block on its own page. Nearly the F051 COMP1',
            '// layout - enable bit 0, INMSEL at [6:4] (4 is PA4, 5 is PA5,',
            '// 6 is PA0), output at bit 14 - gating EXTI line 19, which has',
            '// its own vector (CMP1_IRQn 70) rather than sharing an EXINT',
            '// one. CMP2 in the upper half is unused by every target.',
            'comp: Miscellaneous.AM32_AT32_Cmp @ sysbus <0x40002400, +0x400>',
        ] + [
            '    phase%sInmsel: %d' % (p, cfg['comps'][p][0]) for p in 'ABC'
        ] + [
            '    0 -> exti@19',
        ]
    if cfg['family'] == 'f421':
        return [
            '// SCFG and CMP share the first APB2 page, as on the F051, but',
            '// the register is Artery\'s own: one ctrlsts word at 0x1C with',
            '// the inverting select at [6:4] (4=PA4, 5=PA5, 6=PA0, 7=PA2)',
            '// and the output at bit 30. EXTI line 21, ADC1_CMP_IRQn.',
            'syscfgcomp: Miscellaneous.AM32_AT32_Cmp @ sysbus <0x40010000, +0x400>',
        ] + [
            '    phase%sInmsel: %d' % (p, cfg['comps'][p][0]) for p in 'ABC'
        ] + [
            '    ctrlstsOffset: 0x1C',
            '    outputBit: 30',
            '    polarityBit: 15',
            '    0 -> exti@21',
        ]
    if cfg['family'] in ('f051', 'e230'):
        return [
            '// SYSCFG and COMP share a register page on the F051. Line 21 is',
            "// COMP1's EXTI line. The phase map is CSR[6:4], the COMP1 INMSEL",
            '// field: 4 is PA4, 5 is PA5, 6 is PA0.',
            'syscfgcomp: Miscellaneous.AM32_STM32F0_SysCfgComp @ sysbus <0x40010000, +0x400>',
        ] + [
            '    phase%sInmsel: %d' % (p, cfg['comps'][p][0]) for p in 'ABC'
        ] + [
            '    0 -> exti@21',
            '    1 -> exti@22',
        ]
    if cfg['family'] == 'l431':
        return [
            '// Two comparators as on the G0, but the inverting input is the',
            '// (INMSEL, INMESEL) pair - CSR[6:4] and CSR[26:25] - because',
            '// IO2..IO5 all share INMSEL 7. COMP1 is EXTI line 21, COMP2 is',
            '// line 22, the same lines as the F051.',
            'comp: Miscellaneous.AM32_STM32L4_Comp @ sysbus <0x40010200, +0x100>',
        ] + [
            '    phase%sInmsel: %d' % (p, cfg['comps'][p][0]) for p in 'ABC'
        ] + [
            '    phase%sInmesel: %d' % (p, cfg['comps'][p][1]) for p in 'ABC'
        ] + [
            '    phase%sComp: %d' % (p, cfg['comp_of'][p]) for p in 'ABC'
        ] + [
            '    mainComp: %d' % cfg['main_comp'],
            '    0 -> exti@21',
            '    1 -> exti@22',
        ]
    if cfg['family'] == 'g431':
        return [
            '// The G4 keeps the G0 comparator layout - four-bit INMSEL at',
            '// [7:4], output at bit 30 - so the G0 model serves, but the',
            '// EXTI lines are the F051/L431 ones: COMP1 is 21, COMP2 is 22.',
            '// Every G431 group splits the phases across both comparators.',
            'comp: Miscellaneous.AM32_STM32G0_Comp @ sysbus <0x40010200, +0x100>',
        ] + [
            '    phase%sInmsel: %d' % (p, cfg['comps'][p][0]) for p in 'ABC'
        ] + [
            '    phase%sComp: %d' % (p, cfg['comp_of'][p]) for p in 'ABC'
        ] + [
            '    mainComp: %d' % cfg['main_comp'],
            '    0 -> exti@21',
            '    1 -> exti@22',
        ]
    return [
        '// Two separate comparators on the G0, next to each other rather',
        '// than sharing the SYSCFG page. COMP1 is EXTI line 17, COMP2 is',
        '// line 18. The phase map is CSR[7:4], the INMSEL field: 6 is IO1,',
        '// 7 is IO2, 8 is IO3 (PB3, PB7 and PA2 on COMP2).',
        'comp: Miscellaneous.AM32_STM32G0_Comp @ sysbus <0x40010200, +0x100>',
    ] + [
        '    phase%sInmsel: %d' % (p, cfg['comps'][p][0]) for p in 'ABC'
    ] + [
        '    phase%sComp: %d' % (p, cfg['comp_of'][p]) for p in 'ABC'
    ] + [
        '    mainComp: %d' % cfg['main_comp'],
        '    0 -> exti@17',
        '    1 -> exti@18',
    ]


def adc_block(cfg, spec):
    '''the ADC declaration(s). One instance almost everywhere; the G431
       USE_ADC_1_2 targets convert temperature and the NTC on ADC1 and
       voltage and current on ADC2, each with its own DMA channel.'''
    head = [
        '// The stock ADC models have no DMA output, and AM32 reads its',
        '// conversions only through DMA into ADCDataDMA[], so against them',
        '// the firmware saw no voltage or current at all. There is',
        '// deliberately no external event frequency: AM32 starts',
        '// conversions in software from the 1kHz loop, and modelling a',
        '// hardware trigger as well made sequences overlap forever.',
    ]
    if spec.get('wch_adc'):
        return head + [
            'adc: Analog.AM32_WCH_Adc @ sysbus 0x%08X' % spec['adc_base'],
            '    voltageChannel: %d' % cfg['voltage_channel'],
            '    currentChannel: %d' % cfg['current_channel'],
            '    voltageDivider: %d' % cfg['voltage_divider'],
            '    millivoltPerAmp: %d' % cfg['millivolt_per_amp'],
            '    currentOffsetMv: %d' % cfg['current_offset'],
            '    temperatureChannel: %d' % spec['temp_channel'],
        ] + ([
            # the F415 sensor voltage RISES with temperature, unlike the
            # WCH/GD default the model inverts
            '    tempSlopeTenthsMvPerC: %d' % spec['temp_slope_tenths'],
        ] if 'temp_slope_tenths' in spec else []) + ([
            # the F421 USE_NTC targets read their temperature from an
            # external NTC; unmapped it reads 0, which decodes to 400C
            # and the thermal clamp cuts the duty before the motor starts
            '    ntcChannel: %d' % cfg['ntc_channel'],
            '    ntcCounts: %d' % cfg['ntc_counts'],
        ] if cfg['ntc_channel'] >= 0 else []) + [
            '    0 -> dma@%d' % spec['adc_dma'],
        ]
    cal = [
        '    temperatureChannel: %d' % spec['temp_channel'],
        '    tsCal1: 0x%08X' % spec['ts_cal'][0],
        '    tsCal2: 0x%08X' % spec['ts_cal'][1],
        '    tsCal2Temp: %d' % spec['ts_cal'][2],
        '    tsCalVrefMv: %d' % spec['ts_cal'][3],
    ]
    if not cfg['adc12']:
        return head + [
            'adc: Analog.AM32_STM32F0_ADC @ sysbus 0x%08X' % spec['adc_base'],
            '    voltageChannel: %d' % cfg['voltage_channel'],
            '    currentChannel: %d' % cfg['current_channel'],
            '    voltageDivider: %d' % cfg['voltage_divider'],
            '    millivoltPerAmp: %d' % cfg['millivolt_per_amp'],
            '    currentOffsetMv: %d' % cfg['current_offset'],
        ] + cal + ([
            '    sqrSequencer: true',
        ] if spec['adc_sqr'] else []) + [
            '    0 -> dma@%d' % spec['adc_dma'],
            '    1 -> %s' % spec['adc_irq'],
        ]
    return head + [
        '// ADC1 converts the temperature sensor and the NTC (the NTC has',
        '// no model mapping and reads 0; converted_degrees uses the',
        '// internal sensor). Sized to stop short of ADC2 at +0x100.',
        'adc: Analog.AM32_STM32F0_ADC @ sysbus <0x%08X, +0x100>'
        % spec['adc_base'],
        '    voltageChannel: -1',
        '    currentChannel: -1',
        '    voltageDivider: %d' % cfg['voltage_divider'],
        '    millivoltPerAmp: %d' % cfg['millivolt_per_amp'],
        '    currentOffsetMv: %d' % cfg['current_offset'],
    ] + cal + [
        '    sqrSequencer: true',
        '    0 -> dma@%d' % spec['adc_dma'],
        '    1 -> %s' % spec['adc_irq'],
        '',
        '// ADC2: voltage and current, transferred on DMA1 channel 4',
        'adc2: Analog.AM32_STM32F0_ADC @ sysbus <0x%08X, +0x100>'
        % (spec['adc_base'] + 0x100),
        '    voltageChannel: %d' % cfg['voltage_channel'],
        '    currentChannel: %d' % cfg['current_channel'],
        '    voltageDivider: %d' % cfg['voltage_divider'],
        '    millivoltPerAmp: %d' % cfg['millivolt_per_amp'],
        '    currentOffsetMv: %d' % cfg['current_offset'],
        '    temperatureChannel: -1',
        '    sqrSequencer: true',
        '    0 -> dma@3',
        '',
        '// the ADC12 common registers, write-readback only',
        'adccommon: Miscellaneous.AM32_RegisterFile @ sysbus <0x%08X, +0x100>'
        % (spec['adc_base'] + 0x300),
    ]


def platform(cfg, sigrok=False):
    fam = cfg['family']
    if fam == 'a153':
        return a153_platform(cfg, sigrok)
    spec = FAMILY[fam]
    others = [t for t in STOCK_TIMER[fam] if t != cfg['timer']]
    cap = 'timer%s' % re.sub(r'\D', '', cfg['timer'])
    tp = cfg['throttle_pin']
    # per-phase low-side AFs that differ from the timer AF, so the
    # bridge holds each pin to exactly the mux that routes its channel
    low_afs = [(ph, af) for ph, af in sorted(cfg['low_af'].items())
               if af is not None and af != spec['timer_af']]
    L = [
        '// GENERATED by Mcu/Renode/gen_target.py from Inc/targets.h -',
        '// edit that, or the generator, not this file.',
        '//',
        '// target %s (%s), DEAD_TIME %s' % (cfg['target'], cfg['name'],
                                             cfg['dead_time']),
        '// throttle in on %s, captured by %s_CH%d into DMA1 channel %d'
        % (tp, cfg['timer'], cfg.get('capture_channel', 1),
           cfg['dma_channel'] + 1),
        ('// comparator: A=%s B=%s C=%s (INMSEL[.INMESEL]), on COMP%s/%s/%s'
         % (tuple(inmsel_name(cfg['comps'][p]) for p in 'ABC')
            + tuple(cfg['comp_of'][p] for p in 'ABC'))) if cfg['comps']
        else ('// comparator: external, on GPIO pins %s/%s/%s (EXTI %d/%d/%d)'
              % tuple([cfg['bemf'][p][0] for p in 'ABC']
                      + [cfg['bemf'][p][1] for p in 'ABC'])
              if cfg.get('bemf')
              else '// comparator: OPA-routed, fixed per family'),
        '// bridge: %s' % ('gate driver PWM + enable per phase'
                           if cfg['enable_bridge'] else
                           ('high and low side, low side inverted'
                            if cfg['inverted_low'] else 'high and low side')),
        '',
        # absolute: these are generated into a scratch or obj directory,
        # so a path relative to the platforms tree would not resolve
        'using "%s"' % os.path.join(HERE, 'platforms',
                                    spec.get('base_repl',
                                             'stm32%s_base.repl' % fam)),
        '',
        '// Throttle capture. The stock timer model has no input capture, so',
        '// this is ours; it raises a DMA request rather than an interrupt',
        '// per edge, as the firmware programs it to.',
        '//',
        '// The stock alternate-function pin map for this timer is',
        '// deliberately absent: a later connection block replaces an',
        '// earlier one, so including it would silently override the DMA and',
        '// NVIC wiring here and captures would go to GPIO pins instead.',
        '%s: Timers.AM32_STM32_CaptureTimer @ sysbus 0x%08X'
        % (cap, cfg['timer_addr']),
        '    frequency: %d' % spec['timer_hz'],
    ] + ([
        '    channel: %d' % cfg['capture_channel'],
    ] if cfg.get('capture_channel', 1) != 1 else []) + ([
        # so a capture only happens when the pin is actually routed to
        # this timer, not merely toggling
        '    inputBase: 0x%08X' % cfg['input_base'],
        '    inputPin: %d' % cfg['input_pin'],
        '    inputAf: %d' % cfg['input_af'],
    ] if cfg['input_af'] is not None else []) + [
        '    0 -> dma@%d' % cfg['dma_channel'],
        '    1 -> nvic@%d' % cfg['timer_irq'],
        # output 2 is the bidirectional dshot reply, going back to the
        # generator, which owns the shared wire. It belongs in this
        # declaration rather than a later "timerN:" block: a connection
        # block replaces an earlier one outright, so a separate block
        # here would silently drop the dma and nvic lines above.
        '    2 -> throttle@0',
        '',
    ]

    # On F031 and G031 the non-capture member is the periodic 20kHz
    # control-loop timer. A general STM32 timer creates four unused
    # capture/compare clock entries and updates them on every tick; the
    # basic model represents the update interrupt with one entry.
    basic_other = fam in ('f031', 'g031')
    # The representative target captures throttle on only one member of these
    # timer pairs. Keep the alternate register-visible without paying for the
    # stock model's five scheduled channel/counter entries.
    freerunning_other = fam in ('g071', 'f415', 'f421')

    # whichever timers this target is not capturing with
    for other in others:
        oname, oaddr, oirq, oaf = STOCK_TIMER[fam][other]
        L += [
            '%s: Timers.%s @ sysbus 0x%08X'
            % (oname, ('AM32_STM32_BasicTimer' if basic_other else
                       'AM32_STM32_FreeRunningTimer' if freerunning_other else
                       'STM32_Timer'), oaddr),
            '    frequency: %d' % spec['timer_hz'],
            '    initialLimit: 0xFFFF',
            '    -> nvic@%d' % oirq,
            '',
        ]
        # the F1-generation families have no per-pin AF map to restate
        if oaf and not basic_other and not freerunning_other:
            L += ['%s:' % oname] + oaf + ['']

    L += [
        '// the nvic lines the DMA channels raise: the capture channel',
        '// always, plus any per-channel interrupt the family services',
        'dma:',
        '    %d -> nvic@%d' % (cfg['dma_channel'],
                               spec.get('dma_irq_map', {}).get(
                                   cfg['dma_channel'], spec['dma_irq'])),
    ] + [
        '    %d -> nvic@%d' % (ch, irq) for ch, irq in spec['extra_dma_irqs']
    ] + [
        '',
    ] + comp_block(cfg) + [
        '',
        '// Couples the emulated bridge to the SITL motor physics.',
        '// LibraryPath and ConfigPath are set from the .resc, since they',
        '// are absolute paths. The phase pins are not the same on every',
        '// target: many rotate the phases across these six pins, so they',
        '// are stated rather than defaulted.',
        'bridge: Miscellaneous.AM32_F051_Bridge @ sysbus 0x%08X' % spec['bridge'],
        '    batchUs: 20',
        '    timerHz: %d' % spec['timer_hz'],
        '    timerAf: %d' % spec['timer_af'],
        '    gpioABase: 0x%08X' % spec['gpio_a'],
        '    syscfgBase: 0x%08X' % spec['syscfg'],
        '    gpioBBase: 0x%08X' % (spec['gpio_a'] + 0x400),
        '    gpioCBase: 0x%08X' % (spec['gpio_a'] + 0x800),
    ] + ([
        '    gpioFBase: 0x%08X' % spec['gpio_f'],
    ] if spec.get('gpio_f') else []) + [
        '    phase%s%s: "%s"' % (p, side.capitalize(), cfg['pins'][p + side])
        for p in 'ABC' for side in ('HIGH', 'LOW')
    ] + ([
        '    f1Gpio: true',
    ] if spec.get('f1_gpio') else []) + ([
        '    topology: "enable"',
    ] if cfg['enable_bridge'] else []) + ([
        '    invertedLow: true',
    ] if cfg['inverted_low'] else []) + ([
        '    invertedHigh: true',
    ] if cfg['inverted_high'] else []) + [
        '    lowAf%s: %d' % (ph, af) for ph, af in low_afs
    ] + [
        '',
    ] + adc_block(cfg, spec) + [
        '',
        'throttle: Miscellaneous.AM32ThrottleGenerator @ sysbus 0x%08X'
        % spec['throttle'],
    ] + ([
        # TIM15 can consume a complete DShot frame and reply DMA at once,
        # avoiding tens of thousands of host clock callbacks per second.
        '    batchDshotFrames: true',
    ] if fam == 'l431' else []) + [
        '    0 -> %s@0 | gpioPort%s@%s%s' % (
            cap, tp[1], tp[2:], ' | sigrok@0' if sigrok else ''),
        '',
        '// The other half of the shared wire: what the ESC itself drives',
        '// on the signal pin. The firmware only drives it for a',
        '// bidirectional dshot reply, which leaves through the reply',
        '// source instead, but the bootloader bit bangs its serial',
        '// answer straight onto the pin, and that has to reach the',
        '// generator to be decoded.',
        'gpioPort%s:' % tp[1],
        '    %s -> exti@%s | throttle@0' % (tp[2:], tp[2:]),
        '',
        '// Serves the SITL wire protocols to sitl_gui.py. The ports are',
        '// left closed here and opened from the .resc, so a run that is',
        '// not driving a GUI cannot collide with a real SITL on the same',
        '// machine.',
        'guilink: Miscellaneous.AM32_GuiLink @ sysbus 0x%08X' % spec['guilink'],
        '    eepromAddress: 0x%08X' % cfg['eeprom_addr'],
        '    eepromSize: %d' % EEPROM_SIZE,
        '',
    ] + ([
        '// Bridges the bxCAN to the SITL multicast CAN bus, so',
        '// dronecan_gui_tool on mcast:N sees the emulated ESC. The',
        '// registers are a debug window; the socket stays closed until',
        '// the launcher sets Bus.',
        'canmcast: CAN.AM32_CanMcast @ sysbus 0x%08X' % (spec['guilink'] + 0x400),
        '',
    ] if cfg['dronecan'] else []) + (sigrok_block(spec) if sigrok else []) \
      + ws2812_block(cfg, spec, sigrok)
    return '\n'.join(L)


def sigrok_block(spec):
    return [
        '// Event-driven logic analyser: real wire edges plus bridge state',
        '// changes, rasterised only when an ipdbg-la client captures.',
        'sigrok: Miscellaneous.AM32_Sigrok @ sysbus 0x%08X'
        % (spec['guilink'] + 0x1000),
        '',
    ]


def ws2812_block(cfg, spec, sigrok=False):
    '''the LED strip decoder and the pin rewiring that feeds it. The
       strip pin keeps its EXTI route: a connection block REPLACES the
       base's, so the whole port B map is restated with the one pin
       fanned out.'''
    pin = cfg['ws2812_pin']
    if pin is None:
        return []
    routes = []
    for p in range(16):
        if p == pin:
            routes.append('    %d -> exti@%d | ws2812@0%s'
                          % (p, p, ' | sigrok@1' if sigrok else ''))
        else:
            routes.append('    %d -> exti@%d' % (p, p))
    return [
        '// Decodes the WS2812 strip AM32 bit-bangs on PB%d; the GUI' % pin,
        '// shows the colour through the guilink device-info reply.',
        'ws2812: Miscellaneous.AM32_Ws2812 @ sysbus 0x%08X'
        % (spec['guilink'] + 0x800),
        '',
        'gpioPortB:',
    ] + routes + ['']


def bootloader_lma_segments(bootloader_elf, outdir, target):
    '''Extract initialized data whose ELF virtual address is RAM but whose
       physical/load address is flash. Renode's physical-address LoadELF
       zero-fills the segment's whole RAM-sized p_memsz at p_paddr, corrupting
       an application immediately above the bootloader. Loading the ELF by
       virtual address avoids that, and these small files restore only the
       real p_filesz initializer bytes at their flash load addresses.'''
    with open(bootloader_elf, 'rb') as f:
        data = f.read()
    if len(data) < 52 or data[:4] != b'\x7fELF':
        raise Unsupported('%s is not an ELF file' % bootloader_elf)
    if data[4] != 1 or data[5] != 1:
        raise Unsupported('%s is not a little-endian ELF32 file'
                          % bootloader_elf)
    phoff = struct.unpack_from('<I', data, 28)[0]
    phentsize, phnum = struct.unpack_from('<HH', data, 42)
    if phentsize < 32 or phoff + phentsize * phnum > len(data):
        raise Unsupported('%s has an invalid program header table'
                          % bootloader_elf)
    segments = []
    for i in range(phnum):
        off = phoff + i * phentsize
        p_type, p_offset, p_vaddr, p_paddr, p_filesz, _p_memsz = \
            struct.unpack_from('<IIIIII', data, off)
        if p_type != 1 or p_filesz == 0 or p_paddr == p_vaddr:
            continue
        if p_offset + p_filesz > len(data):
            raise Unsupported('%s has an invalid load segment' % bootloader_elf)
        path = os.path.abspath(os.path.join(
            outdir, '%s_bootloader_lma_%d.bin' % (target, len(segments))))
        with open(path, 'wb') as f:
            f.write(data[p_offset:p_offset + p_filesz])
        segments.append((path, p_paddr))
    return segments


def bootloader_script(cfg, bootloader_elf, lma_segments=None):
    '''Commands appended to a generated target script when a bootloader is
       requested. LoadELF makes it the initial program; replacing the family
       reset macro makes watchdog/NVIC resets return there too, as hardware
       does. The application ELF remains loaded above the bootloader so the
       bootloader can validate and jump to it normally.'''
    if bootloader_elf is None:
        return []
    elf = os.path.abspath(bootloader_elf)
    lma_loads = ['sysbus LoadBinary @%s 0x%08X' % item
                 for item in (lma_segments or [])]
    if cfg['family'] == 'v203':
        # CH32V203 executes through the zero-based flash alias and has no
        # Cortex-M vector table. Its CPU reset does not reload an ELF entry
        # point, so reload the bootloader just as the family script normally
        # reloads the application.
        return [
            '',
            '# Boot from the supplied CH32V203 bootloader. The app loaded by',
            '# the family script remains at 0x1000.',
            'sysbus LoadELF @%s true' % elf,
        ] + lma_loads + [
            'macro reset',
            '"""',
            '    sysbus LoadELF @%s true' % elf,
        ] + ['    %s' % line for line in lma_loads] + [
            '"""',
            '',
        ]
    boot_base = 0 if cfg['family'] == 'a153' else 0x08000000
    lines = [
        '',
        '# Load by virtual address so ELF RAM/BSS segments go to RAM rather',
        '# than zero-filling their flash LMA over the application. Explicit',
        '# LoadBinary commands below restore initialized-data bytes at LMA.',
        'sysbus LoadELF @%s true' % elf,
    ] + lma_loads + [
        'cpu VectorTableOffset 0x%08X' % boot_base,
        'macro reset',
        '"""',
        '    cpu VectorTableOffset 0x%08X' % boot_base,
        '"""',
    ]
    if cfg['family'] in ('f051', 'f031'):
        # These Cortex-M0 parts have no VTOR. On hardware initAfterJump()
        # copies the application vectors to SRAM and remaps address zero;
        # Renode does not model that SYSCFG remap, so move its equivalent
        # vector-table override when the application reaches the same point.
        lines += [
            'cpu AddSymbolHook "initAfterJump" '
            '"cpu.VectorTableOffset = 0x%08X"' % cfg['app_base'],
        ]
    return lines + ['']


def script(cfg, repl_path, bootloader_elf=None, bootloader_lmas=None):
    return '\n'.join([
        ':name: AM32 %s' % cfg['target'],
        ':description: boots an AM32 %s firmware ELF' % cfg['target'],
        '',
        '# GENERATED by Mcu/Renode/gen_target.py. Set $repo, $elf and',
        '# $eeprom before including it; see am32_%s.resc for what they'
        % cfg['family'],
        '# mean.',
        '',
        '$repo?=@.',
        '$platform=@%s' % repl_path,
        # the eeprom address is per target, not per family: a 128k part
        # keeps its settings at 0x0801F800 where a 64k one uses 0x0800F800
        '$eeprom_addr=0x%08X' % cfg['eeprom_addr'],
        # so is the app base: DroneCAN builds link above a 16K bootloader
        '$app_base=0x%08X' % cfg['app_base'],
        'include $repo/Mcu/Renode/scripts/am32_%s.resc' % cfg['family'],
        '',
    ] + ([
        # after the include, so the machine exists. The hub is wiring
        # only; no host socket opens until something sets canmcast Bus.
        'emulation CreateCANHub "canhub"',
        'connector Connect sysbus.%s canhub' % FAMILY[cfg['family']]['can_name'],
        'connector Connect sysbus.canmcast canhub',
        '',
    ] if cfg['dronecan'] else []) + bootloader_script(
        cfg, bootloader_elf, bootloader_lmas))


def throttle_address(target, nm='arm-none-eabi-gcc'):
    '''where to write a pulse width to drive the throttle generator. Not
       a constant: it sits at 0x50000000 on the F051, which is where the
       G0 puts GPIOA, so the G0 moves it to 0x60000000.'''
    return FAMILY[config(target, nm)['family']]['throttle']


def capture_timer_name(target, nm='arm-none-eabi-gcc'):
    '''Renode peripheral name of the peripheral holding the Reply*
       telemetry: the input capture timer on the STM32 families, but
       LPSPI0 on the a153, where the bidirectional dshot reply leaves
       through the SPI rather than the timer'''
    cfg = config(target, nm)
    if cfg['family'] == 'a153':
        return 'lpspi0'
    return 'timer%s' % re.sub(r'\D', '', cfg['timer'])


def generate(target, outdir, nm='arm-none-eabi-gcc', sigrok=False,
             bootloader_elf=None):
    '''write the pair, return (resc, repl). Raises Unsupported.'''
    cfg = config(target, nm)
    os.makedirs(outdir, exist_ok=True)
    repl = os.path.join(outdir, '%s.repl' % target)
    resc = os.path.join(outdir, '%s.resc' % target)
    extra = []
    bootloader_lmas = []
    if bootloader_elf is not None:
        bootloader_lmas = bootloader_lma_segments(
            bootloader_elf, outdir, target)
    if cfg['family'] == 'a153':
        # the ROM flash-driver blob, compiled with the same toolchain;
        # the writes go after the include so the machine exists
        _, extra = a153_rom_api(outdir, nm)
    with open(repl, 'w') as f:
        f.write(platform(cfg, sigrok))
    with open(resc, 'w') as f:
        f.write(script(cfg, repl, bootloader_elf, bootloader_lmas)
                + '\n'.join(extra))
    return resc, repl


def default_eeprom(path, model, extra=None):
    '''An eeprom is not optional: Renode zero-fills unbacked memory where
       erased flash reads 0xFF, so a missing one sends loadEEpromSettings()
       down the migration path. INPUT_SIGNAL_TYPE 0 is mandatory too - the
       default is DSHOT_IN, and with dshot set detectInput() never calls
       checkServo(), so a servo signal is ignored with no diagnostic.
       extra: additional overrides, e.g. CAN_NODE for a DroneCAN target.'''
    sys.path.insert(0, os.path.join(REPO, 'Mcu', 'SITL'))
    try:
        import sitl_params
    except ImportError:
        raise Unsupported('cannot build a default eeprom (Mcu/SITL not '
                          'importable); pass --eeprom')
    overrides = {'INPUT_SIGNAL_TYPE': 0}
    overrides.update(extra or {})
    try:
        import json
        motor = json.load(open(model)).get('motor', {})
    except (OSError, ValueError):
        motor = {}
    # the firmware is tuned for the motor it thinks it has; leaving these
    # at the defaults is worth 20% of measured rpm
    for name, (want, _help) in sitl_params.model_checks(motor).items():
        overrides[name] = want
    with open(path, 'wb') as f:
        f.write(bytes(sitl_params.build_image(overrides)))
    return overrides


# Firmware globals worth watching, in print order. Read by name from the
# ELF symbol table, at the right width per symbol - desync_happened is a
# byte here but a word under DRONECAN_SUPPORT, and reading four bytes of
# a one byte variable returns neighbouring globals.
WATCH = ['armed', 'running', 'inputSet', 'input', 'adjusted_input',
         'duty_cycle', 'commutation_interval', 'zero_crosses', 'step',
         'bemf_timeout_happened', 'desync_happened',
         # what the firmware makes of the ADC: 10mV, 10mA, degrees
         'battery_voltage', 'actual_current', 'converted_degrees',
         # which input protocol detectInput() settled on, and how the
         # dshot decode is faring; absent from the symbol table on
         # targets built without them, which write_status() tolerates
         'dshot', 'servoPwm', 'dshot_telemetry', 'smallestnumber',
         'average_signal_pulse', 'dshot_frametime', 'dshot_goodcounts',
         'dshot_badcounts']

STATUS_PY = r'''
# GENERATED by Mcu/Renode/gen_target.py. Loaded into the Renode monitor
# so status() and watch() are available at the prompt.
#
# This is IronPython 2 inside the monitor: print is a statement.
SYMS = %(syms)s
_sb = monitor.Machine['sysbus']
_bridge = monitor.Machine['sysbus.bridge']

def rd(name):
    """one firmware global, read at its own width"""
    addr, size = SYMS[name]
    if size == 1:
        return _sb.ReadByte(addr)
    if size == 2:
        return _sb.ReadWord(addr)
    return _sb.ReadDoubleWord(addr)

def motor():
    """what the physics says the motor is really doing"""
    return {'rpm': _bridge.Rpm, 'theta': _bridge.Theta,
            'ia': _bridge.CurrentA, 'ib': _bridge.CurrentB,
            'ic': _bridge.CurrentC}

FIELDS = %(fields)s

def status():
    """one line of firmware state plus motor truth"""
    out = []
    for f in FIELDS:
        try:
            out.append('%%s=%%d' %% (f, rd(f)))
        except KeyError:
            pass
    m = motor()
    print ' '.join(out)
    print 'rpm=%%.1f theta=%%.2f ia=%%.2f ib=%%.2f ic=%%.2f' %% (
        m['rpm'], m['theta'], m['ia'], m['ib'], m['ic'])

def csv_header():
    return ','.join(['t_s'] + FIELDS + ['rpm', 'ia', 'ib', 'ic'])

def csv_row():
    t = monitor.Machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds
    vals = [str(t)]
    for f in FIELDS:
        try:
            vals.append(str(rd(f)))
        except KeyError:
            vals.append('')
    m = motor()
    vals += ['%%.3f' %% m['rpm'], '%%.4f' %% m['ia'], '%%.4f' %% m['ib'],
             '%%.4f' %% m['ic']]
    return ','.join(vals)

_log = []

def sample():
    """append one CSV row; call between RunFor steps"""
    _log.append(csv_row())

def save(path):
    """write everything sample() collected, for plotting"""
    f = open(path, 'w')
    f.write(csv_header() + '\n')
    for r in _log:
        f.write(r + '\n')
    f.close()
    print 'wrote %%d rows to %%s' %% (len(_log), path)
'''


def symbol_addresses(elf, names, nm='arm-none-eabi-nm'):
    '''address of each named symbol, for the ones the ELF has. Absent
       symbols are simply left out: a target built without one should
       lose that readout, not fail to start.'''
    try:
        out = subprocess.check_output([nm, elf]).decode()
    except (OSError, subprocess.CalledProcessError):
        return {}
    found = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) == 3 and f[2] in names:
            found[f[2]] = int(f[0], 16)
    return found


def write_status(path, elf, nm='arm-none-eabi-nm'):
    '''per-target monitor helpers, with the symbol table baked in'''
    try:
        out = subprocess.check_output([nm, '-S', elf]).decode()
    except (OSError, subprocess.CalledProcessError):
        raise Unsupported('%s not usable; needed for status()' % nm)
    syms = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) == 4 and f[3] in WATCH:
            syms[f[3]] = (int(f[0], 16), int(f[1], 16))
    have = [f for f in WATCH if f in syms]
    with open(path, 'w') as f:
        f.write(STATUS_PY % {'syms': repr(syms), 'fields': repr(have)})
    return have


def has_debug_info(elf, readelf):
    '''True if the ELF carries DWARF. Without it gdb can only show
       addresses, so it is worth failing early rather than after two
       windows have opened.'''
    try:
        out = subprocess.check_output([readelf, '-S', elf],
                                      stderr=subprocess.DEVNULL).decode()
    except (OSError, subprocess.CalledProcessError):
        return None  # cannot tell; do not block on it
    return '.debug_info' in out


def find_gdb(explicit):
    '''prefer the toolchain the firmware was built with'''
    if explicit:
        return explicit
    for osdir in ('linux', 'macos'):
        cand = os.path.join(REPO, 'tools', osdir,
                            'xpack-arm-none-eabi-gcc-10.3.1-2.3', 'bin',
                            'arm-none-eabi-gdb')
        if os.path.exists(cand):
            return cand
    return 'arm-none-eabi-gdb'


GDB_LAUNCH = r'''#!/bin/bash
# GENERATED by Mcu/Renode/gen_target.py.
# Waits for Renode's gdb stub to listen, then attaches.
echo "waiting for the Renode gdb server on port $PORT ..."
for i in $(seq 1 300); do
    if (exec 3<>/dev/tcp/127.0.0.1/$PORT) 2>/dev/null; then
        exec 3<&-
        break
    fi
    sleep 0.1
done
exec "$GDB" \
    -ex "set confirm off" \
    -ex "set pagination off" \
    -ex "target remote 127.0.0.1:$PORT" \
    "$ELF"
'''


def write_gdb_launcher(path, gdb, elf, port):
    with open(path, 'w') as f:
        f.write(GDB_LAUNCH.replace('$PORT', str(port))
                          .replace('$GDB', gdb).replace('$ELF', elf))
    os.chmod(path, 0o755)


def find_elf(target):
    '''newest firmware ELF in obj/ for exactly this target, or None.
       A bare glob is not enough: AM32_X_*.elf also matches the CAN
       sibling AM32_X_CAN_*.elf, which sorts after every version of X.
       Versions compare numerically, not lexically: 2.20 is newer than
       2.9.'''
    pat = re.compile(r'AM32_%s_([0-9]+(?:\.[0-9]+)*)\.elf$' % re.escape(target))
    found = []
    for f in glob.glob(os.path.join(REPO, 'obj', 'AM32_%s_*.elf' % target)):
        m = pat.search(os.path.basename(f))
        if m:
            found.append((tuple(int(p) for p in m.group(1).split('.')), f))
    return max(found)[1] if found else None


def find_renode(explicit=None):
    '''the renode to launch: an explicit path wins, else a dotnet
    portable installed under tools/linux (not vendored - a 77MB download
    from the renode release page - but 1.76x faster on this workload
    with bit-identical results), else the vendored mono portable, else
    whatever $PATH has.'''
    if explicit:
        return explicit
    for pattern in ('renode_*dotnet*', 'renode_*portable'):
        found = sorted(glob.glob(os.path.join(
            REPO, 'tools', 'linux', pattern, 'renode')))
        if found:
            return found[-1]
    return 'renode'


def renode_env():
    '''Environment for launching Renode. The bundled mono defaults to a
    ~4MB SGen nursery, and Renode allocates on every emulated bus access
    - about 3GB per simulated second while the motor spins - so the
    default collects ~800 times a simulated second and spends roughly a
    third of the run suspending threads for the collector. A 64MB nursery
    measured 1.58x faster on the spin test, results bit-identical. An
    explicit MONO_GC_PARAMS in the caller's environment wins.'''
    env = dict(os.environ)
    env.setdefault('MONO_GC_PARAMS', 'nursery-size=64m')
    # for a CoreCLR renode: DllImport("am32sim") does not consult the
    # RTLD_GLOBAL namespace there, so the library has to be findable by
    # name. Harmless under the bundled mono.
    obj = os.path.join(REPO, 'obj')
    prior = env.get('LD_LIBRARY_PATH')
    env['LD_LIBRARY_PATH'] = obj if not prior else obj + os.pathsep + prior
    return env


def launch_gui(port, state_port, can_bus=-1):
    '''start Mcu/SITL/sitl_gui.py against the link ports. It needs PySide6,
       which the SITL keeps in its own venv, so prefer that interpreter -
       the GUI's own diagnostic for a missing PySide6 tells you to run it
       from there anyway. can_bus >= 0 enables the DroneCAN panel on the
       matching mcast bus, for targets whose CAN is emulated.'''
    gui = os.path.join(REPO, 'Mcu', 'SITL', 'sitl_gui.py')
    venv = os.path.join(REPO, 'Mcu', 'SITL', 'venv', 'bin', 'python3')
    python = venv if os.path.exists(venv) else sys.executable
    cmd = [python, gui, '--port', str(port), '--state-port', str(state_port),
           '--backend', 'renode']
    if can_bus >= 0:
        cmd += ['--renode-can', '--can-uri', 'mcast:%d' % can_bus]
    try:
        return subprocess.Popen(cmd)
    except OSError as e:
        print('could not start the GUI (%s): %s' % (' '.join(cmd), e))
        return None


def find_terminal():
    '''an xterm to put gdb in, or None to run it inline'''
    for t in ('xterm', 'x-terminal-emulator', 'gnome-terminal', 'konsole'):
        for d in os.environ.get('PATH', '').split(os.pathsep):
            if os.path.exists(os.path.join(d, t)):
                return os.path.join(d, t)
    return None


def all_targets(nm='arm-none-eabi-gcc'):
    try:
        out = subprocess.check_output(['make', 'targets'], cwd=REPO,
                                      stderr=subprocess.DEVNULL).decode()
    except (OSError, subprocess.CalledProcessError):
        return []
    # Not a name test. Most targets are named after their MCU, but not
    # all - STELLAR_G071_V1 ends in the board revision - and a suffix
    # match silently drops those from every sweep driven by this list.
    # Cheap substring prefilter, then ask the preprocessor what the
    # target really is.
    cand = [t for t in out.split()
            if 'F051' in t or 'G071' in t or 'L431' in t or 'G431' in t
            or 'V203' in t or 'F031' in t or 'G031' in t or 'E230' in t
            or 'A153' in t or 'F415' in t or 'F421' in t]
    found = []
    for t in sorted(set(cand)):
        try:
            config(t, nm)
        except Unsupported:
            continue
        except Exception as e:
            # a generator bug must not silently shrink the sweep list
            print('WARNING: %s dropped from the list: %s' % (t, e),
                  file=sys.stderr)
            continue
        found.append(t)
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target', nargs='?')
    ap.add_argument('--outdir', default=None,
                    help='default: a gitignored obj/renode/ in the repo')
    ap.add_argument('--nm', default='arm-none-eabi-gcc',
                    help='compiler used to preprocess Inc/targets.h')
    ap.add_argument('--nm-bin', default='arm-none-eabi-nm',
                    help='reads the ELF symbol table for status()')
    ap.add_argument('--list', action='store_true',
                    help='the targets this can emulate')
    ap.add_argument('--run', action='store_true',
                    help='launch renode on the generated platform')
    ap.add_argument('--gdb', action='store_true',
                    help='also start a gdb server and attach gdb in a '
                         'terminal window (implies --run)')
    ap.add_argument('--gui', action='store_true',
                    help='serve the SITL wire protocols and open sitl_gui.py '
                         'on them (implies --run)')
    ap.add_argument('--link', action='store_true',
                    help='serve the SITL wire protocols without opening a GUI, '
                         'for driving the emulated ESC from a script')
    ap.add_argument('--sigrok', action='store_true',
                    help='serve a live renode-la logic analyser on TCP '
                         '(implies --run)')
    ap.add_argument('--sigrok-port', type=int, default=4242,
                    help='TCP port for the renode-la server (default 4242)')
    ap.add_argument('--sigrok-sample-rate', type=int, default=10000000,
                    help='sample rate advertised to the client in Hz; the '
                         'frontend may select another (default 10000000)')
    ap.add_argument('--gui-port', type=int, default=57733,
                    help='udp port carrying throttle in and telemetry out')
    ap.add_argument('--gui-state-port', type=int, default=57734,
                    help='udp port carrying physics samples, eeprom and model')
    ap.add_argument('--gui-dshot-us', type=int, default=250,
                    help='dshot frame period on the emulated wire, in virtual '
                         'microseconds. Every edge is a timer event, so raising '
                         'this is the cheapest way to buy emulation speed when '
                         'the wire is not what is under test (default 250, '
                         '4kHz)')
    ap.add_argument('--can-bus', type=int, default=0,
                    help='mcast CAN bus number for a DroneCAN target: the '
                         'emulated bxCAN appears on 239.65.82.<N>:57732, '
                         'where dronecan_gui_tool mcast:<N> sees it '
                         '(default 0; -1 leaves the bus disconnected)')
    ap.add_argument('--can-node', type=int, default=11,
                    help='DroneCAN node id written into the generated '
                         'eeprom of a CAN target (0 = dynamic allocation, '
                         'which needs an allocator on the bus)')
    ap.add_argument('--gdb-port', type=int, default=3333)
    ap.add_argument('--gdb-bin', default=None,
                    help='default: the toolchain under tools/')
    ap.add_argument('--readelf', default='arm-none-eabi-readelf')
    ap.add_argument('--no-xterm', action='store_true',
                    help='print the gdb command instead of opening a window')
    # the skip hook rewrites PC to return early from delayMillis, which
    # is confusing to single step through
    ap.add_argument('--no-skip-delays', action='store_true',
                    help='emulate the busy-wait delays instead of skipping '
                         'them; slower boot, but honest to step through')
    ap.add_argument('--renode', default=None,
                    help='renode binary to launch; default prefers a '
                         'dotnet portable under tools/linux/ (1.76x '
                         'faster, install it from the renode release '
                         'page), falling back to the vendored mono one')
    ap.add_argument('--cpusel', type=int, default=None, metavar='N',
                    help='pin only Renode (including its in-process motor '
                         'simulator) to host CPU N; the GUI and gdb remain '
                         'unrestricted')
    ap.add_argument('--elf', default=None,
                    help='default: whatever obj/ holds for the target')
    ap.add_argument('--bootloader-elf', default=None,
                    help='also load this bootloader ELF and start/reset the '
                         'MCU in it (default: no bootloader)')
    ap.add_argument('--eeprom', default=None,
                    help='default: generated to match --model')
    ap.add_argument('--model', default=os.path.join(
        REPO, 'Mcu', 'SITL', 'models', 'vimdrones_nano_2216.json'),
        help='motor the physics simulates, and what the eeprom is tuned for')
    # extra monitor commands, repeatable, so a run can be scripted rather
    # than interactive. Not argparse.REMAINDER: after a positional that
    # swallows --run and --eeprom themselves.
    ap.add_argument('--exec', dest='commands', action='append', default=[],
                    metavar='CMD', help='monitor command to run after loading')
    args = ap.parse_args()

    if args.cpusel is not None:
        if args.cpusel < 0:
            ap.error('--cpusel must be a non-negative host CPU number')
        try:
            allowed = os.sched_getaffinity(0)
        except AttributeError:
            ap.error('--cpusel needs host CPU-affinity support')
        if args.cpusel not in allowed:
            ap.error('--cpusel %d is not in this process affinity mask (%s)'
                     % (args.cpusel, ','.join(str(cpu)
                                              for cpu in sorted(allowed))))
        print('Renode host CPU affinity: %d' % args.cpusel)

    if args.list:
        for t in all_targets(args.nm):
            print(t)
        return 0
    if not args.target:
        ap.error('a target is required unless --list')

    outdir = args.outdir or os.path.join(REPO, 'obj', 'renode')
    if args.bootloader_elf is not None:
        if not os.path.isfile(args.bootloader_elf):
            ap.error('no bootloader ELF at %s' % args.bootloader_elf)
        args.bootloader_elf = os.path.abspath(args.bootloader_elf)
    try:
        cfg = config(args.target, args.nm)
        resc, repl = generate(args.target, outdir, args.nm,
                              sigrok=args.sigrok,
                              bootloader_elf=args.bootloader_elf)
    except Unsupported as e:
        print('SKIP: %s' % e)
        return 77
    print(repl)
    print(resc)

    if not (args.run or args.gdb or args.gui or args.link or args.sigrok):
        return 0

    elf = args.elf
    if elf is None:
        elf = find_elf(args.target)
        if elf is None:
            print('no firmware in obj/ for %s; build it or pass --elf'
                  % args.target)
            return 1
    if not os.path.exists(elf):
        print('no firmware at %s' % elf)
        return 1

    # the mcast scheme is 239.65.82.<bus>, one octet only for 0..9, and
    # a DroneCAN node id is 7 bits with 0 meaning dynamic allocation
    if args.can_bus > 9:
        ap.error('--can-bus must be 0..9, or negative to disconnect')
    if not 0 <= args.can_node <= 127:
        ap.error('--can-node must be 0..127')
    if args.sigrok:
        if not 1 <= args.sigrok_port <= 65535:
            ap.error('--sigrok-port must be 1..65535')
        if not 1 <= args.sigrok_sample_rate <= 1000000000:
            ap.error('--sigrok-sample-rate must be 1..1000000000 Hz')

    # an eeprom is required, not optional; without one Renode fails with
    # "Parameters did not match the signature" from LoadBinary, which
    # says nothing about the actual problem
    eeprom = args.eeprom
    if eeprom is None:
        eeprom = os.path.join(outdir, '%s_eeprom.bin' % args.target)
        # A fixed node id, because an anonymous node does nothing until
        # a DNA allocator answers, and a bare bench run has none. Input
        # type 5 (dronecan only) as a real CAN ESC would be configured:
        # it turns the capture interrupts off in DroneCAN_Startup(), and
        # without that the throttle generator's self-started zero-servo
        # signal fights set_input() over newinput and the flapping input
        # keeps resetting the arming counter.
        extra = ({'CAN_NODE': args.can_node, 'INPUT_SIGNAL_TYPE': 5}
                 if cfg['dronecan'] else None)
        try:
            default_eeprom(eeprom, args.model, extra)
        except Unsupported as e:
            print('SKIP: %s' % e)
            return 77
        print(eeprom)

    setup = '$repo=@%s; $elf=@%s; $eeprom=@%s; include @%s' % (
        REPO, elf, eeprom, resc)
    if not args.no_skip_delays:
        setup += '; cpu AddSymbolHook "delayMillis" "execfile(\'%s\')"' % (
            os.path.join(HERE, 'scripts', 'skip_delays.py'))

    # without the physics the bridge never starts and the motor cannot
    # turn, so a bare --run would boot and then look broken
    so = os.path.join(REPO, 'obj', 'libam32sim.so')
    if os.path.exists(so):
        setup += '; bridge LibraryPath "%s"; bridge ConfigPath "%s"' % (
            so, args.model)
    else:
        print('no %s, so the motor will not turn; build it with '
              '"make -C Mcu/Renode/sim"' % so)

    # the emulated ESC on the SITL's multicast CAN bus, where
    # dronecan_gui_tool mcast:N (and the GUI's DroneCAN panel) can see it
    if cfg['dronecan'] and args.can_bus >= 0:
        setup += '; canmcast Bus %d' % args.can_bus

    # status()/watch() at the monitor prompt, since there is no GUI
    status_py = os.path.join(outdir, '%s_status.py' % args.target)
    try:
        write_status(status_py, elf, args.nm_bin)
        setup += '; python "execfile(\'%s\')"' % status_py
    except Unsupported as e:
        print('no status() helpers: %s' % e)

    gdb_proc = None
    if args.gdb:
        dbg = has_debug_info(elf, args.readelf)
        if dbg is False:
            print('%s has no .debug_info; gdb would only show addresses.\n'
                  'The AM32 makefile builds with -g3, so rebuild the target.'
                  % elf)
            return 1
        if dbg is None:
            print('could not run %s, so not checking the ELF for debug info'
                  % args.readelf)
        gdb = find_gdb(args.gdb_bin)
        launcher = os.path.join(outdir, '%s_gdb.sh' % args.target)
        write_gdb_launcher(launcher, gdb, elf, args.gdb_port)
        # halted at reset, so gdb gets control before any code runs
        setup += '; machine StartGdbServer %d' % args.gdb_port

        term = None if args.no_xterm else find_terminal()
        if term is None:
            print('run this in another window to attach:\n    %s' % launcher)
        else:
            gdb_proc = subprocess.Popen(
                [term, '-title', 'gdb %s' % args.target, '-e', launcher])
            print('gdb attaching in %s; -O3 build, so expect inlined frames '
                  'and optimised-out locals' % os.path.basename(term))

    gui_proc = None
    if args.gui or args.link:
        setup += '; guilink DshotFrameUs %d' % args.gui_dshot_us
        setup += '; guilink AppBase 0x%08X' % cfg['app_base']
        setup += '; guilink LoopHz %d' % cfg['loop_hz']
        # so a client can say what firmware is running and how far
        # through arming it is, neither of which is on the wire
        addrs = symbol_addresses(elf, ('filename', 'armed_timeout_count',
                                       'armed', 'eepromBuffer'), args.nm_bin)
        for prop, sym in (('FirmwareNameAddress', 'filename'),
                          ('ArmedCountAddress', 'armed_timeout_count'),
                          ('ArmedAddress', 'armed'),
                          ('EepromBufferAddress', 'eepromBuffer')):
            if sym in addrs:
                setup += '; guilink %s 0x%08X' % (prop, addrs[sym])
        setup += '; guilink InputPort %d; guilink StatePort %d' % (
            args.gui_port, args.gui_state_port)
        # Under gdb the machine is deliberately halted at reset so the
        # debugger gets control first; otherwise there is nothing to wait
        # for and a GUI attached to a stopped machine looks broken.
        if not args.gdb:
            setup += '; start'
    if args.gui:
        gui_proc = launch_gui(args.gui_port, args.gui_state_port,
                              can_bus=args.can_bus if cfg['dronecan'] else -1)

    if args.sigrok:
        setup += '; sigrok SampleRate %d' % args.sigrok_sample_rate
        setup += '; sigrok DeviceName "AM32 %s"' % args.target
        setup += '; sigrok Port %d' % args.sigrok_port
        if not args.gdb and not (args.gui or args.link):
            setup += '; start'
        print('connect with:')
        print('    pulseview -d renode-la:conn=tcp/127.0.0.1/%d'
              % args.sigrok_port)
        print('advertising %d Hz; the capture is continuous, so a lower '
              'rate buys a longer window' % args.sigrok_sample_rate)
        print('channels: CH0=input wire, CH1=WS2812 data, '
              'CH2..4=A mode, CH5..7=B mode, CH8..10=C mode, '
              'CH11=comparator, CH12..13=sensed phase')

    for c in args.commands:
        setup += '; %s' % c
    cmd = [find_renode(args.renode), '--disable-xwt', '--console',
           '-e', setup]
    call_args = {'env': renode_env()}
    if args.cpusel is not None:
        # Apply affinity in the forked child immediately before exec. The
        # motor simulator is a shared library inside Renode, so it shares
        # this mask; the separately launched Qt GUI and gdb terminal inherit
        # the launcher's unrestricted mask instead.
        def select_renode_cpu():
            os.sched_setaffinity(0, {args.cpusel})
        call_args['preexec_fn'] = select_renode_cpu
    try:
        return subprocess.call(cmd, **call_args)
    finally:
        for p in (gdb_proc, gui_proc):
            if p is not None and p.poll() is None:
                p.terminate()


if __name__ == '__main__':
    sys.exit(main())
