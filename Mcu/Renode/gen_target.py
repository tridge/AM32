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
    gen_target.py TARGET --run --exec CMD    ... and script it
    gen_target.py TARGET --gui               ... driven by Mcu/SITL/sitl_gui.py
    gen_target.py --list                     targets this can emulate

F051 and G071 targets work; those are the two AM32 MCU families with a
Renode platform base so far. Anything else exits 77, as the test harness
does for a skip.
'''

import argparse
import glob
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))

# every macro the platform description needs. Read from the preprocessor
# rather than assumed, including the PHASE_*_COMP fallbacks targets.h
# applies at the bottom of the file.
WANTED = [
    'MCU_F051', 'MCU_G071',
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

# The comparator's inverting-input selection, as the value of the INMSEL
# field, per family. The F0 names its choices after the pin; the G0 uses
# the LL driver's IO1/IO2/IO3, which are PB3/PB7/PA2 on COMP2.
INMSEL = {
    'f051': {'COMP_PA4': 4, 'COMP_PA5': 5, 'COMP_PA0': 6},
    'g071': {'LL_COMP_INPUT_MINUS_IO1': 6, 'LL_COMP_INPUT_MINUS_IO2': 7,
             'LL_COMP_INPUT_MINUS_IO3': 8},
}

# the capture timer, as (address, nvic line), per family
CAPTURE_TIMER = {
    'f051': {'TIM3': (0x40000400, 16), 'TIM15': (0x40014000, 20)},
    'g071': {'TIM3': (0x40000400, 16), 'TIM16': (0x40014400, 21)},
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
}

# Everything that differs between the two MCU families, in one place, so
# a third family is a table entry plus a base .repl rather than a new
# code path.
#   dma_irq   nvic line the capture DMA channel raises
#   adc_dma   DMA channel index the ADC transfers on (0 based)
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
        # temperature sensor channel, its factory calibration pair, the
        # temperature the second point was taken at, and the Vref+ the
        # calibration was done with
        'temp_channel': 16,
        'ts_cal': (0x1FFFF7B8, 0x1FFFF7C2, 110, 3300),
        # no PA11/PA12 phase remap on this family
        'syscfg': 0,
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
        'temp_channel': 12,
        'ts_cal': (0x1FFF75A8, 0x1FFF75CA, 130, 3000),
    },
}


# EEprom_t.buffer in Inc/eeprom.h - the settings block the firmware
# reads at boot, and what the GUI parameter editor fetches and writes
EEPROM_SIZE = 192

# where the application is linked, above the 4K bootloader region. Also
# what the .resc points the reset vector at, since no bootloader is
# loaded. A PC below this is executing in the bootloader region.
APP_BASE = 0x08001000


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
    '''LL_ADC_CHANNEL_6 -> 6. The macro expands to a bitfield expression
       rather than a plain number, so the name is what carries it.'''
    if not macro or not macro.startswith(prefix):
        raise Unsupported('cannot read %s from %s' % (what, macro))
    try:
        return int(macro[len(prefix):])
    except ValueError:
        raise Unsupported('cannot read %s from %s' % (what, macro))


def pin_name(port, pin):
    '''GPIOA + LL_GPIO_PIN_10 -> "PA10"'''
    if not port.startswith('GPIO') or not pin.startswith('LL_GPIO_PIN_'):
        raise Unsupported('cannot read pin %s %s' % (port, pin))
    return 'P%s%s' % (port[4:], pin[len('LL_GPIO_PIN_'):])


def capture_input_af(family, timer, port, pin):
    '''alternate function that routes <timer>_CH1 to P<port><pin>.

       Parsed out of STOCK_TIMER's channel 0 line rather than restated:
       those "gpioPortA#02@0" entries are already the per pin AF map, and
       the table holds an entry for whichever timer is the capture timer
       on this target. Returns None if the pin is not a CH1 option, which
       leaves the gate off rather than guessing.'''
    spec = STOCK_TIMER.get(family, {}).get(timer)
    if spec is None:
        return None
    want = 'gpioPort%s#%02d@' % (port, pin)
    for entry in spec[3][0].split('|'):
        entry = entry.strip().lstrip('0 ->').strip()
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
        raise Unsupported('%s is not an F051 or G071 target; those are the '
                          'only AM32 MCU families with a Renode platform '
                          'base so far' % target)

    timer = m.get('IC_TIMER_REGISTER')
    if timer not in CAPTURE_TIMER[family]:
        raise Unsupported('capture timer %s is not modelled on the %s'
                          % (timer, family))
    chan = m.get('INPUT_DMA_CHANNEL', '')
    if not chan.startswith('LL_DMA_CHANNEL_'):
        raise Unsupported('cannot read DMA channel %s' % chan)

    inmsel = INMSEL[family]
    comps = {}
    for ph in 'ABC':
        c = m.get('PHASE_%s_COMP' % ph)
        if c not in inmsel:
            raise Unsupported('comparator input %s for phase %s is not one '
                              'of %s' % (c, ph, '/'.join(sorted(inmsel))))
        comps[ph] = inmsel[c]

    # which comparator senses each phase. Only N_VARIANT targets split
    # them; everything else uses MAIN_COMP throughout, and the F051 has
    # only COMP1.
    main = 1 if family == 'f051' else comp_number(m.get('MAIN_COMP', 'COMP2'))
    comp_of = {}
    for ph in 'ABC':
        if 'N_VARIANT' in m:
            comp_of[ph] = comp_number(m.get('PHASE_%s_COMP_NUMBER' % ph, ''))
        else:
            comp_of[ph] = main

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

    return {
        'target': target,
        'family': family,
        'name': m.get('FILE_NAME', target).strip('"').strip(),
        'voltage_channel': suffix_number(m.get('VOLTAGE_ADC_CHANNEL'),
                                         'LL_ADC_CHANNEL_', 'voltage channel'),
        'current_channel': suffix_number(m.get('CURRENT_ADC_CHANNEL'),
                                         'LL_ADC_CHANNEL_', 'current channel'),
        'voltage_divider': number('TARGET_VOLTAGE_DIVIDER', 110),
        'millivolt_per_amp': number('MILLIVOLT_PER_AMP', 20),
        'current_offset': number('CURRENT_OFFSET', 0),
        'timer': timer,
        'timer_addr': CAPTURE_TIMER[family][timer][0],
        'timer_irq': CAPTURE_TIMER[family][timer][1],
        # reference manual counts channels from 1, Renode from 0
        'dma_channel': int(chan[len('LL_DMA_CHANNEL_'):]) - 1,
        'throttle_pin': pin_name(m['INPUT_PIN_PORT'], m['INPUT_PIN']),
        'input_base': FAMILY[family]['gpio_a']
                      + 0x400 * (ord(m['INPUT_PIN_PORT'][4:]) - ord('A')),
        'input_pin': int(m['INPUT_PIN'][len('LL_GPIO_PIN_'):]),
        'input_af': capture_input_af(
            family, timer, m['INPUT_PIN_PORT'][4:],
            int(m['INPUT_PIN'][len('LL_GPIO_PIN_'):])),
        'dead_time': m.get('DEAD_TIME', '?'),
        'loop_hz': number('LOOP_FREQUENCY_HZ', 20000),
        'eeprom_addr': eeprom_addr,
        'comps': comps,
        'comp_of': comp_of,
        'main_comp': main,
        'enable_bridge': enable_bridge,
        'inverted_low': 'USE_INVERTED_LOW' in m,
        'inverted_high': 'USE_INVERTED_HIGH' in m,
        'pins': pins,
    }


def comp_number(macro):
    '''COMP2 -> 2'''
    if macro not in ('COMP1', 'COMP2'):
        raise Unsupported('comparator %r is not COMP1 or COMP2' % macro)
    return int(macro[4:])


def comp_block(cfg):
    '''the comparator declaration, which is the biggest family split'''
    if cfg['family'] == 'f051':
        return [
            '// SYSCFG and COMP share a register page on the F051. Line 21 is',
            "// COMP1's EXTI line. The phase map is CSR[6:4], the COMP1 INMSEL",
            '// field: 4 is PA4, 5 is PA5, 6 is PA0.',
            'syscfgcomp: Miscellaneous.AM32_STM32F0_SysCfgComp @ sysbus <0x40010000, +0x400>',
        ] + [
            '    phase%sInmsel: %d' % (p, cfg['comps'][p]) for p in 'ABC'
        ] + [
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
        '    phase%sInmsel: %d' % (p, cfg['comps'][p]) for p in 'ABC'
    ] + [
        '    phase%sComp: %d' % (p, cfg['comp_of'][p]) for p in 'ABC'
    ] + [
        '    mainComp: %d' % cfg['main_comp'],
        '    0 -> exti@17',
        '    1 -> exti@18',
    ]


def platform(cfg):
    fam = cfg['family']
    spec = FAMILY[fam]
    others = [t for t in STOCK_TIMER[fam] if t != cfg['timer']]
    cap = 'timer%s' % cfg['timer'][3:]
    tp = cfg['throttle_pin']
    L = [
        '// GENERATED by Mcu/Renode/gen_target.py from Inc/targets.h -',
        '// edit that, or the generator, not this file.',
        '//',
        '// target %s (%s), DEAD_TIME %s' % (cfg['target'], cfg['name'],
                                             cfg['dead_time']),
        '// throttle in on %s, captured by %s_CH1 into DMA1 channel %d'
        % (tp, cfg['timer'], cfg['dma_channel'] + 1),
        '// comparator: A=%s B=%s C=%s (INMSEL), on COMP%s/%s/%s'
        % (tuple(cfg['comps'][p] for p in 'ABC')
           + tuple(cfg['comp_of'][p] for p in 'ABC')),
        '// bridge: %s' % ('gate driver PWM + enable per phase'
                           if cfg['enable_bridge'] else
                           ('high and low side, low side inverted'
                            if cfg['inverted_low'] else 'high and low side')),
        '',
        # absolute: these are generated into a scratch or obj directory,
        # so a path relative to the platforms tree would not resolve
        'using "%s"' % os.path.join(HERE, 'platforms', 'stm32%s_base.repl' % fam),
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

    # whichever timers this target is not capturing with, declared stock
    for other in others:
        oname, oaddr, oirq, oaf = STOCK_TIMER[fam][other]
        L += [
            '%s: Timers.STM32_Timer @ sysbus 0x%08X' % (oname, oaddr),
            '    frequency: %d' % spec['timer_hz'],
            '    initialLimit: 0xFFFF',
            '    -> nvic@%d' % oirq,
            '',
            '%s:' % oname,
        ] + oaf + ['']

    L += [
        '// the nvic line the capture DMA channel raises',
        'dma:',
        '    %d -> nvic@%d' % (cfg['dma_channel'], spec['dma_irq']),
        '',
    ] + comp_block(cfg) + [
        '',
        '// Couples the emulated bridge to the SITL motor physics.',
        '// LibraryPath and ConfigPath are set from the .resc, since they',
        '// are absolute paths. The phase pins are not the same on every',
        '// target: many rotate the phases across these six pins, so they',
        '// are stated rather than defaulted.',
        'bridge: Miscellaneous.AM32_F051_Bridge @ sysbus 0x%08X' % spec['bridge'],
        '    batchUs: 10',
        '    timerHz: %d' % spec['timer_hz'],
        '    gpioABase: 0x%08X' % spec['gpio_a'],
        '    syscfgBase: 0x%08X' % spec['syscfg'],
        '    gpioBBase: 0x%08X' % (spec['gpio_a'] + 0x400),
        '    gpioCBase: 0x%08X' % (spec['gpio_a'] + 0x800),
    ] + [
        '    phase%s%s: "%s"' % (p, side.capitalize(), cfg['pins'][p + side])
        for p in 'ABC' for side in ('HIGH', 'LOW')
    ] + ([
        '    topology: "enable"',
    ] if cfg['enable_bridge'] else []) + ([
        '    invertedLow: true',
    ] if cfg['inverted_low'] else []) + ([
        '    invertedHigh: true',
    ] if cfg['inverted_high'] else []) + [
        '',
        '// The stock ADC models have no DMA output, and AM32 reads its',
        '// conversions only through DMA into ADCDataDMA[], so against them',
        '// the firmware saw no voltage or current at all. There is',
        '// deliberately no external event frequency: AM32 starts',
        '// conversions in software from the 1kHz loop, and modelling a',
        '// hardware trigger as well made sequences overlap forever.',
        'adc: Analog.AM32_STM32F0_ADC @ sysbus 0x40012400',
        '    voltageChannel: %d' % cfg['voltage_channel'],
        '    currentChannel: %d' % cfg['current_channel'],
        '    voltageDivider: %d' % cfg['voltage_divider'],
        '    millivoltPerAmp: %d' % cfg['millivolt_per_amp'],
        '    currentOffsetMv: %d' % cfg['current_offset'],
        '    temperatureChannel: %d' % spec['temp_channel'],
        '    tsCal1: 0x%08X' % spec['ts_cal'][0],
        '    tsCal2: 0x%08X' % spec['ts_cal'][1],
        '    tsCal2Temp: %d' % spec['ts_cal'][2],
        '    tsCalVrefMv: %d' % spec['ts_cal'][3],
        '    0 -> dma@%d' % spec['adc_dma'],
        '    1 -> %s' % spec['adc_irq'],
        '',
        'throttle: Miscellaneous.AM32ThrottleGenerator @ sysbus 0x%08X'
        % spec['throttle'],
        '    0 -> %s@0 | gpioPort%s@%s' % (cap, tp[1], tp[2:]),
        '',
        '// Serves the SITL wire protocols to sitl_gui.py. The ports are',
        '// left closed here and opened from the .resc, so a run that is',
        '// not driving a GUI cannot collide with a real SITL on the same',
        '// machine.',
        'guilink: Miscellaneous.AM32_GuiLink @ sysbus 0x%08X' % spec['guilink'],
        '    eepromAddress: 0x%08X' % cfg['eeprom_addr'],
        '    eepromSize: %d' % EEPROM_SIZE,
        '',
    ]
    return '\n'.join(L)


def script(cfg, repl_path):
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
        'include $repo/Mcu/Renode/scripts/am32_%s.resc' % cfg['family'],
        '',
    ])


def throttle_address(target, nm='arm-none-eabi-gcc'):
    '''where to write a pulse width to drive the throttle generator. Not
       a constant: it sits at 0x50000000 on the F051, which is where the
       G0 puts GPIOA, so the G0 moves it to 0x60000000.'''
    return FAMILY[config(target, nm)['family']]['throttle']


def capture_timer_name(target, nm='arm-none-eabi-gcc'):
    '''Renode peripheral name of the input capture timer, which varies by
       target: TIM15 on most, TIM2/TIM3 on others'''
    return 'timer%s' % config(target, nm)['timer'][3:]


def generate(target, outdir, nm='arm-none-eabi-gcc'):
    '''write the pair, return (resc, repl). Raises Unsupported.'''
    cfg = config(target, nm)
    os.makedirs(outdir, exist_ok=True)
    repl = os.path.join(outdir, '%s.repl' % target)
    resc = os.path.join(outdir, '%s.resc' % target)
    with open(repl, 'w') as f:
        f.write(platform(cfg))
    with open(resc, 'w') as f:
        f.write(script(cfg, repl))
    return resc, repl


def default_eeprom(path, model):
    '''An eeprom is not optional: Renode zero-fills unbacked memory where
       erased flash reads 0xFF, so a missing one sends loadEEpromSettings()
       down the migration path. INPUT_SIGNAL_TYPE 0 is mandatory too - the
       default is DSHOT_IN, and with dshot set detectInput() never calls
       checkServo(), so a servo signal is ignored with no diagnostic.'''
    sys.path.insert(0, os.path.join(REPO, 'Mcu', 'SITL'))
    try:
        import sitl_params
    except ImportError:
        raise Unsupported('cannot build a default eeprom (Mcu/SITL not '
                          'importable); pass --eeprom')
    overrides = {'INPUT_SIGNAL_TYPE': 0}
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


def launch_gui(port, state_port):
    '''start Mcu/SITL/sitl_gui.py against the link ports. It needs PySide6,
       which the SITL keeps in its own venv, so prefer that interpreter -
       the GUI's own diagnostic for a missing PySide6 tells you to run it
       from there anyway.'''
    gui = os.path.join(REPO, 'Mcu', 'SITL', 'sitl_gui.py')
    venv = os.path.join(REPO, 'Mcu', 'SITL', 'venv', 'bin', 'python3')
    python = venv if os.path.exists(venv) else sys.executable
    cmd = [python, gui, '--port', str(port), '--state-port', str(state_port),
           '--backend', 'renode']
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
    cand = [t for t in out.split() if 'F051' in t or 'G071' in t]
    found = []
    for t in sorted(set(cand)):
        try:
            config(t, nm)
        except Unsupported:
            continue
        except Exception:
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
                    help='F051 targets, which are the ones this can emulate')
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
    ap.add_argument('--elf', default=None,
                    help='default: whatever obj/ holds for the target')
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

    if args.list:
        for t in all_targets(args.nm):
            print(t)
        return 0
    if not args.target:
        ap.error('a target is required unless --list')

    outdir = args.outdir or os.path.join(REPO, 'obj', 'renode')
    try:
        resc, repl = generate(args.target, outdir, args.nm)
    except Unsupported as e:
        print('SKIP: %s' % e)
        return 77
    print(repl)
    print(resc)

    if not (args.run or args.gdb or args.gui or args.link):
        return 0

    elf = args.elf
    if elf is None:
        found = sorted(glob.glob(os.path.join(REPO, 'obj',
                                              'AM32_%s_*.elf' % args.target)))
        if not found:
            print('no firmware in obj/ for %s; build it or pass --elf'
                  % args.target)
            return 1
        elf = found[-1]
    if not os.path.exists(elf):
        print('no firmware at %s' % elf)
        return 1

    # an eeprom is required, not optional; without one Renode fails with
    # "Parameters did not match the signature" from LoadBinary, which
    # says nothing about the actual problem
    eeprom = args.eeprom
    if eeprom is None:
        eeprom = os.path.join(outdir, '%s_eeprom.bin' % args.target)
        try:
            default_eeprom(eeprom, args.model)
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
        setup += '; guilink AppBase 0x%08X' % APP_BASE
        setup += '; guilink LoopHz %d' % config(args.target, args.nm)['loop_hz']
        # so a client can say what firmware is running and how far
        # through arming it is, neither of which is on the wire
        addrs = symbol_addresses(elf, ('filename', 'armed_timeout_count',
                                       'armed'), args.nm_bin)
        for prop, sym in (('FirmwareNameAddress', 'filename'),
                          ('ArmedCountAddress', 'armed_timeout_count'),
                          ('ArmedAddress', 'armed')):
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
        gui_proc = launch_gui(args.gui_port, args.gui_state_port)

    for c in args.commands:
        setup += '; %s' % c
    cmd = [os.path.join(REPO, 'tools', 'linux', 'renode_1.16.1_portable',
                        'renode'), '--disable-xwt', '--console', '-e', setup]
    try:
        return subprocess.call(cmd)
    finally:
        for p in (gdb_proc, gui_proc):
            if p is not None and p.poll() is None:
                p.terminate()


if __name__ == '__main__':
    sys.exit(main())
