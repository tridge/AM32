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
    gen_target.py --list                     targets this can emulate

Only F051 targets work: the Renode platform base is an STM32F051 and no
other AM32 MCU family has one yet. Anything else exits 77, as the test
harness does for a skip.
'''

import argparse
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
BASE_PLATFORM = os.path.join(HERE, 'platforms', 'stm32f051_base.repl')

# every macro the platform description needs. Read from the preprocessor
# rather than assumed, including the PHASE_*_COMP fallbacks targets.h
# applies at the bottom of the file.
WANTED = [
    'MCU_F051',
    'IC_TIMER_REGISTER', 'INPUT_DMA_CHANNEL', 'INPUT_PIN', 'INPUT_PIN_PORT',
    'DEAD_TIME', 'FILE_NAME',
    'PHASE_A_COMP', 'PHASE_B_COMP', 'PHASE_C_COMP',
    'PHASE_A_GPIO_HIGH', 'PHASE_A_GPIO_PORT_HIGH',
    'PHASE_A_GPIO_LOW', 'PHASE_A_GPIO_PORT_LOW',
    'PHASE_B_GPIO_HIGH', 'PHASE_B_GPIO_PORT_HIGH',
    'PHASE_B_GPIO_LOW', 'PHASE_B_GPIO_PORT_LOW',
    'PHASE_C_GPIO_HIGH', 'PHASE_C_GPIO_PORT_HIGH',
    'PHASE_C_GPIO_LOW', 'PHASE_C_GPIO_PORT_LOW',
]

# COMP1 CSR[6:4], the INMSEL field, per comparator input pin
INMSEL = {'COMP_PA4': 4, 'COMP_PA5': 5, 'COMP_PA0': 6}

# the capture timer, as (address, nvic line). Renode counts DMA channels
# from 0 where the reference manual counts from 1.
CAPTURE_TIMER = {'TIM3': (0x40000400, 16), 'TIM15': (0x40014000, 20)}

# stock declaration and alternate-function map for whichever of the two
# is NOT the capture timer, lifted from Renode's own stm32f0.repl
STOCK_TIMER = {
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
}


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


def pin_name(port, pin):
    '''GPIOA + LL_GPIO_PIN_10 -> "PA10"'''
    if not port.startswith('GPIO') or not pin.startswith('LL_GPIO_PIN_'):
        raise Unsupported('cannot read pin %s %s' % (port, pin))
    return 'P%s%s' % (port[4:], pin[len('LL_GPIO_PIN_'):])


def config(target, nm='arm-none-eabi-gcc'):
    '''everything the .repl needs, or Unsupported with the reason'''
    m = macros(target, nm)
    if 'FILE_NAME' not in m:
        raise Unsupported('%s is not a target in Inc/targets.h' % target)
    if 'MCU_F051' not in m:
        raise Unsupported('%s is not an F051 target; the Renode platform '
                          'base is an STM32F051 and no other AM32 MCU '
                          'family has one yet' % target)

    timer = m.get('IC_TIMER_REGISTER')
    if timer not in CAPTURE_TIMER:
        raise Unsupported('capture timer %s is not modelled' % timer)
    chan = m.get('INPUT_DMA_CHANNEL', '')
    if not chan.startswith('LL_DMA_CHANNEL_'):
        raise Unsupported('cannot read DMA channel %s' % chan)

    comps = {}
    for ph in 'ABC':
        c = m.get('PHASE_%s_COMP' % ph)
        if c not in INMSEL:
            raise Unsupported('comparator input %s for phase %s is not one '
                              'of PA0/PA4/PA5' % (c, ph))
        comps[ph] = INMSEL[c]

    pins = {}
    for ph in 'ABC':
        for side in ('HIGH', 'LOW'):
            pins[ph + side] = pin_name(m['PHASE_%s_GPIO_PORT_%s' % (ph, side)],
                                       m['PHASE_%s_GPIO_%s' % (ph, side)])

    return {
        'target': target,
        'name': m.get('FILE_NAME', target).strip('"').strip(),
        'timer': timer,
        'timer_addr': CAPTURE_TIMER[timer][0],
        'timer_irq': CAPTURE_TIMER[timer][1],
        # reference manual counts channels from 1, Renode from 0
        'dma_channel': int(chan[len('LL_DMA_CHANNEL_'):]) - 1,
        'throttle_pin': pin_name(m['INPUT_PIN_PORT'], m['INPUT_PIN']),
        'dead_time': m.get('DEAD_TIME', '?'),
        'comps': comps,
        'pins': pins,
    }


def platform(cfg):
    other = 'TIM15' if cfg['timer'] == 'TIM3' else 'TIM3'
    oname, oaddr, oirq, oaf = STOCK_TIMER[other]
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
        '// comparator: A=%s B=%s C=%s (COMP1 CSR[6:4])'
        % tuple(cfg['comps'][p] for p in 'ABC'),
        '',
        # absolute: these are generated into a scratch or obj directory,
        # so a path relative to the platforms tree would not resolve
        'using "%s"' % BASE_PLATFORM,
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
        '    frequency: 48000000',
        '    0 -> dma@%d' % cfg['dma_channel'],
        '    1 -> nvic@%d' % cfg['timer_irq'],
        '',
        '%s: Timers.STM32_Timer @ sysbus 0x%08X' % (oname, oaddr),
        '    frequency: 48000000',
        '    initialLimit: 0xFFFF',
        '    -> nvic@%d' % oirq,
        '',
        '%s:' % oname,
    ] + oaf + [
        '',
        '// DMA1_Channel4_5_IRQn for the capture channel.',
        'dma:',
        '    %d -> nvic@11' % cfg['dma_channel'],
        '',
        '// SYSCFG and COMP share a register page on the F051. Line 21 is',
        "// COMP1's EXTI line. The phase map is CSR[6:4], the COMP1 INMSEL",
        '// field: 4 is PA4, 5 is PA5, 6 is PA0.',
        'syscfgcomp: Miscellaneous.AM32_STM32F0_SysCfgComp @ sysbus <0x40010000, +0x400>',
    ] + [
        '    phase%sInmsel: %d' % (p, cfg['comps'][p]) for p in 'ABC'
    ] + [
        '    0 -> exti@21',
        '    1 -> exti@22',
        '',
        '// Couples the emulated bridge to the SITL motor physics.',
        '// LibraryPath and ConfigPath are set from the .resc, since they',
        '// are absolute paths. The phase pins are not the same on every',
        '// F051 target: a third of them rotate the phases across these six',
        '// pins, so they are stated rather than defaulted.',
        'bridge: Miscellaneous.AM32_F051_Bridge @ sysbus 0x50000400',
        '    batchUs: 10',
    ] + [
        '    phase%s%s: "%s"' % (p, side.capitalize(), cfg['pins'][p + side])
        for p in 'ABC' for side in ('HIGH', 'LOW')
    ] + [
        '',
        'throttle:',
        '    0 -> %s@0 | gpioPort%s@%s' % (cap, tp[1], tp[2:]),
        '',
    ]
    return '\n'.join(L)


def script(cfg, repl_path):
    return '\n'.join([
        ':name: AM32 %s' % cfg['target'],
        ':description: boots an AM32 %s firmware ELF' % cfg['target'],
        '',
        '# GENERATED by Mcu/Renode/gen_target.py. Set $repo, $elf and',
        '# $eeprom before including it; see am32_f051.resc for what they',
        '# mean.',
        '',
        '$repo?=@.',
        '$platform=@%s' % repl_path,
        'include $repo/Mcu/Renode/scripts/am32_f051.resc',
        '',
    ])


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


def all_targets():
    try:
        out = subprocess.check_output(['make', 'targets'], cwd=REPO,
                                      stderr=subprocess.DEVNULL).decode()
    except (OSError, subprocess.CalledProcessError):
        return []
    return sorted(set(t for t in out.split() if t.endswith('F051')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target', nargs='?')
    ap.add_argument('--outdir', default=None,
                    help='default: a gitignored obj/renode/ in the repo')
    ap.add_argument('--nm', default='arm-none-eabi-gcc')
    ap.add_argument('--list', action='store_true',
                    help='F051 targets, which are the ones this can emulate')
    ap.add_argument('--run', action='store_true',
                    help='launch renode on the generated platform')
    ap.add_argument('--elf', default=None)
    ap.add_argument('--eeprom', default=None)
    # extra monitor commands, repeatable, so a run can be scripted rather
    # than interactive. Not argparse.REMAINDER: after a positional that
    # swallows --run and --eeprom themselves.
    ap.add_argument('--exec', dest='commands', action='append', default=[],
                    metavar='CMD', help='monitor command to run after loading')
    args = ap.parse_args()

    if args.list:
        for t in all_targets():
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

    if not args.run:
        return 0

    elf = args.elf or os.path.join(REPO, 'obj',
                                   'AM32_%s_2.20.elf' % args.target)
    if not os.path.exists(elf):
        print('no firmware at %s; build it or pass --elf' % elf)
        return 1
    setup = '$repo=@%s; $elf=@%s; %sinclude @%s' % (
        REPO, elf, '$eeprom=@%s; ' % args.eeprom if args.eeprom else '', resc)
    for c in args.commands:
        setup += '; %s' % c
    cmd = [os.path.join(REPO, 'tools', 'linux', 'renode_1.16.1_portable',
                        'renode'), '--disable-xwt', '--console', '-e', setup]
    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
