#!/usr/bin/env python3
'''
CI test runner for the AM32 Renode harness: boots a real firmware ELF on
an emulated STM32F051, feeds it a servo throttle signal and asserts that
it arms and then spins a simulated motor closed loop on BEMF.

Mirrors Mcu/SITL/run_ci_tests.py in style. Only needs the python
standard library, plus arm-none-eabi-nm for the symbol table.

usage: run_renode_tests.py [--target ...] [--elf ...] [--renode ...]
                           [--model ...]
exits non-zero if any test fails, or 77 if the harness cannot run.
'''

import argparse
import glob
import json
import os
import re
import subprocess
import sys
import tempfile

# lives at the module root, next to the .repl/.resc it drives, mirroring
# Mcu/SITL/run_ci_tests.py. Not under a tools/ subdirectory: .gitignore
# has an unanchored "tools/" for the toolchain tree, which silently
# swallows any such path anywhere in the repo.
HERE = os.path.dirname(os.path.abspath(__file__))
RENODE_DIR = HERE
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))

sys.path.insert(0, os.path.join(REPO, 'Mcu', 'SITL'))
import sitl_params
sys.path.insert(0, RENODE_DIR)
import gen_target

failures = []


def check(name, cond, detail):
    print('%s: %s (%s)' % ('PASS' if cond else 'FAIL', name, detail))
    sys.stdout.flush()
    if not cond:
        failures.append(name)


def skip(reason):
    print('SKIP: %s' % reason)
    sys.exit(77)


def symbols(elf, nm):
    '''address and size for every symbol, so a byte is read as a byte.
       desync_happened is uint8_t here but uint32_t under
       DRONECAN_SUPPORT, and reading the wrong width returns garbage.'''
    out = subprocess.check_output([nm, '-S', elf]).decode()
    syms = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) == 4:
            syms[f[3]] = (int(f[0], 16), int(f[1], 16))
    return syms


def build_library():
    so = os.path.join(REPO, 'obj', 'libam32sim.so')
    r = subprocess.run(['make', '-s'], cwd=os.path.join(RENODE_DIR, 'sim'),
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        print(r.stdout.decode())
        skip('could not build the motor library')
    return so


def find_renode(explicit):
    if explicit:
        return explicit
    for cand in [os.path.join(REPO, 'tools', 'linux', 'renode_1.16.1_portable', 'renode'),
                 'renode']:
        if cand == 'renode' or os.path.exists(cand):
            return cand
    return None


READER = r'''
import sys
sb = monitor.Machine['sysbus']
b = monitor.Machine['sysbus.bridge']
S = %s
def rd(name):
    addr, size = S[name]
    if size == 1:
        return sb.ReadByte(addr)
    if size == 2:
        return sb.ReadWord(addr)
    return sb.ReadDoubleWord(addr)
def report(tag):
    print 'RESULT %%s armed=%%d running=%%d zero_crosses=%%d bemf_timeout=%%d desync=%%d rpm=%%d' %% (
        tag, rd('armed'), rd('running'), rd('zero_crosses'),
        rd('bemf_timeout_happened'), rd('desync_happened'), int(b.Rpm))
'''


def run(renode, target_resc, elf, eeprom, model, so, syms, scratch, physics=True):
    want = ['armed', 'running', 'zero_crosses', 'bemf_timeout_happened',
            'desync_happened']
    missing = [n for n in want if n not in syms]
    if missing:
        skip('firmware symbols not found: %s' % ', '.join(missing))
    table = dict((n, syms[n]) for n in want)

    reader = os.path.join(scratch, 'reader.py')
    with open(reader, 'w') as f:
        f.write(READER % repr(table))

    resc = os.path.join(scratch, 'spin.resc')
    with open(resc, 'w') as f:
        f.write('\n'.join([
            '$repo=@%s' % REPO,
            '$elf=@%s' % elf,
            '$eeprom=@%s' % eeprom,
            'include @%s' % target_resc,
            'logLevel 3',
            'cpu AddSymbolHook "delayMillis" "execfile(\'%s/Mcu/Renode/scripts/skip_delays.py\')"' % REPO,
            'bridge LibraryPath "%s"' % so if physics else '',
            'bridge ConfigPath "%s"' % model if physics else '',
            'python "execfile(\'%s\')"' % reader,
            # past the ~0.62s of frames and the ~1.02s counter gate
            'emulation RunFor "2.5"',
            'python "report(\'armed\')"',
            # 1300us: above the 1100us dead band, low enough to stay in
            # the startup ramp rather than saturating
            'sysbus WriteDoubleWord 0x50000000 1300',
            'emulation RunFor "1.5"',
            'python "report(\'spin\')"',
            'quit',
            '']))

    cmd = [renode, '--disable-xwt', '--console', '-e', 'include @%s' % resc]
    try:
        r = subprocess.run(cmd, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=900)
    except FileNotFoundError:
        skip('renode not found; pass --renode')
    except subprocess.TimeoutExpired:
        check('run completes', False, 'renode timed out')
        return {}
    out = r.stdout.decode(errors='replace')
    if 'compilation errors' in out or 'Errors during compilation' in out:
        print(out[-3000:])
        check('peripherals compile', False, 'see output above')
        return {}

    results = {}
    for m in re.finditer(r'RESULT (\w+) (.*)', out):
        results[m.group(1)] = dict(
            (k, int(v)) for k, v in re.findall(r'(\w+)=(-?\d+)', m.group(2)))
    if not results:
        print(out[-3000:])
        check('run produces results', False, 'no RESULT lines; see output above')
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', default='FD6288_F051',
                    help='any F051 target in Inc/targets.h')
    # defaults to whatever obj/ holds for the target, so the firmware
    # version does not have to be tracked here
    ap.add_argument('--elf', default=None)
    ap.add_argument('--renode', default=None)
    ap.add_argument('--nm', default='arm-none-eabi-nm')
    # preprocesses Inc/targets.h to build the platform; not the same
    # tool as --nm, which reads the ELF symbol table
    ap.add_argument('--gcc', default='arm-none-eabi-gcc')
    ap.add_argument('--model', default=os.path.join(
        REPO, 'Mcu', 'SITL', 'models', 'vimdrones_nano_2216.json'))
    # leaves the bridge unstarted, so there is no motor to sense. Used to
    # confirm the spin assertions can actually fail.
    ap.add_argument('--no-physics', action='store_true')
    args = ap.parse_args()

    if args.elf is None:
        found = sorted(glob.glob(os.path.join(REPO, 'obj',
                                              'AM32_%s_*.elf' % args.target)))
        if not found:
            skip('no firmware in obj/ for %s; build it first' % args.target)
        args.elf = found[-1]
    if not os.path.exists(args.elf):
        skip('no firmware at %s' % args.elf)
    renode = find_renode(args.renode)
    if renode is None:
        skip('renode not installed')
    try:
        syms = symbols(args.elf, args.nm)
    except (OSError, subprocess.CalledProcessError):
        skip('%s not usable' % args.nm)

    so = build_library()

    with tempfile.TemporaryDirectory() as scratch:
        # the platform is generated from Inc/targets.h rather than kept
        # in the tree, so a new target needs nothing written by hand
        try:
            target_resc, _ = gen_target.generate(args.target, scratch, args.gcc)
        except gen_target.Unsupported as e:
            skip(str(e))

        eeprom = os.path.join(scratch, 'eeprom.bin')
        # INPUT_SIGNAL_TYPE 0 is mandatory: the default is DSHOT_IN, and
        # with dshot set detectInput() never calls checkServo(), so a
        # servo signal is ignored with no diagnostic
        overrides = {'INPUT_SIGNAL_TYPE': 0}
        # MOTOR_KV and MOTOR_POLES have to agree with the motor being
        # simulated or the firmware is tuned for a different machine:
        # AM32 scales low rpm power protection from MOTOR_KV, and poles
        # scales every reported rpm and the commutation timing. Taken
        # from the model rather than left at the defaults, which is worth
        # 20% of measured rpm on this model. sitl_params.model_checks()
        # is the SITL's own rule for this, reused rather than restated.
        try:
            motor = json.load(open(args.model)).get('motor', {})
        except (OSError, ValueError):
            motor = {}
        for name, (want, _help) in sitl_params.model_checks(motor).items():
            overrides[name] = want
        image = sitl_params.build_image(overrides)
        bad = sitl_params.mismatches(image, motor)
        if bad:
            skip('eeprom disagrees with the model on %s' % ', '.join(bad))
        print('model %s: %s' % (os.path.basename(args.model),
                                ', '.join('%s=%d' % (k, v)
                                          for k, v in sorted(overrides.items()))))
        with open(eeprom, 'wb') as f:
            f.write(bytes(image))

        res = run(renode, target_resc, args.elf, eeprom, args.model, so, syms,
                  scratch, physics=not args.no_physics)

    a = res.get('armed', {})
    s = res.get('spin', {})
    if a:
        check('arms on a servo signal', a.get('armed') == 1,
              'armed=%d' % a.get('armed', -1))
        check('does not spin unarmed', a.get('running') == 0 and a.get('rpm', 0) == 0,
              'running=%d rpm=%d' % (a.get('running', -1), a.get('rpm', -1)))
    if s:
        check('motor runs', s.get('running') == 1,
              'running=%d' % s.get('running', -1))
        # the real assertion: turning at a sane speed. A stuck or
        # desynced motor still reports running.
        rpm = s.get('rpm', 0)
        check('motor spins', 500 < rpm < 20000, 'rpm=%d' % rpm)
        # closed loop, not blind commutation. 1.5s at this speed is
        # thousands of crossings; anything in the hundreds means BEMF
        # sensing is working rather than the startup ramp limping along.
        zc = s.get('zero_crosses', 0)
        check('commutates on BEMF', zc > 500, 'zero_crosses=%d' % zc)
        check('no BEMF timeouts', s.get('bemf_timeout') == 0,
              'bemf_timeout_happened=%d' % s.get('bemf_timeout', -1))
        check('no desyncs', s.get('desync') == 0,
              'desync_happened=%d' % s.get('desync', -1))

    if failures:
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('\nall tests passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
