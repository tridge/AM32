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


# Virtual time makes a run reproducible, so a target's speed is a fixed
# number rather than a range. 2% leaves room for a compiler or model
# nudge while still catching a partly dead bridge, which costs ~24%.
SPIN_TOLERANCE = 0.02
EXPECTED_SPIN = os.path.join(HERE, 'data', 'expected_spin.json')


def expected(target):
    '''recorded rpm and zero crossings, or None if not recorded yet'''
    try:
        with open(EXPECTED_SPIN) as f:
            return json.load(f).get(target)
    except (OSError, ValueError):
        return None


# Targets that cannot pass the spin assertions for a firmware reason, not
# an emulator gap. Skipped rather than left failing so a red sweep still
# means something.
UNSPINNABLE = {
    'DT160_64K_G071':
        'DEAD_TIME 210 is the only value in either family past 127, where '
        'the BDTR.DTG encoding stops being linear: (32+18)*8 = 400 ticks, '
        '6.25us at 64MHz. That is the whole startup ramp duty at ARR 2665, '
        'so no phase ever drives. Upstream (Alka) confirms the target is '
        'for slot car ESCs, which only run at 100% throttle. Full throttle '
        'does not rescue it here, measured: the ramp duty is fixed before '
        'commanded throttle applies, so 2000us and 1300us give byte '
        'identical rpm and rotor angle.',
}


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
ic = monitor.Machine['sysbus.%s']
S = %s
def report_reply(tag):
    # what the ESC drove back on the shared wire, GCR decoded by the
    # capture timer from the levels it output
    print 'RESULT %%s replies=%%d types=%%d gcr_errors=%%d crc_errors=%%d frame=%%d' %% (
        tag, ic.ReplyCount, ic.ReplyTypeMask, ic.ReplyGcrErrors,
        ic.ReplyCrcErrors, ic.LastReplyFrame)
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


def run(renode, target_resc, elf, eeprom, model, so, syms, scratch,
        throttle_addr, timer_name, physics=True, dshot=0, bidir=False,
        edt=False):
    want = ['armed', 'running', 'zero_crosses', 'bemf_timeout_happened',
            'desync_happened']
    missing = [n for n in want if n not in syms]
    if missing:
        skip('firmware symbols not found: %s' % ', '.join(missing))
    table = dict((n, syms[n]) for n in want)

    reader = os.path.join(scratch, 'reader.py')
    with open(reader, 'w') as f:
        f.write(READER % (timer_name, repr(table)))

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
            # dshot is selected before the run so detectInput() sees it
            # from the first capture; servo needs nothing, being the
            # generator's default
            'sysbus WriteDoubleWord 0x%08X %d' % (throttle_addr + 8, dshot)
            if dshot else '',
            # inverted framing: the ESC recognises bidirectional mode
            # from the idle-high line and then replies on the same wire
            'sysbus WriteDoubleWord 0x%08X 1' % (throttle_addr + 16)
            if bidir else '',
            # past the ~0.62s of frames and the ~1.02s counter gate
            'emulation RunFor "2.5"',
            'python "report(\'armed\')"',
            # dshot command 13 enables extended telemetry, and only
            # counts while armed and stopped. It has to be repeated 6
            # times, which at 4kHz is well inside this window.
            'sysbus WriteDoubleWord 0x%08X 13' % (throttle_addr + 12)
            if edt else '',
            'emulation RunFor "0.2"' if edt else '',
            # 632 is what AM32 makes of a 1300us servo pulse, which is
            # above the dead band but still inside the startup ramp; the
            # dshot value lands on the same internal scale
            'sysbus WriteDoubleWord 0x%08X 632' % (throttle_addr + 12)
            if dshot else
            'sysbus WriteDoubleWord 0x%08X 1300' % throttle_addr,
            'emulation RunFor "1.5"',
            'python "report(\'spin\')"',
            'python "report_reply(\'reply\')"' if bidir else '',
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
                    help='any F051 or G071 target in Inc/targets.h')
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
    # drives the ESC with dshot at this bitrate instead of a servo pulse,
    # which exercises detectInput()/checkDshot() and computeDshotDMA() on
    # real captures rather than the two edge servo path.
    #
    # 150 is deliberately absent: AM32's input auto-detection cannot see
    # it. checkDshot() classifies on the smallest gap between consecutive
    # edges and accepts 1-3 or 4-8 counts at the detection prescaler of
    # CPU_FREQUENCY_MHZ/6. Detection runs at zero throttle, where every
    # bit is a zero, so that gap is a zero's high time - 0.375 of a bit
    # period, 13 counts for dshot150 at 48MHz, measured. The SITL reached
    # the same conclusion independently; see Mcu/SITL/sitl_gui.py.
    ap.add_argument('--dshot', type=int, default=0, choices=(0, 300, 600),
                    help='dshot bitrate in kbaud; 0 (default) is servo')
    # inverted dshot, where the ESC answers on the same wire. Exercises
    # sendDshotDma(), which reuses the capture timer as a PWM output
    ap.add_argument('--bdshot', action='store_true',
                    help='bidirectional dshot; implies --dshot 600 if unset')
    # extended telemetry rides on the bidirectional reply, interleaved
    # one frame in two with eRPM
    ap.add_argument('--edt', action='store_true',
                    help='enable extended dshot telemetry; implies --bdshot')
    args = ap.parse_args()
    if args.target in UNSPINNABLE:
        skip('%s: %s' % (args.target, UNSPINNABLE[args.target]))
    if args.edt:
        args.bdshot = True
    if args.bdshot and not args.dshot:
        args.dshot = 600

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
            throttle_addr = gen_target.throttle_address(args.target, args.gcc)
            timer_name = gen_target.capture_timer_name(args.target, args.gcc)
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
                  scratch, throttle_addr, timer_name,
                  physics=not args.no_physics,
                  dshot=args.dshot, bidir=args.bdshot, edt=args.edt)

    a = res.get('armed', {})
    s = res.get('spin', {})
    if a:
        check('arms on a %s signal'
              % ('%sdshot%d' % ('bi' if args.bdshot else '', args.dshot)
                 if args.dshot else 'servo'),
              a.get('armed') == 1,
              'armed=%d' % a.get('armed', -1))
        check('does not spin unarmed', a.get('running') == 0 and a.get('rpm', 0) == 0,
              'running=%d rpm=%d' % (a.get('running', -1), a.get('rpm', -1)))
    if s:
        check('motor runs', s.get('running') == 1,
              'running=%d' % s.get('running', -1))
        # A range this wide passes almost anything that turns, which is
        # not enough: with two of three high sides dead a target still
        # limps at 76% speed and sails through. So when the target has a
        # recorded figure, hold it to that instead.
        rpm = s.get('rpm', 0)
        want = expected(args.target)
        if want is None:
            check('motor spins', 500 < rpm < 20000,
                  'rpm=%d, no recorded figure for this target' % rpm)
        else:
            check('motor spins at the recorded speed',
                  abs(rpm - want['rpm']) <= SPIN_TOLERANCE * want['rpm'],
                  'rpm=%d, expected %d' % (rpm, want['rpm']))
        # closed loop, not blind commutation. 1.5s at this speed is
        # thousands of crossings; anything in the hundreds means BEMF
        # sensing is working rather than the startup ramp limping along.
        zc = s.get('zero_crosses', 0)
        if want is None:
            check('commutates on BEMF', zc > 500, 'zero_crosses=%d' % zc)
        else:
            check('commutates on BEMF at the recorded rate',
                  abs(zc - want['zero_crosses'])
                  <= SPIN_TOLERANCE * want['zero_crosses'],
                  'zero_crosses=%d, expected %d' % (zc, want['zero_crosses']))
        check('no BEMF timeouts', s.get('bemf_timeout') == 0,
              'bemf_timeout_happened=%d' % s.get('bemf_timeout', -1))
        check('no desyncs', s.get('desync') == 0,
              'desync_happened=%d' % s.get('desync', -1))

    r = res.get('reply', {})
    if r:
        replies = r.get('replies', 0)
        check('replies on the shared wire', replies > 100,
              'replies=%d' % replies)
        # a decode that lines up on the wrong period still produces
        # frames, so the line code and the CRC both have to hold
        check('reply line code is legal', r.get('gcr_errors') == 0,
              'gcr_errors=%d' % r.get('gcr_errors', -1))
        check('reply CRC is correct', r.get('crc_errors') == 0,
              'crc_errors=%d' % r.get('crc_errors', -1))
    # the payload is the electrical period in us, mantissa and shift.
    # Checking it against the physics closes the loop: the firmware
    # sensed the emulated motor and reported the speed it is turning.
    # Only with plain bidirectional dshot, where every frame is eRPM;
    # extended telemetry interleaves and the last frame could be either.
    if r and not args.edt:
        frame = r.get('frame', 0)
        payload = frame >> 4
        period = (payload & 0x1FF) << (payload >> 9)
        poles = int(motor.get('poles', 14))
        erpm = 60e6 / period if period else 0
        want = s.get('rpm', 0)
        got = erpm / (poles / 2)
        check('reply reports the measured rpm',
              want and abs(got - want) < 0.02 * want,
              'frame=0x%04X period=%dus -> %drpm, physics %drpm'
              % (frame, period, got, want))
    if r and args.edt:
        # each telemetry kind carries a different top nibble, so one run
        # shows whether all three went out. The divisors are 40 frames
        # for current and 200 for voltage and temperature.
        types = r.get('types', 0)
        for name, nibble in (('temperature', 2), ('voltage', 4),
                             ('current', 6)):
            check('sends %s telemetry' % name, types & (1 << nibble),
                  'type mask 0x%04X' % types)

    if failures:
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('\nall tests passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
