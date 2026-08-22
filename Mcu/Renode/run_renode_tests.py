#!/usr/bin/env python3
'''
CI test runner for the AM32 Renode harness: boots a real firmware ELF on
an emulated STM32F051, feeds it a servo throttle signal and asserts that
it arms and then spins a simulated motor closed loop on BEMF.

Mirrors Mcu/SITL/run_ci_tests.py in style. Only needs the python
standard library, plus arm-none-eabi-nm for the symbol table.

The throttle is written straight into the generator's registers from the
monitor, so a run is scripted in virtual time and takes seconds. --link
and --gui instead drive it the way a person does, over the udp ports the
guilink peripheral serves: slower, wall clock bound, and the only way to
cover the path sitl_gui.py actually uses.

usage: run_renode_tests.py [--target ...] [--elf ...] [--renode ...]
                           [--model ...] [--link | --gui]
exits non-zero if any test fails, or 77 if the harness cannot run.
'''

import argparse
import collections
import glob
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time

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

# Targets whose firmware cannot ARM on a wire that is bidirectional from
# power-on, found by this harness and reproduced deterministically. The
# reply plumbing is still asserted on these; the arm/spin checks are not.
BDSHOT_NOARM = {
    'FRDM_A153':
        'a three-way race in the A153 port loses arming: zero_input_count '
        'passes 30 within the first ~30 frames, then at ~100 frames '
        'CTIMER0_IRQHandler\'s inverted-dshot autodetect flips the capture '
        'polarity and resets zero_input_count to 0 as part of its state '
        'flush, and within a few more frames dshot_telemetry latches '
        '(computeDshotDMA\'s own 100-count threshold, never reset) - from '
        'then on transfercomplete() returns down the telemetry branch '
        'before the zero_input_count++ block, so the count freezes low. '
        'The 1s arming gate needs >30, fails, and clears inputSet; '
        're-detection can succeed, but with dshot_telemetry still latched '
        'the counter can never rebuild, so the cycle repeats forever. The '
        'STM32 ports arm because they have no polarity-flip handler '
        'resetting the counter mid-race.',
}


# Families whose bidirectional-dshot eRPM reply is off by a fixed
# factor ON REAL HARDWARE, so the reply check must expect it rather
# than fail. The F415 sums commutation intervals assuming 0.5us ticks
# ("COMMUTATION INTERVAL IS 0.5US INCREMENTS", Src/main.c), but its
# INTERVAL_TIMER runs at 144MHz/75 = 1.92MHz, 0.5208us a tick - so the
# reported period is 24/25 of the true one and the eRPM reads 25/24
# high. A firmware trait, not an emulation artefact.
REPLY_RPM_SCALE = {'f415': 25.0 / 24.0}

# what AM32 makes of a 1300us servo pulse, the same internal throttle the
# scripted tests use, so a link run is comparable with them
LINK_DSHOT_VALUE = 632

# Wider than SPIN_TOLERANCE, and not for want of determinism in the
# emulator: a real client decides when to change the throttle, so the
# simulated instant it lands on moves between runs and the settle window
# starts from a slightly different place. The speed itself still has to
# be the recorded one, not merely "turning".
LINK_TOLERANCE = 0.05

# the DroneCAN throttle the --can test commands: 2400/8191 maps onto
# AM32 input 633, one count from the 632 the dshot tests use, so the
# recorded spin figures apply to both
CAN_THROTTLE = 2400 / 8191.0
# node id written into the test eeprom; 0 would need a DNA allocator
CAN_NODE_ID = 11

# CAN targets whose motor cannot yet start in the emulation: the --can
# transport and protocol checks still run and must pass, only the spin
# assertions are withheld, with the reason printed
CAN_NOSPIN = {
    'SEQURE_G431_CAN':
        'NO_POLLING_START locks into the low-speed rocking resonance '
        'described in the README; comparator front-end measurements from '
        'the real board are pending',
}


def report_link(res, target, motor, bidir=True):
    a = res.get('armed', {})
    s = res.get('spin', {})
    if not s:
        if not failures:
            check('link run produces a result', False, 'no samples')
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1

    check('the state stream carries the bus voltage',
          8.0 < a.get('vbus', 0) < 30.0, 'vbus=%.2fV' % a.get('vbus', 0))
    check('does not spin unarmed', a.get('sim_rpm', 1) < 1.0,
          'rpm=%.0f' % a.get('sim_rpm', -1))
    check('the motor audio stream delivers physics samples',
          res.get('audio_samples', 0) > 1000,
          'samples=%d' % res.get('audio_samples', 0))
    check('the motor audio stream carries a non-silent signal',
          res.get('audio_peak', 0) > 1e-6,
          'peak=%g' % res.get('audio_peak', 0))

    rpm = s.get('sim_rpm', 0)
    want = expected(target)
    if want is None:
        check('setpoints over the link spin the motor', 500 < rpm < 20000,
              'rpm=%.0f, no recorded figure for this target' % rpm)
    else:
        check('setpoints over the link spin it at the recorded speed',
              abs(rpm - want['rpm']) <= LINK_TOLERANCE * want['rpm'],
              'rpm=%.0f, expected %d' % (rpm, want['rpm']))
    # the reply the client decoded against the motor the client is
    # watching: both ends of the link, checked against each other
    if bidir:
        telem = s.get('telem_rpm', 0)
        check('telemetry reaches the client', s.get('replies', 0) > 100,
              'replies=%d' % s.get('replies', 0))
        check('the reported rpm is the rpm being simulated',
              rpm > 0 and abs(telem - rpm) < 0.05 * rpm,
              'telemetry %.0f, physics %.0f' % (telem, rpm))
        check('no reply CRC failures', s.get('badcrc') == 0,
              'badcrc=%d' % s.get('badcrc', -1))
    else:
        print('NOTE: link telemetry not asserted: the wire is plain dshot '
              'here because the target cannot arm on a bidirectional one '
              '(see BDSHOT_NOARM)')

    if failures:
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('\nall tests passed')
    return 0


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
    # the launcher's rule, shared: dotnet portable if installed, else
    # the vendored mono one
    return gen_target.find_renode(explicit)


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

    cmd = [renode, '--config', os.path.join(scratch, 'renode-config'),
           '--disable-xwt', '--console', '-e', 'include @%s' % resc]
    try:
        r = subprocess.run(cmd, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=900,
                           env=gen_target.renode_env())
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


def run_link(renode, target_resc, elf, eeprom, model, so, scratch,
             port, state_port, dshot_us, value, seconds,
             gcc='arm-none-eabi-gcc', nm='arm-none-eabi-nm', bidir=True):
    '''Drive the target the way the GUI does: setpoints in over udp,
       BDShot telemetry and physics samples back, with nothing scripted
       through the monitor. That covers what the scripted tests cannot -
       that the link latches setpoints onto the generator, that the
       replies reach a client and that the state stream carries the same
       motor the firmware is sensing.

       Paced by SIMULATED time read out of the state stream, not by the
       wall clock, so it holds the same throttle for the same emulated
       interval as the scripted tests however slowly the host runs.
       `seconds` is only a backstop.'''
    from sitl_gui_backend import AudioStream, DshotPanel, SimStream
    import sitl_dshot as sd

    proc = start_renode(renode, target_resc, elf, eeprom, model, so, scratch,
                        port, state_port, dshot_us, gcc, nm)

    # drained from the start: a full stdout pipe stalls the emulator, and
    # there is nothing to wait for in it - the port messages are Info and
    # this run sets logLevel 3
    tail = collections.deque(maxlen=40)
    threading.Thread(target=drain, args=(proc, tail), daemon=True).start()

    ds = sim = audio = None
    try:
        deadline = time.time() + seconds
        ds = DshotPanel('127.0.0.1', port)
        ds.ptype = sd.TYPE_DSHOT300
        ds.bidir = bidir
        ds.rate = 500
        ds.value = 0
        ds.enabled = True
        sim = SimStream('127.0.0.1', state_port, period_us=1000)
        sim.enabled = True
        audio = AudioStream('127.0.0.1', state_port)

        # the readiness check as well as the first timestamp: samples only
        # flow once the state port is open and the physics has started
        t0 = wait_sim(sim, 0.0, deadline, ds)
        if t0 is None:
            print('\n'.join(tail))
            check('state stream delivers samples', False, 'no samples arrived')
            return {}
        # the same windows the scripted test uses: past the frames and the
        # counter gate at zero, then a settle at throttle
        if wait_sim(sim, t0 + 2.5, deadline, ds) is None:
            check('reaches the arming window', False,
                  'simulated time did not advance')
            return {}
        armed = link_sample(ds, sim)
        ds.value = value
        if wait_sim(sim, t0 + 4.0, deadline, ds) is None:
            check('reaches the settled window', False,
                  'simulated time did not advance')
            return {}
        spin = link_sample(ds, sim)

        # pacing (state cmd 2): the GUI's speedup slider slows the
        # emulation for slow motion by holding simulated time to a
        # fraction of wall time. Wall clock is in the measurement, so
        # the bounds are generous, and a host too slow to make a 0.05x
        # target visible reports that instead of failing on machine
        # speed.
        def measured_ratio(window):
            a = sim.latest()[0]
            wa = time.time()
            time.sleep(window)
            return (sim.latest()[0] - a) / (time.time() - wa)

        free = measured_ratio(3.0)
        if free < 0.08:
            check('the link paces the emulation', True,
                  'unpaced %.3fx is too slow to show a 0.05x target; '
                  'not asserted' % free)
        else:
            sim.set_speedup(0.05)
            time.sleep(1.0)  # let the pace anchor take hold
            paced = measured_ratio(4.0)
            sim.set_speedup(0.0)
            check('the link paces the emulation', 0.01 <= paced <= 0.10,
                  'unpaced %.3fx, paced %.3fx against a 0.05x target'
                  % (free, paced))
        if free >= 1.10:
            sim.set_speedup(1.0)
            time.sleep(1.0)
            realtime = measured_ratio(3.0)
            sim.set_speedup(0.0)
            check('the link paces a fast emulator to realtime',
                  0.90 <= realtime <= 1.10,
                  'unpaced %.3fx, 1x target produced %.3fx' %
                  (free, realtime))
        else:
            check('the link paces a fast emulator to realtime', True,
                  'unpaced %.3fx cannot demonstrate a 1x cap; not asserted'
                  % free)
        audio_batches = audio.take_batches()
        audio_samples = sum(len(vals) for _, vals in audio_batches)
        audio_peak = max((abs(v) for _, vals in audio_batches for v in vals),
                         default=0.0)
        return {'armed': armed, 'spin': spin,
                'audio_samples': audio_samples, 'audio_peak': audio_peak}
    finally:
        for c in (ds, sim, audio):
            if c is not None:
                c.running = False
        proc.terminate()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()


def run_can(renode, target_resc, elf, eeprom, model, so, scratch,
            port, state_port, bus, seconds,
            gcc='arm-none-eabi-gcc', nm='arm-none-eabi-nm'):
    '''Drive the target over DroneCAN: the emulated bxCAN is bridged to
       the mcast bus, and the GUI's own CanPanel arms and throttles it
       there, exactly as dronecan_gui_tool or a flight controller would.
       The guilink state stream is only read, for the physics rpm the
       telemetry is checked against.

       Paced by the ESC's own NodeStatus uptime, which is simulated
       time, so emulation speed does not matter; `seconds` is only a
       backstop.'''
    from sitl_gui_backend import CanPanel, SimStream

    proc = start_renode(renode, target_resc, elf, eeprom, model, so, scratch,
                        port, state_port, 250, gcc, nm, can_bus=bus)

    tail = collections.deque(maxlen=40)
    threading.Thread(target=drain, args=(proc, tail), daemon=True).start()

    can = sim = None
    try:
        deadline = time.time() + seconds
        can = CanPanel('mcast:%d' % bus)
        if not can.started.wait(10.0):
            check('the DroneCAN node starts', False,
                  'CanPanel did not come up within 10s')
            return {}
        if can.error is not None:
            check('the DroneCAN node starts', False, can.error)
            return {}
        can.rate = 100          # wall clock; well inside the 250ms
        # a REAL unarmed test: full throttle commanded while the
        # ArmingStatus stream says disarmed. Firmware that ignored
        # REQUIRE_ARMING would spin here.
        can.armed = False       # simulated-time RawCommand failsafe
        can.throttle = CAN_THROTTLE
        can.enabled = True
        sim = SimStream('127.0.0.1', state_port, period_us=1000)
        sim.enabled = True

        def wait_uptime(until):
            '''ESC uptime is simulated seconds; None on the backstop'''
            while time.time() < deadline:
                if can.node_id is not None and can.uptime >= until:
                    return can.uptime
                time.sleep(0.2)
            return None

        # esc.Status is what identifies the node, so this is also the
        # "the ESC is alive on the bus" gate
        if wait_uptime(3) is None:
            print('\n'.join(tail))
            check('the ESC appears on the mcast bus', False,
                  'no esc.Status within the backstop')
            return {}
        first = can.uptime
        if wait_uptime(first + 3) is None:
            check('reaches the unarmed window', False, 'uptime stalled')
            return {}
        unarmed = dict(can.status)
        # arming needs about 1.5 simulated seconds at zero throttle
        can.throttle = 0.0
        can.armed = True
        if wait_uptime(first + 7) is None:
            check('reaches the arming window', False, 'uptime stalled')
            return {}
        armed = dict(can.status)
        can.throttle = CAN_THROTTLE
        if wait_uptime(first + 13) is None:
            check('reaches the settled window', False, 'uptime stalled')
            return {}
        s = sim.latest()
        spin = dict(can.status)
        spin['sim_rpm'] = (s[1] * 60.0 / (2 * 3.14159265358979)
                           if s is not None else -1)
        spin['esc_frames'] = can.esc_rate.count
        return {'unarmed': unarmed, 'armed': armed, 'spin': spin,
                'node_id': can.node_id}
    finally:
        for c in (can, sim):
            if c is not None:
                c.running = False
        proc.terminate()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()


def report_can(res, target, node_id):
    u = res.get('unarmed')
    a = res.get('armed')
    s = res.get('spin')
    if u is None or a is None or s is None:
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    check('the ESC appears on the mcast bus as node %d' % node_id,
          res.get('node_id') == node_id,
          'esc.Status from node %s' % res.get('node_id'))
    check('refuses full throttle while disarmed', u.get('rpm', -1) == 0,
          'rpm=%s with RawCommand %d and ArmingStatus disarmed'
          % (u.get('rpm'), int(8191 * CAN_THROTTLE)))
    check('the telemetry carries the bus voltage',
          8.0 < a.get('voltage', 0) < 30.0, 'voltage=%.1f' % a.get('voltage', 0))
    check('telemetry reaches the client', s.get('esc_frames', 0) > 100,
          'esc.Status frames=%d' % s.get('esc_frames', 0))
    if target in CAN_NOSPIN:
        # the CAN transport itself is fully asserted above; the motor
        # start is a known, documented gap on this target
        print('NOTE: spin not asserted: %s' % CAN_NOSPIN[target])
    else:
        want = expected(target)
        rpm = s.get('rpm', 0)
        if want is not None:
            check('RawCommand spins it at the recorded speed',
                  abs(rpm - want['rpm']) <= LINK_TOLERANCE * want['rpm'],
                  'rpm=%d, expected %d' % (rpm, want['rpm']))
        else:
            check('RawCommand spins it', 500 < rpm < 20000,
                  'rpm=%d, no recorded figure for this target' % rpm)
        sim_rpm = s.get('sim_rpm', 0)
        check('the reported rpm is the rpm being simulated',
              sim_rpm > 0 and abs(rpm - sim_rpm) <= LINK_TOLERANCE * sim_rpm,
              'telemetry %d, physics %d' % (rpm, sim_rpm))
    if failures:
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('\nall tests passed')
    return 0


def run_gui(renode, target_resc, elf, eeprom, model, so, scratch,
            port, state_port, dshot_us, value, seconds, gui_python,
            gcc='arm-none-eabi-gcc', nm='arm-none-eabi-nm'):
    """The same link, driven by the real GUI instead of by its backend
       classes: sitl_gui.py under Qt's offscreen platform, scripted
       through its control port. That covers what run_link cannot - that
       the UI a person actually uses works against the emulator, not just
       the protocol underneath it."""
    gui = os.path.join(REPO, 'Mcu', 'SITL', 'sitl_gui.py')
    control = free_port()
    proc = start_renode(renode, target_resc, elf, eeprom, model, so, scratch,
                        port, state_port, dshot_us, gcc, nm)
    tail = collections.deque(maxlen=40)
    threading.Thread(target=drain, args=(proc, tail), daemon=True).start()

    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = 'offscreen'
    gp = subprocess.Popen(
        [gui_python, gui, '--backend', 'renode', '--port', str(port),
         '--state-port', str(state_port), '--control-port', str(control)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

    sock = None
    replies = []
    try:
        deadline = time.time() + seconds
        # the first launch builds a Qt font cache, which is slow
        while time.time() < deadline:
            try:
                sock = socket.create_connection(('127.0.0.1', control), timeout=5)
                break
            except OSError:
                if gp.poll() is not None:
                    check('the GUI starts', False,
                          (gp.stdout.read() or '')[-500:])
                    return {}
                time.sleep(1.0)
        if sock is None:
            check('the GUI control port comes up', False, 'timed out')
            return {}
        sock.settimeout(None)
        f = sock.makefile('r')
        threading.Thread(target=lambda: [replies.append(l.rstrip()) for l in f],
                         daemon=True).start()

        def send(cmd):
            sock.sendall((cmd + '\n').encode())

        for cmd in ('ds_type dshot300', 'ds_bidir 1', 'ds_enable 1'):
            send(cmd)
            time.sleep(0.2)

        t0 = gui_wait_sim(send, replies, 0.0, deadline)
        if t0 is None:
            print('\n'.join(tail))
            check('the GUI sees the state stream', False, 'no stream_t')
            return {}
        if gui_wait_sim(send, replies, t0 + 2.5, deadline) is None:
            check('the GUI reaches the arming window', False, 'stalled')
            return {}
        send('ds_value %d' % value)
        if gui_wait_sim(send, replies, t0 + 4.0, deadline) is None:
            check('the GUI reaches the settled window', False, 'stalled')
            return {}
        send('status')
        time.sleep(1.0)
        bds = last_reply(replies, 'STATUS BDShot:')
        # ask the GUI to exit rather than killing it, so its own shutdown
        # runs and a traceback on the way out is still visible
        try:
            send('quit')
        except OSError:
            pass
        sock.close()
        sock = None
        try:
            gp.wait(timeout=15)
        except subprocess.TimeoutExpired:
            gp.terminate()
        return {'bds': bds, 'out': gp.stdout.read() or ''}
    finally:
        if sock is not None:
            sock.close()
        if gp.poll() is None:
            gp.kill()
        proc.terminate()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def last_reply(replies, prefix):
    for line in reversed(list(replies)):
        if line.startswith(prefix):
            return line
    return ''


def gui_wait_sim(send, replies, until, deadline):
    """simulated time as the GUI reports it, from its own state stream"""
    while time.time() < deadline:
        send('status')
        time.sleep(1.0)
        line = last_reply(replies, 'STATUS rpmhist:')
        m = re.search(r'stream_t=(-?[\d.]+)', line)
        if m and float(m.group(1)) >= until:
            return float(m.group(1))
    return None


def report_gui(res, target):
    bds = res.get('bds', '')
    if not bds:
        check('the GUI reports telemetry', False, 'no BDShot status line')
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    m = re.search(r'rpm=(\d+)', bds)
    rpm = int(m.group(1)) if m else 0
    want = expected(target)
    if want is None:
        check('the GUI drives the emulated ESC', 500 < rpm < 20000,
              'rpm=%d, no recorded figure for this target' % rpm)
    else:
        check('the GUI drives it to the recorded speed',
              abs(rpm - want['rpm']) <= LINK_TOLERANCE * want['rpm'],
              'rpm=%d, expected %d' % (rpm, want['rpm']))
    check('the GUI shows the motor spinning', 'spinning' in bds, bds)
    check('no reply CRC failures', 'badcrc=0' in bds, bds)
    out = res.get('out', '')
    check('the GUI raises nothing', 'Traceback' not in out,
          out[-500:] if 'Traceback' in out else 'clean')
    if failures:
        print('\n%u test(s) failed: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('\nall tests passed')
    return 0


def start_renode(renode, target_resc, elf, eeprom, model, so, scratch,
                 port, state_port, dshot_us, gcc='arm-none-eabi-gcc',
                 nm='arm-none-eabi-nm', can_bus=None):
    # what a client needs to say which firmware is running and how far
    # through arming it is; absent symbols simply lose that readout
    addrs = gen_target.symbol_addresses(
        elf, ('filename', 'armed_timeout_count', 'armed', 'eepromBuffer'),
        nm)
    app_base = gen_target.APP_BASE
    try:
        cfg = gen_target.config(os.path.basename(target_resc)
                                .replace('.resc', ''), gcc)
    except gen_target.Unsupported:
        cfg = None
    if cfg is not None:
        app_base = cfg['app_base']
    info = ['guilink AppBase 0x%08X' % app_base]
    if cfg is not None:
        info.append('guilink LoopHz %d' % cfg['loop_hz'])
    for prop, sym in (('FirmwareNameAddress', 'filename'),
                      ('ArmedCountAddress', 'armed_timeout_count'),
                      ('ArmedAddress', 'armed'),
                      ('EepromBufferAddress', 'eepromBuffer')):
        if sym in addrs:
            info.append('guilink %s 0x%08X' % (prop, addrs[sym]))
    resc = os.path.join(scratch, 'link.resc')
    with open(resc, 'w') as f:
        f.write('\n'.join([
            '$repo=@%s' % REPO,
            '$elf=@%s' % elf,
            '$eeprom=@%s' % eeprom,
            'include @%s' % target_resc,
            'logLevel 3',
            'cpu AddSymbolHook "delayMillis" "execfile(\'%s/Mcu/Renode/scripts/skip_delays.py\')"' % REPO,
            'bridge LibraryPath "%s"' % so,
            'bridge ConfigPath "%s"' % model,
            'guilink DshotFrameUs %d' % dshot_us,
            '\n'.join(info),
            'guilink InputPort %d' % port,
            'guilink StatePort %d' % state_port,
        ] + ([
            'canmcast Bus %d' % can_bus,
        ] if can_bus is not None else []) + [
            'start',
            '']))
    # stdin stays open: the monitor treats EOF as "quit", and this run has
    # to outlive the command that started it
    return subprocess.Popen(
        [renode, '--config', os.path.join(scratch, 'renode-link-config'),
         '--disable-xwt', '--console', '-e', 'include @%s' % resc],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, env=gen_target.renode_env())


def drain(proc, tail):
    for line in proc.stdout:
        tail.append(line.decode(errors='replace').rstrip())


def wait_sim(sim, until, deadline, ds=None):
    '''simulated time from the state stream, once it has reached `until`.
       None if the wall clock backstop expires first.'''
    said = 0.0
    while time.time() < deadline:
        s = sim.latest()
        if s is not None and s[0] >= until:
            return s[0]
        # a run is minutes long and mostly waiting; say where it is, so a
        # stall is distinguishable from slow progress
        if time.time() - said > 15.0:
            said = time.time()
            print('  ... simulated %.2fs of %.2fs, %d replies'
                  % (s[0] if s else 0.0, until,
                     ds.replies.count if ds else 0))
            sys.stdout.flush()
        time.sleep(0.2)
    return None


def link_sample(ds, sim):
    s = sim.latest()
    return {
        'sim_rpm': s[1] * 60.0 / (2 * 3.14159265358979),
        'vbus': s[10],
        'telem_rpm': ds.rpm,
        'replies': ds.replies.count,
        'badcrc': ds.badcrc,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', default='FD6288_F051',
                    help='any target in Inc/targets.h whose MCU family has '
                         'a platform base (gen_target.py --list shows them)')
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
    # the GUI's path rather than the monitor's: udp setpoints in,
    # telemetry and physics samples back
    ap.add_argument('--link', action='store_true',
                    help='drive through the guilink udp ports as sitl_gui.py '
                         'does, instead of scripting the monitor. Costs about a '
                         'minute a target rather than seconds, because a real '
                         'client has to be on the other end')
    # off the SITL's own 57733/57734 so a running SITL or GUI does not
    # collide with a test
    ap.add_argument('--link-port', type=int, default=57833)
    ap.add_argument('--link-state-port', type=int, default=57834)
    ap.add_argument('--link-dshot-us', type=int, default=1000,
                    help='dshot frame period on the wire, virtual us. 1000 is '
                         '1kHz: four times cheaper to emulate than the 250 the '
                         'GUI defaults to, and the wire rate is not what this '
                         'test is about')
    ap.add_argument('--gui', action='store_true',
                    help='drive the real Mcu/SITL/sitl_gui.py against the link, '
                         'under Qt offscreen and scripted through its control '
                         'port. Implies --link')
    ap.add_argument('--gui-python', default=None,
                    help='interpreter with PySide6; default the SITL venv')
    # DroneCAN over the emulated bxCAN, bridged to the SITL mcast bus -
    # the L431 (bxCAN) and G431 (FDCAN) _CAN targets have the peripheral
    ap.add_argument('--can', action='store_true',
                    help='arm and throttle over DroneCAN through the mcast '
                         'CAN bridge, as dronecan_gui_tool would; needs the '
                         'python dronecan package')
    ap.add_argument('--can-bus', type=int, default=7,
                    help='mcast bus number for the --can test, off the '
                         'default bus 0 so a live SITL or GUI on the same '
                         'machine is not disturbed')
    ap.add_argument('--link-seconds', type=float, default=600,
                    help='wall clock backstop; the test itself is paced by '
                         'simulated time')
    args = ap.parse_args()
    if args.target in UNSPINNABLE:
        skip('%s: %s' % (args.target, UNSPINNABLE[args.target]))
    if args.edt:
        args.bdshot = True
    if args.bdshot and not args.dshot:
        args.dshot = 600
    if args.can:
        try:
            import dronecan  # noqa: F401
        except ImportError:
            skip('the python dronecan package is not installed')
        if not 0 <= args.can_bus <= 9:
            skip('--can-bus must be 0..9 (the mcast scheme is '
                 '239.65.82.<bus>)')

    if args.elf is None:
        args.elf = gen_target.find_elf(args.target)
        if args.elf is None:
            skip('no firmware in obj/ for %s; build it first' % args.target)
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
            target_cfg = gen_target.config(args.target, args.gcc)
            if args.can and not target_cfg['dronecan']:
                skip('%s has no CAN peripheral' % args.target)
        except gen_target.Unsupported as e:
            skip(str(e))

        eeprom = os.path.join(scratch, 'eeprom.bin')
        # INPUT_SIGNAL_TYPE 0 is mandatory: the default is DSHOT_IN, and
        # with dshot set detectInput() never calls checkServo(), so a
        # servo signal is ignored with no diagnostic. The CAN test wants
        # type 5 (dronecan only) instead - without it the throttle
        # generator's self-started zero-servo signal fights the CAN
        # input over newinput and the ESC never arms - plus a fixed
        # node id so no DNA allocator is needed.
        if args.can:
            overrides = {'INPUT_SIGNAL_TYPE': 5, 'CAN_NODE': CAN_NODE_ID}
        else:
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

        if args.can:
            res = run_can(renode, target_resc, args.elf, eeprom, args.model,
                          so, scratch, args.link_port, args.link_state_port,
                          args.can_bus, args.link_seconds, args.gcc, args.nm)
            return report_can(res, args.target, CAN_NODE_ID)

        if args.gui:
            # the SITL keeps PySide6 in a venv of its own, but a system
            # python that has it works just as well; the GUI prints its
            # own diagnostic if neither does
            venv = os.path.join(REPO, 'Mcu', 'SITL', 'venv', 'bin', 'python3')
            gui_python = args.gui_python or (
                venv if os.path.exists(venv) else sys.executable)
            res = run_gui(renode, target_resc, args.elf, eeprom, args.model,
                          so, scratch, args.link_port, args.link_state_port,
                          args.link_dshot_us, LINK_DSHOT_VALUE,
                          args.link_seconds, gui_python, args.gcc, args.nm)
            return report_gui(res, args.target)

        if args.link:
            link_bidir = args.target not in BDSHOT_NOARM
            res = run_link(renode, target_resc, args.elf, eeprom, args.model,
                           so, scratch, args.link_port, args.link_state_port,
                           args.link_dshot_us, LINK_DSHOT_VALUE,
                           args.link_seconds, args.gcc, args.nm,
                           bidir=link_bidir)
            return report_link(res, args.target, motor, bidir=link_bidir)

        res = run(renode, target_resc, args.elf, eeprom, args.model, so, syms,
                  scratch, throttle_addr, timer_name,
                  physics=not args.no_physics,
                  dshot=args.dshot, bidir=args.bdshot, edt=args.edt)

    a = res.get('armed', {})
    s = res.get('spin', {})
    noarm = args.bdshot and args.target in BDSHOT_NOARM
    if noarm:
        print('NOTE: arming and spin not asserted: %s'
              % BDSHOT_NOARM[args.target])
    if a:
        if not noarm:
            check('arms on a %s signal'
                  % ('%sdshot%d' % ('bi' if args.bdshot else '', args.dshot)
                     if args.dshot else 'servo'),
                  a.get('armed') == 1,
                  'armed=%d' % a.get('armed', -1))
        check('does not spin unarmed', a.get('running') == 0 and a.get('rpm', 0) == 0,
              'running=%d rpm=%d' % (a.get('running', -1), a.get('rpm', -1)))
    if s and not noarm:
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
    if r and not args.edt and not noarm:
        frame = r.get('frame', 0)
        payload = frame >> 4
        period = (payload & 0x1FF) << (payload >> 9)
        poles = int(motor.get('poles', 14))
        erpm = 60e6 / period if period else 0
        # some families misreport by a fixed factor on real hardware
        # too; hold the reply to what the real ESC would say
        scale = REPLY_RPM_SCALE.get(target_cfg['family'], 1.0)
        want = s.get('rpm', 0) * scale
        got = erpm / (poles / 2)
        check('reply reports the measured rpm',
              want and abs(got - want) < 0.02 * want,
              'frame=0x%04X period=%dus -> %drpm, physics %drpm%s'
              % (frame, period, got, s.get('rpm', 0),
                 '' if scale == 1.0 else
                 ' (x%.4f firmware scale -> %drpm expected)'
                 % (scale, want)))
    if r and args.edt and not noarm:
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
