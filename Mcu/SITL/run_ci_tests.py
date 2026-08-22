#!/usr/bin/env python3
'''
CI test runner for the AM32 SITL: starts the simulator, drives it
through the PWM/DShot and DroneCAN input paths and asserts on the
results. Only needs the python standard library; the DroneCAN test runs
when the dronecan package is importable and is skipped otherwise.

usage: run_ci_tests.py [--sitl path/to/elf]
exits non-zero if any test fails.
'''

import argparse
import glob
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sitl_dshot as sd
import sitl_params
import sitl_tones
from sitl_fourway_server import crc16_xmodem
from sitl_gui_backend import EepromClient, SimStream, ToneStream, AudioStream

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT_PORT = 57833
STATE_PORT = 57834
CAN_URI = 'mcast:7'

failures = []


def check(name, cond, detail):
    status = 'PASS' if cond else 'FAIL'
    print('%s: %s (%s)' % (status, name, detail))
    sys.stdout.flush()
    if not cond:
        failures.append(name)


class Sitl(object):
    def __init__(self, sitl_path, extra_args=(), nosleep=True):
        # --nosleep busy-waits to hold the ratio steady, but that pins a
        # core. With a heavy client (pydronecan runs its IO in a separate
        # process) on a CPU-starved CI runner it can starve the client so
        # commands never reach the firmware, which then no-signal resets.
        # Such tests pass nosleep=False to let the SITL yield the core.
        args = [sitl_path, '--input-port', str(INPUT_PORT),
                '--state-port', str(STATE_PORT)] + list(extra_args)
        if nosleep:
            args.append('--nosleep')
        self.log = open('sitl_ci.log', 'ab')
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=self.log)
        time.sleep(0.5)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        # SIGTERM (not kill) so a coverage build can flush its .gcda via
        # the SITL's signal handler; fall back to kill if it lingers
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self.log.close()
        for f in os.listdir('.'):
            if f.startswith('am32_eeprom.bin'):
                os.unlink(f)


class Sender(object):
    '''background frame sender with rate catch-up, like the GUI'''

    def __init__(self, ptype, bidir=False, rate=500.0):
        self.port = sd.InputPort('127.0.0.1', INPUT_PORT)
        self.ptype = ptype
        self.bidir = bidir
        self.rate = rate
        self.value = 1000 if ptype == sd.TYPE_PWM else 0
        self.cmds = []
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        nxt = time.time()
        try:
            self._run()
        except OSError:
            pass  # socket closed by stop()

    def _run(self):
        nxt = time.time()
        while self.running:
            now = time.time()
            burst = 0
            while now >= nxt and burst < 10:
                nxt += 1.0 / self.rate
                if self.cmds:
                    self.port.send_dshot(self.cmds.pop(0), ptype=self.ptype,
                                         telem=True, bidir=self.bidir)
                elif self.ptype == sd.TYPE_PWM:
                    self.port.send_pwm(int(self.value))
                else:
                    self.port.send_dshot(int(self.value), ptype=self.ptype,
                                         bidir=self.bidir)
                burst += 1
            if now - nxt > 0.25:
                nxt = now
            time.sleep(0.0005)

    def stop(self):
        self.running = False
        self.port.close()


def sim_sleep(sim, secs, wall_cap=90.0):
    """wait for `secs` of SIMULATION time, polling the state stream. The
    tests drive the firmware over wall time but the firmware lives in
    sim time, so a coverage or sanitizer build (slower than real time)
    would otherwise arm/spin-up too little. Falls back to wall time if
    no samples arrive. SimStream timestamps (smp[0]) are already seconds"""
    t0 = None
    deadline = time.monotonic() + wall_cap
    while time.monotonic() < deadline:
        smp = sim.latest()
        if smp is not None:
            if t0 is None or smp[0] < t0:
                t0 = smp[0]              # (re-)anchor; a reboot resets sim time
            elif smp[0] - t0 >= secs:
                return
        time.sleep(0.01)


def rpm_from_state(sim, window=1.0):
    w = sim.window(window)
    if not w:
        return -1
    return sum(s[1] for s in w) / len(w) * 60.0 / 6.28318


def sim_time(sim):
    '''latest simulation timestamp on the state stream, or None'''
    w = sim.window(0.05)
    return w[-1][0] if w else None


def test_dshot(sitl_path, name, ptype, bidir, edt, value, rpm_lo, rpm_hi, input_type=1):
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', str(input_type)]):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(ptype, bidir=bidir)
        try:
            sim_sleep(sim, 2.2)                # arm at zero throttle
            if edt:
                # the firmware wants 6 repeats while armed and stopped, and
                # ignores the command once spinning. Frames can be lost on
                # overloaded runners, so confirm the latch from the EDT
                # frames in the replies and retry before throttling up
                for _ in range(6):
                    tx.port.get_replies()
                    tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 20
                    sim_sleep(sim, 0.5)
                    got = [sd.decode_reply(r[3], edt_expected=True)
                           for r in tx.port.get_replies()]
                    if any(k not in ('erpm', 'badcrc') for k, v in got):
                        break
            tx.value = value
            sim_sleep(sim, 4.0)
            rpm = rpm_from_state(sim)
            check(name + ' rpm', rpm_lo <= rpm <= rpm_hi,
                  'rpm=%.0f expected %d..%d' % (rpm, rpm_lo, rpm_hi))
            if bidir:
                replies = tx.port.reply_count
                check(name + ' bdshot replies', replies > 500,
                      'replies=%d' % replies)
                erpm = [sd.decode_reply(r[3], edt_expected=edt)
                        for r in tx.port.get_replies()]
                rpms = [sd.erpm_period_to_rpm(v) for k, v in erpm if k == 'erpm']
                if rpms:
                    check(name + ' bdshot rpm agrees',
                          abs(rpms[-1] - rpm) < max(200, rpm * 0.05),
                          'bdshot=%.0f state=%.0f' % (rpms[-1], rpm))
                if edt:
                    edt_vals = dict((k, v) for k, v in erpm
                                    if k in ('temp', 'volt', 'current'))
                    check(name + ' edt values',
                          edt_vals.get('temp') == 38 and 11 < edt_vals.get('volt', 0) < 13,
                          'edt=%s' % edt_vals)
            # motor must stop again at zero throttle
            tx.value = 1000 if ptype == sd.TYPE_PWM else 0
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim, 0.3)
            check(name + ' stops', rpm < 500, 'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def wait_for_notes(stream, expected, timeout=10.0):
    '''wait until the expected note sequence appears in the tone stream.
    Durations come from the simulated timestamps, so this is exact under
    --nosleep too'''
    deadline = time.time() + timeout
    notes = []
    while time.time() < deadline:
        with stream.lock:
            events = list(stream.events)
        notes = sitl_tones.events_to_notes(events)
        found = sitl_tones.match_notes(notes, expected,
                                       freq_tol=0.02, dur_tol=0.05)
        if found:
            return found, notes
        time.sleep(0.05)
    return None, notes


def test_startup_tune(sitl_path):
    '''the boot tune must arrive on the tone event stream: 3 rising
    notes from TIM1 PSC 55/40/25 at ARR 6665, 200ms each'''
    expected = [(428.6, 0.2), (585.4, 0.2), (923.2, 0.2)]
    tones = ToneStream('127.0.0.1', STATE_PORT)
    try:
        with Sitl(sitl_path, ['--can-uri', 'none']):
            found, notes = wait_for_notes(tones, expected)
            check('startup tune', found is not None,
                  'notes=%s' % ['%.1fHz %.3fs' % n[:2] for n in notes])
    finally:
        tones.close()


def test_beacon_tone(sitl_path):
    '''DShot beacon command 1 plays playDefaultTone: PSC 50 then 30,
    150ms each'''
    expected = [(470.6, 0.15), (774.3, 0.15)]
    tones = ToneStream('127.0.0.1', STATE_PORT)
    try:
        with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
            sim = SimStream('127.0.0.1', STATE_PORT, period_us=1000)
            sim.enabled = True
            tx = Sender(sd.TYPE_DSHOT600)
            try:
                sim_sleep(sim, 2.2)            # arm at zero throttle
                tx.cmds = [1] * 8              # DSHOT_CMD_BEACON1
                found, notes = wait_for_notes(tones, expected, timeout=25.0)
                check('beacon tone', found is not None,
                      'notes=%s' % ['%.1fHz %.3fs' % n[:2] for n in notes])
            finally:
                tx.stop()
                sim.close()
    finally:
        tones.close()


def test_physics_audio(sitl_path):
    '''the physics audio stream must carry the boot tune: some 100ms
    sim-time window has the first note frequency (428.6Hz) dominant'''
    audio = AudioStream('127.0.0.1', STATE_PORT)
    try:
        with Sitl(sitl_path, ['--can-uri', 'none']):
            deadline = time.time() + 12
            batches = []
            ok = False
            while time.time() < deadline and not ok:
                time.sleep(0.5)
                batches += audio.take_batches()
                buckets = {}
                for t0, vals in batches:
                    for i, v in enumerate(vals):
                        t = t0 + i * 20833
                        buckets.setdefault(t // 100000000, []).append(v)
                for vals in buckets.values():
                    if len(vals) < 2000:
                        continue
                    g1 = sitl_tones.goertzel(vals, 428.6)
                    g2 = sitl_tones.goertzel(vals, 700.0)
                    if g1 > 1e-3 and g1 > 5 * g2:
                        ok = True
                        break
            check('physics audio boot tune', ok,
                  '%d batches captured' % len(batches))
    finally:
        audio.close()


def can_state_stream(name):
    '''probe for CAN-enabled tests: some CI runners (github macos) have
    no multicast capable route and the SITL cannot bring CAN up. Returns
    a live SimStream, or None after printing a SKIP with the SITL log
    for diagnosis'''
    sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
    sim.enabled = True
    deadline = time.time() + 5
    while time.time() < deadline and not sim.samples:
        time.sleep(0.2)
    if not sim.samples:
        print('SKIP: %s, SITL state stream never started with CAN enabled, '
              'multicast is probably unavailable on this host. SITL log tail:'
              % name)
        sys.stdout.flush()
        os.system('tail -5 sitl_ci.log')
        sim.close()
        return None
    return sim


def test_dronecan(sitl_path):
    try:
        import dronecan
    except ImportError:
        print('SKIP: dronecan not installed, DroneCAN test skipped')
        return
    with Sitl(sitl_path, ['--can-uri', CAN_URI, '--node-id', '10'],
              nosleep=False):
        sim = can_state_stream('dronecan')
        if sim is None:
            return
        node = dronecan.make_node(CAN_URI, node_id=100, bitrate=1000000)
        status = {}

        def on_esc(e):
            status['rpm'] = e.message.rpm
            status['voltage'] = e.message.voltage

        node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc)
        # the phases are timed on SIMULATION time, not wall clock: the
        # firmware arms only after a full second of zero throttle in its
        # own time (main.c armed_timeout_count), and this SITL is paced,
        # so a loaded runner that holds well under 1x used to end the
        # wall-clock zero phase before arming completed - the ESC then
        # ignored the throttle for the rest of the run and read rpm=0
        ZERO_S, RUN_S = 2.5, 5.5
        t0 = time.time()
        nxt, sim0, elapsed = t0, None, 0.0
        while time.time() - t0 < 120:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                t = sim_time(sim)
                if t is not None:
                    if sim0 is None:
                        sim0 = t
                    elapsed = t - sim0
                thr = 0.35 if elapsed > ZERO_S else 0.0
                node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[int(8191 * thr)]))
                if elapsed >= ZERO_S + RUN_S:
                    break
            time.sleep(0.001)
        # read rpm from the state stream while the motor is still driven;
        # commands have stopped, so a wait here would let it coast down
        rpm = rpm_from_state(sim)
        # the pacing ratio is in the detail so a future failure here says
        # whether the runner simply ran out of wall clock
        paced = '%.1fs sim in %.1fs wall' % (elapsed, time.time() - t0)
        check('dronecan rpm', 2500 <= rpm <= 5000, 'rpm=%.0f (%s)' % (rpm, paced))
        check('dronecan telemetry', 2500 <= status.get('rpm', -1) <= 5000
              and 11 < status.get('voltage', 0) < 13,
              'esc.Status=%s (%s)' % (status, paced))
        node.close()
        sim.close()


def test_dshot_direction(sitl_path):
    """DShot command 8 reverses direction; the rotor omega changes sign"""
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600)
        try:
            sim_sleep(sim, 2.2)                   # arm
            tx.value = 400
            sim_sleep(sim, 3.0)
            fwd = sim.latest()[1] if sim.latest() else 0
            # a loaded runner can drop enough input frames that the
            # repeated command never registers; command 8 is absolute
            # (not a toggle) so re-sending it is safe
            for attempt in range(3):
                tx.value = 0
                sim_sleep(sim, 1.5)
                tx.cmds = [8] * 20                # reverse direction
                sim_sleep(sim, 1.5)
                tx.cmds = []
                tx.value = 400
                sim_sleep(sim, 3.0)
                rev = sim.latest()[1] if sim.latest() else 0
                if fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50:
                    break
                if attempt < 2:
                    print('  reverse attempt %d failed, retrying'
                          % (attempt + 1), flush=True)
            check('dshot reverse direction',
                  fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50,
                  'omega fwd=%.0f rev=%.0f' % (fwd, rev))
        finally:
            tx.stop()
            sim.close()


def test_bidirectional(sitl_path):
    """3D/bidirectional mode: command 10 enables it, then a reverse-half
    throttle spins the motor backwards"""
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600, bidir=True)
        try:
            sim_sleep(sim, 2.2)
            # command 10 is absolute (3D on, not a toggle) so a retry
            # can safely re-send it if a loaded runner dropped the burst
            for attempt in range(3):
                tx.cmds = [10] * 20               # enable 3D
                sim_sleep(sim, 1.5)
                tx.cmds = []
                tx.value = 1400                   # forward half
                sim_sleep(sim, 3.0)
                fwd = sim.latest()[1] if sim.latest() else 0
                tx.value = 0
                sim_sleep(sim, 1.5)
                tx.value = 600                    # reverse half
                sim_sleep(sim, 3.0)
                rev = sim.latest()[1] if sim.latest() else 0
                if fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50:
                    break
                if attempt < 2:
                    print('  bidirectional attempt %d failed, retrying'
                          % (attempt + 1), flush=True)
                    tx.value = 0
                    sim_sleep(sim, 1.5)
            check('bidirectional reverse',
                  fwd * rev < 0 and abs(fwd) > 50 and abs(rev) > 50,
                  'omega fwd=%.0f rev=%.0f' % (fwd, rev))
        finally:
            tx.stop()
            sim.close()


def test_dshot_edt_toggle(sitl_path):
    """EDT enable (13) then disable (14) command path"""
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=1000)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600, bidir=True)
        try:
            sim_sleep(sim, 2.2)
            tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 10
            sim_sleep(sim, 1.0)
            tx.cmds = [12] * 10                   # save settings to eeprom
            sim_sleep(sim, 1.0)
            tx.cmds = [sd.DSHOT_CMD_EDT_DISABLE] * 10
            sim_sleep(sim, 1.0)
            tx.cmds = []
            tx.value = 300
            sim_sleep(sim, 2.0)
            # the point is exercising the paths without a desync/crash;
            # a sanitizer/coverage build is what gains from this
            check('dshot edt toggle survives', tx.port.reply_count > 100,
                  'replies=%d' % tx.port.reply_count)
        finally:
            tx.stop()
            sim.close()


def test_debugger_pause(sitl_path):
    """a stopped process (debugger breakpoint, SIGSTOP, laptop suspend)
    must freeze the simulation and resume cleanly: no watchdog reset,
    no catch-up sprint compressing the backlog, motor still spinning.
    SIGSTOP/SIGCONT exercises exactly the wall-clock jump a debugger
    causes, without needing gdb on the CI runner"""
    if not hasattr(signal, 'SIGSTOP'):
        print('SKIP: no SIGSTOP on this platform, debugger pause test skipped')
        return
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']) as s:
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600)
        try:
            sim_sleep(sim, 2.2)                # arm at zero throttle
            tx.value = 800
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim)
            check('debugger pause spins', 2000 <= rpm <= 4000, 'rpm=%.0f' % rpm)
            t_stop = sim.latest()[0]
            s.proc.send_signal(signal.SIGSTOP)
            time.sleep(3.0)                    # wall time; sim is frozen
            frozen = sim.latest()[0]
            s.proc.send_signal(signal.SIGCONT)
            wall0 = time.monotonic()
            sim_sleep(sim, 2.0)
            wall_elapsed = time.monotonic() - wall0
            rpm = rpm_from_state(sim)
            after = sim.latest()[0]
            check('debugger pause freezes sim time',
                  0 <= frozen - t_stop < 0.1,
                  'sim advanced %.2fs during a 3s stop' % (frozen - t_stop))
            # at speedup 1 simulated time may never advance faster than
            # the wall clock: a catch-up sprint through the 3s backlog
            # would, so paced resumption is provable from the wall time
            # the post-resume simulated seconds took
            check('debugger pause resumes paced',
                  wall_elapsed > (after - frozen) * 0.9,
                  'sim advanced %.2fs in %.2fs of wall' % (after - frozen,
                                                           wall_elapsed))
            check('debugger pause still spinning', 2000 <= rpm <= 4000,
                  'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def test_stuck_rotor(sitl_path, protection):
    """stuck rotor (prop blocked mid-flight, e.g. hitting a tree
    branch; state port cmd 7): with STUCK_ROTOR_PROTECTION on the
    firmware must cut the output after the commutation timeouts and
    stay off until the throttle is cycled through zero; with it off
    the motor must restart by itself once the obstruction clears"""
    name = 'stuck rotor protection %s' % ('on' if protection else 'off')
    offset = dict((p[1], p[0])
                  for p in sitl_params.PARAMS)['STUCK_ROTOR_PROTECTION']
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        ok, msg = EepromClient('127.0.0.1', STATE_PORT).set(
            offset, bytes([1 if protection else 0]))
        check(name + ' eeprom set', ok, msg)
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(sd.TYPE_DSHOT600)
        try:
            sim_sleep(sim, 2.2)                # arm at zero throttle
            tx.value = 800
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim)
            check(name + ' spins', 2000 <= rpm <= 4000, 'rpm=%.0f' % rpm)
            sim.set_stuck(1.0)                 # the prop hits the branch
            sim_sleep(sim, 3.0)
            rpm = rpm_from_state(sim, 0.5)
            check(name + ' stalls', abs(rpm) < 100, 'rpm=%.0f' % rpm)
            sim.set_stuck(0.0)                 # the obstruction clears
            sim_sleep(sim, 4.0)
            rpm = rpm_from_state(sim, 0.5)
            if protection:
                # latched off although the throttle is still raised
                check(name + ' stays off', abs(rpm) < 100, 'rpm=%.0f' % rpm)
                tx.value = 0                   # throttle cycle resets it
                sim_sleep(sim, 1.0)
                tx.value = 800
                sim_sleep(sim, 3.0)
                rpm = rpm_from_state(sim)
                check(name + ' recovers after throttle cycle',
                      2000 <= rpm <= 4000, 'rpm=%.0f' % rpm)
            else:
                check(name + ' restarts by itself', 2000 <= rpm <= 4000,
                      'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def test_dronecan_params(sitl_path):
    """DroneCAN parameter get/set round-trip through the firmware"""
    try:
        import dronecan
    except ImportError:
        print('SKIP: dronecan not installed, DroneCAN param test skipped')
        return
    with Sitl(sitl_path, ['--can-uri', 'mcast:4', '--node-id', '40'],
              nosleep=False):
        sim = can_state_stream('dronecan param set')
        if sim is None:
            return
        sim.close()
        node = dronecan.make_node('mcast:4', node_id=101)
        try:
            time.sleep(2.0)
            got = {}

            def getset(req, key):
                done = {}
                def cb(e):
                    done['e'] = e
                node.request(req, 40, cb)
                t0 = time.time()
                while 'e' not in done and time.time() - t0 < 3:
                    try:
                        node.spin(0.05)
                    except Exception:
                        pass
                got[key] = done.get('e')

            # read TELEM_RATE, set it, read it back
            gp = dronecan.uavcan.protocol.param.GetSet.Request
            getset(gp(name=b'TELEM_RATE'), 'read')
            newval = dronecan.uavcan.protocol.param.Value(integer_value=77)
            getset(gp(name=b'TELEM_RATE', value=newval), 'set')
            getset(gp(name=b'TELEM_RATE'), 'reread')
            ok = (got['reread'] is not None and
                  got['reread'].response.value.integer_value == 77)
            check('dronecan param set', ok,
                  'reread=%s' % (got['reread'].response.value.integer_value
                                 if got['reread'] else None))
        finally:
            node.close()


def test_dataset_params():
    '''every calibration dataset's sitl.param must still build an eeprom
    image, and must describe the motor its model simulates. This is what
    keeps the parameter files from ageing: a renamed or dropped setting
    fails to parse here instead of silently meaning something else'''
    import json
    from run_calibration_tests import DATASETS
    for ds in DATASETS:
        param = os.path.join(ds['data'], 'sitl.param')
        try:
            image = sitl_params.image_from_param_file(param)
        except (OSError, ValueError, KeyError) as ex:
            check('dataset params %s' % ds['name'], False, str(ex))
            continue
        model = json.load(open(ds['model'])).get('motor', {})
        # the stored Kv is the real ESC's setting and the model's is the
        # fitted effective value, so they differ a little by design; the
        # band is the one Mcu/SITL/Src/eeprom.c warns outside of, where
        # low rpm power protection starts clamping duty
        ee_kv = sitl_params.byte_to_kv(image[
            sitl_params.PARAMS_BY_NAME['MOTOR_KV'][0]])
        poles = image[sitl_params.PARAMS_BY_NAME['MOTOR_POLES'][0]]
        bad = []
        if not model['kv'] * 0.8 <= ee_kv <= model['kv'] * 1.25:
            bad.append('MOTOR_KV %d vs model %.0f' % (ee_kv, model['kv']))
        if poles != model['poles']:
            bad.append('MOTOR_POLES %d vs model %d' % (poles, model['poles']))
        if len(image) != sitl_params.EEPROM_SIZE:
            bad.append('image is %d bytes' % len(image))
        check('dataset params %s' % ds['name'], not bad,
              '; '.join(bad) if bad else
              '%d settings, %d Kv, %d poles'
              % (len(sitl_params.parse_param_file(param)), ee_kv, poles))


def test_fc_capture(sitl_path):
    '''scripts/esc_capture_fc.py against the fake-Betaflight MSP stub:
    the FC capture path must produce the calibration JSONL with sane
    values at full telemetry rate'''
    try:
        import pty  # noqa: F401  (POSIX only)
        import serial  # noqa: F401
    except ImportError as ex:
        print('SKIP: fc capture, %s' % ex)
        sys.stdout.flush()
        return
    import json
    import msp_stub_fc
    tool = os.path.join(HERE, '..', '..', 'scripts', 'esc_capture_fc.py')
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1']):
        stub = msp_stub_fc.MspStubFC(sitl_port=INPUT_PORT)
        try:
            r = subprocess.run(
                [sys.executable, tool, 'sweep', '--port', stub.slave_path,
                 '--log', 'fc_capture_sweep.jsonl',
                 '--levels', '0.25,0.35', '--hold', '4.0',
                 '--arm-time', '3.0', '--countdown', '0', '--poles', '14',
                 # this backend runs on the wall clock (a real FC has no
                 # sim time), so a sanitizer or coverage build that lags
                 # wall time needs slack in the stall guards
                 '--stall-timeout', '3.0', '--status-timeout', '10.0'],
                timeout=180, capture_output=True, text=True)
            check('fc capture exits cleanly', r.returncode == 0,
                  'exit=%s stderr=%s' % (r.returncode, r.stderr[-300:].strip()))
            if r.returncode != 0:
                return
            rows = [json.loads(line) for line in open('fc_capture_sweep.jsonl')]
            st = [row for row in rows if row['type'] == 'status']
            span = st[-1]['t'] - st[0]['t'] if len(st) > 1 else 0
            rate = len(st) / span if span else 0
            check('fc capture telemetry rate', rate > 40,
                  'rate=%.0fHz rows=%d' % (rate, len(st)))
            # steady rpm at the end of the 0.35 hold: the sim can lag
            # wall time here, so measure the last second of the segment
            marks = [row for row in rows if row['type'] == 'cmd'
                     and row.get('throttle') == 0.35]
            t35 = marks[0]['t']
            tend = min([row['t'] for row in rows
                        if row['type'] == 'cmd' and row['t'] > t35]
                       + [st[-1]['t']])
            seg = [row['rpm'] for row in st if tend - 1.0 < row['t'] <= tend]
            rpm = sum(seg) / len(seg) if seg else 0
            check('fc capture rpm', 2500 < rpm < 4500, 'rpm=%.0f' % rpm)
            # EDT voltage arrives as whole volts through Betaflight
            volts = [row['volt'] for row in st if row['t'] > t35]
            check('fc capture edt voltage', volts and 10 <= max(volts) <= 14,
                  'volt=%s' % (max(volts) if volts else None))
            currs = [row['curr'] for row in st if row['t'] > t35]
            check('fc capture edt current', currs and 0 < max(currs) < 20,
                  'curr=%s' % (max(currs) if currs else None))
        finally:
            stub.close()


class FourWayClient(object):
    '''the configurator side of the link: MSP on a serial port, then
    BLHeli 4-way after MSP_SET_PASSTHROUGH. The framing here is written
    from the protocol rather than shared with sitl_fourway_server, so the
    two implementations have to agree'''

    MSP_SET_PASSTHROUGH = 245

    def __init__(self, path):
        import tty
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)

    def close(self):
        os.close(self.fd)

    def _read(self, n, timeout=3.0):
        out = b''
        deadline = time.time() + timeout
        while len(out) < n and time.time() < deadline:
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if ready:
                out += os.read(self.fd, n - len(out))
        return out

    def msp(self, cmd, payload=b''):
        hdr = struct.pack('<BB', len(payload), cmd)
        ck = 0
        for b in hdr + payload:
            ck ^= b
        os.write(self.fd, b'$M<' + hdr + payload + bytes([ck]))
        head = self._read(5)
        if head[:3] != b'$M>':
            raise IOError('bad msp reply %r' % head)
        size = head[3]
        return self._read(size + 1)[:size]

    def passthrough(self):
        return self.msp(self.MSP_SET_PASSTHROUGH)[0]

    def cmd(self, command, address=0, params=b'\x00'):
        '''one 4-way transaction, returning (params, ack)'''
        body = (bytes([0x2F, command, (address >> 8) & 0xFF, address & 0xFF,
                       len(params) & 0xFF]) + bytes(params))
        os.write(self.fd, body + struct.pack('>H', crc16_xmodem(body)))
        head = self._read(5)
        if len(head) != 5 or head[0] != 0x2E:
            raise IOError('bad 4-way reply %r' % head)
        size = head[4] or 256
        rest = self._read(size + 3)
        if len(rest) != size + 3:
            raise IOError('short 4-way reply')
        if struct.unpack('>H', rest[-2:])[0] != crc16_xmodem(head + rest[:-2]):
            raise IOError('4-way reply crc error')
        return rest[:size], rest[size]


def test_fc_fourway(sitl_path, bootloader):
    '''the fake FC\'s 4-way passthrough: a configurator speaking MSP to
    the stub must reach the ESC bootloader through it, exactly as it
    would through a real flight controller'''
    if bootloader is None:
        print('SKIP: fc 4-way, no --bootloader given')
        sys.stdout.flush()
        return
    try:
        import pty  # noqa: F401  (POSIX only)
    except ImportError as ex:
        print('SKIP: fc 4-way, %s' % ex)
        sys.stdout.flush()
        return
    import msp_stub_fc
    from sitl_fourway_server import (CMD_DEVICE_INIT_FLASH, CMD_DEVICE_READ,
                                     CMD_DEVICE_WRITE, CMD_INTERFACE_EXIT,
                                     CMD_INTERFACE_TEST_ALIVE, ACK_OK)
    with Sitl(sitl_path, ['--can-uri', 'none', '--bootloader', bootloader],
              nosleep=False):
        stub = msp_stub_fc.MspStubFC(sitl_port=INPUT_PORT,
                                     state_port=STATE_PORT, motor=False)
        client = None
        try:
            client = FourWayClient(stub.slave_path)
            check('fc 4-way esc count', client.passthrough() == 1, '')

            info, ack = client.cmd(CMD_DEVICE_INIT_FLASH, params=b'\x00')
            # escDeviceInfo_t: signature (little endian), pin code, boot pages
            sig = info[0] | (info[1] << 8) if len(info) == 4 else 0
            check('fc 4-way init flash', ack == ACK_OK and (sig & 0xFF) == 0x06,
                  'ack=0x%02x info=%s' % (ack, info.hex()))
            check('fc 4-way boot pin', len(info) == 4 and (info[2] & 0x0F) < 16,
                  'pin=0x%02x' % (info[2] if len(info) == 4 else 0))

            _, ack = client.cmd(CMD_INTERFACE_TEST_ALIVE)
            check('fc 4-way keep alive', ack == ACK_OK, 'ack=0x%02x' % ack)

            # the v3 devinfo block, read through the magic address as a
            # configurator does, tells us where the eeprom lives
            block, ack = client.cmd(CMD_DEVICE_READ, address=0x23,
                                    params=bytes([27]))
            m1, m2 = struct.unpack('<II', block[0:8]) if len(block) >= 8 else (0, 0)
            check('fc 4-way devinfo read',
                  ack == ACK_OK and (m1, m2) == (0x5925E3DA, 0x4EB863D9),
                  'ack=0x%02x magic=0x%08x,0x%08x' % (ack, m1, m2))
            if ack != ACK_OK or len(block) < 27:
                return
            eeprom_start = struct.unpack('<H', block[23:25])[0]

            settings, ack = client.cmd(CMD_DEVICE_READ, address=eeprom_start,
                                       params=bytes([48]))
            check('fc 4-way eeprom read', ack == ACK_OK and len(settings) == 48,
                  'ack=0x%02x len=%d' % (ack, len(settings)))

            # write it back with one byte changed and read it again: the
            # whole set address / set buffer / program sequence in one go
            written = bytearray(settings)
            written[0] = 0x01
            written[26] = (written[26] + 1) & 0xFF
            _, ack = client.cmd(CMD_DEVICE_WRITE, address=eeprom_start,
                                params=bytes(written))
            check('fc 4-way eeprom write', ack == ACK_OK, 'ack=0x%02x' % ack)
            back, ack = client.cmd(CMD_DEVICE_READ, address=eeprom_start,
                                   params=bytes([48]))
            # byte 2 is BOOT_LOADER_REVISION, which the bootloader stamps
            # with its own version as it programs the page
            check('fc 4-way eeprom readback',
                  ack == ACK_OK and len(back) == 48
                  and back[:2] + back[3:] == bytes(written[:2] + written[3:]),
                  'ack=0x%02x back=%s' % (ack, back.hex()))
            check('fc 4-way bootloader version stamp',
                  len(back) == 48 and back[2] not in (0x00, 0xFF),
                  'version=0x%02x' % (back[2] if len(back) == 48 else 0))

            _, ack = client.cmd(CMD_INTERFACE_EXIT)
            check('fc 4-way exit', ack == ACK_OK, 'ack=0x%02x' % ack)
        finally:
            if client is not None:
                client.close()
            stub.close()


def test_fc_reconnect():
    """a configurator may open several passthrough sessions against one
    FC - a browser does it every time you reconnect - and each has to
    work. Needs no ESC: the interface commands are answered by the FC
    itself"""
    try:
        import pty  # noqa: F401  (POSIX only)
    except ImportError as ex:
        print('SKIP: fc reconnect, %s' % ex)
        sys.stdout.flush()
        return
    import msp_stub_fc
    from sitl_fourway_server import (CMD_INTERFACE_EXIT,
                                     CMD_INTERFACE_TEST_ALIVE,
                                     CMD_PROTOCOL_GET_VERSION, ACK_OK,
                                     PROTOCOL_VERSION)
    stub = msp_stub_fc.MspStubFC(sitl_port=INPUT_PORT, motor=False)
    client = None
    try:
        client = FourWayClient(stub.slave_path)
        for session in (1, 2, 3):
            ok = True
            detail = ''
            try:
                count = client.passthrough()
                version, ack = client.cmd(CMD_PROTOCOL_GET_VERSION)
                ok = (count == 1 and ack == ACK_OK
                      and version == bytes([PROTOCOL_VERSION]))
                _, ack = client.cmd(CMD_INTERFACE_TEST_ALIVE)
                ok = ok and ack == ACK_OK
                _, ack = client.cmd(CMD_INTERFACE_EXIT)
                ok = ok and ack == ACK_OK
            except IOError as ex:
                ok, detail = False, str(ex)
            check('fc 4-way session %u' % session, ok, detail)
    finally:
        if client is not None:
            client.close()
        stub.close()


def test_usbip_device(unix=False):
    '''the virtual USB serial device: enumeration and both data
    directions, driven straight over the USB/IP socket so it needs no
    vhci_hcd and no root. Both transports, since the export can be a
    unix socket (no port to collide with anything) or tcp'''
    import sitl_usbip

    name = '@am32-usbip-test.%u' % os.getpid()
    what = 'usbip unix' if unix else 'usbip tcp'
    server = sitl_usbip.UsbipServer(unix_path=name if unix else None,
                                    port=None if unix else 0, serial='TEST')
    if unix:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(5.0)
        if unix:
            sock.connect(sitl_usbip.socket_address(name))
        else:
            sock.connect(('127.0.0.1', server.port))

        def recv(n):
            out = b''
            while len(out) < n:
                b = sock.recv(n - len(out))
                if not b:
                    raise IOError('usbip connection closed')
                out += b
            return out

        seq = [0]

        def submit(direction, ep, length, setup=b'\0' * 8, data=b''):
            seq[0] += 1
            sock.sendall(struct.pack('>IIIII', 1, seq[0], 0, direction, ep)
                         + struct.pack('>Iiiii8s', 0, length, 0, 0, 0, setup)
                         + data)
            hdr = recv(48)
            command, sq = struct.unpack('>II', hdr[:8])
            status, actual = struct.unpack('>ii', hdr[20:28])
            payload = recv(actual) if direction == 1 and actual > 0 else b''
            return command, sq, status, payload

        # import the device, as the vhci driver does when it attaches
        sock.sendall(struct.pack('>HHI', 0x0111, 0x8003, 0)
                     + b'1-1'.ljust(32, b'\0'))
        version, code, status = struct.unpack('>HHI', recv(8))
        dev = recv(312)
        vid, pid = struct.unpack('>HH', dev[300:304])
        check('%s import' % what, code == 0x0003 and status == 0
              and (vid, pid) == (0x1209, 0x0001),
              'code=0x%04x status=%u id=%04x:%04x' % (code, status, vid, pid))

        # GET_DESCRIPTOR(device), the host\'s first control transfer
        setup = struct.pack('<BBHHH', 0x80, 6, 0x0100, 0, 18)
        _, _, st, desc = submit(1, 0, 18, setup)
        check('%s device descriptor' % what,
              st == 0 and len(desc) == 18 and desc[1] == 1
              and struct.unpack('<HH', desc[8:12]) == (0x1209, 0x0001),
              'status=%d len=%d' % (st, len(desc)))

        setup = struct.pack('<BBHHH', 0x80, 6, 0x0200, 0, 255)
        _, _, st, cfg = submit(1, 0, 255, setup)
        total = struct.unpack('<H', cfg[2:4])[0] if len(cfg) >= 4 else 0
        check('%s config descriptor' % what,
              st == 0 and len(cfg) == total and cfg[4] == 2
              and bytes([0x0A, 0x00, 0x00]) in cfg,
              'status=%d len=%d total=%d ifaces=%d'
              % (st, len(cfg), total, cfg[4] if len(cfg) > 4 else 0))

        # host to device, then device to host on the bulk pair
        _, _, st, _ = submit(0, sitl_usbip.EP_BULK, 5, data=b'hello')
        got = server.read(1.0)
        check('%s bulk out' % what, st == 0 and got == b'hello',
              'status=%d got=%r' % (st, got))

        # a read urb queued before there is anything to send must be
        # completed by the write, not answered empty
        seq[0] += 1
        pending = seq[0]
        sock.sendall(struct.pack('>IIIII', 1, pending, 0, 1,
                                 sitl_usbip.EP_BULK)
                     + struct.pack('>Iiiii8s', 0, 64, 0, 0, 0, b'\0' * 8))
        time.sleep(0.2)
        server.write(b'world')
        hdr = recv(48)
        sq = struct.unpack('>I', hdr[4:8])[0]
        actual = struct.unpack('>i', hdr[24:28])[0]
        payload = recv(actual)
        check('%s bulk in' % what, sq == pending and payload == b'world',
              'seq=%u payload=%r' % (sq, payload))

        # the notification endpoint never completes, so the host has to
        # be able to take its urb back
        seq[0] += 1
        intr = seq[0]
        sock.sendall(struct.pack('>IIIII', 1, intr, 0, 1, sitl_usbip.EP_INTR)
                     + struct.pack('>Iiiii8s', 0, 8, 0, 0, 0, b'\0' * 8))
        time.sleep(0.2)
        seq[0] += 1
        sock.sendall(struct.pack('>IIIII', 2, seq[0], 0, 0, 0)
                     + struct.pack('>I24s', intr, b''))
        hdr = recv(48)
        command = struct.unpack('>I', hdr[:4])[0]
        st = struct.unpack('>i', hdr[20:24])[0]
        check('%s unlink' % what, command == 4 and st != 0,
              'command=%u status=%d' % (command, st))
    finally:
        sock.close()
        server.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    # don't hardcode the firmware version in the default binary path
    pat = os.path.join(HERE, '..', '..', 'obj', 'AM32_AM32_SITL_CAN_*.elf')
    hits = sorted(glob.glob(pat))
    ap.add_argument('--sitl', default=os.path.normpath(hits[0]) if hits else None)
    ap.add_argument('--bootloader', default=None,
                    help='bootloader SITL elf from the am32-bootloader repo, '
                         'enables the 4-way passthrough test')
    args = ap.parse_args()

    if not os.path.exists(args.sitl):
        print('SITL binary not found: %s' % args.sitl)
        sys.exit(2)

    test_dshot(args.sitl, 'dshot600 bidir edt', sd.TYPE_DSHOT600,
               bidir=True, edt=True, value=800, rpm_lo=2000, rpm_hi=4000)
    test_dshot(args.sitl, 'dshot300', sd.TYPE_DSHOT300,
               bidir=False, edt=False, value=600, rpm_lo=2000, rpm_hi=4500)
    test_dshot(args.sitl, 'pwm', sd.TYPE_PWM,
               bidir=False, edt=False, value=1500, rpm_lo=3000, rpm_hi=7000,
               input_type=2)
    test_startup_tune(args.sitl)
    test_beacon_tone(args.sitl)
    test_physics_audio(args.sitl)
    test_dronecan(args.sitl)
    test_dshot_direction(args.sitl)
    test_bidirectional(args.sitl)
    test_dshot_edt_toggle(args.sitl)
    test_stuck_rotor(args.sitl, protection=True)
    test_stuck_rotor(args.sitl, protection=False)
    test_debugger_pause(args.sitl)
    test_dronecan_params(args.sitl)
    test_dataset_params()
    test_fc_capture(args.sitl)
    test_fc_fourway(args.sitl, args.bootloader)
    test_fc_reconnect()
    test_usbip_device(unix=False)
    test_usbip_device(unix=True)

    if failures:
        print('\n%d FAILED: %s' % (len(failures), ', '.join(failures)))
        sys.exit(1)
    print('\nall tests passed')


if __name__ == '__main__':
    main()
