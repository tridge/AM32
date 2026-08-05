#!/usr/bin/env python3
'''
Run the SITL test scripts against REAL ESC firmware emulated in Renode:

    run_test.py TARGET test.scr [more.scr ...]

The script language, outputs (timeline, change log, CSV, PNG, HTML
report) and almost all of the machinery are Mcu/SITL/run_test.py's -
this wraps its Runner around an emulated MCU running the target's own
firmware ELF, so the same test exercises the real register-level code.

What differs from the SITL, and why a script may need care:

- firmware variables resolve against the target ELF with nm. Sizes
  come from the symbol table, so name(u16) overrides are rarely
  needed. sitl_tone_active is SITL-only: when the ELF lacks a name on
  the known-SITL-only list it reads as constant 0, so the idiomatic
  "wait sitl_tone_active == 0" passes immediately (the arming beeps
  cost virtual time here, covered by waiting for armed == 1).
- eepromBuffer writes land in the emulated settings flash AND the
  firmware's in-RAM eepromBuffer, which is what a runtime DroneCAN
  parameter write does - the change is live with no reboot. Derived
  values that only loadEEpromSettings() computes at boot still need a
  scripted reset, exactly as they would on the bench.
- reset is a machine reset. The virtual clock keeps running, so the
  report's time axis is continuous with no epoch stitching.
- speedup <1 paces the emulation to that fraction of the wall clock;
  0 or >=1 free-runs (an emulated MCU cannot exceed real time).

The throttle protocol, physics stream, eeprom access and reset all ride
the same UDP wire protocols the SITL serves, provided in the emulator
by AM32_GuiLink; the firmware variable watch uses the same packets with
addresses in place of names (state cmd 8).
'''

import argparse
import collections
import importlib.util
import json
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SITL_DIR = os.path.join(REPO, 'Mcu', 'SITL')
sys.path.insert(0, SITL_DIR)
sys.path.insert(0, HERE)

import gen_target
import run_renode_tests as harness
import sitl_params

# the SITL runner, under a name that cannot shadow this file
_spec = importlib.util.spec_from_file_location(
    'sitl_run_test', os.path.join(SITL_DIR, 'run_test.py'))
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)

TestError = core.TestError

# SITL-only exports: real firmware has no such symbol, and a script
# waiting on one means "wait out the beeps", which reads as constant 0
# here (see the module docstring). Registered as pseudo physics
# channels so parsing accepts them; RenodeRunner.series() intercepts.
SITL_ONLY_ZERO = ('sitl_tone_active',)
for _name in SITL_ONLY_ZERO:
    core.PHYS_CHANNELS[_name] = None


def nm_symbols(elf, nm='arm-none-eabi-nm'):
    '''name -> (address, size) for the data symbols of the firmware ELF'''
    out = subprocess.check_output([nm, '-S', elf], text=True)
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        # only lines with a size field: "addr size type name"
        if len(parts) == 4:
            try:
                syms[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
            except ValueError:
                continue
    return syms


class RenodeWatchStream(core.WatchStream):
    '''the SITL watch client, subscribing with (size, address) entries:
    the emulator has no symbol table, names are resolved here from the
    ELF. Reply and data packets are identical to the SITL's'''

    def __init__(self, host, port, variables, addresses,
                 min_period_ns=1000000):
        pkt = struct.pack('<HBBI', self.MAGIC_CMD, 8, len(variables),
                          min_period_ns)
        for (name, size, _signed, _ffmt), addr in zip(variables, addresses):
            pkt += struct.pack('<BI', size, addr)
        # skip the base constructor's name packing but keep its plumbing
        self.addr = (host, port)
        self.variables = variables
        self.series = [[] for _ in variables]
        self.resolved = None
        self.lock = threading.Lock()
        self.running = True
        self.pkt = pkt
        import socket as _socket
        self.sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._subscriber, daemon=True).start()


class RenodeRunner(core.Runner):
    '''core.Runner with the SITL process swapped for a Renode machine'''

    def __init__(self, args, stmts, eeprom_fields, syms):
        self.syms = syms
        self.const_zero = set()
        for st in stmts:
            for ref in getattr(st, 'refs', []) + \
                    ([st.ref] if st.kind == 'wait' else []):
                if ref.name in SITL_ONLY_ZERO:
                    self.const_zero.add(ref.name)
        # the base resolves sizes through elf_symbol_sizes on args.sitl;
        # feed it the ARM ELF's table instead (the base parser is
        # 64-bit-ELF-only)
        core.elf_symbol_sizes = \
            lambda path: dict((n, s) for n, (_a, s) in syms.items())
        super().__init__(args, stmts, eeprom_fields)

    # ---- variable access ----

    def series(self, ref, since=None):
        if ref.name in self.const_zero:
            return [(since if since is not None else 0.0, 0.0)]
        return super().series(ref, since)

    # ---- the emulated ESC ----

    def run(self):
        args = self.args
        ptype_name = 'DSHOT600'
        bidir = False
        for st in self.stmts:
            if st.kind == 'throttle_type':
                ptype_name, bidir = st.ptype, st.bidir
                break
        ptype, input_type = core.THROTTLE_TYPES[ptype_name]
        self.input_type = input_type
        self.githash = core.git_hash()

        # the eeprom the firmware boots from: input auto-detect (the
        # generator's frames are real wire signals, the firmware detects
        # them as hardware would) plus the motor model's KV and poles,
        # without which the firmware is tuned for a different machine
        overrides = {'INPUT_SIGNAL_TYPE': 0}
        try:
            motor = json.load(open(args.model)).get('motor', {})
        except (OSError, ValueError):
            motor = {}
        for name, (want, _help) in sitl_params.model_checks(motor).items():
            overrides[name] = want
        self.eeprom_overrides = overrides
        eeprom = os.path.join(args.outdir, self.base + '_eeprom.bin')
        with open(eeprom, 'wb') as f:
            f.write(bytes(sitl_params.build_image(overrides)))

        renode = harness.find_renode(args.renode)
        if renode is None:
            raise TestError('renode not installed')
        so = harness.build_library()
        scratch = tempfile.mkdtemp(prefix='renode_test_')
        try:
            target_resc, _ = gen_target.generate(args.target, scratch,
                                                 args.gcc)
        except gen_target.Unsupported as ex:
            raise TestError(str(ex))

        proc = harness.start_renode(
            renode, target_resc, args.sitl, eeprom, os.path.abspath(args.model),
            so, scratch, args.link_port, args.link_state_port, args.dshot_us,
            args.gcc, args.nm)
        tail = collections.deque(maxlen=40)
        threading.Thread(target=harness.drain, args=(proc, tail),
                         daemon=True).start()

        self.sim = core.SimStream('127.0.0.1', args.link_state_port,
                                  period_us=args.sample_us, maxlen=4000000)
        self.sim.enabled = True
        addresses = [self.syms[name][0] for name, _s, _sg, _f
                     in self.watch_vars]
        self.watch = RenodeWatchStream('127.0.0.1', args.link_state_port,
                                       self.watch_vars, addresses,
                                       min_period_ns=args.watch_period_us
                                       * 1000)
        try:
            # the emulated firmware has to boot and the bridge start
            # before samples flow
            deadline = time.time() + 60
            while time.time() < deadline and not self.sim.samples:
                time.sleep(0.05)
            if not self.sim.samples:
                raise TestError('no state stream from the emulator; '
                                'log tail:\n' + '\n'.join(tail))
            if self.watch_vars:
                resolved = self.watch.wait_resolved()
                if resolved is None:
                    raise TestError('the emulator never answered the '
                                    'variable watch request')
                if not all(resolved):
                    raise TestError('the emulator rejected a watch entry')
            self.sender = core.Sender(ptype, bidir=bidir)
            self.log('start: %s on %s, input %s%s%s'
                     % (os.path.basename(args.sitl), args.target,
                        ptype_name, ' bidir' if bidir else '',
                        (', git %s' % self.githash) if self.githash else ''))
            for st in self.stmts:
                self.execute(st)
        finally:
            if self.sender:
                self.sender.stop()
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            self.t_end = self.sim_now() or 0.0
            self.sim.close()
            self.watch.close()

    # ---- statement handling ----

    def execute(self, st):
        if st.kind == 'eepromdefaults':
            # like the base, but keeping the motor model's KV and poles:
            # losing those would silently mistune the firmware
            img = sitl_params.build_image(self.eeprom_overrides)
            ok, msg = core.EepromClient('127.0.0.1',
                                        self.args.link_state_port) \
                .set(0, bytes(img))
            self.log('eepromdefaults: %s' % msg, None if ok else 'FAIL')
            return
        super().execute(st)

    def do_reset(self):
        # a machine reset inside the emulator: the virtual clock keeps
        # running, so no epoch freezing or offsetting is needed - the
        # firmware reboots and the time axis stays continuous by itself
        pkt = struct.pack('<HBB', 0x5353, 9, 0)
        self.watch.sock.sendto(pkt,
                               ('127.0.0.1', self.args.link_state_port))
        self.sim_sleep(0.05)
        self.log('reset (machine reset; the clock is continuous)')


def run_one(args, script, outdir, eeprom_fields, syms):
    '''run one test script against the emulated target'''
    stmts = core.parse_script(script, eeprom_fields)
    base = os.path.splitext(os.path.basename(script))[0]
    os.makedirs(outdir, exist_ok=True)

    run_args = argparse.Namespace(**vars(args))
    run_args.script, run_args.outdir = script, outdir
    runner = RenodeRunner(run_args, stmts, eeprom_fields, syms)
    runner.base = base
    runner.run()

    report = os.path.join(outdir, base + '_report.txt')
    csvf = os.path.join(outdir, base + '_vars.csv')
    png = os.path.join(outdir, base + '.png')
    html = os.path.join(outdir, base + '.html')
    runner.write_report(report, script)
    runner.write_csv(csvf)
    runner.write_html(html, script)
    have_png = False
    if runner.graph_refs:
        try:
            runner.write_graph(png, '%s on %s' % (os.path.basename(script),
                                                  args.target))
            have_png = True
        except ImportError:
            print('(no matplotlib, skipping the PNG; the HTML report '
                  'has the interactive graph)')
    print('\nreport: %s' % report)
    print('html:   file://%s' % os.path.abspath(html))
    print('vars:   %s' % csvf)
    if have_png:
        print('graph:  %s' % png)
    print('FAIL (%d)' % len(runner.failures) if runner.failures else 'PASS')
    return len(runner.failures)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('target', help='ESC target name, as gen_target.py '
                                   '--list shows')
    ap.add_argument('scripts', nargs='+', metavar='script.scr')
    ap.add_argument('--elf', help='firmware ELF (default: newest build '
                                  'for the target in obj/)')
    ap.add_argument('--outdir', default=None,
                    help='where the reports, graphs and CSVs go '
                         '(default test_outputs/<target>/<test name>/)')
    ap.add_argument('--model', default=os.path.join(
        SITL_DIR, 'models', 'vimdrones_nano_2216.json'))
    ap.add_argument('--renode', help='renode binary')
    ap.add_argument('--sample-us', type=int, default=500,
                    help='physics stream sample period')
    ap.add_argument('--watch-period-us', type=int, default=1000,
                    help='per variable change coalescing interval')
    ap.add_argument('--dshot-us', type=int, default=250,
                    help='dshot frame period on the emulated wire')
    ap.add_argument('--link-port', type=int, default=57853)
    ap.add_argument('--link-state-port', type=int, default=57854)
    ap.add_argument('--gcc', default='arm-none-eabi-gcc')
    ap.add_argument('--nm', default='arm-none-eabi-nm')
    args = ap.parse_args()

    elf = args.elf or gen_target.find_elf(args.target)
    if elf is None:
        print('no firmware in obj/ for %s; building it' % args.target)
        r = subprocess.run(['make', '-C', REPO, '-j8', args.target])
        if r.returncode != 0:
            sys.exit('error: make %s failed' % args.target)
        elf = gen_target.find_elf(args.target)
    if elf is None or not os.path.exists(elf):
        sys.exit('error: no firmware ELF for %s' % args.target)
    # the base runner calls the ELF args.sitl; here it is the firmware
    args.sitl = os.path.abspath(elf)
    try:
        syms = nm_symbols(args.sitl, args.nm)
    except (OSError, subprocess.CalledProcessError) as ex:
        sys.exit('error: cannot read symbols from %s: %s' % (elf, ex))

    # the throttle sender reads these module globals
    core.INPUT_PORT = args.link_port
    core.STATE_PORT = args.link_state_port

    eeprom_fields = core.parse_eeprom_layout(
        os.path.join(REPO, 'Inc', 'eeprom.h'))

    results = {}
    for script in args.scripts:
        base = os.path.splitext(os.path.basename(script))[0]
        if len(args.scripts) > 1:
            print('\n=== %s ===' % script)
        outdir = os.path.join(args.outdir, base) if args.outdir \
            and len(args.scripts) > 1 else \
            (args.outdir or os.path.join('test_outputs',
                                         args.target, base))
        try:
            results[script] = run_one(args, script, outdir,
                                      eeprom_fields, syms)
        except TestError as ex:
            print('error: %s' % ex, file=sys.stderr)
            results[script] = -1
        except OSError as ex:
            print('error: %s: %s' % (script, ex), file=sys.stderr)
            results[script] = -1

    if len(results) > 1:
        print('\n--- summary ---')
        for script, n in results.items():
            print('%-50s %s' % (script, 'PASS' if n == 0 else
                                'ERROR' if n < 0 else 'FAIL (%d)' % n))
    if any(n != 0 for n in results.values()):
        sys.exit(1)


if __name__ == '__main__':
    main()
