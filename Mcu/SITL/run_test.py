#!/usr/bin/env python3
'''
Scripted SITL test runner: executes a plain-language test script
against the simulator and produces an analysis report - a timeline of
when each scripted event happened (in simulated time), a change log of
every watched firmware variable, a CSV of those changes, a PNG graph
and a self-contained interactive HTML report (open the printed file://
URL: cursor readout of every variable, drag/wheel zoom, the timeline
and change log alongside the graph). The point is turning "the brake
mode misbehaves" into a picture plus "zero_throttle_brake_active went
1 at t=6.2031".

usage: run_test.py test1.scr [test2.scr ...] [--sitl elf] [--outdir d]
each test runs in sequence; outputs go to test_outputs/<test name>/
unless --outdir says otherwise

Script language, one statement per line, '#' comments:

  throttle type PWM|DSHOT150|DSHOT300|DSHOT600 [bidir]
                                 input protocol (default DSHOT600)
  throttle <0..1> [ramp=<s>]     throttle fraction; PWM maps to
                                 1000+1000*f us, dshot to 48..2047, 0 is
                                 off. ramp=0.3 slews there linearly over
                                 0.3s of simulated time
  eepromBuffer.<field> = <int>   set an ESC setting; fields (and nested
                                 ones like servo.dead_band or tune[3])
                                 come from Inc/eeprom.h, the firmware
                                 reloads settings on write
  eepromdefaults                 reset every setting to the defaults
                                 (the input protocol is kept)
  reset                          reset the ESC, like a power cycle:
                                 saved eeprom changes survive, the
                                 report's time axis stays continuous
  graph <expr[:N]> ...           variables to plot; ':2', ':3', ...
                                 put a series on its own extra Y axis
                                 (drawn on the right). An expr can clamp
                                 for display: MIN(e_rpm,1000) caps at
                                 1000, MAX(x,0) floors, and they nest:
                                 MIN(MAX(x,0),1000)
  watch <var> ...                log a variable's changes without plotting
  wait <var> <op> <value> [timeout=<s>]
                                 wait (sim time, default timeout 5s) for
                                 ==, !=, <, <=, > or >=; a timeout is a
                                 FAIL but the script continues so the
                                 report still shows what happened
  delay <s>                      let the simulation run for s seconds
  note <text>                    timeline/graph annotation
  graphtag <text>                prominent vertical text tag on the
                                 graph at the current time, e.g.
                                 graphtag brake_on_zero=3 comp_pwm=1
                                 to label a test section at a glance
  speedup <x>                    simulation speedup (0 = free run)
  model <path.json>              load a motor model

A <var> is a firmware global (resolved by name in the SITL via the
state-port variable watch, any global works: armed, e_rpm, newinput,
duty_cycle...) or one of the physics stream channels: rpm, omega,
theta, theta_e, iu, iv, iw, vu, vv, vw, vbus, ibus, comp_out.
Firmware variables default to unsigned, with the width taken from the
ELF symbol table; append a type for signed or float ones, e.g.
actual_current(i16).

Limitations: the throttle sender and ramps run on the wall clock, so
under `speedup` far from 1 (or 0, free-run) ramps coarsen and the
500Hz frame rate shrinks in simulated terms; a value pulse shorter
than --watch-period-us can escape the change log and waits; firmware
variable watching needs an ELF with dlsym (not the Windows build);
up to 16 firmware variables per run.

The SITL also exports sitl_tone_active: 1 while any beep plays. The
tunes run to completion inside an interrupt handler, so throttle input
is deaf until they end - start a test with

  throttle 0.0
  wait armed == 1 timeout=3
  wait sitl_tone_active == 0 timeout=2

and the ESC is armed, silent and listening.
'''

import argparse
import difflib
import glob
import os
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sitl_dshot as sd
import sitl_params
from sitl_gui_backend import EepromClient, SimStream


class TestError(Exception):
    '''a user-facing problem (script syntax, unknown variable or
    field, missing binary): reported as a message, not a traceback'''


def git_hash():
    '''short hash of the tree run_test lives in, with a trailing +
    when tracked files are modified'''
    try:
        r = subprocess.run(['git', '-C', HERE, 'rev-parse', '--short',
                            'HEAD'], capture_output=True, text=True,
                           timeout=10)
        if r.returncode != 0:
            return ''
        d = subprocess.run(['git', '-C', HERE, 'status', '--porcelain',
                            '-uno'], capture_output=True, text=True,
                           timeout=10)
        dirty = d.returncode == 0 and d.stdout.strip() != ''
        return r.stdout.strip() + ('+' if dirty else '')
    except OSError:
        return ''

INPUT_PORT = 57843
STATE_PORT = 57844

# physics stream channels by sample index (SimStream tuples); rpm is
# derived from omega
PHYS_CHANNELS = {
    'omega': 1, 'theta': 2, 'theta_e': 3,
    'iu': 4, 'iv': 5, 'iw': 6, 'vu': 7, 'vv': 8, 'vw': 9,
    'vbus': 10, 'ibus': 11, 'comp_phase': 13, 'comp_out': 14,
    'rpm': None,
}

TYPES = {
    'u8': (1, False, None), 'i8': (1, True, None),
    'u16': (2, False, None), 'i16': (2, True, None),
    'u32': (4, False, None), 'i32': (4, True, None),
    'u64': (8, False, None), 'i64': (8, True, None),
    'f32': (4, False, '<f'), 'f64': (8, False, '<d'),
}

THROTTLE_TYPES = {
    'PWM': (sd.TYPE_PWM, 2), 'DSHOT150': (sd.TYPE_DSHOT150, 1),
    'DSHOT300': (sd.TYPE_DSHOT300, 1), 'DSHOT600': (sd.TYPE_DSHOT600, 1),
}

# matplotlib's default cycle, shared with the HTML report so the PNG
# and the interactive graph use the same colours per series
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']


def decimate(pts, maxn):
    '''bucketed min/max decimation: keeps spikes a stride would drop'''
    if len(pts) <= maxn:
        return pts
    nb = max(1, maxn // 2)
    n = len(pts)
    out = []
    for b in range(nb):
        seg = pts[b * n // nb:(b + 1) * n // nb]
        if not seg:
            continue
        lo = min(seg, key=lambda p: p[1])
        hi = max(seg, key=lambda p: p[1])
        pair = sorted({lo, hi}, key=lambda p: p[0])
        out += pair
    return out


def elf_symbol_sizes(path):
    '''name -> st_size from a 64-bit little-endian ELF symbol table.
    Only sizes are wanted (addresses are resolved in-process by the
    SITL), so PIE vs non-PIE does not matter here'''
    syms = {}
    with open(path, 'rb') as f:
        data = f.read()
    if data[:4] != b'\x7fELF' or data[4] != 2 or data[5] != 1:
        return syms
    e_shoff, = struct.unpack_from('<Q', data, 0x28)
    e_shentsize, e_shnum = struct.unpack_from('<HH', data, 0x3a)
    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        sh_type, = struct.unpack_from('<I', data, off + 4)
        sh_offset, sh_size = struct.unpack_from('<QQ', data, off + 24)
        sh_link, = struct.unpack_from('<I', data, off + 40)
        sections.append((sh_type, sh_offset, sh_size, sh_link))
    for sh_type, sh_offset, sh_size, sh_link in sections:
        if sh_type != 2:  # SHT_SYMTAB
            continue
        str_off, str_size = sections[sh_link][1:3]
        strtab = data[str_off:str_off + str_size]
        for off in range(sh_offset, sh_offset + sh_size, 24):
            st_name, = struct.unpack_from('<I', data, off)
            st_size, = struct.unpack_from('<Q', data, off + 16)
            if st_name == 0:
                continue
            name = strtab[st_name:strtab.index(b'\0', st_name)]
            syms[name.decode(errors='replace')] = st_size
    return syms


def parse_eeprom_layout(header_path):
    '''field path -> (offset, size) from the EEprom_u struct in
    Inc/eeprom.h, so a script can name any setting the firmware can.
    Nested struct members (servo.dead_band, can.telem_rate) are
    buffered until the closing brace names the struct; every array
    element is addressable as name[i]'''
    sizes = {'uint8_t': 1, 'int8_t': 1, 'char': 1, 'uint16_t': 2,
             'uint32_t': 4}
    fields = {}
    offset = 0
    pending = None  # members of an open nested struct
    started = False
    in_outer = False
    for line in open(header_path):
        line = line.split('//')[0].strip()
        if line.startswith('typedef union'):
            started = True
            continue
        if not started:
            continue
        if line.startswith('struct {'):
            # the first struct is the union's own layout struct, not a
            # nested member struct
            if not in_outer:
                in_outer = True
            else:
                pending = []
            continue
        if line.startswith('}') and pending is not None:
            prefix = line.strip('} ;')
            for name, off, size in pending:
                fields['%s.%s' % (prefix, name)] = (off, size)
            pending = None
            continue
        if line.startswith('uint8_t buffer['):
            break
        parts = line.rstrip(';').split()
        if len(parts) != 2 or parts[0] not in sizes:
            continue
        size = sizes[parts[0]]
        name, count = parts[1], 1
        if '[' in name:
            name, count = name.split('[')
            count = int(count.rstrip(']'))
        for i in range(count):
            key = '%s[%d]' % (name, i) if count > 1 else name
            if pending is not None:
                pending.append((key, offset, size))
            else:
                fields[key] = (offset, size)
            offset += size
    return fields


class WatchStream(object):
    '''subscriber for the SITL variable watch (state port cmd 8):
    change events for named firmware globals, timestamped in simulated
    time. Keeps the full series per variable'''

    MAGIC_CMD = 0x5353
    MAGIC_REPLY = 0x5359
    MAGIC_DATA = 0x535a
    EVENT = struct.Struct('<QIQ')

    def __init__(self, host, port, variables, min_period_ns=1000000):
        '''variables: list of (name, size, signed, float_fmt)'''
        self.addr = (host, port)
        self.variables = variables
        self.series = [[] for _ in variables]
        self.resolved = None
        self.lock = threading.Lock()
        self.running = True
        pkt = struct.pack('<HBBI', self.MAGIC_CMD, 8, len(variables),
                          min_period_ns)
        for name, size, _signed, _ffmt in variables:
            pkt += struct.pack('<B', size) + name.encode() + b'\0'
        if len(pkt) > 500:
            raise ValueError('watch list too long for one packet')
        self.pkt = pkt
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._subscriber, daemon=True).start()

    def _subscriber(self):
        while self.running:
            try:
                self.sock.sendto(self.pkt, self.addr)
            except OSError:
                pass
            time.sleep(0.5)

    def _convert(self, raw, var):
        _name, size, signed, ffmt = var
        if ffmt is not None:
            return struct.unpack(ffmt, struct.pack('<Q', raw)[:size])[0]
        if signed and raw >= 1 << (size * 8 - 1):
            raw -= 1 << (size * 8)
        return raw

    def _reader(self):
        while self.running:
            try:
                d = self.sock.recv(4096)
            except OSError:
                if not self.running:
                    return
                continue
            if len(d) < 4:
                continue
            magic, _b2, count = struct.unpack('<HBB', d[:4])
            if magic == self.MAGIC_REPLY:
                self.resolved = list(d[4:4 + count])
                continue
            if magic != self.MAGIC_DATA:
                continue
            with self.lock:
                for k in range(count):
                    off = 4 + k * self.EVENT.size
                    if off + self.EVENT.size > len(d):
                        break
                    t_ns, idx, raw = self.EVENT.unpack_from(d, off)
                    if idx >= len(self.variables):
                        continue
                    val = self._convert(raw, self.variables[idx])
                    self.series[idx].append((t_ns * 1e-9, val))

    def wait_resolved(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.resolved is not None:
                return self.resolved
            time.sleep(0.05)
        return None

    def get(self, idx):
        with self.lock:
            return list(self.series[idx])

    def close(self):
        self.running = False
        self.sock.close()


class Sender(object):
    '''background input frame sender at 500Hz, like the CI harness'''

    def __init__(self, ptype, bidir=False):
        self.port = sd.InputPort('127.0.0.1', INPUT_PORT)
        self.ptype = ptype
        self.bidir = bidir
        self.value = 1000 if ptype == sd.TYPE_PWM else 0
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        nxt = time.time()
        while self.running:
            now = time.time()
            burst = 0
            while now >= nxt and burst < 10:
                nxt += 1.0 / 500
                try:
                    if self.ptype == sd.TYPE_PWM:
                        self.port.send_pwm(int(self.value))
                    else:
                        self.port.send_dshot(int(self.value),
                                             ptype=self.ptype,
                                             bidir=self.bidir)
                except OSError:
                    # transient (e.g. ICMP unreachable while the ESC
                    # re-execs through a reset): keep sending, only a
                    # stop() ends the stream
                    if not self.running:
                        return
                burst += 1
            if now - nxt > 0.25:
                nxt = now
            time.sleep(0.0005)

    def stop(self):
        self.running = False
        self.port.close()


def parse_var_expr(s):
    '''a variable expression: NAME, NAME(type), or MIN(expr,const) /
    MAX(expr,const) which clamp the plotted values - MIN caps, MAX
    floors, and they nest: MIN(MAX(x,0),1000). Returns
    (name, type_override, transforms)'''
    s = s.strip()
    for fn in ('MIN', 'MAX'):
        if s.startswith(fn + '(') and s.endswith(')'):
            inner = s[len(fn) + 1:-1]
            depth, split = 0, -1
            for i, ch in enumerate(inner):
                if ch == '(':
                    depth += 1
                elif ch == ')':
                    depth -= 1
                elif ch == ',' and depth == 0:
                    split = i
            if split < 0:
                raise ValueError('%s() takes two arguments: %s' % (fn, s))
            name, t, transforms = parse_var_expr(inner[:split])
            const = float(inner[split + 1:])
            return name, t, transforms + [(fn.lower(), const)]
    t = None
    if s.endswith(')') and '(' in s:
        s, t = s[:-1].split('(', 1)
        if t not in TYPES:
            raise ValueError('unknown type %r (known: %s)'
                             % (t, ' '.join(sorted(TYPES))))
    return s, t, []


class VarRef(object):
    '''a variable reference in the script: an expression (see
    parse_var_expr) with an optional :axis suffix on graph lines'''

    def __init__(self, token, allow_axis=False):
        self.axis = 1
        if allow_axis and ':' in token:
            token, axis = token.rsplit(':', 1)
            self.axis = int(axis)
            if self.axis < 1:
                raise ValueError('axis must be 1 or higher: %s' % token)
        self.label = token
        self.name, self.type_override, self.transforms = \
            parse_var_expr(token)
        self.is_physics = self.name in PHYS_CHANNELS

    def key(self):
        return self.name


OPS = {
    '==': lambda a, b: a == b, '!=': lambda a, b: a != b,
    '<': lambda a, b: a < b, '<=': lambda a, b: a <= b,
    '>': lambda a, b: a > b, '>=': lambda a, b: a >= b,
}


class Statement(object):
    def __init__(self, lineno, line, kind, **kw):
        self.lineno = lineno
        self.line = line
        self.kind = kind
        self.__dict__.update(kw)


def parse_script(path, eeprom_fields):
    stmts = []
    for lineno, raw in enumerate(open(path), 1):
        line = raw.split('#')[0].strip()
        if not line:
            continue
        toks = line.split()

        def err(msg):
            raise TestError('%s:%d: %s\n    %s' % (path, lineno, msg, line))

        try:
            if toks[0] == 'throttle' and toks[1] == 'type':
                if toks[2].upper() not in THROTTLE_TYPES:
                    err('unknown throttle type %r (known: %s)'
                        % (toks[2], ' '.join(THROTTLE_TYPES)))
                stmts.append(Statement(lineno, line, 'throttle_type',
                                       ptype=toks[2].upper(),
                                       bidir='bidir' in toks[3:]))
            elif toks[0] == 'throttle':
                ramp = 0.0
                for t in toks[2:]:
                    if t.startswith('ramp='):
                        ramp = float(t[len('ramp='):])
                    else:
                        err('unexpected %r (only ramp=<s> goes after '
                            'the throttle fraction)' % t)
                stmts.append(Statement(lineno, line, 'throttle',
                                       fraction=float(toks[1]), ramp=ramp))
            elif toks[0].startswith('eepromBuffer.'):
                field = toks[0][len('eepromBuffer.'):]
                if field not in eeprom_fields:
                    close = difflib.get_close_matches(field, eeprom_fields,
                                                      n=3, cutoff=0.5)
                    close += [f for f in eeprom_fields
                              if field in f and f not in close]
                    err('unknown eeprom field %r%s\n'
                        '  (fields come from the EEprom_u struct in '
                        'Inc/eeprom.h of this tree)'
                        % (field, (', did you mean: %s' % ', '.join(close))
                           if close else ''))
                if len(toks) < 3 or toks[1] != '=':
                    err('expected eepromBuffer.%s = <value>' % field)
                value = int(toks[2], 0)
                size = eeprom_fields[field][1]
                if not 0 <= value < 256 ** size:
                    err('%d does not fit the %d byte field %s (0..%d)'
                        % (value, size, field, 256 ** size - 1))
                stmts.append(Statement(lineno, line, 'eeprom', field=field,
                                       value=value))
            elif toks[0] == 'eepromdefaults':
                stmts.append(Statement(lineno, line, 'eepromdefaults'))
            elif toks[0] == 'reset':
                stmts.append(Statement(lineno, line, 'reset'))
            elif toks[0] == 'graph':
                refs = [VarRef(t, allow_axis=True) for t in toks[1:]]
                stmts.append(Statement(lineno, line, 'graph', refs=refs))
            elif toks[0] == 'watch':
                refs = [VarRef(t) for t in toks[1:]]
                stmts.append(Statement(lineno, line, 'watch', refs=refs))
            elif toks[0] == 'wait':
                timeout = 5.0
                cond = toks[1:]
                if cond and cond[-1].startswith('timeout='):
                    timeout = float(cond.pop()[len('timeout='):])
                if len(cond) != 3 or cond[1] not in OPS:
                    err('expected: wait <var> <op> <value> [timeout=<s>]')
                stmts.append(Statement(lineno, line, 'wait',
                                       ref=VarRef(cond[0]), op=cond[1],
                                       value=float(cond[2]),
                                       timeout=timeout))
            elif toks[0] == 'delay':
                stmts.append(Statement(lineno, line, 'delay',
                                       seconds=float(toks[1])))
            elif toks[0] == 'note':
                stmts.append(Statement(lineno, line, 'note',
                                       text=line[len('note'):].strip()))
            elif toks[0] == 'graphtag':
                stmts.append(Statement(lineno, line, 'graphtag',
                                       text=line[len('graphtag'):].strip()))
            elif toks[0] == 'speedup':
                stmts.append(Statement(lineno, line, 'speedup',
                                       factor=float(toks[1])))
            elif toks[0] == 'model':
                stmts.append(Statement(lineno, line, 'model', path=toks[1]))
            else:
                known = ('throttle', 'eepromBuffer.<field> = <v>',
                         'eepromdefaults', 'reset', 'graph', 'watch',
                         'wait', 'delay', 'note', 'graphtag', 'speedup',
                         'model')
                err('unknown statement %r (known statements: %s)'
                    % (toks[0], ', '.join(known)))
        except TestError:
            raise
        except (IndexError, ValueError) as ex:
            err(str(ex) or 'malformed statement')
    return stmts


class Runner(object):
    def __init__(self, args, stmts, eeprom_fields):
        self.args = args
        self.stmts = stmts
        self.eeprom_fields = eeprom_fields
        self.timeline = []       # (t, text, status) status None/PASS/FAIL
        self.gtags = []          # (t, text) graphtag annotations
        self.failures = []
        self.sender = None
        self.graph_refs = []
        self.watch_refs = []
        self.cur_fraction = 0.0
        # an ESC reset restarts simulated time at zero; the report's
        # time axis stays continuous by freezing the collected series
        # and offsetting the new epoch
        self.t_offset = 0.0
        self.frozen_phys = []
        self.frozen_watch = None  # sized once the watch list is known

        # every firmware variable the script mentions gets watched
        refs = {}
        for st in stmts:
            for ref in getattr(st, 'refs', []) + \
                    ([st.ref] if st.kind == 'wait' else []):
                if not ref.is_physics:
                    refs.setdefault(ref.key(), ref)
            if st.kind == 'graph':
                # a sweep style script repeats its graph line per
                # section; the same expression on the same axis is
                # still one series
                for ref in st.refs:
                    if not any(r.label == ref.label and r.axis == ref.axis
                               for r in self.graph_refs):
                        self.graph_refs.append(ref)
            if st.kind == 'watch':
                self.watch_refs += st.refs
        self.fw_refs = list(refs.values())

        syms = elf_symbol_sizes(args.sitl)
        variables = []
        for ref in self.fw_refs:
            if ref.type_override:
                size, signed, ffmt = TYPES[ref.type_override]
            elif ref.name in syms and syms[ref.name] in (1, 2, 4, 8):
                size, signed, ffmt = syms[ref.name], False, None
            elif not syms:
                raise TestError('cannot read the ELF symbol table of %s; '
                                'give %s an explicit type, e.g. %s(u16)'
                                % (args.sitl, ref.name, ref.name))
            else:
                close = difflib.get_close_matches(ref.name, syms,
                                                  n=3, cutoff=0.6)
                raise TestError(
                    'unknown variable %r: not a firmware global in %s%s\n'
                    '  and not a physics channel (%s)'
                    % (ref.name, os.path.basename(args.sitl),
                       (', did you mean: %s' % ', '.join(close))
                       if close else '',
                       ' '.join(sorted(PHYS_CHANNELS))))
            variables.append((ref.name, size, signed, ffmt))
        if len(variables) > 16:
            raise TestError('%d firmware variables to watch, the SITL '
                            'supports 16 per run - drop some graph/watch/'
                            'wait variables' % len(variables))
        self.watch_vars = variables
        self.frozen_watch = [[] for _ in variables]
        self.fw_index = dict((r.name, i)
                             for i, r in enumerate(self.fw_refs))

    # ---- simulated time helpers ----

    def sim_now(self):
        s = self.sim.latest()
        return self.t_offset + s[0] if s else None

    def sim_sleep(self, secs, wall_cap=None):
        wall_cap = wall_cap or max(30.0, secs * 5 + 20)
        t0 = None
        deadline = time.monotonic() + wall_cap
        while time.monotonic() < deadline:
            t = self.sim_now()
            if t is not None:
                if t0 is None or t < t0:
                    t0 = t
                elif t - t0 >= secs:
                    return
            time.sleep(0.005)

    # ---- variable access ----

    def phys_samples(self):
        '''frozen plus live physics samples, on the continuous clock'''
        with self.sim.lock:
            live = list(self.sim.samples)
        return self.frozen_phys + \
            [(self.t_offset + s[0],) + s[1:] for s in live]

    def watch_series(self, idx):
        '''frozen plus live change events, on the continuous clock'''
        return self.frozen_watch[idx] + \
            [(self.t_offset + t, v) for t, v in self.watch.get(idx)]

    def series(self, ref, since=None):
        '''[(t, value)] for a variable, optionally from sim time. A
        wait polls this every few ms, so the frozen pre-reset epochs
        (all earlier than t_offset) are skipped when the window starts
        after them, and the window is cut before any transform'''
        skip_frozen = since is not None and since >= self.t_offset
        if ref.is_physics:
            idx = PHYS_CHANNELS[ref.name]
            with self.sim.lock:
                live = list(self.sim.samples)
            smps = [(self.t_offset + s[0],) + s[1:] for s in live]
            if not skip_frozen:
                smps = self.frozen_phys + smps
            if since is not None:
                smps = [s for s in smps if s[0] >= since]
            if ref.name == 'rpm':
                out = [(s[0], s[1] * 60.0 / 6.283185) for s in smps]
            else:
                out = [(s[0], s[idx]) for s in smps]
        else:
            i = self.fw_index[ref.name]
            out = [(self.t_offset + t, v) for t, v in self.watch.get(i)]
            if not skip_frozen:
                out = self.frozen_watch[i] + out
            if since is not None:
                out = [p for p in out if p[0] >= since]
        for op, const in ref.transforms:
            f = min if op == 'min' else max
            out = [(t, f(v, const)) for t, v in out]
        return out

    def latest(self, ref):
        s = self.series(ref)
        return s[-1] if s else None

    # ---- execution ----

    def log(self, text, status=None, t=None):
        t = t if t is not None else (self.sim_now() or 0.0)
        self.timeline.append((t, text, status))
        marker = {'PASS': ' [ok]', 'FAIL': ' [FAIL]'}.get(status, '')
        print('t=%8.4f  %s%s' % (t, text, marker))
        sys.stdout.flush()
        if status == 'FAIL':
            self.failures.append(text)

    def run(self):
        args = self.args
        # the input protocol must be known at SITL start (--input-type
        # seeds the eeprom): find the first throttle type statement
        ptype_name = 'DSHOT600'
        bidir = False
        for st in self.stmts:
            if st.kind == 'throttle_type':
                ptype_name, bidir = st.ptype, st.bidir
                break
        ptype, input_type = THROTTLE_TYPES[ptype_name]
        self.input_type = input_type
        self.githash = git_hash()

        eeprom = os.path.join(args.outdir, self.base + '_eeprom.bin')
        for suffix in ('', '.bkup', '.lock'):
            if os.path.exists(eeprom + suffix):
                os.unlink(eeprom + suffix)
        sitl_log = os.path.join(args.outdir, self.base + '_sitl.log')
        cmd = [args.sitl, '--input-port', str(INPUT_PORT),
               '--state-port', str(STATE_PORT), '--can-uri', 'none',
               '--input-type', str(input_type), '--eeprom', eeprom]
        if not args.realtime:
            cmd.append('--nosleep')
        proc = subprocess.Popen(cmd, stdout=open(sitl_log, 'wb'),
                                stderr=subprocess.STDOUT)
        self.sim = SimStream('127.0.0.1', STATE_PORT,
                             period_us=args.sample_us, maxlen=4000000)
        self.sim.enabled = True
        self.watch = WatchStream('127.0.0.1', STATE_PORT, self.watch_vars,
                                 min_period_ns=args.watch_period_us * 1000)
        try:
            deadline = time.time() + 10
            while time.time() < deadline and not self.sim.samples:
                time.sleep(0.05)
            if not self.sim.samples:
                raise TestError('no state stream from the SITL; log tail:\n'
                                + open(sitl_log, errors='replace').read()[-500:])
            if self.watch_vars:
                resolved = self.watch.wait_resolved()
                if resolved is None:
                    raise TestError('the SITL never answered the variable '
                                    'watch request; is it an old binary?')
                if len(resolved) != len(self.watch_vars):
                    raise TestError('the SITL acknowledged %d of %d watch '
                                    'variables' % (len(resolved),
                                                   len(self.watch_vars)))
                for ok, (name, _s, _sg, _f) in zip(resolved, self.watch_vars):
                    if not ok:
                        raise TestError(
                            'the SITL cannot resolve %r - not a firmware '
                            'global in this build' % name)
            self.sender = Sender(ptype, bidir=bidir)
            self.log('start: %s, input %s%s%s'
                     % (os.path.basename(args.sitl), ptype_name,
                        ' bidir' if bidir else '',
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

    def execute(self, st):
        if st.kind == 'throttle_type':
            if self.sender.ptype != THROTTLE_TYPES[st.ptype][0]:
                self.log('throttle type %s: changing the protocol needs a '
                         'restart, put it before the first throttle'
                         % st.ptype, 'FAIL')
            return
        if st.kind == 'throttle':
            self.do_throttle(st)
        elif st.kind == 'eeprom':
            off, size = self.eeprom_fields[st.field]
            data = st.value.to_bytes(size, 'little')
            ok, msg = EepromClient('127.0.0.1', STATE_PORT).set(off, data)
            self.log('eepromBuffer.%s = %d (byte %d): %s'
                     % (st.field, st.value, off, msg),
                     None if ok else 'FAIL')
        elif st.kind == 'eepromdefaults':
            img = sitl_params.build_image(
                {'INPUT_SIGNAL_TYPE': self.input_type})
            ok, msg = EepromClient('127.0.0.1', STATE_PORT).set(0, img)
            self.log('eepromdefaults: %s' % msg, None if ok else 'FAIL')
        elif st.kind == 'reset':
            self.do_reset()
        elif st.kind in ('graph', 'watch'):
            self.log(st.line)
        elif st.kind == 'wait':
            self.do_wait(st)
        elif st.kind == 'delay':
            self.log('delay %g' % st.seconds)
            self.sim_sleep(st.seconds)
        elif st.kind == 'note':
            self.log('note: %s' % st.text)
        elif st.kind == 'graphtag':
            self.gtags.append((self.sim_now() or 0.0, st.text))
            self.log('graphtag: %s' % st.text)
        elif st.kind == 'speedup':
            self.sim.set_speedup(st.factor)
            self.log('speedup %g' % st.factor)
        elif st.kind == 'model':
            self.sim.load_model(os.path.abspath(st.path))
            self.log('model %s' % st.path)
            time.sleep(0.5)

    def set_fraction(self, f):
        if self.sender.ptype == sd.TYPE_PWM:
            self.sender.value = 1000 + round(1000 * f)
        else:
            self.sender.value = 0 if f <= 0 else \
                min(2047, 48 + round(f * 1999))
        self.cur_fraction = f

    def do_throttle(self, st):
        proto = 'pwm' if self.sender.ptype == sd.TYPE_PWM else 'dshot'
        if st.ramp > 0:
            f0, f1 = self.cur_fraction, st.fraction
            self.log('throttle %g -> %g ramp=%gs' % (f0, f1, st.ramp))
            t0 = self.sim_now() or 0.0
            wall_deadline = time.monotonic() + max(30.0, st.ramp * 5 + 20)
            while time.monotonic() < wall_deadline:
                dt = (self.sim_now() or t0) - t0
                if dt >= st.ramp:
                    break
                self.set_fraction(f0 + (f1 - f0) * dt / st.ramp)
                time.sleep(0.005)
        self.set_fraction(st.fraction)
        self.log('throttle %g (%s %d)'
                 % (st.fraction, proto, self.sender.value))

    @staticmethod
    def _split_at_regression(seq, key):
        """index of the first backward time jump, or len(seq): raw
        simulated time never decreases within one boot, so a regression
        is the reboot boundary"""
        for j in range(1, len(seq)):
            if key(seq[j]) < key(seq[j - 1]) - 0.5:
                return j
        return len(seq)

    def do_reset(self):
        pkt = struct.pack('<HBB', 0x5353, 9, 0)
        self.watch.sock.sendto(pkt, ('127.0.0.1', STATE_PORT))
        # a reboot is detected by its raw clock restarting: the newest
        # sample's time drops far below the highest seen. Outage-length
        # or cutoff heuristics are not needed and cannot misfire
        with self.sim.lock:
            high = self.sim.samples[-1][0] if self.sim.samples else 0.0
        deadline = time.monotonic() + 15
        next_kick = 0.0
        rebooted = False
        while time.monotonic() < deadline:
            s = self.sim.latest()
            if s is not None:
                if s[0] < high - 0.5:
                    rebooted = True
                    break
                high = max(high, s[0])
            # kick the watch subscription so the boot's early
            # transitions are captured sooner than the 0.5s keepalive
            if time.monotonic() >= next_kick:
                next_kick = time.monotonic() + 0.1
                try:
                    self.watch.sock.sendto(self.watch.pkt, self.watch.addr)
                except OSError:
                    pass
            time.sleep(0.02)
        if not rebooted:
            self.log('reset: no reboot observed within 15s (cmd lost?), '
                     'timestamps after this point are unreliable', 'FAIL')
            return
        # freeze the old epoch on the continuous clock, splitting each
        # stream exactly at its own regression point
        raw_mark = 0.0
        with self.watch.lock:
            for i in range(len(self.watch_vars)):
                seq = self.watch.series[i]
                j = self._split_at_regression(seq, lambda e: e[0])
                if j > 0:
                    raw_mark = max(raw_mark, seq[j - 1][0])
                self.frozen_watch[i] += [(self.t_offset + t, v)
                                         for t, v in seq[:j]]
                self.watch.series[i] = seq[j:]
        with self.sim.lock:
            seq = list(self.sim.samples)
            j = self._split_at_regression(seq, lambda e: e[0])
            if j > 0:
                raw_mark = max(raw_mark, seq[j - 1][0])
            self.frozen_phys += [(self.t_offset + e[0],) + e[1:]
                                 for e in seq[:j]]
            self.sim.samples.clear()
            self.sim.samples.extend(seq[j:])
        t_mark = self.t_offset + raw_mark
        self.t_offset = t_mark
        self.log('reset (sim clock continues from %.4f)' % t_mark)

    def do_wait(self, st):
        t0 = self.sim_now() or 0.0
        op = OPS[st.op]
        wall_deadline = time.monotonic() + max(30.0, st.timeout * 5 + 20)
        while True:
            # scan the recorded series so the satisfied time is the
            # crossing's own timestamp, not the polling instant
            for t, val in self.series(st.ref, since=t0):
                if op(val, st.value):
                    self.log('wait %s: satisfied after %.4fs (%s=%g)'
                             % (st.line[5:], t - t0, st.ref.label, val),
                             'PASS', t=t)
                    return
            # a firmware variable may already hold a satisfying value
            # from before t0, with no change event after it
            if not st.ref.is_physics:
                last = self.latest(st.ref)
                if last is not None and last[0] <= t0 \
                        and op(last[1], st.value):
                    self.log('wait %s: already true (%s=%g)'
                             % (st.line[5:], st.ref.label, last[1]),
                             'PASS', t=t0)
                    return
            now = self.sim_now() or t0
            if now - t0 > st.timeout or time.monotonic() > wall_deadline:
                last = self.latest(st.ref)
                self.log('wait %s: TIMEOUT after %.4fs (%s=%s)'
                         % (st.line[5:], now - t0, st.ref.label,
                            last[1] if last else 'no data'), 'FAIL', t=now)
                return
            time.sleep(0.005)

    # ---- outputs ----

    def change_log(self, cap=200):
        '''chronological change list of all watched firmware variables'''
        events = []
        suppressed = {}
        for i, ref in enumerate(self.fw_refs):
            series = self.watch_series(i)
            prev = None
            shown = 0
            for t, val in series:
                if prev is not None and shown >= cap:
                    suppressed[ref.name] = suppressed.get(ref.name, 0) + 1
                    prev = val
                    continue
                events.append((t, ref.name, prev, val))
                prev = val
                shown += 1
        events.sort(key=lambda e: e[0])
        return events, suppressed

    def write_report(self, path, script_path):
        lines = []
        lines.append('AM32 SITL test report')
        lines.append('script:  %s' % script_path)
        lines.append('sitl:    %s' % self.args.sitl)
        lines.append('git:     %s' % (self.githash or 'unknown'))
        lines.append('date:    %s' % time.strftime('%Y-%m-%d %H:%M:%S'))
        lines.append('result:  %s' % ('FAIL (%d)' % len(self.failures)
                                      if self.failures else 'PASS'))
        lines.append('')
        lines.append('--- script ---')
        lines += [l.rstrip() for l in open(script_path)]
        lines.append('')
        lines.append('--- timeline (simulated seconds) ---')
        for t, text, status in self.timeline:
            marker = {'PASS': ' [ok]', 'FAIL': ' [FAIL]'}.get(status, '')
            lines.append('t=%8.4f  %s%s' % (t, text, marker))
        lines.append('')
        events, suppressed = self.change_log()
        lines.append('--- variable changes (coalesced to %dus) ---'
                     % self.args.watch_period_us)
        for t, name, prev, val in events:
            if prev is None:
                lines.append('t=%8.4f  %s = %g (initial)' % (t, name, val))
            else:
                lines.append('t=%8.4f  %s: %g -> %g' % (t, name, prev, val))
        for name, n in sorted(suppressed.items()):
            lines.append('(%s: %d further changes suppressed, '
                         'see the CSV/graph)' % (name, n))
        if self.failures:
            lines.append('')
            lines.append('--- failures ---')
            lines += self.failures
        open(path, 'w').write('\n'.join(lines) + '\n')

    def marker_events(self):
        '''the timeline entries worth marking on a graph'''
        out = []
        for t, text, status in self.timeline:
            if not text.startswith(('throttle ', 'eepromBuffer.', 'note:',
                                    'wait ')):
                continue
            if text.startswith('wait ') and status is None:
                continue
            out.append((t, text, status))
        return out

    def write_html(self, path, script_path):
        '''self-contained interactive report: the timeline and change
        log next to a zoomable graph with a cursor value readout. No
        external resources, so it works from a file:// URL offline'''
        import json
        series = []
        for i, ref in enumerate(self.graph_refs):
            pts = self.series(ref)
            if not pts:
                continue
            step = not ref.is_physics
            if step:
                pts = pts + [(self.t_end, pts[-1][1])]
            else:
                pts = decimate(pts, 20000)
            series.append({
                'name': ref.label, 'axis': ref.axis, 'step': step,
                'color': COLORS[i % len(COLORS)],
                'points': [[round(t, 6), round(v, 5)] for t, v in pts],
            })
        events, suppressed = self.change_log(cap=1000)
        payload = {
            'title': os.path.basename(script_path),
            'script_path': script_path,
            'sitl': self.args.sitl,
            'git': self.githash,
            'date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'result': ('FAIL (%d)' % len(self.failures)
                       if self.failures else 'PASS'),
            'series': series,
            'markers': [{'t': t, 'text': text, 'status': status}
                        for t, text, status in self.marker_events()],
            'tags': [{'t': t, 'text': text} for t, text in self.gtags],
            'timeline': [{'t': t, 'text': text, 'status': status}
                         for t, text, status in self.timeline],
            'changes': [{'t': t, 'name': name, 'prev': prev, 'val': val}
                        for t, name, prev, val in events],
            'suppressed': suppressed,
            'script': open(script_path).read(),
        }
        import html as html_mod
        data = json.dumps(payload).replace('</', '<\\/')
        page = HTML_PAGE.replace('__TITLE__', html_mod.escape(
            os.path.basename(script_path)))
        page = page.replace('__DATA__', data)
        open(path, 'w').write(page)

    def write_csv(self, path):
        with open(path, 'w') as f:
            f.write('t,variable,value\n')
            for i, ref in enumerate(self.fw_refs):
                for t, val in self.watch_series(i):
                    f.write('%.6f,%s,%g\n' % (t, ref.name, val))

    def write_graph(self, path, title):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax1 = plt.subplots(figsize=(12, 6.5))
        axes = {1: ax1}
        axis_color = {}
        handles = []
        for i, ref in enumerate(self.graph_refs):
            # one shared colour sequence: a twin axis restarts the
            # matplotlib cycle, which would repeat colours
            color = COLORS[i % len(COLORS)]
            ax = axes.get(ref.axis)
            if ax is None:
                ax = ax1.twinx()
                # third and later axes get their spine pushed outwards
                n_right = sum(1 for a in axes if a != 1)
                if n_right:
                    ax.spines['right'].set_position(('outward',
                                                     58 * n_right))
                axes[ref.axis] = ax
            axis_color.setdefault(ref.axis, color)
            pts = self.series(ref)
            if not pts:
                continue
            ts = [p[0] for p in pts]
            vs = [p[1] for p in pts]
            if not ref.is_physics:
                # change events plot as steps, held to the end of the run
                ts.append(self.t_end)
                vs.append(vs[-1])
                h, = ax.plot(ts, vs, drawstyle='steps-post',
                             label=ref.label, color=color)
            else:
                h, = ax.plot(ts, vs, label=ref.label, alpha=0.85,
                             color=color)
            handles.append(h)
        # event markers: throttle changes, eeprom writes, notes and
        # wait outcomes, labelled vertically at the top
        for t, text in self.gtags:
            ax1.axvline(t, color='#8899bb', alpha=0.5, linewidth=1)
            ax1.annotate(text, xy=(t, 0.5),
                         xycoords=('data', 'axes fraction'),
                         rotation=90, va='center', ha='right',
                         fontsize=9.5, color='#223', alpha=0.95)
        for n_mark, (t, text, status) in enumerate(self.marker_events()):
            color = {'FAIL': 'red', 'PASS': 'green'}.get(status, 'gray')
            ax1.axvline(t, color=color, linestyle=':', alpha=0.6,
                        linewidth=1)
            label = text if len(text) < 42 else text[:39] + '...'
            # alternate the label side so close-together markers stay
            # readable
            ha = 'right' if n_mark % 2 == 0 else 'left'
            ax1.annotate(label, xy=(t, 1.0),
                         xycoords=('data', 'axes fraction'),
                         rotation=90, va='top', ha=ha, fontsize=6.5,
                         color=color, alpha=0.9)
        ax1.set_xlabel('simulated time (s)')
        for a, ax in axes.items():
            ax.set_ylabel(', '.join(r.label for r in self.graph_refs
                                    if r.axis == a))
            # with more than one right axis, colour each by its first
            # series so the columns of ticks stay attributable
            if a != 1 and len(axes) > 2:
                ax.tick_params(axis='y', colors=axis_color[a])
                ax.yaxis.label.set_color(axis_color[a])
        ax1.grid(True, alpha=0.3)
        ax1.set_title(title)
        if handles:
            ax1.legend(handles=handles, loc='best', fontsize=8)
        fig.tight_layout()
        fig.savefig(path, dpi=110)


# the interactive report page. Entirely self-contained (inline CSS, JS
# and data) so a file:// URL works with no network and no install
HTML_PAGE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>__TITLE__ - AM32 SITL test report</title>
<style>
  :root { --fg:#1a1a1a; --bg:#ffffff; --muted:#667; --line:#d8dce2;
          --panel:#f6f7f9; --pass:#1a7f37; --fail:#c62828; }
  * { box-sizing: border-box; }
  body { margin: 0 auto; padding: 14px 18px 40px; max-width: 1280px;
         color: var(--fg); background: var(--bg);
         font: 14px/1.45 system-ui, sans-serif; }
  h1 { font-size: 19px; margin: 0 0 2px; }
  h2 { font-size: 15px; margin: 22px 0 8px; }
  .meta { color: var(--muted); font-size: 12.5px; margin-bottom: 10px; }
  .badge { display: inline-block; padding: 1px 9px; border-radius: 10px;
           color: #fff; font-size: 12.5px; font-weight: 600;
           vertical-align: 2px; }
  .badge.pass { background: var(--pass); } .badge.fail { background: var(--fail); }
  #failures { display: none; margin: 0 0 10px; padding: 7px 12px;
              border: 1px solid #e5b4b4; border-radius: 4px;
              background: #fdf3f3; font: 12.5px/1.6 ui-monospace, monospace; }
  #failures div { cursor: pointer; } #failures div:hover { text-decoration: underline; }
  #chartwrap { position: relative; }
  #chart { width: 100%; height: 440px; display: block; cursor: crosshair;
           border: 1px solid var(--line); border-radius: 4px;
           background: #fff; touch-action: none; }
  #tip { position: absolute; pointer-events: none; display: none;
         background: rgba(28,30,34,0.93); color: #eef; padding: 6px 9px;
         border-radius: 5px; font: 12px/1.5 ui-monospace, monospace;
         white-space: pre; z-index: 5; }
  #legend { margin: 6px 0 0; font-size: 13px; }
  #legend label { margin-right: 16px; cursor: pointer; user-select: none; }
  #legend .sw { display: inline-block; width: 11px; height: 11px;
                border-radius: 2px; margin-right: 5px; vertical-align: -1px; }
  .hint { color: var(--muted); font-size: 12px; margin-top: 4px; }
  .cols { display: grid; grid-template-columns: 1fr 1fr; gap: 22px; }
  @media (max-width: 950px) { .cols { grid-template-columns: 1fr; } }
  table { border-collapse: collapse; width: 100%;
          font: 12.5px/1.5 ui-monospace, monospace; }
  td { padding: 1px 10px 1px 0; vertical-align: top; }
  td.t { color: var(--muted); white-space: nowrap; }
  tr.click { cursor: pointer; } tr.click:hover { background: var(--panel); }
  tr.failrow { background: #fdf3f3; }
  .ok { color: var(--pass); } .bad { color: var(--fail); font-weight: 600; }
  .scroll { max-height: 420px; overflow-y: auto;
            border: 1px solid var(--line); border-radius: 4px; padding: 6px 10px; }
  pre { background: var(--panel); border: 1px solid var(--line);
        border-radius: 4px; padding: 10px 12px; overflow-x: auto;
        font: 12.5px/1.5 ui-monospace, monospace; }
</style>
</head>
<body>
<h1 id="title"></h1>
<div class="meta" id="meta"></div>
<div id="failures"></div>
<div id="chartwrap">
  <canvas id="chart"></canvas>
  <div id="tip"></div>
</div>
<div id="legend"></div>
<div class="hint">drag to zoom a time range &middot; mouse wheel zooms around
the cursor &middot; double-click resets &middot; click pins the cursor
&middot; click a timeline row to jump there</div>
<div class="cols">
  <section>
    <h2>Timeline (simulated seconds)</h2>
    <div class="scroll"><table id="timeline"></table></div>
  </section>
  <section>
    <h2>Variable changes</h2>
    <div class="scroll"><table id="changes"></table></div>
  </section>
</div>
<h2>Script</h2>
<pre id="script"></pre>
<script>
const DATA = __DATA__;

const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');
const tip = document.getElementById('tip');
// one tick column per right hand axis (2, 3, ...)
const RIGHT_AXES = [...new Set(DATA.series.filter(s => s.axis > 1)
                               .map(s => s.axis))].sort((a, b) => a - b);
const AXIS_COLOR = {};
for (const s of DATA.series)
  if (!(s.axis in AXIS_COLOR)) AXIS_COLOR[s.axis] = s.color;
const PAD = { l: 64, r: RIGHT_AXES.length ? 12 + 54 * RIGHT_AXES.length : 24,
              t: 10, b: 26 };
let xfull = [Infinity, -Infinity];
for (const s of DATA.series) {
  if (!s.points.length) continue;
  xfull[0] = Math.min(xfull[0], s.points[0][0]);
  xfull[1] = Math.max(xfull[1], s.points[s.points.length - 1][0]);
}
for (const m of DATA.markers) {
  xfull[0] = Math.min(xfull[0], m.t); xfull[1] = Math.max(xfull[1], m.t);
}
if (!isFinite(xfull[0])) xfull = [0, 1];
if (xfull[1] - xfull[0] < 1e-6) xfull[1] = xfull[0] + 1;
let x0 = xfull[0], x1 = xfull[1];
let cursorT = null, pinnedT = null, drag = null;
const visible = DATA.series.map(() => true);

function fmt(v) {
  if (v === null || v === undefined) return '-';
  const a = Math.abs(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 1) return +v.toFixed(2) + '';
  return +v.toFixed(4) + '';
}
function niceTicks(lo, hi, n) {
  const span = hi - lo || 1;
  const step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  let step = 10 * mag;
  for (const m of [10, 5, 2, 1]) if (step0 <= m * mag) step = m * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step)
    out.push(+v.toPrecision(12));
  return out;
}
// value at t: held for step series, interpolated for continuous ones
function valueAt(s, t) {
  const p = s.points;
  if (!p.length || t < p[0][0]) return null;
  let lo = 0, hi = p.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (p[mid][0] <= t) lo = mid; else hi = mid - 1;
  }
  if (!s.step && lo < p.length - 1 && p[lo + 1][0] > p[lo][0])
    return p[lo][1] + (p[lo + 1][1] - p[lo][1]) *
           (t - p[lo][0]) / (p[lo + 1][0] - p[lo][0]);
  return p[lo][1];
}
function ydomains() {
  const dom = {};
  DATA.series.forEach((s, i) => {
    if (!visible[i]) return;
    let lo = Infinity, hi = -Infinity;
    let prev = null;
    for (const [t, v] of s.points) {
      if (t > x1) {
        // the segment bridging out of the window still draws into it
        if (prev !== null && prev[0] <= x1) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
        break;
      }
      if (t >= x0 && prev !== null && prev[0] < x0) { lo = Math.min(lo, prev[1]); hi = Math.max(hi, prev[1]); }
      if (t >= x0) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      prev = [t, v];
    }
    if (prev !== null && prev[0] < x0) { lo = Math.min(lo, prev[1]); hi = Math.max(hi, prev[1]); }
    if (!isFinite(lo)) return;
    if (!(s.axis in dom)) dom[s.axis] = [lo, hi];
    else { dom[s.axis][0] = Math.min(dom[s.axis][0], lo);
           dom[s.axis][1] = Math.max(dom[s.axis][1], hi); }
  });
  for (const a in dom) {
    let [lo, hi] = dom[a];
    if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.06;
    dom[a] = [lo - pad, hi + pad];
  }
  return dom;
}
function draw() {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== W * dpr) { canvas.width = W * dpr; canvas.height = H * dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const pw = W - PAD.l - PAD.r, ph = H - PAD.t - PAD.b;
  const X = t => PAD.l + (t - x0) / (x1 - x0) * pw;
  const dom = ydomains();
  const Y = {};
  for (const a in dom) {
    const [lo, hi] = dom[a];
    Y[a] = v => PAD.t + ph - (v - lo) / (hi - lo) * ph;
  }
  // grid and x ticks
  ctx.font = '11px system-ui'; ctx.fillStyle = '#667';
  ctx.strokeStyle = '#eceef2'; ctx.lineWidth = 1;
  for (const t of niceTicks(x0, x1, 8)) {
    const px = X(t);
    ctx.beginPath(); ctx.moveTo(px, PAD.t); ctx.lineTo(px, PAD.t + ph); ctx.stroke();
    ctx.textAlign = 'center';
    ctx.fillText(+t.toPrecision(8) + 's', px, H - 8);
  }
  const drawYAxis = (a, xpos, align, color) => {
    if (!dom[a]) return;
    for (const v of niceTicks(dom[a][0], dom[a][1], 6)) {
      const py = Y[a](v);
      if (a === 1) { ctx.strokeStyle = '#f2f3f6';
        ctx.beginPath(); ctx.moveTo(PAD.l, py); ctx.lineTo(PAD.l + pw, py); ctx.stroke(); }
      ctx.textAlign = align; ctx.fillStyle = color;
      ctx.fillText(fmt(v), xpos, py + 3.5);
    }
  };
  drawYAxis(1, PAD.l - 8, 'right', '#667');
  RIGHT_AXES.forEach((a, k) => {
    // colour the tick columns by their first series once there is
    // more than one right axis to tell apart
    const color = RIGHT_AXES.length > 1 ? AXIS_COLOR[a] : '#667';
    drawYAxis(a, PAD.l + pw + 8 + 54 * k, 'left', color);
  });
  ctx.strokeStyle = '#c8ccd4';
  ctx.strokeRect(PAD.l + 0.5, PAD.t + 0.5, pw - 1, ph - 1);
  // series
  ctx.save();
  ctx.beginPath(); ctx.rect(PAD.l, PAD.t, pw, ph); ctx.clip();
  DATA.series.forEach((s, i) => {
    if (!visible[i] || !Y[s.axis]) return;
    const y = Y[s.axis];
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.6; ctx.beginPath();
    let started = false, prevY = null;
    for (const [t, v] of s.points) {
      const px = X(t), py = y(v);
      if (!started) { ctx.moveTo(px, py); started = true; }
      else if (s.step) { ctx.lineTo(px, prevY); ctx.lineTo(px, py); }
      else ctx.lineTo(px, py);
      prevY = py;
    }
    ctx.stroke();
  });
  // event markers
  for (const m of DATA.markers) {
    const px = X(m.t);
    if (px < PAD.l || px > PAD.l + pw) continue;
    ctx.strokeStyle = m.status === 'FAIL' ? '#c62828'
                    : m.status === 'PASS' ? '#1a7f37' : '#99a';
    ctx.setLineDash([2, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px, PAD.t); ctx.lineTo(px, PAD.t + ph); ctx.stroke();
    ctx.setLineDash([]);
  }
  // graphtag annotations: prominent vertical section labels
  ctx.font = '12px system-ui';
  for (const g of DATA.tags) {
    const px = X(g.t);
    if (px < PAD.l || px > PAD.l + pw) continue;
    ctx.strokeStyle = '#8899bb'; ctx.globalAlpha = 0.55; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px, PAD.t); ctx.lineTo(px, PAD.t + ph); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.save();
    ctx.translate(px - 5, PAD.t + ph / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center'; ctx.fillStyle = '#223';
    ctx.fillText(g.text, 0, 0);
    ctx.restore();
  }
  ctx.font = '11px system-ui';
  // pinned + live cursor
  for (const [t, col] of [[pinnedT, '#c8a200'], [cursorT, '#555']]) {
    if (t === null || t < x0 || t > x1) continue;
    ctx.strokeStyle = col; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(X(t), PAD.t); ctx.lineTo(X(t), PAD.t + ph); ctx.stroke();
  }
  // drag zoom rectangle
  if (drag && drag.zooming) {
    ctx.fillStyle = 'rgba(80,120,200,0.15)';
    const a = Math.min(drag.px, drag.cur), b = Math.max(drag.px, drag.cur);
    ctx.fillRect(a, PAD.t, b - a, ph);
  }
  ctx.restore();
}
function showTip(evx, evy, t) {
  const lines = ['t = ' + t.toFixed(4) + 's'];
  DATA.series.forEach((s, i) => {
    if (!visible[i]) return;
    lines.push(s.name + ' = ' + fmt(valueAt(s, t)));
  });
  for (const m of DATA.markers)
    if (Math.abs((m.t - x0) / (x1 - x0) - (t - x0) / (x1 - x0)) *
        (canvas.clientWidth - PAD.l - PAD.r) < 4)
      lines.push('▸ ' + m.text);
  tip.textContent = lines.join('\n');
  tip.style.display = 'block';
  const wrap = document.getElementById('chartwrap').getBoundingClientRect();
  let lx = evx - wrap.left + 14, ly = evy - wrap.top + 12;
  if (lx + tip.offsetWidth > wrap.width - 8) lx -= tip.offsetWidth + 26;
  tip.style.left = lx + 'px'; tip.style.top = ly + 'px';
}
function evT(ev) {
  const r = canvas.getBoundingClientRect();
  const pw = canvas.clientWidth - PAD.l - PAD.r;
  return x0 + (ev.clientX - r.left - PAD.l) / pw * (x1 - x0);
}
canvas.addEventListener('mousemove', ev => {
  const r = canvas.getBoundingClientRect();
  if (drag) {
    drag.cur = ev.clientX - r.left;
    if (Math.abs(drag.cur - drag.px) > 5) drag.zooming = true;
  }
  cursorT = Math.max(x0, Math.min(x1, evT(ev)));
  showTip(ev.clientX, ev.clientY, cursorT);
  draw();
});
canvas.addEventListener('mouseleave', () => {
  cursorT = null; tip.style.display = 'none'; draw();
});
canvas.addEventListener('mousedown', ev => {
  const r = canvas.getBoundingClientRect();
  drag = { px: ev.clientX - r.left, cur: ev.clientX - r.left,
           t: evT(ev), zooming: false };
});
window.addEventListener('mouseup', ev => {
  if (!drag) return;
  if (drag.zooming) {
    let ta = Math.min(drag.t, evT(ev)), tb = Math.max(drag.t, evT(ev));
    ta = Math.max(ta, xfull[0]); tb = Math.min(tb, xfull[1]);
    if (tb - ta > (xfull[1] - xfull[0]) * 1e-6) { x0 = ta; x1 = tb; }
  } else {
    pinnedT = (pinnedT !== null &&
               Math.abs(pinnedT - drag.t) < (x1 - x0) * 0.005)
              ? null : drag.t;
  }
  drag = null; draw();
});
canvas.addEventListener('wheel', ev => {
  ev.preventDefault();
  const t = evT(ev), f = ev.deltaY > 0 ? 1.25 : 0.8;
  let nx0 = t - (t - x0) * f, nx1 = t + (x1 - t) * f;
  nx0 = Math.max(nx0, xfull[0] - (xfull[1] - xfull[0]) * 0.05);
  nx1 = Math.min(nx1, xfull[1] + (xfull[1] - xfull[0]) * 0.05);
  if (nx1 - nx0 > (xfull[1] - xfull[0]) * 1e-6) { x0 = nx0; x1 = nx1; }
  draw();
}, { passive: false });
canvas.addEventListener('dblclick', () => {
  x0 = xfull[0]; x1 = xfull[1]; draw();
});
window.addEventListener('resize', draw);

// header
document.getElementById('title').innerHTML =
  esc(DATA.title) + ' &nbsp; <span class="badge ' +
  (DATA.result === 'PASS' ? 'pass' : 'fail') + '">' + DATA.result + '</span>';
document.getElementById('meta').textContent =
  DATA.script_path + '  ·  ' + DATA.sitl +
  (DATA.git ? '  ·  git ' + DATA.git : '') + '  ·  ' + DATA.date;
// legend
const leg = document.getElementById('legend');
DATA.series.forEach((s, i) => {
  const lab = document.createElement('label');
  const cb = document.createElement('input');
  cb.type = 'checkbox'; cb.checked = true;
  cb.addEventListener('change', () => { visible[i] = cb.checked; draw(); });
  const sw = document.createElement('span');
  sw.className = 'sw'; sw.style.background = s.color;
  lab.append(cb, ' ', sw, s.name +
             (s.axis > 1 ? (RIGHT_AXES.length > 1 ? ' (y' + s.axis + ')'
                                                  : ' (right)') : ''));
  leg.appendChild(lab);
});
// tables
function esc(s) {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
}
const tl = document.getElementById('timeline');
tl.innerHTML = DATA.timeline.map(e =>
  '<tr class="click' + (e.status === 'FAIL' ? ' failrow' : '') +
  '" data-t="' + e.t + '"><td class="t">' + e.t.toFixed(4) +
  '</td><td>' + esc(e.text) +
  (e.status === 'PASS' ? ' <span class="ok">[ok]</span>'
   : e.status === 'FAIL' ? ' <span class="bad">[FAIL]</span>' : '') +
  '</td></tr>').join('');
tl.addEventListener('click', ev => {
  const tr = ev.target.closest('tr'); if (!tr) return;
  pinnedT = +tr.dataset.t;
  if (pinnedT < x0 || pinnedT > x1) { x0 = xfull[0]; x1 = xfull[1]; }
  draw();
  canvas.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
});
const chg = document.getElementById('changes');
chg.innerHTML = DATA.changes.map(c =>
  '<tr class="click" data-t="' + c.t + '"><td class="t">' + c.t.toFixed(4) +
  '</td><td>' + esc(c.name) + (c.prev === null
    ? ' = ' + fmt(c.val) + ' (initial)'
    : ': ' + fmt(c.prev) + ' → ' + fmt(c.val)) + '</td></tr>').join('')
  + Object.entries(DATA.suppressed).map(([n, k]) =>
    '<tr><td></td><td>(' + esc(n) + ': ' + k +
    ' further changes suppressed, see the CSV)</td></tr>').join('');
chg.addEventListener('click', ev => {
  const tr = ev.target.closest('tr'); if (!tr || !tr.dataset.t) return;
  pinnedT = +tr.dataset.t; draw();
});
document.getElementById('script').textContent = DATA.script;
// failures deserve the top of the page, not a scroll hunt through the
// timeline; clicking one pins the graph cursor at it
const failBox = document.getElementById('failures');
const failEntries = DATA.timeline.filter(e => e.status === 'FAIL');
if (failEntries.length) {
  failBox.style.display = 'block';
  failBox.innerHTML = failEntries.map(e =>
    '<div data-t="' + e.t + '"><span class="bad">FAIL</span> t=' +
    e.t.toFixed(4) + '  ' + esc(e.text) + '</div>').join('');
  failBox.addEventListener('click', ev => {
    const row = ev.target.closest('div[data-t]'); if (!row) return;
    pinnedT = +row.dataset.t;
    if (pinnedT < x0 || pinnedT > x1) { x0 = xfull[0]; x1 = xfull[1]; }
    draw();
  });
}
draw();
</script>
</body>
</html>
'''


def run_one(args, script, outdir, eeprom_fields):
    '''run one test script; returns the number of failures'''
    stmts = parse_script(script, eeprom_fields)
    base = os.path.splitext(os.path.basename(script))[0]
    os.makedirs(outdir, exist_ok=True)

    run_args = argparse.Namespace(**vars(args))
    run_args.script, run_args.outdir = script, outdir
    runner = Runner(run_args, stmts, eeprom_fields)
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
            runner.write_graph(png, os.path.basename(script))
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
    ap.add_argument('scripts', nargs='+', metavar='script.scr')
    pat = os.path.join(HERE, '..', '..', 'obj', 'AM32_AM32_SITL_CAN_*.elf')
    hits = sorted(glob.glob(pat), key=os.path.getmtime)
    ap.add_argument('--sitl', default=os.path.normpath(hits[-1]) if hits
                    else None, help='SITL binary (default: newest in obj/)')
    ap.add_argument('--outdir', default=None,
                    help='where the reports, graphs and CSVs go '
                         '(default test_outputs/<test name>/)')
    ap.add_argument('--sample-us', type=int, default=500,
                    help='physics stream sample period')
    ap.add_argument('--watch-period-us', type=int, default=1000,
                    help='per variable change coalescing interval')
    ap.add_argument('--realtime', action='store_true',
                    help='pace the simulation to the wall clock '
                         '(default free-runs with --nosleep)')
    args = ap.parse_args()

    if not args.sitl or not os.path.exists(args.sitl):
        sys.exit('error: SITL binary not found '
                 '(build with: make AM32_SITL_CAN)')
    args.sitl = os.path.abspath(args.sitl)

    eeprom_fields = parse_eeprom_layout(
        os.path.join(HERE, '..', '..', 'Inc', 'eeprom.h'))

    results = {}
    for script in args.scripts:
        base = os.path.splitext(os.path.basename(script))[0]
        if len(args.scripts) > 1:
            print('\n=== %s ===' % script)
        outdir = os.path.join(args.outdir, base) if args.outdir \
            and len(args.scripts) > 1 else \
            (args.outdir or os.path.join('test_outputs', base))
        try:
            results[script] = run_one(args, script, outdir, eeprom_fields)
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
