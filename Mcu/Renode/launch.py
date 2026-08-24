#!/usr/bin/env python3
'''
Renode ESC lab: pick a hardware target, a bootloader and a firmware,
press Start, and get an emulated ESC a configurator can talk to.

The emulator runs gen_target.py TARGET --link, serving the SITL wire
protocols. The configurator port is served by Mcu/SITL on those ports,
on a pty or - so a browser can reach it - on a virtual USB serial
device attached through vhci_hcd. The protocol choice picks what sits
on that port: the fake flight controller (MSP with BLHeli 4-way
passthrough to the emulated bootloader, as a real FC provides) or a
direct single-wire adapter (the raw bootloader protocol with the
adapter's self-echo, as a USB linker soldered to the signal pad
provides). This is the rig for developing am32.tridgell.net against
emulated CAN and non-CAN ESCs with no hardware.

Bootloaders are matched to the target automatically: the ELF must be
built for the target's signal pin (a PA2 bootloader on a PB4 target
answers nothing), so the list only offers AM32_<MCU>_BOOTLOADER_<PIN>*
builds from the bootloader repo's obj directory.

The Renode monitor is served on a telnet port and polled once a
second, so the status panel shows the live PC (labelled when it is
executing inside the bootloader), the emulation speed against real
time, the retired instruction rate and the machine's virtual time.

with --control-port N the UI can be driven over a localhost TCP
connection (one command per line), for scripted tests:
  target NAME, bootloader auto|none|PATH, firmware auto|none|PATH,
  eeprom defaults|blank, conf off|serial|usb, protocol 4way|direct,
  canbus N, download-renode, start, stop, status, quit
replies are prefixed OK/ERR/STATUS.
'''

import argparse
import errno
import glob
import hashlib
import json
import os
import platform
import queue
import re
import signal
import socket
import struct
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import urllib.parse
import urllib.request
import zipfile

from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SITL_DIR = os.path.join(REPO, 'Mcu', 'SITL')
sys.path.insert(0, SITL_DIR)

# the bootloader family names as its obj files spell them
FAMILY_MCU = {
    'f051': 'F051', 'f031': 'F031', 'e230': 'E230', 'f415': 'F415',
    'f421': 'F421', 'g071': 'G071', 'g431': 'G431', 'l431': 'L431',
    'v203': 'V203', 'a153': 'A153',
}

SITL_MAGIC = 0x4453
STATE_MAGIC = 0x5353

# -- Renode telnet monitor, for live PC / speedup metrics --------------
# (the same approach as ArduPilot's Renode launcher: poll the monitor
# once a second and derive realtime speed from virtual-vs-wall deltas)

ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[ -/]*[@-~]')
PROMPT_RE = re.compile(r'\([^)]+\)\s*$')
METRICS_COMMAND = (
    'cpu PC; cpu PerformanceInMips; cpu ExecutedInstructions; '
    'emulation GetTimeSourceInfo'
)

RENODE_DOWNLOAD_BASE = 'https://firmware.ardupilot.org/Tools/Renode/'
RENODE_LATEST_URL = urllib.parse.urljoin(RENODE_DOWNLOAD_BASE, 'latest.json')
RENODE_SELECTION = 'selected.json'


def default_renode_cache():
    root = os.environ.get('XDG_CACHE_HOME')
    if root:
        return Path(root).expanduser() / 'ardupilot' / 'renode'
    return Path.home() / '.cache' / 'ardupilot' / 'renode'


def host_target(system=None, machine=None):
    '''platform and architecture names used by the download manifest'''
    system = (system or platform.system()).lower()
    machine = (machine or platform.machine()).lower()
    platforms = {'linux': 'linux', 'darwin': 'macos', 'windows': 'windows'}
    architectures = {
        'amd64': 'x86_64',
        'x64': 'x86_64',
        'x86_64': 'x86_64',
        'aarch64': 'aarch64' if system == 'linux' else 'arm64',
        'arm64': 'aarch64' if system == 'linux' else 'arm64',
    }
    if system not in platforms or machine not in architectures:
        raise RuntimeError('no ArduPilot Renode download for %s/%s' %
                           (system, machine))
    return platforms[system], architectures[machine]


def select_renode_package(latest, system=None, machine=None):
    '''select the portable package for this host from latest.json'''
    wanted_platform, wanted_architecture = host_target(system, machine)
    for artifact in latest.get('artifacts', []):
        target = artifact.get('target', {})
        if (target.get('platform') != wanted_platform or
                target.get('architecture') != wanted_architecture):
            continue
        packages = artifact.get('packages', [])
        if wanted_platform == 'windows':
            packages = [package for package in packages
                        if package.get('filename', '').endswith('.zip')]
        elif wanted_platform == 'linux':
            packages = [package for package in packages
                        if package.get('filename', '').endswith('.tar.gz')]
        else:
            raise RuntimeError(
                'automatic Renode installation is not yet supported on macOS')
        if len(packages) != 1:
            raise RuntimeError(
                'latest.json has no unique portable package for %s/%s' %
                (wanted_platform, wanted_architecture))
        package = dict(packages[0])
        filename = package.get('filename')
        digest = package.get('sha256')
        size = package.get('size')
        if (not isinstance(filename, str) or Path(filename).name != filename or
                not isinstance(digest, str) or
                not re.fullmatch(r'[0-9a-fA-F]{64}', digest) or
                not isinstance(size, int) or size <= 0):
            raise RuntimeError(
                'latest.json has invalid portable package metadata')
        runtime_identifier = target.get('runtime_identifier')
        if (not isinstance(runtime_identifier, str) or
                not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*',
                                 runtime_identifier)):
            raise RuntimeError('latest.json has no valid runtime identifier')
        package['runtime_identifier'] = runtime_identifier
        package['platform'] = wanted_platform
        package['architecture'] = wanted_architecture
        return package
    raise RuntimeError('latest.json has no package for %s/%s' %
                       (wanted_platform, wanted_architecture))


def fetch_renode_latest(opener=None):
    '''fetch uncached current-version metadata from firmware.ardupilot.org'''
    opener = opener or urllib.request.urlopen
    separator = '&' if '?' in RENODE_LATEST_URL else '?'
    url = '%s%st=%u' % (RENODE_LATEST_URL, separator, time.time_ns())
    request = urllib.request.Request(
        url, headers={'Cache-Control': 'no-cache', 'Pragma': 'no-cache'})
    with opener(request, timeout=30) as response:
        data = response.read()
    latest = json.loads(data.decode('utf-8'))
    if latest.get('schema_version') != 1:
        raise RuntimeError('unsupported Renode latest.json schema')
    revision = latest.get('source', {}).get('revision')
    if not revision or not re.fullmatch(r'[0-9a-fA-F]{7,64}', revision):
        raise RuntimeError('latest.json has no valid source revision')
    return latest


def renode_cache_key(latest, package):
    revision = latest['source']['revision']
    runtime = package['runtime_identifier']
    digest = package['sha256'][:12]
    return '%s-%s-%s' % (runtime, revision[:12], digest)


def verified_renode_install(cache, install_name, executable_name,
                            expected=None):
    '''executable from one valid, contained cache install, or None'''
    try:
        cache = Path(cache).expanduser().resolve()
        if (not isinstance(install_name, str) or
                not isinstance(executable_name, str)):
            return None
        install = (cache / install_name).resolve()
        executable = (install / executable_name).resolve()
        if (not install.is_relative_to(cache.resolve()) or
                not executable.is_relative_to(install) or
                not executable.is_file()):
            return None
        manifest = json.loads((install / 'ardupilot-renode.json').read_text())
        if (not isinstance(manifest, dict) or
                manifest.get('executable') != executable_name):
            return None
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError):
        return None
    if expected is not None:
        if any(manifest.get(key) != value
               for key, value in expected.items()):
            return None
    return executable


def cached_renode(cache, latest=None, package=None):
    '''a verified cache executable, optionally requiring the latest build'''
    cache = Path(cache).expanduser()
    try:
        selection = json.loads((cache / RENODE_SELECTION).read_text())
        if not isinstance(selection, dict):
            return None
        install_name = selection.get('install')
        executable_name = selection.get('executable')
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    expected = None
    if latest is not None and package is not None:
        expected = {
            'revision': latest['source']['revision'],
            'filename': package['filename'],
            'sha256': package['sha256'],
            'runtime_identifier': package['runtime_identifier'],
        }
    return verified_renode_install(cache, install_name, executable_name,
                                   expected)


def download_file(url, destination, size, sha256, progress=None, opener=None):
    opener = opener or urllib.request.urlopen
    request = urllib.request.Request(
        url, headers={'Cache-Control': 'no-cache'})
    digest = hashlib.sha256()
    received = 0
    with (opener(request, timeout=60) as response,
          destination.open('wb') as output):
        while True:
            block = response.read(1024 * 1024)
            if not block:
                break
            output.write(block)
            digest.update(block)
            received += len(block)
            if progress:
                progress(received, size)
    if received != size:
        raise RuntimeError('Renode download is %u bytes; expected %u' %
                           (received, size))
    if digest.hexdigest().lower() != sha256.lower():
        raise RuntimeError(
            'Renode download SHA-256 does not match latest.json')


def extract_renode(archive, destination, package):
    destination.mkdir()
    if package['filename'].endswith('.tar.gz'):
        with tarfile.open(archive, 'r:gz') as bundle:
            bundle.extractall(destination, filter='data')
        executable_name = 'renode'
    elif package['filename'].endswith('.zip'):
        with zipfile.ZipFile(archive) as bundle:
            for member in bundle.infolist():
                target = (destination / member.filename).resolve()
                if not target.is_relative_to(destination.resolve()):
                    raise RuntimeError('unsafe path in Renode zip package')
            bundle.extractall(destination)
        executable_name = 'renode.exe'
    else:
        raise RuntimeError(
            'unsupported Renode package %s' % package['filename'])
    candidates = [path for path in destination.rglob(executable_name)
                  if path.is_file()]
    if len(candidates) != 1:
        raise RuntimeError('downloaded package has %u %s executables' %
                           (len(candidates), executable_name))
    executable = candidates[0]
    if package['platform'] != 'windows':
        executable.chmod(executable.stat().st_mode | 0o111)
    return executable


def install_current_renode(cache, latest=None, progress=None, opener=None):
    '''ensure the cache holds the freshly queried current Renode package'''
    cache = Path(cache).expanduser().resolve()
    latest = latest or fetch_renode_latest(opener)
    package = select_renode_package(latest)
    executable = cached_renode(cache, latest, package)
    if executable is not None:
        return executable, latest, False

    cache.mkdir(parents=True, exist_ok=True)
    install_name = renode_cache_key(latest, package)
    install = cache / install_name
    with tempfile.TemporaryDirectory(
            prefix='.download-', dir=cache) as temporary:
        temporary = Path(temporary)
        archive = temporary / package['filename']
        filename = package['filename']
        if Path(filename).name != filename:
            raise RuntimeError('invalid Renode package filename')
        url = urllib.parse.urljoin(RENODE_DOWNLOAD_BASE,
                                   urllib.parse.quote(filename))
        download_file(url, archive, package['size'], package['sha256'],
                      progress, opener)
        payload = temporary / 'payload'
        executable = extract_renode(archive, payload, package)
        manifest = {
            'revision': latest['source']['revision'],
            'renode_version': latest.get('renode_version'),
            'filename': filename,
            'sha256': package['sha256'],
            'runtime_identifier': package['runtime_identifier'],
            'executable': str(executable.relative_to(payload)),
        }
        (payload / 'ardupilot-renode.json').write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        expected = {
            key: manifest[key] for key in (
                'revision', 'filename', 'sha256', 'runtime_identifier')
        }
        while True:
            try:
                payload.rename(install)
                break
            except OSError as error:
                if error.errno not in (errno.EEXIST, errno.ENOTEMPTY):
                    raise
                existing = verified_renode_install(
                    cache, install_name, manifest['executable'], expected)
                if existing is not None:
                    break
                # Preserve an unexpected/corrupt directory for diagnosis and
                # install atomically beside it. This also avoids collisions
                # between launchers downloading the same build concurrently.
                install_name = '%s-%u' % (
                    renode_cache_key(latest, package), time.time_ns())
                install = cache / install_name

    selection = {
        'install': install_name,
        'executable': manifest['executable'],
    }
    selection_tmp = cache / (RENODE_SELECTION + '.tmp')
    selection_tmp.write_text(json.dumps(selection, indent=2) + '\n')
    selection_tmp.replace(cache / RENODE_SELECTION)
    return install / manifest['executable'], latest, True


def parse_elapsed(value):
    '''Renode's [days.]HH:MM:SS.s elapsed-time format, in seconds'''
    fields = value.strip().split(':')
    if len(fields) != 3:
        raise ValueError('bad elapsed time %s' % value)
    days = 0
    hours = fields[0]
    if '.' in hours:
        days_text, hours = hours.split('.', 1)
        days = int(days_text)
    return (days * 86400 + int(hours) * 3600 + int(fields[1]) * 60 +
            float(fields[2]))


def clean_monitor_text(data):
    text = data.decode('utf-8', errors='replace')
    text = ANSI_RE.sub('', text).replace('\r', '')
    # telnet negotiation bytes decode as replacement characters
    return text.replace('\ufffd', '')


def parse_metrics(text):
    values = re.findall(r'(?m)^\s*(0x[0-9A-Fa-f]+)\s*$', text)
    virtual = re.search(r'(?m)^Elapsed Virtual Time:\s*(\S+)\s*$', text)
    host = re.search(r'(?m)^Elapsed Host Time:\s*(\S+)\s*$', text)
    if len(values) < 3 or virtual is None or host is None:
        raise ValueError('incomplete Renode monitor metrics')
    return {
        'pc': int(values[0], 16),
        'mips': int(values[1], 16),
        'instructions': int(values[2], 16),
        'virtual_seconds': parse_elapsed(virtual.group(1)),
    }


class MonitorClient:
    '''small, single-threaded client for Renode's telnet monitor'''

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self, timeout=45):
        self.close()
        self.sock = socket.create_connection((self.host, self.port),
                                             timeout=2)
        self.sock.settimeout(0.5)
        self._read_to_prompt(timeout)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def command(self, command, timeout=5):
        if self.sock is None:
            raise OSError('monitor is not connected')
        self.sock.sendall((command + '\n').encode('ascii'))
        return self._read_to_prompt(timeout, expected=command)

    def _read_to_prompt(self, timeout, expected=None):
        deadline = time.monotonic() + timeout
        data = bytearray()
        while time.monotonic() < deadline:
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError('Renode monitor disconnected')
            data.extend(chunk)
            text = clean_monitor_text(data)
            if ((expected is None or expected in text) and
                    PROMPT_RE.search(text)):
                return text
        raise TimeoutError('timed out waiting for the Renode monitor')


def bootloader_dirs(explicit=None):
    '''where bootloader ELFs might live: an explicit dir, the env, then
    the bootloader repo checked out next to this one'''
    cands = []
    if explicit:
        cands.append(explicit)
    env = os.environ.get('AM32_BOOTLOADER_OBJ')
    if env:
        cands.append(env)
    parent = os.path.dirname(REPO)
    for name in ('AM32-bootloader', 'am32-bootloader'):
        cands.append(os.path.join(parent, name, 'obj'))
    return [d for d in cands if os.path.isdir(d)]


def find_bootloaders(family, pin, dirs, dronecan=False):
    '''bootloader images built for this target's MCU and signal pin.

    Ordered so the first entry is the right default: the CAN build for a
    DroneCAN target, the plain default-flash build otherwise. Loading a
    CAN (128K) bootloader on a 64K non-CAN target answers the wire but
    puts the eeprom where neither the firmware nor the configurator
    expects it, so the ordering is load-bearing.
    '''
    mcu = FAMILY_MCU.get(family)
    if mcu is None:
        return []
    hits = []
    for d in dirs:
        for ext in ('elf', 'hex'):
            pat = os.path.join(d, 'AM32_%s_BOOTLOADER_%s*_V*.%s'
                               % (mcu, pin, ext))
            hits += glob.glob(pat)
    # newest version of each variant only, an ELF (which carries the
    # symbols and debug info) beating the hex built beside it
    byvar = {}
    for h in sorted(hits, key=lambda h: (h[:-4], h.endswith('.elf'))):
        var = re.sub(r'_V\d+\.(elf|hex)$', '', os.path.basename(h))
        byvar[var] = h

    def rank(path):
        name = os.path.basename(path)
        is_can = '_CAN_' in name
        is_sized = re.search(r'_\d+K_', name) is not None
        if dronecan:
            return (0 if is_can else 1, name)
        # plain default-flash first, size variants next, CAN last
        return ((2 if is_can else (1 if is_sized else 0)), name)
    return sorted(byvar.values(), key=rank)


class ProcRunner(object):
    '''a child process whose output lines land in a queue, killed as a
    group so renode dies with its launcher'''

    def __init__(self, out_q):
        self.out_q = out_q
        self.proc = None

    def start(self, cmd, cwd=None):
        # stdin must be a pipe we hold open: renode's console exits on
        # EOF, so inheriting a nohup'd or exhausted stdin kills the
        # emulator moments after it starts
        self.proc = subprocess.Popen(
            cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, errors='replace',
            start_new_session=True)
        threading.Thread(target=self._pump, args=(self.proc,),
                         daemon=True).start()

    def _pump(self, proc):
        for line in proc.stdout:
            self.out_q.put(line.rstrip('\n'))
        self.out_q.put('[emulator exited, status %s]' % proc.wait())

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc is None:
            return
        pgid = self.proc.pid
        if self.proc.poll() is None:
            try:
                os.killpg(pgid, signal.SIGTERM)
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            except ProcessLookupError:
                pass
        # The direct child exiting says nothing about its children:
        # gen_target dies on SIGTERM in an instant while renode can
        # linger (and, its stdin gone, spin on the dead console). Sweep
        # the whole group; a kill of an already-empty group is a no-op.
        deadline = time.time() + 5
        while time.time() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.2)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc = None


class Lab(object):
    '''the launcher's state and actions, UI-independent so the control
    port drives exactly what the buttons do'''

    def __init__(self, args):
        self.args = args
        self.log_q = queue.Queue()
        self.runner = ProcRunner(self.log_q)
        self.emulator_ready = False
        self.stub = None
        self.usb_attached = False
        self.target = None
        self.info = None                  # {'family','pin','dronecan'}
        self.bootloader = 'auto'          # auto | none | path
        self.firmware = 'auto'            # auto | none | path
        self.eeprom = 'defaults'          # defaults | blank
        self.metrics = None               # latest monitor sample
        self.generation = 0               # invalidates old pollers
        self.conf = 'serial'              # off | serial | usb
        self.protocol = '4way'            # 4way | direct
        self.can_bus = 0
        self.status = 'stopped'
        self.conf_port = ''               # the pty / tty path once up
        self.bl_dirs = bootloader_dirs(args.bootloader_dir)

    def log(self, msg):
        self.log_q.put(msg)

    # -- queries -------------------------------------------------------

    def target_info(self, target):
        '''family/pin/dronecan for a target, via gen_target --info'''
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, 'gen_target.py'),
             target, '--info'],
            capture_output=True, text=True)
        for line in r.stdout.splitlines():
            line = line.strip()
            if line.startswith('{'):
                d = json.loads(line)
                if 'error' not in d:
                    return d
                self.log('target %s: %s' % (target, d['error']))
        return None

    def matched_bootloaders(self):
        if self.info is None:
            return []
        return find_bootloaders(self.info['family'], self.info['pin'],
                                self.bl_dirs, self.info['dronecan'])

    def pick_bootloader(self):
        '''the ELF to load, or None for app-only, or an error string'''
        if self.bootloader == 'none':
            return None
        if self.bootloader != 'auto':
            if not os.path.isfile(self.bootloader):
                return 'no bootloader at %s' % self.bootloader
            return self.bootloader
        hits = self.matched_bootloaders()
        if not hits:
            return ('no AM32_%s_BOOTLOADER_%s* ELF found; build one in the '
                    'bootloader repo or Browse to it'
                    % (FAMILY_MCU.get(self.info['family'], '?'),
                       self.info['pin']))
        return hits[0]

    # -- lifecycle -----------------------------------------------------

    @staticmethod
    def wait_port_free(port, timeout=8.0, tcp=False):
        '''wait for a port to be bindable; the previous emulator's
        teardown can outlive the Stop click by a moment'''
        deadline = time.time() + timeout
        while True:
            s = socket.socket(socket.AF_INET,
                              socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM)
            try:
                if tcp:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(('127.0.0.1', port))
                return True
            except OSError:
                if time.time() >= deadline:
                    return False
                time.sleep(0.3)
            finally:
                s.close()

    def start(self):
        if self.runner.running():
            return 'already running'
        if self.target is None or self.info is None:
            return 'pick a target first'
        for port in (self.args.gui_port, self.args.state_port):
            if not self.wait_port_free(port):
                return ('udp port %u is still in use - a leftover emulator? '
                        'try: pkill -f renode' % port)
        if not self.wait_port_free(self.args.monitor_port, tcp=True):
            return ('monitor port %u is still in use - a leftover emulator? '
                    'try: pkill -f renode' % self.args.monitor_port)
        bl = self.pick_bootloader()
        if isinstance(bl, str) and not os.path.isfile(bl):
            return bl
        cmd = [sys.executable, os.path.join(HERE, 'gen_target.py'),
               self.target, '--link',
               '--gui-port', str(self.args.gui_port),
               '--gui-state-port', str(self.args.state_port),
               '--monitor-port', str(self.args.monitor_port)]
        if bl is not None:
            cmd += ['--bootloader-elf', bl]
        if self.firmware == 'none':
            if bl is None:
                return ('a blank ESC still needs its bootloader: pick one, '
                        'or pick a firmware')
            cmd += ['--no-firmware']
        elif self.firmware != 'auto':
            if not os.path.isfile(self.firmware):
                return 'no firmware at %s' % self.firmware
            cmd += ['--elf', self.firmware]
        if self.eeprom == 'blank':
            cmd += ['--blank-eeprom']
        if self.info['dronecan']:
            cmd += ['--can-bus', str(self.can_bus)]
        if self.args.renode:
            cmd += ['--renode', self.args.renode]
        self.emulator_ready = False
        self.start_failed = False
        self.conf_port = ''
        self.status = 'starting emulator...'
        self.log('$ ' + ' '.join(cmd))
        self.runner.start(cmd, cwd=REPO)
        threading.Thread(target=self._wait_ready, args=(bl,),
                         daemon=True).start()
        return None

    def _wait_ready(self, bl):
        '''watch for the emulator's input port, then bring up the
        configurator side'''
        deadline = time.time() + 120
        while time.time() < deadline and self.runner.running():
            if self.emulator_ready or self.start_failed:
                break
            time.sleep(0.3)
        if not self.emulator_ready:
            # a half-started emulator (a failed port bind still leaves
            # the machine running) must not linger and block the retry
            self.runner.stop()
            if not self.status.startswith('emulator exited'):
                self.status = 'emulator did not come up'
            return
        self.generation += 1
        threading.Thread(target=self._metrics_loop,
                         args=(self.generation,), daemon=True).start()
        if bl is not None:
            self._enter_bootloader()
        if self.conf == 'off':
            self.status = 'running (no configurator port)'
            return
        try:
            self._start_stub()
        except Exception as ex:
            self.status = 'configurator port failed: %s' % ex
            self.log(self.status)

    def _enter_bootloader(self):
        '''hold the signal wire high and reset, so the ESC is parked in
        the bootloader before the first configurator connect'''
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # type 5 line level, idle high
            s.sendto(struct.pack('<HBBHH', SITL_MAGIC, 5, 4, 1, 0),
                     ('127.0.0.1', self.args.gui_port))
            time.sleep(0.3)
            s.sendto(struct.pack('<HBB', STATE_MAGIC, 9, 0),
                     ('127.0.0.1', self.args.state_port))
        finally:
            s.close()
        self.log('holding the signal wire; ESC reset into the bootloader')

    def _metrics_loop(self, generation):
        '''poll the Renode monitor for PC and timing, and derive the
        realtime speedup from virtual-vs-wall deltas over a sliding
        window (Renode advances in bursts, so an instant ratio just
        flaps around the true speed)'''
        client = MonitorClient('127.0.0.1', self.args.monitor_port)
        history = []
        try:
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                if generation != self.generation or not self.runner.running():
                    return
                try:
                    client.connect()
                    break
                except (OSError, TimeoutError):
                    client.close()
                    time.sleep(0.5)
            else:
                self.log_q.put(('__monitor_error__', generation,
                                'monitor did not become ready'))
                return
            while generation == self.generation and self.runner.running():
                try:
                    current = parse_metrics(client.command(
                        METRICS_COMMAND, timeout=60 if not history else 5))
                except (OSError, TimeoutError, ValueError) as error:
                    self.log_q.put(('__monitor_error__', generation,
                                    str(error)))
                    return
                current['wall_seconds'] = time.monotonic()
                history.append(current)
                while (len(history) > 2 and current['wall_seconds']
                        - history[1]['wall_seconds'] >= 8):
                    history.pop(0)
                if len(history) > 1:
                    base = history[0]
                    wall = current['wall_seconds'] - base['wall_seconds']
                    virtual = (current['virtual_seconds']
                               - base['virtual_seconds'])
                    executed = (current['instructions']
                                - base['instructions'])
                    if wall > 0 and virtual >= 0:
                        current['speedup'] = virtual / wall
                        current['executed_mips'] = executed / wall / 1e6
                self.log_q.put(('__metrics__', generation, current))
                time.sleep(1)
        finally:
            client.close()

    def format_metrics(self):
        '''one status line: PC (with the flash region it is executing
        from), realtime speedup, executed MIPS and virtual time'''
        m = self.metrics
        if m is None:
            return ''
        where = ''
        app_base = (self.info or {}).get('app_base')
        if app_base:
            flash_base = 0x08000000 if app_base >= 0x08000000 else 0
            if flash_base <= m['pc'] < app_base:
                where = ' (bootloader)'
        parts = ['PC 0x%08X%s' % (m['pc'], where)]
        if m.get('speedup') is not None:
            parts.append('%.2fx realtime' % m['speedup'])
        if m.get('executed_mips') is not None:
            parts.append('%.0f of %u MIPS' % (m['executed_mips'], m['mips']))
        parts.append('vt %.1fs' % m['virtual_seconds'])
        return ' | '.join(parts)

    def _start_stub(self):
        import msp_stub_fc
        endpoint = None
        if self.conf == 'usb':
            import sitl_usbip
            # in direct mode the USB ids make the web configurator
            # treat the port as a single-wire adapter, not an FC
            ids = ({'vid': msp_stub_fc.DIRECT_VENDOR_ID,
                    'pid': msp_stub_fc.DIRECT_PRODUCT_ID}
                   if self.protocol == 'direct' else {})
            endpoint = sitl_usbip.UsbipServer(
                unix_path='@am32-renode-usbip.%u.%u' % (os.getuid(),
                                                        os.getpid()),
                serial='RENODE', **ids)
        if self.protocol == 'direct':
            self.stub = msp_stub_fc.DirectBridge(
                sitl_port=self.args.gui_port, endpoint=endpoint,
                verbose=False)
        else:
            self.stub = msp_stub_fc.MspStubFC(
                sitl_port=self.args.gui_port,
                state_port=self.args.state_port,
                motor=False, endpoint=endpoint, verbose=False)
        if self.conf == 'usb':
            import sitl_usbip
            if not sitl_usbip.attach(unix_path=endpoint.unix_path):
                raise RuntimeError('vhci attach refused (is vhci_hcd '
                                   'loaded, and can you become root?)')
            self.usb_attached = True
            tty = sitl_usbip.find_tty('RENODE', timeout=10)
            if tty is None:
                raise RuntimeError('attached but no tty appeared')
            self.conf_port = tty
        else:
            self.conf_port = self.stub.slave_path
        self.status = 'running - configurator port: %s' % self.conf_port
        self.log(self.status)

    def stop(self):
        self.generation += 1
        self.metrics = None
        if self.stub is not None:
            self.stub.close()
            self.stub = None
        if self.usb_attached:
            import sitl_usbip
            sitl_usbip.detach()
            self.usb_attached = False
        self.runner.stop()
        self.emulator_ready = False
        self.conf_port = ''
        self.status = 'stopped'

    def saw_log_line(self, line):
        if 'input port on udp' in line:
            self.emulator_ready = True
        if 'could not bind the input port' in line:
            # the machine keeps running without its ports; fail the
            # start promptly rather than waiting out the ready timeout
            self.status = 'emulator exited: ' + line.strip()
            self.start_failed = True
        if line.startswith('[emulator exited'):
            self.emulator_ready = False
            if self.status.startswith('running'):
                self.status = line[1:-1]


def run_control_server(lab, port, on_command):
    '''the scripted-test interface; on_command marshals a closure onto
    the UI thread and returns its reply'''
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', port))
    srv.listen(4)

    def client(conn):
        f = conn.makefile('rw')
        try:
            for line in f:
                reply = on_command(line.strip())
                f.write(reply + '\n')
                f.flush()
        except OSError:
            pass
        finally:
            conn.close()

    def loop():
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=client, args=(conn,),
                             daemon=True).start()

    threading.Thread(target=loop, daemon=True).start()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gui-port', type=int, default=57833,
                    help='emulator input port (default off the SITL\'s '
                         '57733, so both can run)')
    ap.add_argument('--state-port', type=int, default=57834)
    ap.add_argument('--monitor-port', type=int, default=57835,
                    help='Renode telnet monitor port, polled for the live '
                         'PC / speedup display')
    ap.add_argument('--bootloader-dir', default=None,
                    help='directory of bootloader ELFs (default: the '
                         'bootloader repo next to this one, or '
                         '$AM32_BOOTLOADER_OBJ)')
    ap.add_argument('--renode', default=None,
                    help='renode binary, passed through to gen_target')
    ap.add_argument('--renode-cache',
                    help='download cache (default: ~/.cache/ardupilot/renode)')
    ap.add_argument('--control-port', type=int, default=0,
                    help='TCP port for scripted UI control (default off)')
    args = ap.parse_args()

    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtWidgets import (QApplication, QComboBox, QFileDialog,
                                   QGridLayout, QLabel, QLineEdit,
                                   QPlainTextEdit, QPushButton, QSpinBox,
                                   QWidget)

    renode_cache = (Path(args.renode_cache).expanduser()
                    if args.renode_cache else default_renode_cache())

    app = QApplication(sys.argv)
    lab = Lab(args)
    win = QWidget()
    win.setWindowTitle('AM32 Renode ESC lab')
    grid = QGridLayout(win)

    # -- target --------------------------------------------------------
    grid.addWidget(QLabel('Target'), 0, 0)
    target_filter = QLineEdit()
    target_filter.setPlaceholderText('filter...')
    grid.addWidget(target_filter, 0, 1)
    target_combo = QComboBox()
    grid.addWidget(target_combo, 0, 2, 1, 2)
    info_label = QLabel('pick a target')
    grid.addWidget(info_label, 1, 2, 1, 2)

    all_targets = []

    def load_targets():
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, 'gen_target.py'), '--list'],
            capture_output=True, text=True)
        return [t for t in r.stdout.split() if t]

    def apply_filter():
        pat = target_filter.text().strip().upper()
        target_combo.blockSignals(True)
        target_combo.clear()
        target_combo.addItems([t for t in all_targets if pat in t])
        target_combo.blockSignals(False)
        if target_combo.count():
            target_combo.setCurrentIndex(0)
            target_changed()

    def target_changed():
        t = target_combo.currentText()
        if not t:
            return
        info_label.setText('resolving %s...' % t)

        def resolve():
            info = lab.target_info(t)
            lab.log_q.put(('__target__', t, info))
        threading.Thread(target=resolve, daemon=True).start()

    target_filter.textChanged.connect(apply_filter)
    target_combo.currentIndexChanged.connect(lambda _i: target_changed())

    # -- bootloader ----------------------------------------------------
    grid.addWidget(QLabel('Bootloader'), 2, 0)
    bl_combo = QComboBox()
    grid.addWidget(bl_combo, 2, 1, 1, 2)
    bl_browse = QPushButton('Browse...')
    grid.addWidget(bl_browse, 2, 3)

    def refresh_bootloaders():
        bl_combo.clear()
        hits = lab.matched_bootloaders()
        for h in hits:
            bl_combo.addItem(os.path.basename(h), h)
        bl_combo.addItem('None (boot straight into the app)', 'none')
        if not hits:
            bl_combo.setCurrentIndex(bl_combo.count() - 1)

    def bl_changed():
        data = bl_combo.currentData()
        lab.bootloader = data if data else 'auto'
    bl_combo.currentIndexChanged.connect(lambda _i: bl_changed())

    def browse_bl():
        path, _ = QFileDialog.getOpenFileName(
            win, 'Bootloader image',
            lab.bl_dirs[0] if lab.bl_dirs else REPO,
            'Bootloader (*.elf *.hex *.bin);;All files (*)')
        if path:
            bl_combo.insertItem(0, os.path.basename(path), path)
            bl_combo.setCurrentIndex(0)
    bl_browse.clicked.connect(browse_bl)

    # -- firmware / can ------------------------------------------------
    grid.addWidget(QLabel('Firmware'), 3, 0)
    fw_combo = QComboBox()
    fw_combo.addItem('Auto (newest obj/AM32_<TARGET>_*.elf)', 'auto')
    fw_combo.addItem('None (blank flash, factory-fresh ESC)', 'none')
    fw_combo.setToolTip(
        'None gives a part with only the bootloader: everything else\n'
        'reads erased 0xFF, as an ESC fresh from the factory does.')
    grid.addWidget(fw_combo, 3, 1, 1, 2)
    fw_browse = QPushButton('Browse...')
    grid.addWidget(fw_browse, 3, 3)

    def browse_fw():
        path, _ = QFileDialog.getOpenFileName(
            win, 'Firmware ELF', os.path.join(REPO, 'obj'), 'ELF (*.elf)')
        if path:
            fw_combo.insertItem(0, os.path.basename(path), path)
            fw_combo.setCurrentIndex(0)
    fw_browse.clicked.connect(browse_fw)

    grid.addWidget(QLabel('CAN bus'), 4, 0)
    can_spin = QSpinBox()
    can_spin.setRange(-1, 9)
    can_spin.setValue(8)
    # -1 leaves the mcast socket closed entirely, like a bench ESC with
    # no CAN cable: any traffic on a shared bus - even another rig's -
    # carries RawCommands that boot the app out from under a config
    # session
    can_spin.setSpecialValueText('off')
    can_spin.setToolTip('mcast bus number for DroneCAN targets '
                        '(239.65.82.N, as the SITL and dronecan_gui_tool '
                        'use); disabled for targets with no CAN.\n'
                        'Defaults off bus 0: CAN traffic from anything '
                        'else - an ArduPilot SITL, another bench rig - '
                        'makes the CAN bootloader boot the app instead '
                        'of waiting for the configurator. "off" leaves '
                        'the CAN unconnected entirely.')
    can_spin.setEnabled(False)
    grid.addWidget(can_spin, 4, 1)

    grid.addWidget(QLabel('EEPROM'), 5, 0)
    ee_combo = QComboBox()
    ee_combo.addItem('Defaults, tuned for the simulated motor', 'defaults')
    ee_combo.addItem('Blank (0xFF, factory-fresh ESC)', 'blank')
    ee_combo.setToolTip(
        'The settings area at the end of flash.\n'
        'Defaults: a generated eeprom tuned for the simulated motor.\n'
        'Blank: erased 0xFF, as a factory-fresh ESC ships - what a\n'
        'configurator sees before the first save.')
    grid.addWidget(ee_combo, 5, 1, 1, 2)

    # -- Renode download -----------------------------------------------
    grid.addWidget(QLabel('Renode'), 6, 0)
    renode_path = QLineEdit(args.renode or 'not selected')
    renode_path.setReadOnly(True)
    renode_path.setToolTip('Managed downloads are stored in %s' % renode_cache)
    grid.addWidget(renode_path, 6, 1, 1, 2)
    download_renode = QPushButton('Download Renode')
    grid.addWidget(download_renode, 6, 3)

    # -- configurator port ---------------------------------------------
    grid.addWidget(QLabel('Configurator'), 7, 0)
    conf_combo = QComboBox()
    conf_combo.addItem('Serial port (pty)', 'serial')
    if sys.platform.startswith('linux'):
        conf_combo.addItem('USB device (vhci, for the browser)', 'usb')
    conf_combo.addItem('Off (drive it some other way)', 'off')
    conf_combo.setToolTip(
        'How the configurator reaches the emulated ESC.\n'
        'A pty works for desktop tools; the USB device is a real\n'
        '/dev/ttyACM* Chrome can open, so am32.tridgell.net works.\n'
        'Attaching the USB device asks for root unless the udev rule\n'
        'from sitl_usbip.py --install-rules is in place.')
    grid.addWidget(conf_combo, 7, 1, 1, 2)

    # -- protocol: what sits on that port ------------------------------
    grid.addWidget(QLabel('Protocol'), 8, 0)
    proto_combo = QComboBox()
    proto_combo.addItem('FC with 4-way passthrough', '4way')
    proto_combo.addItem('Direct 1-wire adapter', 'direct')
    proto_combo.setToolTip(
        'What the configurator port pretends to be.\n'
        'FC: MSP with BLHeli 4-way passthrough, as a real flight\n'
        'controller provides.\n'
        'Direct: a single-wire adapter soldered to the signal pad -\n'
        'the raw 19200 baud bootloader protocol, with the self-echo\n'
        'such an adapter produces. The web configurator decides\n'
        'FC-vs-adapter by USB vendor id, so the USB device enumerates\n'
        'accordingly; the Offline-Configurator uses its direct/1-wire\n'
        'checkbox on the pty or tty.')
    grid.addWidget(proto_combo, 8, 1, 1, 2)

    # -- start/stop, status, log ---------------------------------------
    start_btn = QPushButton('Start')
    stop_btn = QPushButton('Stop')
    stop_btn.setEnabled(False)
    grid.addWidget(start_btn, 9, 2)
    grid.addWidget(stop_btn, 9, 3)
    status_label = QLabel('stopped')
    status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
    grid.addWidget(status_label, 9, 0, 1, 2)
    metrics_label = QLabel('')
    metrics_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
    metrics_label.setToolTip(
        'Live from the Renode monitor: current PC (labelled when it is\n'
        'executing the bootloader), emulation speed against real time,\n'
        'instructions actually retired per wall second against the\n'
        "configured PerformanceInMips, and the machine's virtual time.")
    grid.addWidget(metrics_label, 10, 0, 1, 4)
    log_view = QPlainTextEdit()
    log_view.setReadOnly(True)
    log_view.setMaximumBlockCount(2000)
    log_view.setMinimumSize(640, 240)
    grid.addWidget(log_view, 11, 0, 1, 4)

    download_active = False

    def renode_version(latest):
        return '%s (%s)' % (latest.get('renode_version', '?'),
                            latest['source']['revision'][:9])

    def check_renode_download():
        try:
            latest = fetch_renode_latest()
            package = select_renode_package(latest)
            current = cached_renode(renode_cache, latest, package)
            lab.log_q.put(('__renode_check__', current, latest))
        except (OSError, RuntimeError, ValueError,
                json.JSONDecodeError) as error:
            lab.log_q.put(('__renode_download_error__',
                           'update check failed: %s' % error))

    def start_renode_download():
        nonlocal download_active
        if download_active:
            return False
        download_active = True
        download_renode.setEnabled(False)
        download_renode.setText('Checking...')

        def progress(received, total):
            lab.log_q.put(('__renode_download_progress__', received, total))

        def worker():
            try:
                executable, latest, downloaded = install_current_renode(
                    renode_cache, progress=progress)
                lab.log_q.put(('__renode_download_done__',
                               executable, latest, downloaded))
            except (OSError, RuntimeError, ValueError, tarfile.TarError,
                    zipfile.BadZipFile, json.JSONDecodeError) as error:
                lab.log_q.put(('__renode_download_error__', str(error)))

        threading.Thread(target=worker, daemon=True).start()
        return True

    download_renode.clicked.connect(start_renode_download)

    def do_start():
        lab.firmware = fw_combo.currentData() or 'auto'
        lab.eeprom = ee_combo.currentData()
        lab.conf = conf_combo.currentData()
        lab.protocol = proto_combo.currentData()
        lab.can_bus = can_spin.value()
        err = lab.start()
        if err:
            # also into lab.status, which the control port reports
            lab.status = err
            status_label.setText(err)
            return
        start_btn.setEnabled(False)
        stop_btn.setEnabled(True)

    def do_stop():
        lab.stop()
        start_btn.setEnabled(True)
        stop_btn.setEnabled(False)
        status_label.setText(lab.status)

    start_btn.clicked.connect(do_start)
    stop_btn.clicked.connect(do_stop)

    def drain_log():
        nonlocal download_active
        lines = []
        while True:
            try:
                item = lab.log_q.get_nowait()
            except queue.Empty:
                break
            if (isinstance(item, tuple) and
                    item[0].startswith('__renode_download')):
                tag = item[0]
                if tag == '__renode_download_progress__':
                    _tag, received, total = item
                    percent = min(100, received * 100 // max(total, 1))
                    download_renode.setText('Downloading %u%%' % percent)
                elif tag == '__renode_download_done__':
                    _tag, executable, latest, downloaded = item
                    download_active = False
                    args.renode = str(executable)
                    renode_path.setText(args.renode)
                    download_renode.setText('Renode current')
                    download_renode.setEnabled(True)
                    action = 'downloaded' if downloaded else 'using cached'
                    lines.append('[renode] %s %s: %s' %
                                 (action, renode_version(latest), executable))
                elif tag == '__renode_download_error__':
                    download_active = False
                    download_renode.setText('Retry Renode download')
                    download_renode.setEnabled(True)
                    lines.append('[renode] %s' % item[1])
                continue
            if isinstance(item, tuple) and item[0] == '__renode_check__':
                _tag, current, latest = item
                if current is None:
                    label = ('Update Renode' if cached_renode(renode_cache)
                             else 'Download Renode')
                    download_renode.setText(label)
                    download_renode.setToolTip(
                        'Current server version: %s' % renode_version(latest))
                else:
                    if args.renode is None:
                        args.renode = str(current)
                        renode_path.setText(args.renode)
                    selected = (args.renode and
                                Path(args.renode).expanduser() == current)
                    download_renode.setText(
                        'Renode current' if selected else 'Use cached current')
                    download_renode.setToolTip(
                        'Cached server version: %s' % renode_version(latest))
                continue
            if isinstance(item, tuple) and item[0] == '__metrics__':
                if item[1] == lab.generation:
                    lab.metrics = item[2]
                continue
            if isinstance(item, tuple) and item[0] == '__monitor_error__':
                if item[1] == lab.generation:
                    lines.append('[monitor] %s' % item[2])
                continue
            if isinstance(item, tuple) and item[0] == '__target__':
                _tag, t, info = item
                if t != target_combo.currentText():
                    continue
                lab.target, lab.info = t, info
                if info is None:
                    info_label.setText('%s: unsupported' % t)
                    continue
                info_label.setText('%s: %s, signal pin %s%s'
                                   % (t, info['family'].upper(), info['pin'],
                                      ', DroneCAN' if info['dronecan']
                                      else ''))
                can_spin.setEnabled(bool(info['dronecan']))
                refresh_bootloaders()
                continue
            lab.saw_log_line(item)
            lines.append(item)
        if lines:
            log_view.appendPlainText('\n'.join(lines))
        status_label.setText(lab.status)
        metrics_label.setText(lab.format_metrics())

    timer = QTimer()
    timer.timeout.connect(drain_log)
    timer.start(150)

    # -- control port --------------------------------------------------
    pending = queue.Queue()

    def on_command(line):
        done = queue.Queue()
        pending.put((line, done))
        try:
            return done.get(timeout=30)
        except queue.Empty:
            return 'ERR timeout'

    def poll_pending():
        try:
            line, done = pending.get_nowait()
        except queue.Empty:
            return
        done.put(handle_command(line))

    def handle_command(line):
        parts = line.split(None, 1)
        if not parts:
            return 'ERR empty'
        cmd, rest = parts[0], parts[1] if len(parts) > 1 else ''
        if cmd == 'target':
            target_filter.setText(rest)
            if target_combo.findText(rest) < 0:
                return 'ERR no target %s' % rest
            target_combo.setCurrentIndex(target_combo.findText(rest))
            # wait for the async resolve
            deadline = time.time() + 60
            while time.time() < deadline and lab.target != rest:
                drain_log()
                time.sleep(0.2)
            return 'OK' if lab.target == rest else 'ERR resolve timeout'
        if cmd == 'bootloader':
            lab.bootloader = rest or 'auto'
            return 'OK'
        if cmd == 'firmware':
            if rest in ('', 'auto', 'none'):
                fw_combo.setCurrentIndex(fw_combo.findData(rest or 'auto'))
            else:
                fw_combo.insertItem(0, os.path.basename(rest), rest)
                fw_combo.setCurrentIndex(0)
            return 'OK'
        if cmd == 'eeprom':
            i = ee_combo.findData(rest)
            if i < 0:
                return 'ERR eeprom defaults|blank'
            ee_combo.setCurrentIndex(i)
            return 'OK'
        if cmd == 'conf':
            i = conf_combo.findData(rest)
            if i < 0:
                return 'ERR conf off|serial|usb'
            conf_combo.setCurrentIndex(i)
            return 'OK'
        if cmd == 'protocol':
            i = proto_combo.findData(rest)
            if i < 0:
                return 'ERR protocol 4way|direct'
            proto_combo.setCurrentIndex(i)
            return 'OK'
        if cmd == 'canbus':
            can_spin.setValue(int(rest))
            return 'OK'
        if cmd == 'download-renode':
            return ('OK' if start_renode_download()
                    else 'ERR download in progress')
        if cmd == 'start':
            do_start()
            return 'OK' if not start_btn.isEnabled() else ('ERR ' + lab.status)
        if cmd == 'stop':
            do_stop()
            return 'OK'
        if cmd == 'status':
            extra = lab.format_metrics()
            return 'STATUS %s | port=%s | emulator=%s%s' % (
                lab.status, lab.conf_port,
                'up' if lab.emulator_ready else 'down',
                ' | ' + extra if extra else '')
        if cmd == 'quit':
            QTimer.singleShot(100, app.quit)
            return 'OK'
        return 'ERR unknown %s' % cmd

    if args.control_port:
        run_control_server(lab, args.control_port, on_command)
        ptimer = QTimer()
        ptimer.timeout.connect(poll_pending)
        ptimer.start(50)

    # populate targets in the background so the window opens instantly
    def targets_thread():
        found = load_targets()
        lab.log_q.put('%u targets' % len(found))
        all_targets.extend(found)
        # applying the filter must run on the Qt thread
        QTimer.singleShot(0, apply_filter)
    threading.Thread(target=targets_thread, daemon=True).start()
    threading.Thread(target=check_renode_download, daemon=True).start()

    signal.signal(signal.SIGINT, lambda *a: app.quit())
    signal.signal(signal.SIGTERM, lambda *a: app.quit())
    win.show()
    try:
        app.exec()
    finally:
        lab.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
