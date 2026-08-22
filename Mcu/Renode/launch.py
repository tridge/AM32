#!/usr/bin/env python3
'''
Renode ESC lab: pick a hardware target, a bootloader and a firmware,
press Start, and get an emulated ESC a configurator can talk to.

The emulator runs gen_target.py TARGET --link, serving the SITL wire
protocols. The configurator port is Mcu/SITL's fake flight controller on
those ports: MSP with BLHeli 4-way passthrough to the emulated
bootloader, on a pty or - so a browser can reach it - on a virtual USB
serial device attached through vhci_hcd. This is the rig for developing
am32.tridgell.net against emulated CAN and non-CAN ESCs with no
hardware.

Bootloaders are matched to the target automatically: the ELF must be
built for the target's signal pin (a PA2 bootloader on a PB4 target
answers nothing), so the list only offers AM32_<MCU>_BOOTLOADER_<PIN>*
builds from the bootloader repo's obj directory.

with --control-port N the UI can be driven over a localhost TCP
connection (one command per line), for scripted tests:
  target NAME, bootloader auto|none|PATH, firmware auto|PATH,
  conf off|serial|usb, canbus N, start, stop, status, quit
replies are prefixed OK/ERR/STATUS.
'''

import argparse
import glob
import json
import os
import queue
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

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
    '''bootloader ELFs built for this target's MCU and signal pin.

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
        pat = os.path.join(d, 'AM32_%s_BOOTLOADER_%s*_V*.elf' % (mcu, pin))
        hits += glob.glob(pat)
    # newest version of each variant only
    byvar = {}
    for h in sorted(hits):
        var = re.sub(r'_V\d+\.elf$', '', os.path.basename(h))
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
        self.firmware = 'auto'            # auto | path
        self.conf = 'serial'              # off | serial | usb
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
    def wait_port_free(port, timeout=8.0):
        '''wait for a UDP port to be bindable; the previous emulator's
        teardown can outlive the Stop click by a moment'''
        deadline = time.time() + timeout
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
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
        bl = self.pick_bootloader()
        if isinstance(bl, str) and not os.path.isfile(bl):
            return bl
        cmd = [sys.executable, os.path.join(HERE, 'gen_target.py'),
               self.target, '--link',
               '--gui-port', str(self.args.gui_port),
               '--gui-state-port', str(self.args.state_port)]
        if bl is not None:
            cmd += ['--bootloader-elf', bl]
        if self.firmware != 'auto':
            if not os.path.isfile(self.firmware):
                return 'no firmware at %s' % self.firmware
            cmd += ['--elf', self.firmware]
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

    def _start_stub(self):
        import msp_stub_fc
        endpoint = None
        if self.conf == 'usb':
            import sitl_usbip
            endpoint = sitl_usbip.UsbipServer(
                unix_path='@am32-renode-usbip.%u.%u' % (os.getuid(),
                                                        os.getpid()),
                serial='RENODE')
        self.stub = msp_stub_fc.MspStubFC(
            sitl_port=self.args.gui_port, state_port=self.args.state_port,
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
    ap.add_argument('--bootloader-dir', default=None,
                    help='directory of bootloader ELFs (default: the '
                         'bootloader repo next to this one, or '
                         '$AM32_BOOTLOADER_OBJ)')
    ap.add_argument('--renode', default=None,
                    help='renode binary, passed through to gen_target')
    ap.add_argument('--control-port', type=int, default=0,
                    help='TCP port for scripted UI control (default off)')
    args = ap.parse_args()

    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtWidgets import (QApplication, QComboBox, QFileDialog,
                                   QGridLayout, QLabel, QLineEdit,
                                   QPlainTextEdit, QPushButton, QSpinBox,
                                   QWidget)

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
            win, 'Bootloader ELF',
            lab.bl_dirs[0] if lab.bl_dirs else REPO, 'ELF (*.elf)')
        if path:
            bl_combo.insertItem(0, os.path.basename(path), path)
            bl_combo.setCurrentIndex(0)
    bl_browse.clicked.connect(browse_bl)

    # -- firmware / can ------------------------------------------------
    grid.addWidget(QLabel('Firmware'), 3, 0)
    fw_edit = QLineEdit()
    fw_edit.setPlaceholderText('auto: newest obj/AM32_<TARGET>_*.elf')
    grid.addWidget(fw_edit, 3, 1, 1, 2)
    fw_browse = QPushButton('Browse...')
    grid.addWidget(fw_browse, 3, 3)

    def browse_fw():
        path, _ = QFileDialog.getOpenFileName(
            win, 'Firmware ELF', os.path.join(REPO, 'obj'), 'ELF (*.elf)')
        if path:
            fw_edit.setText(path)
    fw_browse.clicked.connect(browse_fw)

    grid.addWidget(QLabel('CAN bus'), 4, 0)
    can_spin = QSpinBox()
    can_spin.setRange(0, 9)
    can_spin.setValue(8)
    can_spin.setToolTip('mcast bus number for DroneCAN targets '
                        '(239.65.82.N, as the SITL and dronecan_gui_tool '
                        'use); disabled for targets with no CAN.\n'
                        'Defaults off bus 0: CAN traffic from anything '
                        'else - an ArduPilot SITL, another bench rig - '
                        'makes the CAN bootloader boot the app instead '
                        'of waiting for the configurator.')
    can_spin.setEnabled(False)
    grid.addWidget(can_spin, 4, 1)

    # -- configurator port ---------------------------------------------
    grid.addWidget(QLabel('Configurator'), 5, 0)
    conf_combo = QComboBox()
    conf_combo.addItem('Serial port (pty)', 'serial')
    if sys.platform.startswith('linux'):
        conf_combo.addItem('USB device (vhci, for the browser)', 'usb')
    conf_combo.addItem('Off (drive it some other way)', 'off')
    conf_combo.setToolTip(
        'The fake flight controller in front of the emulated ESC:\n'
        'MSP plus BLHeli 4-way passthrough, as a real FC provides.\n'
        'A pty works for desktop tools; the USB device is a real\n'
        '/dev/ttyACM* Chrome can open, so am32.tridgell.net works.\n'
        'Attaching the USB device asks for root.')
    grid.addWidget(conf_combo, 5, 1, 1, 2)

    # -- start/stop, status, log ---------------------------------------
    start_btn = QPushButton('Start')
    stop_btn = QPushButton('Stop')
    stop_btn.setEnabled(False)
    grid.addWidget(start_btn, 6, 2)
    grid.addWidget(stop_btn, 6, 3)
    status_label = QLabel('stopped')
    status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
    grid.addWidget(status_label, 6, 0, 1, 2)
    log_view = QPlainTextEdit()
    log_view.setReadOnly(True)
    log_view.setMaximumBlockCount(2000)
    log_view.setMinimumSize(640, 240)
    grid.addWidget(log_view, 7, 0, 1, 4)

    def do_start():
        lab.firmware = fw_edit.text().strip() or 'auto'
        lab.conf = conf_combo.currentData()
        lab.can_bus = can_spin.value()
        err = lab.start()
        if err:
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
        lines = []
        while True:
            try:
                item = lab.log_q.get_nowait()
            except queue.Empty:
                break
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
            fw_edit.setText('' if rest in ('', 'auto') else rest)
            return 'OK'
        if cmd == 'conf':
            i = conf_combo.findData(rest)
            if i < 0:
                return 'ERR conf off|serial|usb'
            conf_combo.setCurrentIndex(i)
            return 'OK'
        if cmd == 'canbus':
            can_spin.setValue(int(rest))
            return 'OK'
        if cmd == 'start':
            do_start()
            return 'OK' if not start_btn.isEnabled() else ('ERR ' + lab.status)
        if cmd == 'stop':
            do_stop()
            return 'OK'
        if cmd == 'status':
            return 'STATUS %s | port=%s | emulator=%s' % (
                lab.status, lab.conf_port,
                'up' if lab.emulator_ready else 'down')
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
