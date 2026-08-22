'''
virtual USB CDC-ACM serial device, exported over USB/IP

The SITL is reachable over UDP, but a browser cannot open a UDP socket
and Chrome's Web Serial only lists devices the kernel enumerated. This
serves a simulated USB serial adapter over the USB/IP protocol; with the
Linux vhci_hcd driver attached to it the device appears as a real
/dev/ttyACM* under /dev/serial/by-id, so any host tool - the web
configurator included - can talk to whatever is on the other end of the
byte stream (msp_stub_fc.py, which is a flight controller as far as a
configurator is concerned).

    python3 Mcu/SITL/msp_stub_fc.py --usbip --attach --no-motor

vhci_hcd is handed a socket to speak USB/IP over, and does not care what
kind: a unix socket works as well as TCP and is the default here, so
nothing has to claim a port or be reachable from the network. TCP
(--usbip-port) is for a client on another machine, or one that can only
attach the usbip way.

The device enumerates as pid.codes 1209:0001 (their test ID, which the
AM32 configurator accepts as a flight controller) with a CDC-ACM
interface: one bulk pair carrying the serial bytes and an unused
interrupt endpoint for the notifications a real ACM device would send.

USB/IP protocol: an op phase (OP_REQ_DEVLIST / OP_REQ_IMPORT, both
answered from the descriptors below) followed by URB submissions,
48 byte big endian headers with the transfer buffer appended. Control
transfers on endpoint 0 are answered from the descriptor tables, bulk
OUT bytes go to the byte stream, and bulk IN submissions are held
pending until there are bytes to complete them with - which is exactly
what a real device does with a queued read.
'''

import argparse
import errno
import glob
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

USBIP_VERSION = 0x0111

OP_REQ_DEVLIST = 0x8005
OP_REP_DEVLIST = 0x0005
OP_REQ_IMPORT = 0x8003
OP_REP_IMPORT = 0x0003

CMD_SUBMIT = 1
CMD_UNLINK = 2
RET_SUBMIT = 3
RET_UNLINK = 4

DIR_OUT = 0
DIR_IN = 1

SPEED_FULL = 2

# endpoints, matching the descriptors below
EP_BULK = 1
EP_INTR = 2

ST_OK = 0
ST_STALL = -errno.EPIPE
ST_UNLINKED = -errno.ECONNRESET

# USB requests
REQ_GET_STATUS = 0x00
REQ_CLEAR_FEATURE = 0x01
REQ_SET_FEATURE = 0x03
REQ_SET_ADDRESS = 0x05
REQ_GET_DESCRIPTOR = 0x06
REQ_GET_CONFIGURATION = 0x08
REQ_SET_CONFIGURATION = 0x09
REQ_GET_INTERFACE = 0x0A
REQ_SET_INTERFACE = 0x0B
# CDC class requests
REQ_SET_LINE_CODING = 0x20
REQ_GET_LINE_CODING = 0x21
REQ_SET_CONTROL_LINE_STATE = 0x22

VENDOR_ID = 0x1209
PRODUCT_ID = 0x0001
BUSID = '1-1'
BUSNUM = 1
DEVNUM = 1

VHCI = '/sys/devices/platform/vhci_hcd.0'
VDEV_ST_NULL = '004'      # a free port in the status table

DEVICE_DESCRIPTOR = struct.pack(
    '<BBHBBBBHHHBBBB',
    18, 1, 0x0200,      # bLength, DEVICE, bcdUSB 2.00
    0x02, 0x00, 0x00,   # class CDC, no subclass or protocol
    64,                 # bMaxPacketSize0
    VENDOR_ID, PRODUCT_ID, 0x0100,
    1, 2, 3,            # iManufacturer, iProduct, iSerialNumber
    1)                  # bNumConfigurations

CONFIG_DESCRIPTOR = b''.join([
    struct.pack('<BBHBBBBB', 9, 2, 67, 2, 1, 0, 0xC0, 50),
    # communication interface, one interrupt endpoint
    struct.pack('<BBBBBBBBB', 9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0),
    bytes([5, 0x24, 0x00, 0x10, 0x01]),        # CDC header
    bytes([5, 0x24, 0x01, 0x00, 0x01]),        # call management
    bytes([4, 0x24, 0x02, 0x02]),              # ACM, supports line coding
    bytes([5, 0x24, 0x06, 0x00, 0x01]),        # union: comm 0, data 1
    struct.pack('<BBBBHB', 7, 5, 0x80 | EP_INTR, 0x03, 8, 255),
    # data interface, the bulk pair carrying the serial bytes
    struct.pack('<BBBBBBBBB', 9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0),
    struct.pack('<BBBBHB', 7, 5, EP_BULK, 0x02, 64, 0),
    struct.pack('<BBBBHB', 7, 5, 0x80 | EP_BULK, 0x02, 64, 0),
])

MANUFACTURER = 'AM32'
PRODUCT = 'AM32 SITL serial'
DEFAULT_SERIAL = 'SITL'


def tty_glob(serial=DEFAULT_SERIAL):
    '''udev names the link after the manufacturer, product and serial'''
    name = ('%s %s %s' % (MANUFACTURER, PRODUCT, serial)).replace(' ', '_')
    return '/dev/serial/by-id/usb-%s-if00' % name


def find_tty(serial=DEFAULT_SERIAL, timeout=10.0):
    '''wait for the attached device to show up as a serial port'''
    deadline = time.time() + timeout
    while True:
        hits = sorted(glob.glob(tty_glob(serial)))
        if hits:
            return hits[0]
        if time.time() >= deadline:
            return None
        time.sleep(0.2)


def default_socket_name():
    '''an abstract socket by default: it lives in the kernel's namespace
    rather than the filesystem, so a killed run leaves nothing behind
    and there is no stale socket to clear out. Per uid, since the
    abstract namespace is shared by everyone in the network namespace'''
    return '@am32-sitl-usbip.%u' % os.getuid()


def socket_address(name):
    '''bind/connect address for a socket name; a leading @ selects the
    abstract namespace (the kernel's own notation is a leading NUL)'''
    return '\0' + name[1:] if name.startswith('@') else name


class UsbipServer(object):
    '''USB/IP server exporting one CDC-ACM device.

    read()/write() are the device end of the serial link: what the host
    writes to the tty arrives from read(), what write() is given is what
    the host reads back.
    '''

    def __init__(self, unix_path=None, host='127.0.0.1', port=None,
                 serial=DEFAULT_SERIAL, log=None, rx_max=65536):
        self.log = log or (lambda s: None)
        self.rx_max = rx_max
        self.serial = serial
        self.strings = [MANUFACTURER, PRODUCT, serial]
        self.rx = b''
        self.rx_lock = threading.Condition()
        self.tx_held = b''       # bytes with no urb to carry them yet
        self.pending = []        # bulk IN urbs waiting for data
        self.intr = []           # interrupt IN urbs, never completed
        self.send_lock = threading.Lock()
        self.conn = None
        self.running = True
        self.attached = threading.Event()
        self.line_coding = struct.pack('<IBBB', 115200, 0, 0, 8)
        # a unix socket by default: vhci_hcd is given the connected
        # socket, so nothing needs a port or has to be on the network
        self.unix_path = None if port is not None else (
            unix_path or default_socket_name())
        self.host = host
        self.port = port
        if self.unix_path is not None:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            if not self.unix_path.startswith('@'):
                if os.path.exists(self.unix_path):
                    # a leftover from a killed run; a live one would
                    # have failed the bind below anyway
                    os.unlink(self.unix_path)
            try:
                self.sock.bind(socket_address(self.unix_path))
            except OSError as ex:
                raise OSError('cannot export USB/IP on %s (%s), another '
                              'instance is probably running'
                              % (self.unix_path, ex))
            if not self.unix_path.startswith('@'):
                # a filesystem socket can be locked down to us; an
                # abstract one is reachable by anyone in the network
                # namespace, like the SITL's own udp ports
                os.chmod(self.unix_path, 0o600)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.sock.bind((host, port))
            except OSError as ex:
                # 3240 is the well known USB/IP port, so usbipd or
                # another exporter may already have it
                raise OSError('cannot serve USB/IP on %s:%u (%s), try '
                              'another port' % (host, port, ex))
            self.port = self.sock.getsockname()[1]
        self.sock.listen(1)
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    @property
    def endpoint(self):
        '''where a client attaches to us'''
        if self.unix_path is not None:
            return self.unix_path
        return '%s:%u' % (self.host, self.port)

    @property
    def path(self):
        '''the serial device the host sees, once it has attached us'''
        hits = sorted(glob.glob(tty_glob(self.serial)))
        return hits[0] if hits else '%s (not attached)' % self.endpoint

    # -- serial side ---------------------------------------------------

    def read(self, timeout=0.1):
        '''bytes the host wrote, or b'' if none arrive within timeout'''
        with self.rx_lock:
            if not self.rx:
                self.rx_lock.wait(timeout)
            out, self.rx = self.rx, b''
            return out

    def drain(self):
        '''whatever has already arrived, without waiting'''
        with self.rx_lock:
            out, self.rx = self.rx, b''
            return out

    def write(self, data):
        '''send bytes to the host, completing its queued read urbs'''
        data = bytes(data)
        while data:
            with self.send_lock:
                if not self.pending:
                    # no reader: a real device NAKs until the host asks
                    # again, so hold the bytes for the next urb
                    self.tx_held += data
                    return
                seqnum, length = self.pending.pop(0)
                chunk, data = data[:length], data[length:]
                self._ret_submit(seqnum, ST_OK, chunk)

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        if self.unix_path is not None and not self.unix_path.startswith('@'):
            try:
                os.unlink(self.unix_path)
            except OSError:
                pass
        with self.send_lock:
            if self.conn is not None:
                try:
                    self.conn.close()
                except OSError:
                    pass

    # -- USB/IP --------------------------------------------------------

    def _serve(self):
        while self.running:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            self.log('connection from %s' % (addr or self.endpoint,))
            self.conn = conn
            try:
                self._session(conn)
            except (OSError, struct.error) as ex:
                self.log('session ended: %s' % ex)
            finally:
                self.attached.clear()
                with self.send_lock:
                    self.conn = None
                    self.pending = []
                    self.intr = []
                    self.tx_held = b''
                try:
                    conn.close()
                except OSError:
                    pass
                self.log('detached')

    def _recv(self, conn, n):
        out = b''
        while len(out) < n:
            b = conn.recv(n - len(out))
            if not b:
                raise OSError('connection closed')
            out += b
        return out

    def _session(self, conn):
        # op phase: the client either lists our devices or imports one
        while True:
            head = self._recv(conn, 8)
            version, code, _status = struct.unpack('>HHI', head)
            if code == OP_REQ_DEVLIST:
                conn.sendall(struct.pack('>HHII', USBIP_VERSION,
                                         OP_REP_DEVLIST, 0, 1)
                             + self._usb_device() + self._usb_interfaces())
                continue
            if code != OP_REQ_IMPORT:
                self.log('unexpected op code 0x%04x' % code)
                return
            busid = self._recv(conn, 32).split(b'\0')[0].decode()
            if busid != BUSID:
                conn.sendall(struct.pack('>HHI', USBIP_VERSION,
                                         OP_REP_IMPORT, 1))
                return
            conn.sendall(struct.pack('>HHI', USBIP_VERSION, OP_REP_IMPORT, 0)
                         + self._usb_device())
            break

        self.log('attached, device is now enumerating')
        self.attached.set()
        # urb phase
        while self.running:
            hdr = self._recv(conn, 48)
            command, seqnum, _devid, direction, ep = struct.unpack('>IIIII',
                                                                   hdr[:20])
            if command == CMD_SUBMIT:
                (_flags, length, _start, _npkt, _interval) = struct.unpack(
                    '>Iiiii', hdr[20:40])
                setup = hdr[40:48]
                data = self._recv(conn, length) if direction == DIR_OUT and length else b''
                self._submit(seqnum, direction, ep, length, setup, data)
            elif command == CMD_UNLINK:
                victim = struct.unpack('>I', hdr[20:24])[0]
                self._unlink(seqnum, victim)
            else:
                self.log('unknown usbip command %u' % command)
                return

    def _usb_device(self):
        path = '/sys/devices/platform/vhci_hcd.0/usb%u/%s' % (BUSNUM, BUSID)
        return struct.pack('>256s32sIIIHHHBBBBBB',
                           path.encode(), BUSID.encode(),
                           BUSNUM, DEVNUM, SPEED_FULL,
                           VENDOR_ID, PRODUCT_ID, 0x0100,
                           0x02, 0x00, 0x00,   # device class/subclass/proto
                           1, 1, 2)            # config value, configs, ifaces

    @staticmethod
    def _usb_interfaces():
        return (bytes([0x02, 0x02, 0x01, 0]) +   # communication
                bytes([0x0A, 0x00, 0x00, 0]))    # data

    def _ret_submit(self, seqnum, status, data=b''):
        '''caller must hold send_lock'''
        hdr = (struct.pack('>IIIII', RET_SUBMIT, seqnum, 0, 0, 0)
               + struct.pack('>iiiii8s', status, len(data), 0, 0, 0, b''))
        conn = self.conn
        if conn is None:
            return
        try:
            conn.sendall(hdr + data)
        except OSError as ex:
            self.log('send failed: %s' % ex)

    def _submit(self, seqnum, direction, ep, length, setup, data):
        if ep == 0:
            with self.send_lock:
                status, reply = self._control(setup, data, length)
                self._ret_submit(seqnum, status, reply)
            return

        if ep == EP_INTR:
            # the notification endpoint: nothing ever happens on it, so
            # the urb stays queued until the host unlinks it
            with self.send_lock:
                self.intr.append(seqnum)
            return

        if ep != EP_BULK:
            with self.send_lock:
                self._ret_submit(seqnum, ST_STALL)
            return

        if direction == DIR_OUT:
            with self.rx_lock:
                if len(self.rx) + len(data) <= self.rx_max:
                    self.rx += data
                else:
                    self.log('rx overflow, dropping %u bytes' % len(data))
                self.rx_lock.notify_all()
            with self.send_lock:
                self._ret_submit(seqnum, ST_OK)
            return

        # a read: complete it now if we are holding bytes, else queue it
        with self.send_lock:
            if self.tx_held:
                chunk = self.tx_held[:length]
                self.tx_held = self.tx_held[length:]
                self._ret_submit(seqnum, ST_OK, chunk)
            else:
                self.pending.append((seqnum, length))

    def _unlink(self, seqnum, victim):
        with self.send_lock:
            found = False
            for i, (sq, _len) in enumerate(self.pending):
                if sq == victim:
                    del self.pending[i]
                    found = True
                    break
            if not found and victim in self.intr:
                self.intr.remove(victim)
                found = True
            hdr = (struct.pack('>IIIII', RET_UNLINK, seqnum, 0, 0, 0)
                   + struct.pack('>i24s', ST_UNLINKED if found else 0, b''))
            if self.conn is not None:
                try:
                    self.conn.sendall(hdr)
                except OSError:
                    pass

    def _string_descriptor(self, index):
        if index == 0:
            return bytes([4, 3, 0x09, 0x04])       # US English
        if index > len(self.strings):
            return None
        body = self.strings[index - 1].encode('utf-16-le')
        return bytes([len(body) + 2, 3]) + body

    def _control(self, setup, data, length):
        '''answer a control transfer, returning (status, reply bytes)'''
        rtype, request, value, index, wlength = struct.unpack('<BBHHH', setup)
        recipient_std = (rtype & 0x60) == 0
        if recipient_std and request == REQ_GET_DESCRIPTOR:
            dtype, dindex = value >> 8, value & 0xFF
            if dtype == 1:
                return ST_OK, DEVICE_DESCRIPTOR[:wlength]
            if dtype == 2:
                return ST_OK, CONFIG_DESCRIPTOR[:wlength]
            if dtype == 3:
                desc = self._string_descriptor(dindex)
                if desc is None:
                    return ST_STALL, b''
                return ST_OK, desc[:wlength]
            # device qualifier, other speed config, BOS: full speed only
            return ST_STALL, b''
        if recipient_std and request in (REQ_SET_CONFIGURATION,
                                         REQ_SET_INTERFACE,
                                         REQ_SET_ADDRESS,
                                         REQ_CLEAR_FEATURE,
                                         REQ_SET_FEATURE):
            return ST_OK, b''
        if recipient_std and request == REQ_GET_CONFIGURATION:
            return ST_OK, bytes([1])
        if recipient_std and request == REQ_GET_INTERFACE:
            return ST_OK, bytes([0])
        if recipient_std and request == REQ_GET_STATUS:
            return ST_OK, bytes([0, 0])[:wlength]
        # CDC class requests on the communication interface
        if request == REQ_SET_LINE_CODING:
            if len(data) >= 7:
                self.line_coding = data[:7]
            return ST_OK, b''
        if request == REQ_GET_LINE_CODING:
            return ST_OK, self.line_coding[:wlength]
        if request == REQ_SET_CONTROL_LINE_STATE:
            self.log('DTR=%u RTS=%u' % (value & 1, (value >> 1) & 1))
            return ST_OK, b''
        self.log('unhandled control 0x%02x/0x%02x' % (rtype, request))
        return ST_STALL, b''


def _free_vhci_port(speed):
    """a port of the right hub in the vhci status table, or None.

    columns are hub, port, sta, spd, dev, sockfd, local_busid
    """
    want_hs = speed != 5      # only super speed lives on the ss hub
    for line in open(os.path.join(VHCI, 'status')).read().splitlines()[1:]:
        f = line.split()
        if len(f) >= 3 and f[2] == VDEV_ST_NULL and (f[0] == 'hs') == want_hs:
            return int(f[1])
    return None


def attach_socket(sock, devid, speed):
    """hand a connected socket to vhci_hcd, which then enumerates the
    device on it. Needs root, and takes over the socket: the kernel
    keeps its own reference, so we can exit afterwards"""
    port = _free_vhci_port(speed)
    if port is None:
        raise OSError('no free vhci_hcd port, detach something first')
    with open(os.path.join(VHCI, 'attach'), 'w') as f:
        f.write('%u %u %u %u' % (port, sock.fileno(), devid, speed))
    return port


def import_device(unix_path=None, host='127.0.0.1', port=3240, busid=BUSID):
    """connect to an exporter and import a device, returning the socket
    (positioned at the start of the urb phase) with its devid and speed"""
    if unix_path is not None:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(socket_address(unix_path))
    else:
        sock = socket.create_connection((host, port))
    sock.sendall(struct.pack('>HHI', USBIP_VERSION, OP_REQ_IMPORT, 0)
                 + busid.encode().ljust(32, b'\0'))
    reply = b''
    while len(reply) < 8:
        reply += sock.recv(8 - len(reply))
    _version, code, status = struct.unpack('>HHI', reply)
    if code != OP_REP_IMPORT or status != 0:
        sock.close()
        raise OSError('import of %s refused (code 0x%04x status %u)'
                      % (busid, code, status))
    dev = b''
    while len(dev) < 312:
        chunk = sock.recv(312 - len(dev))
        if not chunk:
            sock.close()
            raise OSError('short device reply')
        dev += chunk
    busnum, devnum, speed = struct.unpack('>III', dev[288:300])
    return sock, (busnum << 16) | devnum, speed


UDEV_RULE = (
    '# let members of dialout attach/detach USB/IP devices without root\n'
    'ACTION=="add", SUBSYSTEM=="platform", KERNEL=="vhci_hcd.0", '
    'RUN+="/bin/sh -c \'chgrp dialout /sys%p/attach /sys%p/detach; '
    'chmod g+w /sys%p/attach /sys%p/detach\'"\n')
UDEV_RULE_PATH = '/etc/udev/rules.d/99-vhci-user.rules'
MODULES_LOAD_PATH = '/etc/modules-load.d/vhci-hcd.conf'


def install_rules():
    """one-time root setup after which no attach ever needs root:
    load vhci_hcd at boot and make its attach/detach files writable by
    the dialout group (the same group the resulting tty needs anyway)"""
    if os.geteuid() != 0:
        cmd = privilege_prefix() + [sys.executable,
                                    os.path.abspath(__file__),
                                    '--install-rules']
        return subprocess.run(cmd, check=False).returncode == 0
    with open(UDEV_RULE_PATH, 'w') as f:
        f.write(UDEV_RULE)
    with open(MODULES_LOAD_PATH, 'w') as f:
        f.write('vhci_hcd\n')
    subprocess.run(['modprobe', 'vhci_hcd'], check=False)
    # apply to the already-loaded module too, not just future boots
    for name in ('attach', 'detach'):
        path = os.path.join(VHCI, name)
        if os.path.exists(path):
            subprocess.run(['chgrp', 'dialout', path], check=False)
            subprocess.run(['chmod', 'g+w', path], check=False)
    subprocess.run(['udevadm', 'control', '--reload'], check=False)
    print('installed %s and %s; vhci attach now needs no root'
          % (UDEV_RULE_PATH, MODULES_LOAD_PATH), file=sys.stderr)
    return True


def privilege_prefix():
    """how to run something as root here: nothing if we already are,
    sudo when it needs no password, pkexec when there is a desktop to
    ask on (the GUI has no terminal to type into), else sudo anyway"""
    if os.geteuid() == 0:
        return []
    if subprocess.run(['sudo', '-n', 'true'], check=False,
                      stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL).returncode == 0:
        return ['sudo', '-n']
    if os.environ.get('DISPLAY') and shutil.which('pkexec'):
        return ['pkexec']
    return ['sudo']


def attach(unix_path=None, host='127.0.0.1', port=3240, busid=BUSID):
    """import and attach, without root when the udev rule from
    --install-rules is in place (it makes the vhci attach file group
    writable); otherwise re-run ourselves as root for the sysfs write.
    The socket handed to the kernel must belong to the process doing
    that write, so the whole import runs wherever the write happens.
    """
    if os.geteuid() != 0 and not os.access(os.path.join(VHCI, 'attach'),
                                           os.W_OK):
        cmd = privilege_prefix() + [
            sys.executable, os.path.abspath(__file__), '--attach-to',
            unix_path if unix_path is not None else '%s:%u' % (host, port)]
        return subprocess.run(cmd, check=False).returncode == 0
    sock, devid, speed = import_device(unix_path, host, port, busid)
    try:
        attach_socket(sock, devid, speed)
    except OSError:
        sock.close()
        raise
    # True, not the port number: vhci port 0 is a perfectly good
    # attachment, and callers test the return for truth
    return True


def detach(port=None):
    """detach one vhci port, or every port carrying OUR busid.

    The vhci is shared machine-wide: an ArduPilot Renode CubeOrange (or
    anything else) may be attached alongside, so a blanket detach-all
    would unplug someone else's device. Only ports whose local_busid is
    ours are swept; an explicit port number is trusted as given.
    """
    if os.geteuid() != 0 and not os.access(os.path.join(VHCI, 'detach'),
                                           os.W_OK):
        cmd = privilege_prefix() + [sys.executable, os.path.abspath(__file__),
                                    '--detach']
        if port is not None:
            cmd += [str(port)]
        return subprocess.run(cmd, check=False).returncode == 0
    ports = []
    if port is not None:
        ports = [port]
    else:
        for line in open(os.path.join(VHCI, 'status')).read().splitlines()[1:]:
            f = line.split()
            if len(f) >= 7 and f[2] != VDEV_ST_NULL and f[6] == BUSID:
                ports.append(int(f[1]))
    for p in ports:
        with open(os.path.join(VHCI, 'detach'), 'w') as f:
            f.write('%u' % p)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--socket', default=None,
                    help='unix socket to serve on, @name for the abstract '
                         'namespace (default %s)' % default_socket_name())
    ap.add_argument('--port', type=int, default=None,
                    help='serve on this tcp port instead of a unix socket')
    ap.add_argument('--host', default='127.0.0.1',
                    help='address to serve tcp on')
    ap.add_argument('--serial', default=DEFAULT_SERIAL,
                    help='usb serial string, which names the /dev/serial/'
                         'by-id link; give a second instance its own')
    ap.add_argument('--attach', action='store_true',
                    help='attach to the local vhci_hcd once we are listening '
                         '(needs root, re-runs itself under sudo)')
    ap.add_argument('--attach-to', default=None,
                    help='attach an already exported device and exit; takes '
                         'a unix socket name or host:port')
    ap.add_argument('--detach', nargs='?', type=int, const=-1, default=None,
                    help='detach one vhci port, or every port carrying '
                         'this tool\'s busid (other USB/IP devices on the '
                         'machine are left alone)')
    ap.add_argument('--install-rules', action='store_true',
                    help='one-time root setup: load vhci_hcd at boot and '
                         'make its attach/detach group-writable (dialout), '
                         'so no later attach or detach needs root')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    def log(msg):
        if args.verbose:
            print('usbip: %s' % msg, file=sys.stderr, flush=True)

    if args.install_rules:
        return 0 if install_rules() else 1

    if args.detach is not None:
        return 0 if detach(None if args.detach < 0 else args.detach) else 1

    if args.attach_to is not None:
        # the privileged half of attach(): import and hand the socket to
        # the kernel, which keeps it after we exit
        spec = args.attach_to
        if ':' in spec and not spec.startswith('@') and '/' not in spec:
            host, _, port = spec.rpartition(':')
            ok = attach(host=host, port=int(port))
        else:
            ok = attach(unix_path=spec)
        print('attached' if ok else 'attach failed', file=sys.stderr)
        return 0 if ok else 1

    server = UsbipServer(unix_path=args.socket, host=args.host,
                         port=args.port, serial=args.serial, log=log)
    print('exporting %s on %s' % (BUSID, server.endpoint), file=sys.stderr,
          flush=True)
    if args.attach:
        if not attach(unix_path=server.unix_path, host=args.host,
                      port=server.port):
            print('attach failed', file=sys.stderr)
            server.close()
            return 1
        tty = find_tty(args.serial, timeout=10)
        print('attached as %s' % (tty or 'no tty appeared'), file=sys.stderr,
              flush=True)
    elif server.unix_path is None:
        print('attach with: sudo usbip %sattach -r %s -b %s'
              % ('--tcp-port %u ' % server.port if server.port != 3240 else '',
                 args.host, BUSID), file=sys.stderr, flush=True)
    else:
        print('attach with: %s %s --attach-to %s'
              % (sys.executable, os.path.abspath(__file__), server.unix_path),
              file=sys.stderr, flush=True)

    # loopback: echo what the host writes, so the device can be tested
    # with a terminal before anything is wired to it
    try:
        while True:
            data = server.read(0.2)
            if data:
                server.write(data)
    except KeyboardInterrupt:
        server.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
