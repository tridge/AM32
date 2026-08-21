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

    python3 Mcu/SITL/msp_stub_fc.py --usbip --no-motor
    sudo usbip attach -r 127.0.0.1 -b 1-1

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

STRINGS = ['AM32', 'AM32 SITL serial', 'SITL']

# udev builds this out of the manufacturer, product and serial strings
TTY_GLOB = '/dev/serial/by-id/usb-AM32_AM32_SITL_serial_*-if00'


def find_tty(timeout=10.0):
    '''wait for the attached device to show up as a serial port'''
    deadline = time.time() + timeout
    while True:
        hits = sorted(glob.glob(TTY_GLOB))
        if hits:
            return hits[0]
        if time.time() >= deadline:
            return None
        time.sleep(0.2)


def string_descriptor(index):
    if index == 0:
        return bytes([4, 3, 0x09, 0x04])       # US English
    if index > len(STRINGS):
        return None
    body = STRINGS[index - 1].encode('utf-16-le')
    return bytes([len(body) + 2, 3]) + body


class UsbipServer(object):
    '''USB/IP server exporting one CDC-ACM device.

    read()/write() are the device end of the serial link: what the host
    writes to the tty arrives from read(), what write() is given is what
    the host reads back.
    '''

    def __init__(self, host='127.0.0.1', port=3240, log=None, rx_max=65536):
        self.log = log or (lambda s: None)
        self.rx_max = rx_max
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
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((host, port))
        except OSError as ex:
            # 3240 is the well known USB/IP port, so usbipd or another
            # exporter may already have it
            raise OSError('cannot serve USB/IP on %s:%u (%s), try another '
                          'port' % (host, port, ex))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    @property
    def path(self):
        '''the serial device the host sees, once it has attached us'''
        hits = sorted(glob.glob(TTY_GLOB))
        return hits[0] if hits else 'usbip:%u (not attached)' % self.port

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
            self.log('connection from %s:%u' % addr)
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
                desc = string_descriptor(dindex)
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


def attach(host='127.0.0.1', port=3240, busid=BUSID, sudo=True):
    '''attach the exported device to the local vhci_hcd'''
    # --tcp-port is a global option, it has to come before the command
    cmd = ['usbip']
    if port != 3240:
        cmd += ['--tcp-port', str(port)]
    cmd += ['attach', '-r', host, '-b', busid]
    if sudo:
        cmd = ['sudo'] + cmd
    return subprocess.run(cmd, check=False).returncode == 0


def detach(port=None, sudo=True):
    '''detach every vhci port we own, or one given port number'''
    ports = [port] if port is not None else range(8)
    for p in ports:
        cmd = ['usbip', 'detach', '-p', str(p)]
        subprocess.run(['sudo'] + cmd if sudo else cmd, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='127.0.0.1', help='address to serve on')
    ap.add_argument('--port', type=int, default=3240, help='USB/IP tcp port')
    ap.add_argument('--attach', action='store_true',
                    help='run usbip attach once we are listening (needs root)')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    def log(msg):
        if args.verbose:
            print('usbip: %s' % msg, file=sys.stderr, flush=True)

    server = UsbipServer(host=args.host, port=args.port, log=log)
    print('exporting %s on %s:%u, attach with:' % (BUSID, args.host, args.port),
          file=sys.stderr)
    print('  sudo usbip attach -r %s -b %s' % (args.host, BUSID),
          file=sys.stderr, flush=True)
    if args.attach and not attach(args.host, args.port):
        print('attach failed', file=sys.stderr)
        return 1
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
