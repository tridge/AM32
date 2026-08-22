'''
fake Betaflight FC: a minimal MSP server bridged to the SITL's UDP
DShot input, with BLHeli 4-way passthrough to the simulated ESC

Lets scripts/esc_capture_fc.py run against the SITL binary with no
hardware: the stub answers the MSP preflight queries, streams
bidirectional DShot600 to the SITL from the latest MSP_SET_MOTOR
value, decodes the BDShot/EDT replies and serves them back as
MSP_MOTOR_TELEMETRY, just like a real FC does.

MSP_SET_PASSTHROUGH switches the link into 4-way mode
(sitl_fourway_server.py), so an unmodified ESC configurator can read and
write the settings and flash of a SITL running with --bootloader, the
same way it would through a real flight controller.

With --direct it is a single-wire adapter on the ESC signal pad
instead of a flight controller: the raw 19200 baud bootloader protocol
with the adapter's self-echo, which is how both configurators' direct
modes expect the wire to behave. The line is held idle-high, so the
bootloader stays resident across ESC resets.

Serves MSP on a pty by default (Linux/macOS only), printing the slave
device path to hand to --port. With --usbip it serves on a virtual USB
serial device instead (sitl_usbip.py), which enumerates as a real
/dev/ttyACM* once attached to vhci_hcd and so is reachable from tools
that only accept USB serial ports, the browser included.
'''

import argparse
import os
import pty
import select
import struct
import sys
import threading
import time

import sitl_dshot as sd
import sitl_fourway_server
import sitl_usbip

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FEATURE_CONFIG = 36
MSP_STATUS = 101
MSP_BOXIDS = 119
MSP_MOTOR_CONFIG = 131
MSP_MOTOR_TELEMETRY = 139
MSP_BATTERY_STATE = 130
MSP_SET_MOTOR = 214
MSP_SET_PASSTHROUGH = 245

# USB identity for --direct: a vendor id the web configurator treats as
# a direct single-wire adapter (WCH), with a product id no kernel
# vendor driver claims, so cdc_acm keeps the port
DIRECT_VENDOR_ID = 0x1A86
DIRECT_PRODUCT_ID = 0x0001


class PtyEndpoint(object):
    '''the serial link as a pty, opened by the client by path'''

    def __init__(self):
        self.master, self.slave = pty.openpty()
        # raw mode now: the default line discipline would echo the
        # client's bytes back at us until pyserial reconfigures it
        import tty
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)

    def read(self, timeout=0.1):
        try:
            ready, _, _ = select.select([self.master], [], [], timeout)
        except (OSError, ValueError):
            return b''
        if not ready:
            return b''
        try:
            return os.read(self.master, 4096)
        except OSError:
            return b''

    def drain(self):
        out = b''
        while True:
            chunk = self.read(0)
            if not chunk:
                return out
            out += chunk

    def write(self, data):
        os.write(self.master, data)

    def close(self):
        try:
            os.close(self.master)
            os.close(self.slave)
        except OSError:
            pass


class DirectBridge(object):
    """the serial link wired straight to the ESC's signal pad, the way a
    single-wire USB adapter presents it: no flight controller, no MSP,
    no 4-way - the raw 19200 baud bootloader protocol. TX and RX share
    the wire on such an adapter, so the client reads its own
    transmission back; both configurators' direct modes strip that
    echo. The echo is released at wire pace, not on receipt: the client
    only moves on once its bytes have actually left the wire, and the
    bootloader ends a command frame on line idle, so an instant echo
    lets the client run its next command seamlessly into the previous
    frame and nothing ever parses. The wire idles high, keeping the
    bootloader resident across ESC resets."""

    BAUD = 19200.0
    GAP_S = 0.001                 # FLAG_GAP's leading line idle

    def __init__(self, sitl_host='127.0.0.1', sitl_port=57733,
                 endpoint=None, verbose=False, trace=False):
        self.verbose = verbose
        self.tracing = trace or os.environ.get('AM32_FC_TRACE') == '1'
        self.port = sd.InputPort(sitl_host, sitl_port)
        self.ep = endpoint if endpoint is not None else PtyEndpoint()
        self.running = True
        self.wire_free = 0.0      # when the queued TX will have left
        self.port.send_level(1)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    @property
    def slave_path(self):
        return self.ep.path

    def _trace(self, direction, data):
        if not self.tracing or not data:
            return
        print('DW %s %3u: %s' % (direction, len(data), data.hex(' ')),
              file=sys.stderr, flush=True)

    def _loop(self):
        echoes = []               # (due, bytes) awaiting their wire time
        while self.running:
            timeout = 0.02
            if echoes:
                timeout = max(0.0, min(timeout,
                                       echoes[0][0] - time.time()))
            chunk = self.ep.read(timeout)
            now = time.time()
            if chunk:
                self._trace('rx', chunk)
                # Frame separation as the wire itself would show it: a
                # chunk arriving on an idle wire begins a new command
                # and gets the idle gap the bootloader needs to end the
                # previous frame; one arriving while bytes are still
                # going out continues the frame (a USB packet split
                # mid-command must not be torn apart).
                gap = now >= self.wire_free
                if gap:
                    self.wire_free = now + self.GAP_S
                self.port.send_serial(chunk, idle_high=True, gap=gap)
                self.wire_free = (max(self.wire_free, now)
                                  + len(chunk) * 10.0 / self.BAUD)
                echoes.append((self.wire_free, chunk))
            while echoes and echoes[0][0] <= now:
                _due, data = echoes.pop(0)
                self._trace('echo', data)
                self.ep.write(data)
            out = self.port.drain_serial()
            if out:
                self._trace('tx', out)
                self.ep.write(out)

    def close(self):
        self.running = False
        self.thread.join(0.5)
        self.ep.close()
        self.port.close()


class MspStubFC(object):
    def __init__(self, sitl_host='127.0.0.1', sitl_port=57733,
                 poles=14, rate=500.0, esc_ports=None, state_port=57734,
                 esc_reset=True, motor=True, verbose=False, endpoint=None,
                 trace=False):
        self.poles = poles
        self.rate = rate
        self.verbose = verbose
        self.tracing = trace or os.environ.get('AM32_FC_TRACE') == '1'
        # the ESCs reachable over 4-way passthrough: one SITL input port
        # each, defaulting to the one we drive with DShot
        self.fourway = sitl_fourway_server.FourWayServer(
            esc_ports=esc_ports or [sitl_port], host=sitl_host,
            state_port=state_port, esc_reset=esc_reset, log=self._log)
        self.in_fourway = False
        # set to stop updating the telemetry while still answering MSP,
        # reproducing Betaflight serving its cached values after the
        # BDShot replies stop arriving (it never marks them stale)
        self.freeze = False
        self.port = sd.InputPort(sitl_host, sitl_port)
        self.motor_value = 1000        # latest MSP_SET_MOTOR, motor 1
        self.rpm = 0
        # raw EDT bytes as Betaflight caches them: voltage in 0.25V
        # steps, current in the ESC's own units (AM32: 0.5A), temp in C
        self.volt_raw = 0
        self.curr_raw = 0
        self.temp = 0
        self.invalid = 100.0           # no telemetry until frames arrive
        self.edt_seen = False
        self.last_edt_cmd = 0.0
        self.running = True
        self.ep = endpoint if endpoint is not None else PtyEndpoint()
        # driving DShot at the ESC would fight the 4-way session for the
        # signal wire, so it can be left off for pure configurator work
        self.dshot_thread = None
        if motor:
            self.dshot_thread = threading.Thread(target=self._dshot_loop,
                                                 daemon=True)
            self.dshot_thread.start()
        self.msp_thread = threading.Thread(target=self._msp_loop, daemon=True)
        self.msp_thread.start()

    @property
    def slave_path(self):
        '''the serial device a client should open'''
        return self.ep.path

    def _log(self, msg):
        if self.verbose:
            print('FC: %s' % msg, file=sys.stderr, flush=True)

    def _trace(self, direction, data):
        '''byte level trace of the configurator link, for working out
        which side of a failed session went quiet. AM32_FC_TRACE=1 turns
        it on for a stub started by the GUI, which has no command line'''
        if not self.tracing or not data:
            return
        print('FC %s %3u: %s' % (direction, len(data), data.hex(' ')),
              file=sys.stderr, flush=True)

    def close(self):
        self.running = False
        # Do not close a pty descriptor under a blocking read in another
        # thread: close() itself can wait for that read forever on macOS.
        # _msp_loop polls with a short timeout, so both workers can leave
        # before their descriptors are closed.
        self.msp_thread.join(0.5)
        if self.dshot_thread is not None:
            self.dshot_thread.join(0.5)
        self.fourway.close()
        self.ep.close()
        self.port.close()

    # -- DShot side ----------------------------------------------------

    def _dshot_value(self):
        '''BF motor value 1000..2000 -> 11 bit DShot throttle'''
        v = self.motor_value
        if v <= 1000:
            return 0
        return 48 + int((min(v, 2000) - 1000) * (2047 - 48) / 1000)

    def _dshot_loop(self):
        nxt = time.time()
        while self.running:
            now = time.time()
            burst = 0
            while now >= nxt and burst < 10:
                nxt += 1.0 / self.rate
                value = self._dshot_value()
                # maintain EDT while stopped, like a real FC with
                # dshot_edt on (the firmware ignores commands once
                # spinning)
                if (value == 0 and not self.edt_seen
                        and now - self.last_edt_cmd > 0.5):
                    self.last_edt_cmd = now
                    for _ in range(20):
                        self.port.send_dshot(sd.DSHOT_CMD_EDT_ENABLE,
                                             ptype=sd.TYPE_DSHOT600,
                                             telem=True, bidir=True)
                        burst += 1
                    continue
                self.port.send_dshot(value, ptype=sd.TYPE_DSHOT600,
                                     bidir=True)
                burst += 1
            if now - nxt > 0.25:
                nxt = now
            self._drain_replies()
            time.sleep(0.0005)

    def _drain_replies(self):
        if self.freeze:
            self.port.get_replies()      # discard, keep the cached values
            return
        for r in self.port.get_replies():
            kind, val = sd.decode_reply(r[3], edt_expected=True)
            if kind == 'erpm':
                self.rpm = int(sd.erpm_period_to_rpm(val, self.poles))
                self.invalid = max(0.0, self.invalid - 1.0)
            elif kind == 'temp':
                self.temp = val
                self.edt_seen = True
            elif kind == 'volt':
                self.volt_raw = int(round(val / 0.25))
                self.edt_seen = True
            elif kind == 'current':
                self.curr_raw = int(round(val / 0.5))
                self.edt_seen = True
            elif kind == 'edt':
                self.edt_seen = True
            elif kind == 'badcrc':
                self.invalid = min(100.0, self.invalid + 0.1)

    # -- MSP side ------------------------------------------------------

    def _reply(self, cmd, payload=b''):
        hdr = struct.pack('<BB', len(payload), cmd)
        ck = 0
        for b in hdr + payload:
            ck ^= b
        out = b'$M>' + hdr + payload + bytes([ck])
        self._trace('tx', out)
        self.ep.write(out)

    def _handle(self, cmd, payload):
        if cmd == MSP_API_VERSION:
            self._reply(cmd, struct.pack('<BBB', 0, 1, 46))
        elif cmd == MSP_FC_VARIANT:
            self._reply(cmd, b'BTFL')
        elif cmd == MSP_STATUS:
            # cycletime, i2c errors, sensors, mode flags (disarmed), profile
            self._reply(cmd, struct.pack('<HHHIB', 125, 0, 0, 0, 0))
        elif cmd == MSP_BOXIDS:
            self._reply(cmd, bytes([0]))   # one box: ARM
        elif cmd == MSP_FEATURE_CONFIG:
            self._reply(cmd, struct.pack('<I', 0))   # no 3D mode
        elif cmd == MSP_MOTOR_CONFIG:
            self._reply(cmd, struct.pack('<HHHBBBB', 1070, 2000, 1000,
                                         4, self.poles, 1, 0))
        elif cmd == MSP_MOTOR_TELEMETRY:
            out = bytes([4])
            for i in range(4):
                if i == 0:
                    # matches Betaflight's DShot telemetry serialisation:
                    # voltage is the 0.25V-step EDT value >> 2, current
                    # is the raw EDT byte (msp.c MSP_MOTOR_TELEMETRY)
                    out += struct.pack('<IHBHHH', self.rpm,
                                       int(self.invalid * 100), int(self.temp),
                                       self.volt_raw >> 2,
                                       self.curr_raw, 0)
                else:
                    # unused outputs: no telemetry at all
                    out += struct.pack('<IHBHHH', 0, 10000, 0, 0, 0, 0)
            self._reply(cmd, out)
        elif cmd == MSP_BATTERY_STATE:
            # cells, capacity, voltage in 0.1V, mAh drawn, current in 0.01A
            self._reply(cmd, struct.pack('<BHBHH', 4, 1500, 126, 0, 0))
        elif cmd == MSP_SET_PASSTHROUGH:
            self.fourway.begin()
            self._reply(cmd, bytes([self.fourway.esc_count]))
            self._log('4-way passthrough to %u ESC(s)'
                      % self.fourway.esc_count)
            self.in_fourway = True
        elif cmd == MSP_SET_MOTOR:
            if len(payload) >= 2:
                self.motor_value = struct.unpack('<H', payload[0:2])[0]
            self._reply(cmd)
        else:
            self.ep.write(b'$M!' + struct.pack('<BB', 0, cmd)
                          + bytes([cmd]))

    def _fourway(self, chunk):
        '''run the 4-way session until the client exits the interface'''
        resp = self.fourway.feed(chunk)
        if resp:
            self._trace('tx', resp)
            # A slow transaction (a 256 byte flash chunk is ~140ms of
            # 19200 baud wire time) makes the client retry, and those
            # retries queue up while we work. Answering each one would
            # shift every later response by one, so exact duplicates of
            # the request we just answered are dropped - but ONLY exact
            # duplicates: a client that timed out and moved on has its
            # NEXT command queued here, and discarding that starves the
            # whole session one command at a time.
            pending = self.ep.drain()
            dropped = 0
            last = self.fourway.last_request
            while last and pending.startswith(last):
                pending = pending[len(last):]
                dropped += 1
            if dropped:
                self._log('dropped %u retries of the answered command'
                          % dropped)
            self.ep.write(resp)
            if pending:
                self._trace('rx', pending)
                self._fourway(pending)
                return
        if self.fourway.exited:
            self._log('4-way interface exited')
            self.in_fourway = False

    def _msp_loop(self):
        buf = b''
        while self.running:
            chunk = self.ep.read(0.1)
            if not chunk:
                continue
            self._trace('rx', chunk)
            if self.in_fourway:
                self._fourway(chunk)
                continue
            buf += chunk
            while True:
                start = buf.find(b'$M<')
                if start < 0:
                    buf = b''
                    break
                buf = buf[start:]
                if len(buf) < 5:
                    break
                size = buf[3]
                if len(buf) < 6 + size:
                    break
                cmd = buf[4]
                payload = buf[5:5 + size]
                ck = 0
                for b in buf[3:5 + size]:
                    ck ^= b
                good = ck == buf[5 + size]
                buf = buf[6 + size:]
                if good:
                    self._handle(cmd, payload)
                if self.in_fourway:
                    if buf:
                        self._fourway(buf)
                    buf = b''
                    break


def main():
    parser = argparse.ArgumentParser(
        description='fake Betaflight FC for the AM32 SITL')
    parser.add_argument('--host', default='127.0.0.1', help='SITL host')
    parser.add_argument('--sitl-port', type=int, default=57733,
                        help='SITL input port, the simulated signal wire')
    parser.add_argument('--esc-ports', default=None,
                        help='comma separated SITL input ports of the ESCs '
                             'reachable over 4-way (default --sitl-port)')
    parser.add_argument('--state-port', type=int, default=57734,
                        help='SITL state port, used to reset an ESC into the '
                             'bootloader (0 disables)')
    parser.add_argument('--no-esc-reset', action='store_true',
                        help='never reset an ESC that does not answer')
    parser.add_argument('--no-motor', action='store_true',
                        help='do not drive DShot, 4-way and MSP only')
    parser.add_argument('--direct', action='store_true',
                        help='be a single-wire adapter on the ESC signal '
                             'pad instead of a flight controller: raw '
                             'bootloader protocol with self-echo, no MSP '
                             'or 4-way')
    parser.add_argument('--usbip', action='store_true',
                        help='serve on a virtual USB serial device instead '
                             'of a pty, so tools that only take USB ports '
                             '(a browser) can reach it')
    parser.add_argument('--usbip-socket', default=None,
                        help='unix socket the virtual device is exported on, '
                             '@name for the abstract namespace')
    parser.add_argument('--usbip-port', type=int, default=None,
                        help='export the virtual device on this tcp port '
                             'instead of a unix socket')
    parser.add_argument('--usbip-serial', default=sitl_usbip.DEFAULT_SERIAL,
                        help='usb serial string of the virtual device, which '
                             'names its /dev/serial/by-id link')
    parser.add_argument('--attach', action='store_true',
                        help='with --usbip, attach it to vhci_hcd for you')
    parser.add_argument('--poles', type=int, default=14)
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--trace', action='store_true',
                        help='hex dump every byte to and from the client')
    args = parser.parse_args()

    ports = None
    if args.esc_ports:
        ports = [int(p) for p in args.esc_ports.split(',') if p.strip()]

    endpoint = None
    if args.usbip:
        def log(msg):
            if args.verbose:
                print('usbip: %s' % msg, file=sys.stderr, flush=True)
        ids = ({'vid': DIRECT_VENDOR_ID, 'pid': DIRECT_PRODUCT_ID}
               if args.direct else {})
        endpoint = sitl_usbip.UsbipServer(unix_path=args.usbip_socket,
                                          port=args.usbip_port,
                                          serial=args.usbip_serial, log=log,
                                          **ids)

    if args.direct:
        stub = DirectBridge(sitl_host=args.host, sitl_port=args.sitl_port,
                            endpoint=endpoint, verbose=args.verbose,
                            trace=args.trace)
    else:
        stub = MspStubFC(sitl_host=args.host, sitl_port=args.sitl_port,
                         poles=args.poles, esc_ports=ports,
                         state_port=args.state_port,
                         esc_reset=not args.no_esc_reset,
                         motor=not args.no_motor, verbose=args.verbose,
                         endpoint=endpoint, trace=args.trace)
    if args.usbip:
        print('virtual %s exported on %s'
              % ('adapter' if args.direct else 'FC', endpoint.endpoint),
              file=sys.stderr, flush=True)
        if args.attach:
            if not sitl_usbip.attach(unix_path=endpoint.unix_path,
                                     port=endpoint.port):
                print('attach failed', file=sys.stderr)
                stub.close()
                return 1
        else:
            print('attach it with: %s %s --attach-to %s'
                  % (sys.executable, sitl_usbip.__file__,
                     endpoint.unix_path or '%s:%u' % (args.host,
                                                      endpoint.port)),
                  file=sys.stderr, flush=True)
        if sitl_usbip.find_tty(args.usbip_serial,
                               timeout=10 if args.attach else 60) is None:
            print('no tty appeared, is vhci_hcd loaded?', file=sys.stderr)
    print(stub.slave_path, flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stub.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
