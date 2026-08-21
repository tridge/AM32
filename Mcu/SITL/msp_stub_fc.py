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


class MspStubFC(object):
    def __init__(self, sitl_host='127.0.0.1', sitl_port=57833,
                 poles=14, rate=500.0, esc_ports=None, state_port=57734,
                 esc_reset=True, motor=True, verbose=False, endpoint=None):
        self.poles = poles
        self.rate = rate
        self.verbose = verbose
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
        self.ep.write(b'$M>' + hdr + payload + bytes([ck]))

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
            # a 4-way client is strictly request/response, so anything
            # waiting for us now is a retry of the command we just
            # answered (a 256 byte read is 130ms of 19200 baud wire time,
            # which is close to the client's timeout). Our one reply
            # satisfies it; leaving it queued would answer twice and
            # shift every later response by one.
            stale = self.ep.drain()
            if stale and stale != self.fourway.last_request:
                self._log('discarding %u unexpected bytes' % len(stale))
            self.ep.write(resp)
        if self.fourway.exited:
            self._log('4-way interface exited')
            self.in_fourway = False

    def _msp_loop(self):
        buf = b''
        while self.running:
            chunk = self.ep.read(0.1)
            if not chunk:
                continue
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
    parser.add_argument('--sitl-port', type=int, default=57833,
                        help='SITL input port driven with DShot')
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
    args = parser.parse_args()

    ports = None
    if args.esc_ports:
        ports = [int(p) for p in args.esc_ports.split(',') if p.strip()]

    endpoint = None
    if args.usbip:
        def log(msg):
            if args.verbose:
                print('usbip: %s' % msg, file=sys.stderr, flush=True)
        endpoint = sitl_usbip.UsbipServer(unix_path=args.usbip_socket,
                                          port=args.usbip_port,
                                          serial=args.usbip_serial, log=log)

    stub = MspStubFC(sitl_host=args.host, sitl_port=args.sitl_port,
                     poles=args.poles, esc_ports=ports,
                     state_port=args.state_port,
                     esc_reset=not args.no_esc_reset,
                     motor=not args.no_motor, verbose=args.verbose,
                     endpoint=endpoint)
    if args.usbip:
        print('virtual FC exported on %s' % endpoint.endpoint,
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
