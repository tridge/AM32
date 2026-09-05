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
import collections
import os
import select
import struct
import sys
import threading
import time

import msp_betaflight
import msp_framing
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
        import pty       # POSIX-only; USB endpoints also work on Windows
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
    def __init__(self, sitl_host='127.0.0.1', sitl_port=57733,
                 poles=14, rate=500.0, esc_ports=None, state_port=57734,
                 esc_reset=True, motor=True, verbose=False, endpoint=None,
                 trace=False, config_path=None, on_reboot=None):
        self.on_reboot = on_reboot
        self.config = msp_betaflight.Configuration(poles, config_path)
        self.output_protocol = self.config.protocol
        self.output_bidir = self.config.values['bidir']
        self.output_lock = threading.RLock()
        self.reply_version = 1
        self.cli = False
        self.cli_line = bytearray()
        self.parser = msp_framing.Parser()
        self.commands = collections.deque()
        self.motor_enabled = motor
        self.last_request = time.monotonic()
        self.ready_at = time.monotonic() + 2.0
        self.edt_override = None
        self.edt_commanded = None
        self.last_reply = 0.0
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
        with self.output_lock:
            self._stop_motor()
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
        nxt = time.monotonic()
        while self.running:
            now = time.monotonic()
            with self.output_lock:
                if self.in_fourway or self.cli or self.output_protocol == 9:
                    nxt = now
                elif now >= nxt:
                    # One writer owns the signal wire, including DShot commands.
                    # Stop after loss of MSP polling (e.g. browser disconnected).
                    if now - self.last_request > 2.0:
                        self.motor_value = 1000
                    cfg = self.config.values
                    self.poles = cfg['poles']
                    bidir = self.output_bidir
                    ptype = {5: sd.TYPE_DSHOT150, 6: sd.TYPE_DSHOT300,
                             7: sd.TYPE_DSHOT600}[self.output_protocol]
                    value = self._dshot_value() if now >= self.ready_at else 0
                    edt = self.edt_override if self.edt_override is not None else cfg['edt'] != 'OFF'
                    if (bidir and (self.edt_commanded != edt or (edt and not self.edt_seen))
                            and value == 0
                            and now >= self.ready_at and not self.commands
                            and now - self.last_edt_cmd > 0.5):
                        self.commands.append([13 if edt else 14, 20])
                        self.edt_commanded = edt
                        self.last_edt_cmd = now
                    telem = False
                    if self.commands and now >= self.ready_at:
                        value, count = self.commands[0]
                        telem = True
                        self.commands[0][1] -= 1
                        if count == 1:
                            self.commands.popleft()
                            # EEPROM save/beeps need time before the next command.
                            if value == 12:
                                self.ready_at = now + 0.04
                            elif 1 <= value <= 5:
                                self.ready_at = now + 0.3
                    self.port.send_dshot(value, ptype=ptype, bidir=bidir, telem=telem)
                    nxt = max(nxt + 1.0 / self.rate, now)
                self._drain_replies()
                if now - self.last_reply > 1 and not self.freeze:
                    self.rpm = 0
                    self.invalid = 100.0
            time.sleep(0.0005)

    def _drain_replies(self):
        if self.freeze:
            self.port.get_replies()      # discard, keep the cached values
            return
        for r in self.port.get_replies():
            kind, val = sd.decode_reply(r[3], edt_expected=True)
            if kind != 'badcrc':
                self.last_reply = time.monotonic()
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

    def _reply(self, cmd, payload=b'', error=False):
        out = msp_framing.encode(cmd, payload, self.reply_version,
                                b'!' if error else b'>')
        self._trace('tx', out)
        self.ep.write(out)

    def _stop_motor(self):
        self.motor_value = 1000
        self.commands.clear()

    def _reboot(self, notify=False):
        self._stop_motor()
        self.config.reboot()
        if (self.motor_enabled and
                (self.output_protocol != self.config.protocol or
                 self.output_bidir != self.config.values['bidir'])):
            # AM32 detects the signal rate/polarity at startup. Apply FC
            # settings on reboot, and restart the simulated ESC to detect
            # the new wire format without a manual power cycle.
            self.fourway._reset_esc(0)
        self.output_protocol = self.config.protocol
        self.output_bidir = self.config.values['bidir']
        self.edt_seen = False
        self.edt_override = None
        self.edt_commanded = None
        self.rpm = self.temp = self.volt_raw = self.curr_raw = 0
        self.ready_at = time.monotonic() + 2.0
        self.cli = False
        self.cli_line.clear()
        if notify and self.on_reboot is not None:
            self.on_reboot(self)

    def _handle(self, cmd, payload):
        self.last_request = time.monotonic()
        with self.output_lock:
            try:
                self._dispatch(cmd, payload)
            except (ValueError, struct.error, OSError) as ex:
                self._log('MSP %u: %s' % (cmd, ex))
                self._reply(cmd, error=True)

    def _dispatch(self, cmd, payload):
        if cmd == MSP_SET_PASSTHROUGH:
            self._stop_motor()
            self.in_fourway = True
            self.fourway.begin()
            self._reply(cmd, bytes([self.fourway.esc_count]))
        elif cmd == MSP_SET_MOTOR:
            if len(payload) < 2 or len(payload) > 16 or len(payload) % 2:
                raise ValueError('invalid motor values')
            value = struct.unpack_from('<H', payload)[0]
            if not 1000 <= value <= 2000:
                raise ValueError('invalid motor throttle')
            self.motor_value = value
            self._reply(cmd)
        elif cmd == 104:  # MSP_MOTOR: one active output, seven unused
            self._reply(cmd, struct.pack('<8H', self.motor_value, *([0] * 7)))
        elif cmd == MSP_MOTOR_TELEMETRY:
            # Match BF's EDT wire units, including its integer voltage shift.
            self._reply(cmd, bytes([1]) + struct.pack('<IHBHHH', self.rpm,
                int(self.invalid * 100), int(self.temp), self.volt_raw >> 2,
                self.curr_raw, 0))
        elif cmd == 222:
            self._stop_motor()
            self.config.set_motor(payload)
            self._reply(cmd)
        elif cmd in (37, 43, 62, 91, 93, 95, 217):
            self._stop_motor()
            self.config.set_register(cmd, payload)
            self._reply(cmd)
        elif cmd == 250:  # MSP_EEPROM_WRITE
            self.config.save()
            self._reply(cmd)
        elif cmd == 68:   # FC reboot; the ESC is separately powered
            if payload and payload != b'\0':
                raise ValueError('only normal FC reboot is supported')
            self._reply(cmd, b'\0')
            self._reboot(notify=True)
        elif cmd == 99:   # MSP_ARMING_DISABLE; never emulate RC arming
            if len(payload) != 2:
                raise ValueError('invalid arming request')
            if payload[0]:
                self._stop_motor()
            self._reply(cmd)
        elif cmd in (205, 246):  # stationary ACC calibration / RTC
            self._reply(cmd)
        elif cmd == 0x3003:
            if (len(payload) < 4 or payload[0] not in (0, 1)
                    or payload[1] not in (0, 255) or payload[2] != len(payload) - 3
                    or any(c > 47 for c in payload[3:]) or self.config.protocol == 9):
                raise ValueError('invalid DShot command')
            if self.motor_value > 1000 and any(c != 0 for c in payload[3:]):
                raise ValueError('stop motor before DShot commands')
            for command in payload[3:]:
                if command == 0:
                    self._stop_motor()
                else:
                    if command in (13, 14):
                        self.edt_override = command == 13
                        self.edt_commanded = self.edt_override
                        self.edt_seen = False
                        self.temp = self.volt_raw = self.curr_raw = 0
                    if len(self.commands) >= 32:
                        raise ValueError('DShot command queue full')
                    self.commands.append([command, 20])
            self._reply(cmd)
        elif cmd == 0x3002:
            if payload != bytes([1, 0]):
                raise ValueError('only motor output 1 exists')
            self._reply(cmd)
        else:
            data = self.config.read(cmd, payload,
                                    voltage=self.volt_raw * 0.25 if self.volt_raw else 12.6,
                                    current=self.curr_raw * 0.5)
            if data is None:
                self._log('unsupported MSP %u' % cmd)
            self._reply(cmd, data or b'', error=data is None)

    def _cli_feed(self, data):
        for char in data:
            if char in (10, 13):
                if not self.cli_line:
                    continue
                line = self.cli_line.decode('ascii', errors='replace').strip()
                self.cli_line.clear()
                self.ep.write(b'\r\n')
                self._cli_command(line)
                if self.cli:
                    self.ep.write(b'# ')
            elif char in (8, 127):
                if self.cli_line:
                    self.cli_line.pop()
                    self.ep.write(b'\b \b')
            elif char == 4:  # Ctrl-D exits without saving
                self._reboot()
            elif 32 <= char < 127 and len(self.cli_line) < 256:
                self.cli_line.append(char)
                self.ep.write(bytes([char]))

    def _cli_command(self, line):
        if line.startswith('#'):
            return  # comments also delimit the app's autocomplete queries
        try:
            if line in ('save', 'exit'):
                if line == 'save':
                    self.config.save()
                self.ep.write(b'Rebooting\r\n')
                self._reboot(notify=True)
                return
            if line.startswith('set ') and '=' in line:
                key, value = [s.strip() for s in line[4:].split('=', 1)]
                self.config.cli_set(key, value)
                result = '%s set to %s' % (key, self.config.cli_get()[key])
            elif line in ('get', 'set', 'dump', 'diff', 'dump all', 'diff all') or line.startswith('get '):
                needle = line[4:].strip() if line.startswith('get ') else ''
                values = self.config.cli_get()
                prefix = 'set ' if line.startswith(('dump', 'diff')) else ''
                result = '\r\n'.join('%s%s = %s' % (prefix, k, v)
                                       for k, v in values.items() if needle in k)
                if not result:
                    raise ValueError('unknown setting')
            elif line == 'version':
                result = '# Betaflight / AM32_SITL 4.6.0 - simulated FC, MSP API: 1.46'
            elif line == 'status':
                result = 'AM32 SITL: stationary ACC/GYRO, disarmed, one motor'
            elif line == 'help':
                result = 'get\r\nset\r\nsave\r\nexit\r\nversion\r\nstatus\r\ndump\r\ndiff\r\nhelp'
            elif line == 'mixer list':
                result = 'Available mixers: CUSTOM'
            else:
                raise ValueError('unsupported command')
            self.ep.write((result + '\r\n').encode('ascii'))
        except (ValueError, OSError) as ex:
            self.ep.write(('###ERROR: %s\r\n' % ex).encode('ascii', errors='replace'))

    def _fourway(self, chunk):
        '''run the 4-way session until the client exits the interface'''
        resp = self.fourway.feed(chunk)
        if resp:
            self._trace('tx', resp)
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
            with self.output_lock:
                # Reload ESC settings when returning to motor testing.
                if self.motor_enabled and self.fourway.connected:
                    self.fourway._reset_esc(0)
                self._reboot()
                self.in_fourway = False

    def _msp_loop(self):
        while self.running:
            chunk = self.ep.read(0.1)
            if not chunk:
                continue
            self._trace('rx', chunk)
            if self.in_fourway:
                self._fourway(chunk)
                continue
            if self.cli:
                with self.output_lock:
                    self._cli_feed(chunk)
                continue
            self.parser.buf += chunk
            while True:
                frame = self.parser.next()
                if frame is None:
                    break
                cmd, payload, self.reply_version = frame
                if cmd == 'cli':
                    with self.output_lock:
                        self._stop_motor()
                        self.cli = True
                        self.ep.write(b'\r\nEntering CLI Mode, type \"exit\" to return\r\n# ')
                        self._cli_feed(self.parser.buf)
                        self.parser.buf = b''
                    break
                self._handle(cmd, payload)
                if self.in_fourway:
                    if self.parser.buf:
                        self._fourway(self.parser.buf)
                    self.parser.buf = b''
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
    parser.add_argument('--config', help='persistent simulated FC settings JSON')
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
        vid, pid = sitl_usbip.VENDOR_ID, sitl_usbip.PRODUCT_ID
        if not args.no_motor:
            vid, pid = (sitl_usbip.BETAFLIGHT_VENDOR_ID,
                        sitl_usbip.BETAFLIGHT_PRODUCT_ID)
        def log(msg):
            if args.verbose:
                print('usbip: %s' % msg, file=sys.stderr, flush=True)
        endpoint = sitl_usbip.UsbipServer(unix_path=args.usbip_socket,
                                          port=args.usbip_port,
                                          serial=args.usbip_serial, log=log,
                                          vid=vid, pid=pid)

    stub = MspStubFC(sitl_host=args.host, sitl_port=args.sitl_port,
                     poles=args.poles, esc_ports=ports,
                     state_port=args.state_port,
                     esc_reset=not args.no_esc_reset,
                     motor=not args.no_motor, verbose=args.verbose,
                     endpoint=endpoint, trace=args.trace, config_path=args.config)
    if args.usbip:
        print('virtual FC exported on %s' % endpoint.endpoint,
              file=sys.stderr, flush=True)
        if args.attach:
            if sitl_usbip.attach(unix_path=endpoint.unix_path,
                                 port=endpoint.port) is False:
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
                               timeout=10 if args.attach else 60,
                               vid=endpoint.vid, pid=endpoint.pid) is None:
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
