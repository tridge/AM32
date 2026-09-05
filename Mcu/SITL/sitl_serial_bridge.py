'''
USB linker emulation: a transparent serial pipe to the simulated ESC.

This is the other way a configurator reaches an AM32 ESC. Where
msp_stub_fc.py pretends to be a flight controller and tunnels BLHeli
4-way to the ESC, this pretends to be the 1-wire USB linker soldered
straight onto the signal wire: the host's bytes go out on the wire as
they arrive and the ESC bootloader's answers come back, with no framing
in between. It is what the am32-configurator calls direct mode and what
the desktop Offline-Configurator drives at 19200 baud.

A real linker shorts TX to RX, so the host reads back everything it
sent before the ESC replies. Both configurators rely on that echo to
find the start of a response (the web one requires it), so the bridge
reproduces it.

Serves the pipe on a pty by default, printing the slave device path to
hand to the configurator. With --usbip it serves a virtual USB serial
device instead (sitl_usbip.py), which enumerates as a real /dev/ttyACM*
and so is reachable from a browser.
'''

import argparse
import socket
import struct
import sys
import threading
import time

import sitl_dshot as sd
import sitl_usbip
from msp_stub_fc import PtyEndpoint

# state port RESET (sitl_state.c cmd 9), used to put a running ESC back
# in the bootloader the way unplugging the battery does
STATE_MAGIC_CMD = 0x5353
STATE_CMD_RESET = 9

# how long to wait for the ESC to answer the first command before
# deciding it is running the application, and how long to keep retrying
# once it has been reset
PROBE_TIMEOUT = 0.5
PROBE_RETRY_INTERVAL = 0.1  # shorter than the bootloader's 250 ms fallback
RESET_WINDOW = 3.0

# one byte at 19200 8N1. The bootloader separates a command from the
# buffer that follows it by the gap in between, so the bridge has to
# work out where the wire really went idle: bytes the host hands over
# while the linker is still shifting the previous ones out continue the
# same frame
BYTE_TIME = 10.0 / 19200.0
POLL = 0.002


class SerialBridge(object):
    '''byte pipe between a serial endpoint and the SITL signal wire'''

    def __init__(self, sitl_host='127.0.0.1', sitl_port=57733,
                 state_port=57734, esc_reset=True, echo=True,
                 endpoint=None, verbose=False, trace=False):
        self.host = sitl_host
        self.state_port = state_port
        self.esc_reset = esc_reset
        self.echo = echo
        self.verbose = verbose
        self.tracing = trace
        self.port = sd.InputPort(sitl_host, sitl_port)
        self.ep = endpoint if endpoint is not None else PtyEndpoint()
        # the ESC has answered at least once, so it is in the bootloader
        # and no longer needs the reset treatment
        self.connected = False
        self.reset_done = False
        # the command we are waiting on an answer for, kept so it can be
        # sent again after a reset
        self.pending = b''
        self.pending_at = 0.0
        self.retry_until = 0.0
        # when the bytes handed to the SITL so far finish transmitting,
        # and the echo waiting on that: [(due time, bytes)]
        self.wire_busy_until = 0.0
        self.echo_due = []
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    @property
    def slave_path(self):
        '''the serial device a client should open'''
        return self.ep.path

    def _log(self, msg):
        if self.verbose:
            print('bridge: %s' % msg, file=sys.stderr, flush=True)

    def _trace(self, direction, data):
        if not self.tracing or not data:
            return
        print('bridge %s %3u: %s' % (direction, len(data), data.hex(' ')),
              file=sys.stderr, flush=True)

    def close(self):
        self.running = False
        # never close a pty descriptor under a blocking read in another
        # thread; _loop polls with a short timeout so it can leave first
        self.thread.join(0.5)
        self.ep.close()
        self.port.close()

    def _reset_esc(self):
        '''ask the SITL to reset, which lands it back in the bootloader
        when it was chained with --bootloader'''
        if not self.state_port:
            return
        pkt = struct.pack('<HBB', STATE_MAGIC_CMD, STATE_CMD_RESET, 0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(pkt, (self.host, self.state_port))
        except OSError as ex:
            self._log('state port reset failed: %s' % ex)
        finally:
            sock.close()

    def _to_esc(self, data):
        self._trace('rx', data)
        now = time.time()
        gap = now > self.wire_busy_until
        self.wire_busy_until = max(now, self.wire_busy_until) \
            + len(data) * BYTE_TIME
        self.port.send_serial(data, gap=gap)
        if self.echo:
            # the linker's TX/RX short: the host reads back everything it
            # sent, and the configurator strips exactly that many bytes
            # to find the reply. It comes back at wire speed - echoing
            # instantly would let the host send the buffer upload before
            # the command frame in front of it had finished, leaving the
            # bootloader one long frame it cannot parse
            self.echo_due.append((self.wire_busy_until, data))
        # the frame in flight, kept so it can be sent again after a
        # reset. A frame boundary starts a new one, so a client that
        # retries a probe of its own does not leave the two concatenated
        self.pending = data if gap else (self.pending + data)[-512:]
        self.pending_at = now

    def _flush_echo(self, force=False):
        now = time.time()
        while self.echo_due and (force or now >= self.echo_due[0][0]):
            self.ep.write(self.echo_due.pop(0)[1])

    def _from_esc(self, data):
        self._trace('tx', data)
        # the ESC cannot answer before it has heard the command, so the
        # echo goes out first however fast the simulation is running
        self._flush_echo(force=True)
        self.ep.write(data)
        self.connected = True
        self.pending = b''
        self.retry_until = 0.0

    def _retry(self, now):
        '''nothing came back from the first command: the ESC is running
        the application, so reset it into the bootloader and send that
        command again. Only ever done before the first reply, where the
        command is the harmless device info probe - a resend later could
        repeat a flash write'''
        if self.connected or not self.esc_reset or not self.pending:
            return
        if not self.reset_done:
            if now - self.pending_at < PROBE_TIMEOUT:
                return
            self._log('no answer, resetting the ESC into the bootloader')
            self.reset_done = True
            self._reset_esc()
            self.retry_until = now + RESET_WINDOW
            self.pending_at = now
            return
        # the bootloader takes a moment to come up after the reset, so
        # keep re-probing until it answers or the window closes
        if now > self.retry_until or now - self.pending_at < PROBE_RETRY_INTERVAL:
            return
        self.port.flush_serial()
        self.port.send_serial(self.pending, gap=True)
        self.wire_busy_until = now + len(self.pending) * BYTE_TIME
        self.pending_at = now

    def _loop(self):
        while self.running:
            chunk = self.ep.read(POLL)
            if chunk:
                self._to_esc(chunk)
            reply = self.port.drain_serial()
            if reply:
                self._from_esc(reply)
            else:
                self._flush_echo()
                if self.pending:
                    self._retry(time.time())


def main():
    parser = argparse.ArgumentParser(
        description='USB linker emulation for the AM32 SITL')
    parser.add_argument('--host', default='127.0.0.1', help='SITL host')
    parser.add_argument('--sitl-port', type=int, default=57733,
                        help='SITL input port, the simulated signal wire')
    parser.add_argument('--state-port', type=int, default=57734,
                        help='SITL state port, used to reset the ESC into '
                             'the bootloader (0 disables)')
    parser.add_argument('--no-esc-reset', action='store_true',
                        help='never reset an ESC that does not answer')
    parser.add_argument('--no-echo', action='store_true',
                        help='do not echo the host\'s bytes back, for a '
                             'client that cannot cope with a 1-wire linker')
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
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--trace', action='store_true',
                        help='hex dump every byte to and from the client')
    args = parser.parse_args()

    def log(msg):
        if args.verbose:
            print('usbip: %s' % msg, file=sys.stderr, flush=True)

    endpoint = None
    if args.usbip:
        endpoint = sitl_usbip.UsbipServer(unix_path=args.usbip_socket,
                                          port=args.usbip_port,
                                          serial=args.usbip_serial, log=log,
                                          vid=sitl_usbip.DIRECT_VENDOR_ID,
                                          pid=sitl_usbip.DIRECT_PRODUCT_ID)

    bridge = SerialBridge(sitl_host=args.host, sitl_port=args.sitl_port,
                          state_port=args.state_port,
                          esc_reset=not args.no_esc_reset,
                          echo=not args.no_echo, endpoint=endpoint,
                          verbose=args.verbose, trace=args.trace)
    if args.usbip:
        print('virtual linker exported on %s' % endpoint.endpoint,
              file=sys.stderr, flush=True)
        if args.attach:
            if sitl_usbip.attach(unix_path=endpoint.unix_path,
                                 port=endpoint.port) is False:
                print('attach failed', file=sys.stderr)
                bridge.close()
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
    print(bridge.slave_path, flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
