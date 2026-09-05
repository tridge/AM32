'''
FC side of the BLHeli 4-way protocol, bridged to the SITL ESC.

msp_stub_fc.py switches into this after MSP_SET_PASSTHROUGH, so a
configurator that only knows how to talk to a flight controller (the
web am32-configurator, the desktop Offline-Configurator, esc-configurator)
can drive the simulated ESC with no changes. Each 4-way command becomes
one-wire bootloader transactions on the SITL input port (sitl_fourway.py),
which is what the ESC signal wire is in the simulation.

Frame formats, from am32-configurator src/communication/four_way.ts and
the desktop client fourwayif.cpp:

  request   0x2F cmd addr_hi addr_lo len params[len] crc_hi crc_lo
  response  0x2E cmd addr_hi addr_lo len params[len] ack crc_hi crc_lo

len 0 means 256, and the CRC is CRC16-XMODEM over every preceding byte
(so the response CRC covers the ack). Commands with nothing to return
still carry one zero parameter byte, as Betaflight's do.
'''

import socket
import struct
import time

import sitl_fourway

REQ_MARK = 0x2F
RESP_MARK = 0x2E

# serial_4way.h command set
CMD_INTERFACE_TEST_ALIVE = 0x30
CMD_PROTOCOL_GET_VERSION = 0x31
CMD_INTERFACE_GET_NAME = 0x32
CMD_INTERFACE_GET_VERSION = 0x33
CMD_INTERFACE_EXIT = 0x34
CMD_DEVICE_RESET = 0x35
CMD_DEVICE_INIT_FLASH = 0x37
CMD_DEVICE_ERASE_ALL = 0x38
CMD_DEVICE_PAGE_ERASE = 0x39
CMD_DEVICE_READ = 0x3A
CMD_DEVICE_WRITE = 0x3B
CMD_DEVICE_C2CK_LOW = 0x3C
CMD_DEVICE_READ_EEPROM = 0x3D
CMD_DEVICE_WRITE_EEPROM = 0x3E
CMD_INTERFACE_SET_MODE = 0x3F

ACK_OK = 0x00
ACK_I_INVALID_CMD = 0x02
ACK_I_INVALID_CRC = 0x03
ACK_D_GENERAL_ERROR = 0x0F

# what Betaflight reports for itself
PROTOCOL_VERSION = 107
INTERFACE_NAME = b'm4wFCIntf'
INTERFACE_VERSION = (20, 1)
IM_ARM_BLB = 4

# state port RESET (sitl_state.c cmd 9), used to put a running ESC back
# in the bootloader the way a real FC power cycles it
STATE_MAGIC_CMD = 0x5353
STATE_CMD_RESET = 9


def crc16_xmodem(data, crc=0):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class FourWayServer(object):
    '''4-way passthrough server for one or more simulated ESCs.

    Each ESC is a (host, input_port) pair - one SITL instance. feed()
    takes bytes from the configurator and returns the bytes to send
    back; it never blocks on the configurator, only on the ESC.
    '''

    def __init__(self, esc_ports=None, host='127.0.0.1', state_port=57734,
                 esc_reset=True, log=None, state_ports=None):
        self.host = host
        self.esc_ports = list(esc_ports or [57733])
        if not 1 <= len(self.esc_ports) <= 8 or len(set(self.esc_ports)) != len(self.esc_ports):
            raise ValueError('expected 1..8 distinct ESC signal ports')
        self.state_port = state_port
        self.state_ports = list(state_ports) if state_ports is not None else [
            state_port + port - self.esc_ports[0] if state_port else 0
            for port in self.esc_ports]
        if len(self.state_ports) != len(self.esc_ports):
            raise ValueError('one state port is required per ESC')
        self.esc_reset = esc_reset
        self.log = log or (lambda s: None)
        self.clients = {}
        self.target = 0
        self.connected = set()
        self.exited = False
        self.buf = b''
        self.last_request = b''

    def begin(self):
        '''start a passthrough session.

        A configurator may open several against one FC - a browser does
        it every time you reconnect - and each one starts from scratch:
        the exit flag from the last session would otherwise drop us out
        of 4-way mode again after its first command, and the ESC has to
        be re-selected with cmd_DeviceInitFlash anyway.
        '''
        self.exited = False
        self.buf = b''
        self.last_request = b''
        self.connected = set()
        self.target = 0

    def close(self):
        for c in self.clients.values():
            c.close()
        self.clients = {}
        self.connected = set()

    @property
    def esc_count(self):
        return len(self.esc_ports)

    def _client(self, target):
        '''one-wire client for an ESC index, created on first use'''
        if target not in self.clients:
            self.clients[target] = sitl_fourway.FourWay(
                host=self.host, udp_port=self.esc_ports[target])
        return self.clients[target]

    def _reset_esc(self, target):
        '''ask the SITL to reset, which lands it back in the bootloader
        when it was chained with --bootloader'''
        port = self.state_ports[target]
        if not self.esc_reset or not port:
            return
        pkt = struct.pack('<HBB', STATE_MAGIC_CMD, STATE_CMD_RESET, 0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(pkt, (self.host, port))
        except OSError as ex:
            self.log('state port reset failed: %s' % ex)
        finally:
            sock.close()

    def _connect(self, target):
        '''bootloader probe, resetting the ESC once if it is running the
        application - a real FC pulls the signal line to do the same'''
        fw = self._client(target)
        info = fw.connect(timeout=0.5)
        if info is None:
            self.log('ESC %u did not answer, resetting into the bootloader'
                     % target)
            self._reset_esc(target)
            deadline = time.time() + 3.0
            while info is None and time.time() < deadline:
                # The bootloader's no-CAN fallback boots a valid app after
                # 250 ms. Probe faster than that while the process re-execs;
                # a 500 ms retry can miss the entire configuration window.
                info = fw.connect(timeout=0.1)
        if info is not None:
            self.connected.add(target)
        return info

    # -- protocol ------------------------------------------------------

    def feed(self, data):
        '''consume bytes from the configurator, return bytes to send back'''
        self.buf += data
        out = b''
        while True:
            frame = self._next_frame()
            if frame is None:
                break
            if frame is False:          # bad crc, already answered
                continue
            out += self._handle(frame)
            if self.exited:
                break
        return out

    def _next_frame(self):
        '''pull one complete request out of the buffer.

        None: need more bytes. False: a frame was dropped.
        '''
        # resync on the start byte, tolerating line noise or a partial
        # command left over from the MSP side
        start = self.buf.find(bytes([REQ_MARK]))
        if start < 0:
            self.buf = b''
            return None
        if start > 0:
            self.log('dropping %u bytes before the frame start' % start)
            self.buf = self.buf[start:]
        if len(self.buf) < 5:
            return None
        size = self.buf[4] or 256
        total = 7 + size
        if len(self.buf) < total:
            return None
        frame, self.buf = self.buf[:total], self.buf[total:]
        crc = struct.unpack('>H', frame[-2:])[0]
        if crc != crc16_xmodem(frame[:-2]):
            self.log('bad request crc for command 0x%02x' % frame[1])
            return False
        self.last_request = frame
        return frame

    def _reply(self, cmd, address, params, ack):
        if len(params) > 256:
            raise ValueError('4-way response too long')
        body = (bytes([RESP_MARK, cmd, (address >> 8) & 0xFF, address & 0xFF,
                       len(params) & 0xFF]) + bytes(params) + bytes([ack]))
        return body + struct.pack('>H', crc16_xmodem(body))

    def _handle(self, frame):
        cmd = frame[1]
        address = (frame[2] << 8) | frame[3]
        size = frame[4] or 256
        params = frame[5:5 + size]

        if cmd == CMD_PROTOCOL_GET_VERSION:
            return self._reply(cmd, address, [PROTOCOL_VERSION], ACK_OK)
        if cmd == CMD_INTERFACE_GET_NAME:
            return self._reply(cmd, address, INTERFACE_NAME, ACK_OK)
        if cmd == CMD_INTERFACE_GET_VERSION:
            return self._reply(cmd, address, list(INTERFACE_VERSION), ACK_OK)
        if cmd == CMD_INTERFACE_EXIT:
            self.exited = True
            return self._reply(cmd, address, [0], ACK_OK)
        if cmd == CMD_INTERFACE_SET_MODE:
            return self._reply(cmd, address, [0], ACK_OK)
        if cmd == CMD_DEVICE_C2CK_LOW:
            return self._reply(cmd, address, [0], ACK_OK)

        if cmd == CMD_INTERFACE_TEST_ALIVE:
            if self.target in self.connected:
                # the bootloader answers a keep alive with 0xC1 by protocol,
                # which is what keep_alive() checks for
                if not self._client(self.target).keep_alive(timeout=0.5):
                    self.connected.discard(self.target)
                    return self._reply(cmd, address, [0], ACK_D_GENERAL_ERROR)
            return self._reply(cmd, address, [0], ACK_OK)

        if cmd == CMD_DEVICE_INIT_FLASH:
            target = params[0] if params else 0
            if target >= self.esc_count:
                return self._reply(cmd, address, [0], ACK_I_INVALID_CMD)
            self.target = target
            info = self._connect(target)
            if info is None:
                return self._reply(cmd, address, [0], ACK_D_GENERAL_ERROR)
            self.log('ESC %u connected: %s' % (target, info.hex()))
            return self._reply(cmd, address, self._device_params(info), ACK_OK)

        if cmd == CMD_DEVICE_RESET:
            target = params[0] if params else 0
            if target >= self.esc_count:
                return self._reply(cmd, address, [0], ACK_I_INVALID_CMD)
            self._client(target).run()
            self.connected.discard(target)
            return self._reply(cmd, address, [0], ACK_OK)

        if cmd in (CMD_DEVICE_READ, CMD_DEVICE_READ_EEPROM):
            size = (params[0] or 256) if params else 256
            data = None
            if self.target in self.connected:
                data = self._client(self.target).read_flash(size, addr16=address)
            if data is None:
                return self._reply(cmd, address, [0], ACK_D_GENERAL_ERROR)
            return self._reply(cmd, address, data, ACK_OK)

        if cmd in (CMD_DEVICE_WRITE, CMD_DEVICE_WRITE_EEPROM):
            ok = False
            if self.target in self.connected and params:
                ok = self._client(self.target).write(address, bytes(params))
            return self._reply(cmd, address, [0],
                               ACK_OK if ok else ACK_D_GENERAL_ERROR)

        if cmd == CMD_DEVICE_PAGE_ERASE:
            # the AM32 bootloader erases as part of programming; its
            # CMD_ERASE_FLASH only validates that the page is writable
            page = params[0] if params else 0
            ok = False
            if self.target in self.connected:
                ok = self._client(self.target).erase_flash(address)
            return self._reply(cmd, address, [page],
                               ACK_OK if ok else ACK_D_GENERAL_ERROR)

        if cmd == CMD_DEVICE_ERASE_ALL:
            # never supported by the AM32 bootloader (it would erase itself)
            return self._reply(cmd, address, [0], ACK_I_INVALID_CMD)

        self.log('unknown 4-way command 0x%02x' % cmd)
        return self._reply(cmd, address, [0], ACK_I_INVALID_CMD)

    @staticmethod
    def _device_params(info):
        '''the escDeviceInfo_t a real FC returns from cmd_DeviceInitFlash,
        filled from the bootloader's 9 byte deviceInfo: signature (little
        endian) from bytes 4 and 5, then the boot version slot - which AM32
        uses for the pin code - and the boot pages slot'''
        return bytes([info[5], info[4], info[3], info[6]])
