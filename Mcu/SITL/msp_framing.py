"""Incremental MSPv1, native MSPv2 and MSPv2-over-v1 framing."""

import functools
import operator
import struct


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xd5 if crc & 0x80 else 0)) & 255
    return crc


def xor(data):
    return functools.reduce(operator.xor, data, 0)


def encode(cmd, payload=b'', version=1, direction=b'>'):
    if version != 1:
        body = struct.pack('<BHH', 0, cmd, len(payload)) + payload
        body += bytes([crc8(body)])
        if version == 2:
            return b'$X' + direction + body
        cmd, payload = 255, body
    body = bytes([len(payload), cmd]) + payload
    return b'$M' + direction + body + bytes([xor(body)])


class Parser:
    MAX_PAYLOAD = 1024

    def __init__(self):
        self.buf = b''

    def next(self):
        """Return (command, payload, framing), or None until more bytes arrive.

        A standalone # starts the CLI. Partial headers are retained; invalid
        CRCs/oversized packets are discarded without invoking any command.
        """
        while self.buf:
            if self.buf[0] == ord('#'):
                self.buf = self.buf[1:]
                return ('cli', b'', 1)
            if self.buf[0] != ord('$'):
                self.buf = self.buf[1:]
                continue
            if len(self.buf) < 3:
                return None
            if self.buf[:3] not in (b'$M<', b'$X<'):
                self.buf = self.buf[1:]
                continue
            native = self.buf[1] == ord('X')
            header = 8 if native else 5
            if len(self.buf) < header:
                return None
            if native:
                cmd, size = struct.unpack_from('<HH', self.buf, 4)
            else:
                size, cmd = self.buf[3:5]
            if size > self.MAX_PAYLOAD:
                self.buf = self.buf[1:]
                continue
            total = header + size + 1
            if len(self.buf) < total:
                return None
            data, self.buf = self.buf[:total], self.buf[total:]
            check = crc8(data[3:-1]) if native else xor(data[3:-1])
            if check != data[-1]:
                continue
            payload = data[header:-1]
            if not native and cmd == 255:
                if len(payload) < 6 or crc8(payload[:-1]) != payload[-1]:
                    continue
                cmd, size = struct.unpack_from('<HH', payload, 1)
                if size != len(payload) - 6:
                    continue
                return cmd, payload[5:-1], 3
            return cmd, payload, 2 if native else 1
        return None
