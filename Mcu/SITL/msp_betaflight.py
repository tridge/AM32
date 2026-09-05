"""Betaflight API 1.46 configuration and stationary FC sensor responses.

Only the connection, Setup and Motors workflows are emulated. These are FC
settings, separate from the AM32 EEPROM accessed through 4-way passthrough.
Wire layouts follow Betaflight src/main/msp/msp.c (API 1.46).
"""

import copy
import json
import os
import struct
from pathlib import Path


def pack(fmt, *values):
    return struct.pack('<' + fmt, *values)


def pstring(value):
    value = value.encode('ascii')
    return bytes([len(value)]) + value


class Configuration:
    def __init__(self, poles=14, path=None, motor_count=1):
        if not 1 <= motor_count <= 8:
            raise ValueError('motor count must be 1..8')
        self.motor_count = motor_count
        self.path = Path(path) if path else None
        # Fields irrelevant to an ESC bench are retained so the Motors tab
        # can round-trip them when saving a motor protocol change.
        advanced = pack('BBBBHHBBBBBHHBBB',
                        1, 4, 0, 7, 480, 550, 0, 0, 0, 0, 48, 125, 0, 0, 0, 1)
        pid = bytearray(61)
        pid[47] = 100  # motor_output_limit; no flight dynamics/PID emulation
        self.defaults = {
            'poles': poles, 'bidir': True, 'edt': 'ON',
            'minthrottle': 1070, 'maxthrottle': 2000, 'mincommand': 1000,
            'registers': {
                36: pack('I', 0),             # features (no receiver/3D)
                # With eight populated MSP_MOTOR slots there is no zero
                # sentinel. Betaflight then takes its slider count from the
                # mixer (CUSTOM has zero); use OCTOFLATX for a full bench.
                42: bytes([13 if motor_count == 8 else 23, 0]),
                61: bytes([5, 0, 25]),         # arming config, stays disarmed
                90: advanced,
                92: bytes(49),                # filters (not used by ESC)
                94: bytes(pid),
                124: pack('HHH', 1406, 1514, 1460),
            },
        }
        self.values = copy.deepcopy(self.defaults)
        if self.path and self.path.exists():
            stored = json.loads(self.path.read_text())
            regs = {int(k): bytes.fromhex(v) for k, v in stored.pop('registers').items()}
            if set(stored) != set(self.defaults) - {'registers'}:
                raise ValueError('invalid simulated FC configuration keys')
            for k, v in regs.items():
                if k not in self.defaults['registers'] or len(v) != len(self.defaults['registers'][k]):
                    raise ValueError('invalid simulated FC configuration register')
            self.values.update(stored)
            self.values['registers'].update(regs)
            self.validate()
        # The GUI's ESC count defines the outputs, including when reusing
        # saved FC settings from a bench with a different number of ESCs.
        mixer = self.values['registers'][42]
        self.values['registers'][42] = self.defaults['registers'][42][:1] + mixer[1:]
        self.saved = copy.deepcopy(self.values)

    def validate(self):
        v = self.values
        if not isinstance(v['poles'], int) or not 2 <= v['poles'] <= 100 or v['poles'] % 2:
            raise ValueError('motor_poles must be even, between 2 and 100')
        if type(v['bidir']) is not bool or v['edt'] not in ('OFF', 'ON', 'FORCE'):
            raise ValueError('invalid DShot telemetry setting')
        if self.protocol not in (5, 6, 7, 9):
            raise ValueError('supported protocols: DSHOT150, DSHOT300, DSHOT600, DISABLED')
        for key in ('minthrottle', 'maxthrottle', 'mincommand'):
            if not isinstance(v[key], int) or not 1000 <= v[key] <= 2000:
                raise ValueError('motor endpoints must be between 1000 and 2000')

    @property
    def protocol(self):
        return self.values['registers'][90][3]

    def save(self):
        self.validate()
        if self.path:
            stored = copy.deepcopy(self.values)
            stored['registers'] = {str(k): v.hex() for k, v in stored['registers'].items()}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(self.path.suffix + '.tmp')
            tmp.write_text(json.dumps(stored, indent=2) + '\n')
            os.replace(tmp, self.path)
        self.saved = copy.deepcopy(self.values)

    def reboot(self):
        self.values = copy.deepcopy(self.saved)

    def set_motor(self, payload):
        if len(payload) != 8:
            raise ValueError('expected 8-byte motor config')
        lo, hi, stop, poles, bidir = struct.unpack('<HHHBB', payload)
        old = copy.deepcopy(self.values)
        self.values.update(minthrottle=lo, maxthrottle=hi, mincommand=stop,
                           poles=poles, bidir=bool(bidir))
        try:
            self.validate()
        except ValueError:
            self.values = old
            raise

    def set_register(self, cmd, payload):
        get_cmd = {37: 36, 43: 42, 62: 61, 91: 90, 93: 92, 95: 94, 217: 124}[cmd]
        old = self.values['registers'][get_cmd]
        # ADVANCED_CONFIG has a read-only final debugModeCount byte.
        expected = len(old) - (1 if get_cmd == 90 else 0)
        if len(payload) != expected:
            raise ValueError('invalid configuration payload length')
        if get_cmd == 36 and struct.unpack('<I', payload)[0] & ((1 << 12) | (1 << 27)):
            raise ValueError('3D and UART ESC_SENSOR are not emulated')
        if get_cmd == 42 and payload[0] != self.defaults['registers'][42][0]:
            raise ValueError('set the ESC count in the simulator GUI')
        self.values['registers'][get_cmd] = payload + (old[-1:] if get_cmd == 90 else b'')
        try:
            self.validate()
        except ValueError:
            self.values['registers'][get_cmd] = old
            raise

    def cli_get(self):
        return {'motor_pwm_protocol': {5: 'DSHOT150', 6: 'DSHOT300', 7: 'DSHOT600', 9: 'DISABLED'}[self.protocol],
                'dshot_bidir': 'ON' if self.values['bidir'] else 'OFF',
                'dshot_edt': self.values['edt'], 'motor_poles': str(self.values['poles'])}

    def cli_set(self, key, value):
        value = value.upper()
        if key == 'motor_pwm_protocol':
            protocols = {'DSHOT150': 5, 'DSHOT300': 6, 'DSHOT600': 7, 'DISABLED': 9}
            if value not in protocols:
                raise ValueError('invalid motor protocol')
            data = bytearray(self.values['registers'][90])
            data[3] = protocols[value]
            self.values['registers'][90] = bytes(data)
        elif key == 'dshot_bidir' and value in ('ON', 'OFF'):
            self.values['bidir'] = value == 'ON'
        elif key == 'dshot_edt' and value in ('OFF', 'ON', 'FORCE'):
            self.values['edt'] = value
        elif key == 'motor_poles' and value.isdigit() and 2 <= int(value) <= 100 and int(value) % 2 == 0:
            self.values['poles'] = int(value)
        else:
            raise ValueError('unknown setting or invalid value')

    def read(self, cmd, payload=b'', voltage=12.6, current=0):
        v = self.values
        voltage = max(0, min(655.35, voltage))
        current = max(0, min(327.67, current))
        cv, ca = round(voltage * 100), round(current * 100)
        status = pack('HHHIBH', 500, 0, 0x21, 0, 0, 0)  # ACC + GYRO
        status += bytes([3, 0]) if cmd == 150 else bytes(2)
        status += pack('BBIBH', 0, 26, 0, int(v != self.saved), 25)
        replies = {
            1: bytes([0, 1, 46]), 2: b'BTFL', 3: bytes([4, 6, 0]),
            4: (b'SITL' + pack('HBB', 0, 0, 0) + pstring('AM32_SITL')
                + pstring('AM32 SITL') + pstring('AM32') + bytes(32)
                + pack('BBHIBB', 255, 2, 8000, 0, 0, 0)),
            5: b'Jan 01 202600:00:00' + b'0000000' + bytes(2),
            10: b'AM32 SITL',
            32: pack('BBB HBB HHH', 33, 43, 35, 1500, 1, 1, 330, 430, 350),
            38: bytes(6), 58: bytes(4), 70: bytes(13), 79: bytes(11),
            80: bytes(11), 96: bytes([1, 1, 1, 1]),
            101: status, 150: status,
            102: pack('9h', 0, 0, 512, 0, 0, 0, 0, 0, 0),
            105: pack('8H', 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000),
            108: bytes(6), 109: bytes(6),
            110: pack('BHHhH', min(255, round(voltage * 10)), 0, 0, ca, cv),
            116: b'ARM;', 119: b'\0', 125: pack('BBBH', 0, 0, 0, 50),
            126: bytes([0, 0, 0, 1, 0, 0, 0]),
            128: pack('BB', 10, min(255, round(voltage * 10))),
            129: pack('BHH', 10, 0, ca),
            130: pack('BHBHHBH', max(1, round(voltage / 4.2)), 1500,
                      min(255, round(voltage * 10)), 0, ca, 0, cv),
            131: pack('HHHBBBB', v['minthrottle'], v['maxthrottle'], v['mincommand'],
                      self.motor_count, v['poles'], v['bidir'], 0),
            160: pack('III', 0x414d3332, 0x5349544c, 1),
            240: bytes(4), 254: bytes(8),
            0x3001: bytes([self.motor_count]) + bytes(range(self.motor_count)),
            0x300a: bytes([1, 1, 0, 0, 0]),
            0x300c: bytes([255]) + pstring('SITL'),
        }
        if cmd == 0x3006 and len(payload) == 1:
            return payload + pstring({1: '', 2: 'AM32 SITL', 3: 'SITL', 4: 'SITL', 5: ''}.get(payload[0], ''))
        return v['registers'].get(cmd, replies.get(cmd))
