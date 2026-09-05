"""Regressions for the simulated FC's Betaflight connection and motor control."""
import queue
import socket
import struct
import tempfile
import time
import unittest
from pathlib import Path

import msp_betaflight
import msp_framing as framing
import msp_stub_fc
import sitl_dshot as sd


class Endpoint:
    path = 'test'

    def __init__(self):
        self.rx, self.tx = queue.Queue(), queue.Queue()

    def read(self, timeout=0.1):
        try:
            return self.rx.get(timeout=timeout)
        except queue.Empty:
            return b''

    def write(self, data):
        self.tx.put(data)

    def drain(self):
        return self.read(0)

    def close(self):
        pass


class BetaflightTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.endpoint = Endpoint()
        self.fc = msp_stub_fc.MspStubFC(endpoint=self.endpoint, motor=False,
            config_path=str(Path(self.tmp.name) / 'fc.json'))

    def tearDown(self):
        self.fc.close()
        self.tmp.cleanup()

    def request(self, cmd, payload=b'', version=1, error=False):
        self.endpoint.rx.put(framing.encode(cmd, payload, version, b'<'))
        response = self.endpoint.tx.get(timeout=2)
        self.assertEqual(response[2:3], b'!' if error else b'>')
        # Check framing independently of the server's incremental parser.
        if version == 1:
            self.assertEqual(response[4], cmd)
            self.assertEqual(framing.xor(response[3:]), 0)
            return response[5:-1]
        if version == 3:
            self.assertEqual(response[4], 255)
            self.assertEqual(framing.xor(response[3:]), 0)
            response = b'$X>' + response[5:-1]
        self.assertEqual(struct.unpack_from('<H', response, 4)[0], cmd)
        self.assertEqual(framing.crc8(response[3:-1]), response[-1])
        return response[8:-1]

    def test_handshake_and_stationary_sensors(self):
        self.assertEqual(self.request(1), b'\0\1\x2e')
        self.assertEqual(self.request(2), b'BTFL')
        for cmd in (3, 4, 5, 101, 160, 32, 80, 70, 79, 150, 240, 126, 90):
            self.assertTrue(self.request(cmd), cmd)
        self.assertEqual(self.request(0x3006, b'\2', 2), b'\2\x09AM32 SITL')
        self.request(99, b'\1\0')
        self.request(246, bytes(6))
        self.assertEqual(struct.unpack('<9h', self.request(102)), (0, 0, 512, 0, 0, 0, 0, 0, 0))
        self.assertEqual(self.request(108), bytes(6))
        status = self.request(150)
        self.assertEqual(len(status), 24)
        self.assertEqual(struct.unpack_from('<HI', status, 4), (0x21, 0))
        self.assertEqual(self.request(131)[6], 1)
        self.assertEqual(self.request(0x3001, version=2), b'\1\0')
        self.assertEqual(len(self.request(139)), 14)

    def test_all_framings_and_split_headers(self):
        for version in (1, 2, 3):
            self.assertEqual(self.request(1, version=version), b'\0\1\x2e')
            wire = framing.encode(2, version=version, direction=b'<')
            for byte in wire:
                self.endpoint.rx.put(bytes([byte]))
            self.assertIn(b'BTFL', self.endpoint.tx.get(timeout=2))
        bad = bytearray(framing.encode(214, struct.pack('<H', 1800), direction=b'<'))
        bad[-1] ^= 1
        self.endpoint.rx.put(bytes(bad) + b'noise')
        self.assertEqual(self.request(104)[:2], struct.pack('<H', 1000))
        self.request(214, b'\0', error=True)
        self.request(214, struct.pack('<H', 2500), error=True)
        self.assertEqual(self.fc.motor_value, 1000)
        self.request(0x3fff, version=2, error=True)

    def test_settings_save_reboot_and_reload(self):
        self.request(222, struct.pack('<HHHBB', 1070, 2000, 1000, 12, 0))
        advanced = bytearray(self.request(90)[:-1]); advanced[3] = 6
        self.request(91, bytes(advanced))
        self.request(250)
        self.request(214, struct.pack('<H', 1400))
        self.request(68)
        self.assertEqual(self.fc.motor_value, 1000)
        self.assertEqual(self.request(131)[7:9], bytes([12, 0]))
        c = msp_betaflight.Configuration(path=self.fc.config.path)
        self.assertEqual(c.protocol, 6)
        self.assertEqual(c.values['poles'], 12)
        self.request(222, struct.pack('<HHHBB', 1070, 2000, 1000, 0, 1), error=True)
        self.assertEqual(self.request(131)[7], 12)
        advanced[3] = 8
        self.request(91, bytes(advanced), error=True)
        self.assertEqual(self.request(90)[3], 6)

    def test_cli_and_dshot_commands(self):
        self.request(214, struct.pack('<H', 1500))
        self.endpoint.rx.put(b'#')
        self.assertIn(b'Entering CLI', self.endpoint.tx.get(timeout=2))
        self.assertEqual(self.fc.motor_value, 1000)
        self.endpoint.rx.put(b'set dshot_edt = OFF\r\nsave\r\n')
        deadline = time.monotonic() + 2
        text = b''
        while b'Rebooting' not in text and time.monotonic() < deadline:
            text += self.endpoint.tx.get(timeout=2)
        self.assertIn(b'dshot_edt set to OFF', text)
        self.assertEqual(self.fc.config.values['edt'], 'OFF')
        self.assertFalse(self.fc.cli)
        self.assertEqual(self.request(1), b'\0\1\x2e')
        self.request(0x3003, bytes([1, 255, 1, 13]), 2)
        self.assertTrue(self.fc.edt_override)
        self.request(0x3003, bytes([1, 0, 1, 14]), 3)
        self.assertFalse(self.fc.edt_override)
        self.request(0x3003, bytes([1, 2, 1, 13]), 2, error=True)

    def test_reboot_notification_follows_saved_state(self):
        notifications = []
        self.fc.on_reboot = lambda fc: notifications.append(fc.config.values['poles'])
        self.request(222, struct.pack('<HHHBB', 1070, 2000, 1000, 12, 1))
        self.request(250)
        self.request(68)
        self.assertEqual(notifications, [12])
        self.assertEqual(self.fc.motor_value, 1000)

    def test_motor_output_and_passthrough_exclusion(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 0)); sock.settimeout(1)
        self.fc.close()
        self.fc = msp_stub_fc.MspStubFC(endpoint=self.endpoint,
            sitl_port=sock.getsockname()[1], motor=True)
        try:
            self.fc.ready_at = 0
            self.fc.config.values['edt'] = 'OFF'
            self.request(214, struct.pack('<H', 1500))
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                packet = sd.unpack(sock.recv(1024))
                if packet[2] >> 5 >= 1000:
                    break
            self.assertEqual(packet[0], sd.TYPE_DSHOT600)
            self.assertTrue(packet[1] & sd.FLAG_IDLE_HIGH)
            self.assertGreater(packet[2] >> 5, 1000)
            self.request(245)
            # Drain frames emitted before the passthrough acknowledgement.
            sock.setblocking(False)
            try:
                while True: sock.recv(1024)
            except BlockingIOError:
                pass
            sock.settimeout(0.1)
            with self.assertRaises(socket.timeout): sock.recv(1024)
            self.assertEqual(self.fc.motor_value, 1000)
        finally:
            sock.close()


if __name__ == '__main__':
    unittest.main()
