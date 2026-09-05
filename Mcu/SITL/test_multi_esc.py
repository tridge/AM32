"""Multiple ESC motor outputs, telemetry and configurator target isolation."""
import socket
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import msp_stub_fc
import sitl_dshot as sd
import sitl_fourway_server as fw
import sim_runner
import test_msp_betaflight as single


class MultiEscTests(unittest.TestCase):
    request = single.BetaflightTests.request
    def setUp(self):
        self.sockets = []
        for _ in range(8):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', 0))
            sock.settimeout(0.1)
            self.sockets.append(sock)
        single.BetaflightTests.setUp(self)
        self.fc.close()
        self.fc = msp_stub_fc.MspStubFC(endpoint=self.endpoint, motor=False,
            esc_ports=[s.getsockname()[1] for s in self.sockets], state_port=0)

    def tearDown(self):
        single.BetaflightTests.tearDown(self)
        for sock in self.sockets:
            sock.close()

    def test_eight_outputs_and_atomic_validation(self):
        self.assertEqual(self.request(131)[6], 8)
        self.assertEqual(self.request(42), bytes([13, 0]))
        self.assertEqual(self.request(0x3001, version=2), bytes([8]) + bytes(range(8)))
        values = list(range(1100, 1900, 100))
        self.request(214, struct.pack('<8H', *values))
        self.assertEqual(struct.unpack('<8H', self.request(104)), tuple(values))
        self.request(214, struct.pack('<7H', *values[:7]), error=True)
        self.request(214, struct.pack('<8H', *(values[:7] + [999])), error=True)
        self.assertEqual([m.motor_value for m in self.fc.motors], values)
        self.assertEqual(self.request(245), b'\x08')
        self.assertTrue(all(m.motor_value == 1000 for m in self.fc.motors))

    def test_commands_and_telemetry_are_per_motor(self):
        self.request(0x3003, bytes([1, 7, 1, 14]), 2)
        self.assertEqual([m.edt_override for m in self.fc.motors], [None] * 7 + [False])
        self.assertEqual([list(m.commands) for m in self.fc.motors], [[]] * 7 + [[[14, 20]]])
        self.request(0x3003, bytes([1, 8, 1, 13]), 2, error=True)
        self.request(0x3003, bytes([1, 255, 1, 13]), 2)
        self.assertTrue(all(m.edt_override for m in self.fc.motors))
        for i, motor in enumerate(self.fc.motors):
            motor.rpm, motor.temp, motor.volt_raw, motor.curr_raw = 1000 + i, 30 + i, 48, i
        reply = self.request(139)
        self.assertEqual(len(reply), 1 + 8 * 13)
        for i in range(8):
            rpm, invalid, temp, volt, current, consumption = struct.unpack_from('<IHBHHH', reply, 1 + 13 * i)
            self.assertEqual((rpm, temp, volt, current), (1000 + i, 30 + i, 12, i))

    def test_wire_routing_and_reboot_targets(self):
        values = [1000 + i * 100 for i in range(8)]
        self.request(214, struct.pack('<8H', *values))
        self.fc.ready_at = 0
        self.fc.config.values['edt'] = 'OFF'
        for motor in self.fc.motors:
            motor.edt_commanded = False
            self.fc._send_motor(motor, 1.0)
        for i, sock in enumerate(self.sockets):
            packet = sd.unpack(sock.recv(1024))
            expected = 0 if i == 0 else 48 + int(i * 100 * 1999 / 1000)
            self.assertEqual(packet[2] >> 5, expected)
        self.fc.motor_enabled = True
        with patch.object(self.fc.fourway, '_reset_esc') as reset:
            self.request(222, struct.pack('<HHHBB', 1070, 2000, 1000, 14, 0))
            self.request(250)
            self.request(68)
            self.assertEqual([call.args[0] for call in reset.call_args_list], list(range(8)))
        self.assertTrue(all(m.motor_value == 1000 for m in self.fc.motors))


class MultiEscStorageTests(unittest.TestCase):
    def test_reset_only_reaches_selected_state_port(self):
        sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(8)]
        try:
            for sock in sockets:
                sock.bind(('127.0.0.1', 0))
                sock.settimeout(0.05)
            server = fw.FourWayServer(esc_ports=list(range(18000, 18008)),
                state_ports=[s.getsockname()[1] for s in sockets])
            try:
                server._reset_esc(7)
                self.assertEqual(sockets[7].recv(32), struct.pack('<HBB', fw.STATE_MAGIC_CMD, fw.STATE_CMD_RESET, 0))
                for sock in sockets[:7]:
                    with self.assertRaises(socket.timeout): sock.recv(32)
            finally:
                server.close()
        finally:
            for sock in sockets: sock.close()

    def test_separate_persistent_eeproms(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'sitl').mkdir()
            (root / 'sitl/default_eeprom.bin').write_bytes(b'default')
            with patch.object(sim_runner, '_resource_dir', return_value=tmp), patch.object(sim_runner.sys, 'platform', 'win32'), patch.dict(sim_runner.os.environ, LOCALAPPDATA=str(root / 'user')):
                paths = [Path(sim_runner.bundled_eeprom(i)) for i in range(8)]
                self.assertEqual(len(set(paths)), 8)
                for i, path in enumerate(paths): path.write_bytes(bytes([i]))
                for i in range(8):
                    self.assertEqual(Path(sim_runner.bundled_eeprom(i)).read_bytes(), bytes([i]))


if __name__ == '__main__':
    unittest.main()
