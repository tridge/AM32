"""Portable regressions for Windows packaging and USB/IP protocol replies."""
import os
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import package_windows
import sim_runner
import sitl_usbip as usb


class WindowsPackageTests(unittest.TestCase):
    def test_closing_unattached_exporter_releases_listener(self):
        server = usb.UsbipServer(port=0)
        port = server.port
        server.close()
        self.assertFalse(server.thread.is_alive())
        replacement = usb.UsbipServer(port=port)
        replacement.close()

    @unittest.skipUnless(hasattr(os, 'geteuid'), 'Linux privilege helper')
    def test_linux_attach_retains_owned_port(self):
        result = Mock(returncode=0, stdout='3\n')
        with patch.object(usb, 'IS_WINDOWS', False), patch.object(os, 'geteuid', return_value=1000), patch.object(usb, 'privilege_prefix', return_value=['sudo']), patch.object(usb.subprocess, 'run', return_value=result):
            self.assertEqual(usb.attach(port=3299), 3)
            result.stdout = '0\n'
            self.assertIs(type(usb.attach(port=3299)), int)
            result.returncode = 1
            self.assertIs(usb.attach(port=3299), False)

    def test_usb_identity_matches_enumeration_and_device_list(self):
        # All identities must coexist without changing another exporter's
        # descriptors. Direct mode also selects AM32's raw linker protocol.
        with_id = usb.UsbipServer(port=0, vid=usb.BETAFLIGHT_VENDOR_ID,
                                   pid=usb.BETAFLIGHT_PRODUCT_ID)
        default = usb.UsbipServer(port=0)
        direct = usb.UsbipServer(port=0, vid=usb.DIRECT_VENDOR_ID,
                                pid=usb.DIRECT_PRODUCT_ID)
        try:
            setup = struct.pack('<BBHHH', 0x80, usb.REQ_GET_DESCRIPTOR,
                                0x0100, 0, 18)
            for server, expected in ((with_id, (0x0483, 0x5740)),
                                     (default, (0x1209, 0x0001)),
                                     (direct, (0x1a86, 0x0001))):
                status, descriptor = server._control(setup, b'', 18)
                self.assertEqual(status, usb.ST_OK)
                self.assertEqual(struct.unpack_from('<HH', descriptor, 8), expected)
                self.assertEqual(struct.unpack_from('>HH', server._usb_device(), 300), expected)
                self.assertEqual(server.strings[:2], ['AM32', 'AM32 SITL serial'])
        finally:
            with_id.close()
            default.close()
            direct.close()

    def test_out_completion_reports_bytes_without_returning_payload(self):
        server = usb.UsbipServer.__new__(usb.UsbipServer)
        server.send_lock = threading.Lock()
        server.rx_lock = threading.Condition()
        server.rx = b''
        server.rx_max = 1024
        server.conn = Mock()
        server._submit(17, usb.DIR_OUT, usb.EP_BULK, 4, b'', b'abcd')
        packet = server.conn.sendall.call_args.args[0]
        self.assertEqual(len(packet), 48)
        self.assertEqual(struct.unpack('>ii', packet[20:28]), (0, 4))
        self.assertEqual(server.rx, b'abcd')

    def test_attach_retains_exact_owned_port(self):
        result = Mock(returncode=0, stdout='7\n', stderr='')
        with patch.object(usb, '_windows_usbip_checked', return_value=('usbip.exe', (0,9,7,7))), patch.object(usb.subprocess, 'run', return_value=result) as run:
            self.assertEqual(usb._windows_attach('127.0.0.1', 3299, '1-1'), 7)
            self.assertIn('--once', run.call_args.args[0])
            self.assertIn('3299', run.call_args.args[0])
            result.stdout = 'attached, possibly port 7'
            with self.assertRaises(RuntimeError):
                usb._windows_attach('127.0.0.1', 3299, '1-1')

    def test_detach_never_guesses_port(self):
        for port in (None, True, False):
            with self.assertRaises(RuntimeError):
                usb._windows_detach(port)
        with patch.object(usb, '_windows_usbip_checked', return_value=('usbip.exe', (0,9,7,7))), patch.object(usb.subprocess, 'run', return_value=Mock(returncode=0)) as run:
            usb._windows_detach(7)
            self.assertEqual(run.call_args.args[0], ['usbip.exe', 'detach', '--port', '7'])

    def test_reject_known_bad_driver(self):
        with patch.object(usb, 'windows_usbip_executable', return_value='usbip.exe'), patch.object(usb, 'windows_usbip_version', return_value=(0,9,7,8)):
            with self.assertRaises(RuntimeError):
                usb._windows_usbip_checked()

    def test_eeprom_survives_bundle_replacement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'sitl').mkdir()
            (root / 'sitl/default_eeprom.bin').write_bytes(b'original')
            with patch.object(sim_runner, '_resource_dir', return_value=tmp), patch.object(sim_runner.sys, 'platform', 'win32'), patch.dict(os.environ, LOCALAPPDATA=str(root / 'user data')):
                ee = Path(sim_runner.bundled_eeprom())
                self.assertEqual(ee.read_bytes(), b'original')
                ee.write_bytes(b'user settings')
                (root / 'sitl/default_eeprom.bin').write_bytes(b'new bundle')
                self.assertEqual(Path(sim_runner.bundled_eeprom()).read_bytes(), b'user settings')

    def test_missing_bootloader_is_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            existing = Path(tmp) / 'file'
            existing.touch()
            with self.assertRaisesRegex(RuntimeError, 'bootloader file does not exist'):
                sim_runner.SimRunner().start(str(existing), str(existing), bootloader=str(existing) + '.missing')

    def test_readme_has_windows_line_endings(self):
        with tempfile.TemporaryDirectory() as tmp:
            src, dst = Path(tmp) / 'src', Path(tmp) / 'dst'
            src.write_bytes(b'a\nb\r\nc\n')
            package_windows.text_copy(src, dst)
            self.assertEqual(dst.read_bytes(), b'a\r\nb\r\nc\r\n')


if __name__ == '__main__':
    unittest.main()
