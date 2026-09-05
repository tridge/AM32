#!/usr/bin/env python3
"""Smoke-test the packaged Windows GUI; --usb also tests the installed driver.

Runs against temporary EEPROM storage and owns/stops the GUI and simulator.
The CI test needs no driver installation. On win11 --usb exercises a real COM
port, MSP passthrough, bootloader discovery and settings read/write.
"""
import argparse
import os
from pathlib import Path
import queue
import re
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
import time

import sitl_params
import msp_framing
from sitl_fourway_server import crc16_xmodem


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe', required=True)
    ap.add_argument('--usb', action='store_true')
    args = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix='AM32 packaged test ') as tmp:
        tmp = Path(tmp)
        ee = sitl_params.write_eeprom(Path(__file__).parent / 'data/VIMDRONES_NANO_2216/sitl.param', tmp / 'eeprom.bin')
        # Spaces in the executable path exercise quoting of the process chain.
        exe = tmp / 'am32-sitl-gui.exe'
        shutil.copyfile(args.exe, exe)
        env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
        # Verify that child executables use their bundled runtime, rather
        # than accidentally finding the build machine's Cygwin installation.
        env['PATH'] = os.pathsep.join(part for part in env.get('PATH', '').split(os.pathsep)
                                     if 'cygwin' not in part.lower())
        log = (tmp / 'gui.log').open('w')
        proc = subprocess.Popen([str(exe), '--control-port', '28472', '--port', '18470',
                                 '--state-port', '18471', '--can-uri', 'mcast:8'],
                                env=env, stdout=log, stderr=log)
        sock = None
        serial_port = None
        responses = queue.Queue()
        try:
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError('GUI exited: %s' % proc.returncode)
                try:
                    sock = socket.create_connection(('127.0.0.1', 28472), timeout=1)
                    break
                except OSError:
                    time.sleep(0.2)
            if sock is None:
                raise RuntimeError('GUI control socket never opened')
            sock.settimeout(None)
            def receive():
                try:
                    with sock.makefile('r') as stream:
                        for line in stream:
                            responses.put(line.strip())
                except OSError:
                    pass
            threading.Thread(target=receive, daemon=True).start()
            def command(cmd, prefix=None, timeout=15):
                sock.sendall((cmd + '\n').encode())
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    line = responses.get(timeout=max(0.1, deadline-time.monotonic()))
                    if line.startswith(prefix or 'OK ' + cmd):
                        print(line, flush=True)
                        return line
                    if line.startswith('ERR'):
                        raise RuntimeError(line)
                raise RuntimeError('no reply to ' + cmd)
            command('sim_set eeprom ' + str(ee))
            command('sim_input dshot')
            command('sim_start')
            time.sleep(1)
            logs = command('sim_log', 'STATUS sim_log:')
            assert command('sim_status', 'STATUS sim_process:').endswith('running'), logs
            assert 'bootloader' in logs.lower(), logs
            if args.usb:
                command('usb fourway')
                deadline = time.monotonic() + 40
                while True:
                    status = command('usb_status', 'STATUS usb:')
                    if status.startswith('STATUS usb: COM'):
                        break
                    if 'failed:' in status or time.monotonic() > deadline:
                        raise RuntimeError(status)
                    time.sleep(0.5)
                import serial
                serial_port = serial.Serial(status.split()[-1], 115200, timeout=5, write_timeout=5)
                serial_port.write(b'$M<\x00\xf5\xf5')
                assert serial_port.read(7) == b'$M>\x01\xf5\x01\xf5'
                def fourway(cmd, address=0, data=b'\0'):
                    body = bytes([0x2f, cmd, address >> 8, address & 255, len(data) & 255]) + data
                    serial_port.write(body + struct.pack('>H', crc16_xmodem(body)))
                    head = serial_port.read(5)
                    assert len(head) == 5 and head[0:2] == bytes([0x2e, cmd]), head
                    rest = serial_port.read((head[4] or 256) + 3)
                    assert len(rest) == (head[4] or 256) + 3, rest
                    assert crc16_xmodem(head + rest[:-2]) == struct.unpack('>H', rest[-2:])[0]
                    assert rest[-3] == 0, (cmd, rest.hex())
                    return rest[:-3]
                info = fourway(0x37)
                assert len(info) == 4 and info[0] == 6, info
                print('PASS: Windows COM port MSP and bootloader discovery ' + info.hex(), flush=True)
                dev = fourway(0x3a, 0x23, bytes([27]))
                assert dev[:8] == struct.pack('<II', 0x5925E3DA, 0x4EB863D9)
                address = struct.unpack('<H', dev[23:25])[0]
                settings = bytearray(fourway(0x3a, address, bytes([48])))
                settings[26] = (settings[26] + 1) & 255
                fourway(0x3b, address, settings)
                back = fourway(0x3a, address, bytes([48]))
                assert back[:2] + back[3:] == settings[:2] + settings[3:]
                print('PASS: Windows USB settings read/write/readback', flush=True)
                fourway(0x35)
                fourway(0x34)
                serial_port.close()
                serial_port = None
                command('usb none')
                assert command('usb_status', 'STATUS usb:').endswith('off')
                # Switching modes must detach the first device and create a
                # working direct linker, including its serial echo.
                command('usb serial')
                deadline = time.monotonic() + 40
                while True:
                    status = command('usb_status', 'STATUS usb:')
                    if status.startswith('STATUS usb: COM'):
                        break
                    if 'failed:' in status or time.monotonic() > deadline:
                        raise RuntimeError(status)
                    time.sleep(0.5)
                serial_port = serial.Serial(status.split()[-1], 19200, timeout=8, write_timeout=5)
                probe = bytes(12) + bytes([0x0d]) + b'BLHeli' + bytes([0xf4, 0x7d])
                serial_port.write(probe)
                answer = serial_port.read(len(probe) + 9)
                assert answer[:len(probe)] == probe, answer.hex()
                assert answer[len(probe):len(probe)+3] == b'471' and answer[-1] == 0x30, answer.hex()
                serial_port.write(bytes(4))  # CMD_RUN, returning to the application
                assert serial_port.read(4) == bytes(4)
                serial_port.close()
                serial_port = None
                command('usb none')
                assert command('usb_status', 'STATUS usb:').endswith('off')
                print('PASS: direct USB serial probe, echo and mode switching', flush=True)
                # Restart to load the edited settings, as a real ESC would
                # be power-cycled after configuration.
                command('sim_stop')
                command('sim_start')
                time.sleep(1)
            command('ds_type dshot600')
            command('ds_bidir 1')
            command('ds_value 0')
            command('ds_enable 1')
            time.sleep(6)
            logs = command('sim_log', 'STATUS sim_log:')
            assert 'PWM/DShot input on udp port' in logs, logs
            command('ds_value 1000')
            deadline = time.monotonic() + 15
            while True:
                status = command('status', 'STATUS BDShot')
                rpm = re.search(r'rpm=\s*(\d+)', status)
                if rpm and int(rpm.group(1)) > 1000:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('no motor telemetry: ' + status)
                time.sleep(1)
            print('PASS: bundled firmware runs the motor after bootloader handoff', flush=True)
            command('ds_value 0')
            if args.usb:
                command('ds_enable 0')
                command('usb betaflight')
                deadline = time.monotonic() + 40
                while True:
                    status = command('usb_status', 'STATUS usb:')
                    if status.startswith('STATUS usb: COM'):
                        break
                    if 'failed:' in status or time.monotonic() > deadline:
                        raise RuntimeError(status)
                    time.sleep(0.5)
                serial_port = serial.Serial(status.split()[-1], 115200, timeout=5, write_timeout=5)
                def msp(cmd, data=b'', version=1):
                    serial_port.write(msp_framing.encode(cmd, data, version, b'<'))
                    head = serial_port.read(5 if version == 1 else 8)
                    assert head[:3] == (b'$M>' if version == 1 else b'$X>'), head
                    size = head[3] if version == 1 else struct.unpack_from('<H', head, 6)[0]
                    tail = serial_port.read(size + 1)
                    assert len(tail) == size + 1, tail
                    check = msp_framing.xor if version == 1 else msp_framing.crc8
                    assert check(head[3:] + tail[:-1]) == tail[-1]
                    return tail[:-1]
                assert msp(1) == bytes([0, 1, 46])
                assert msp(131)[6] == 1
                assert struct.unpack('<9h', msp(102)) == (0, 0, 512, 0, 0, 0, 0, 0, 0)
                assert msp(0x3006, b'\2', 2) == b'\2\x09AM32 SITL'
                msp(0x3003, bytes([1, 255, 1, 13]), 2)
                for _ in range(35):
                    msp(150)
                    time.sleep(0.1)
                msp(214, struct.pack('<8H', 1350, *([1000] * 7)))
                deadline = time.monotonic() + 15
                while True:
                    telem = msp(139)
                    if struct.unpack_from('<I', telem, 1)[0] > 1000:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError('Betaflight COM motor did not spin: ' + telem.hex())
                    time.sleep(0.1)
                msp(214, struct.pack('<H', 1000))
                msp(250)
                assert Path(str(ee) + '.fc.json').exists()
                msp(68)
                serial_port.close()
                serial_port = None
                time.sleep(2)
                deadline = time.monotonic() + 40
                while True:
                    status = command('usb_status', 'STATUS usb:')
                    if status.startswith('STATUS usb: COM'):
                        break
                    if 'failed:' in status or time.monotonic() > deadline:
                        raise RuntimeError(status)
                    time.sleep(0.5)
                serial_port = serial.Serial(status.split()[-1], 115200, timeout=5, write_timeout=5)
                assert msp(131)[7:9] == bytes([14, 1])
                serial_port.close()
                serial_port = None
                command('usb none')
                print('PASS: Betaflight COM handshake, sensors, MSPv2, motor/telemetry and FC settings', flush=True)
            command('sim_stop')
            assert command('sim_status', 'STATUS sim_process:').endswith('stopped')
            sock.sendall(b'quit\n')  # quit closes the GUI without an OK reply
            proc.wait(timeout=20)
            assert proc.returncode == 0, proc.returncode
            print('PASS: packaged defaults, paths with spaces, launch and teardown', flush=True)
        except Exception:
            if sock and proc.poll() is None:
                try:
                    command('sim_log', 'STATUS sim_log:', timeout=5)
                except Exception:
                    pass
            raise
        finally:
            if serial_port:
                serial_port.close()
            if proc.poll() is None:
                if sock:
                    try:
                        sock.sendall(b'usb none\nsim_stop\nquit\n')
                        proc.wait(timeout=25)
                    except (OSError, subprocess.TimeoutExpired):
                        pass
                if proc.poll() is None:
                    subprocess.run(['taskkill', '/PID', str(proc.pid), '/T', '/F'], check=False)
                    proc.wait(timeout=10)
            if sock:
                sock.close()
            log.close()
            print((tmp / 'gui.log').read_text(errors='replace')[-5000:], flush=True)


if __name__ == '__main__':
    main()
