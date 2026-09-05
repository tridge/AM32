#!/usr/bin/env python3
"""Exercise up to eight real AM32 instances through one simulated FC.

With --bootloader, also read/write each ESC through the same 4-way requests
used by am32.ca. With --usb, use a real Linux USB/IP CDC serial connection.
"""
import argparse
from contextlib import ExitStack
import glob
from pathlib import Path
import socket
import struct
import tempfile
import time

import msp_framing as framing
from msp_stub_fc import MspStubFC
import sim_runner
import sitl_params
import sitl_usbip as usb
from sitl_fourway_server import crc16_xmodem
from test_msp_betaflight import Endpoint


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sitl')
    ap.add_argument('--bootloader')
    ap.add_argument('--esc-count', type=int, choices=range(1, 9), default=8)
    ap.add_argument('--usb', action='store_true')
    args = ap.parse_args()
    binary = args.sitl or next(iter(glob.glob('obj/AM32_*SITL*.elf')), None)
    if not binary:
        ap.error('build SITL or pass --sitl')
    count = args.esc_count
    with tempfile.TemporaryDirectory(prefix='am32-multi-') as tmp, ExitStack() as stack:
        # Reserve all addresses before creating the fleet so none repeat.
        reservations = []
        for _ in range(count * 2):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', 0))
            reservations.append(sock)
        ports = [s.getsockname()[1] for s in reservations]
        for sock in reservations: sock.close()
        inputs, states = ports[:count], ports[count:]
        runners, launch_args = [], []
        for i in range(count):
            ee = sitl_params.write_eeprom(Path(__file__).parent / 'data/VIMDRONES_NANO_2216/sitl.param', Path(tmp) / ('esc%u.bin' % i))
            runner = sim_runner.SimRunner()
            stack.callback(runner.stop)
            launch = dict(binary=str(Path(binary).resolve()), eeprom=str(ee),
                          input_port=inputs[i], state_port=states[i], can_uri='none', input_type=1,
                          bootloader=str(Path(args.bootloader).resolve()) if args.bootloader else None)
            runner.start(**launch)
            launch_args.append(launch)
            runners.append(runner)
        ep = Endpoint()
        if args.usb:
            ep = usb.UsbipServer(port=0, serial='SITL-MULTI-TEST',
                                 vid=usb.BETAFLIGHT_VENDOR_ID, pid=usb.BETAFLIGHT_PRODUCT_ID)
        fc = MspStubFC(endpoint=ep, esc_ports=inputs, state_ports=states)
        stack.callback(fc.close)
        stream = None
        if args.usb:
            import serial
            # Capture the exact owned port; never detach someone else's device.
            owned = usb.attach(host=ep.host, port=ep.port)
            assert type(owned) is int, 'USB attach did not return its owned port'
            stack.callback(usb.detach, owned)
            tty = usb.find_tty(ep.serial, vid=ep.vid, pid=ep.pid)
            assert tty, 'USB serial device did not appear'
            stream = stack.enter_context(serial.Serial(tty, 115200, timeout=5))
            print('USB serial:', tty, flush=True)

        def exchange(packet, fourway=False):
            if stream is None:
                ep.rx.put(packet)
                return ep.tx.get(timeout=8)
            stream.write(packet)
            head = stream.read(5)
            assert len(head) == 5, head
            size = (head[4] or 256) + 3 if fourway else head[3] + 1
            tail = stream.read(size)
            assert len(tail) == size, tail
            return head + tail

        def request(cmd, payload=b''):
            data = exchange(framing.encode(cmd, payload, direction=b'<'))
            assert data[:3] == b'$M>' and data[4] == cmd and framing.xor(data[3:]) == 0, data
            return data[5:-1]

        def wait_for(description, predicate, timeout=25):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                request(150)
                if predicate():
                    print('PASS:', description, flush=True)
                    return
                time.sleep(0.1)
            raise AssertionError(description + ': RPM=' + str([m.rpm for m in fc.motors]))

        def fourway(cmd, address=0, data=b'\0'):
            body = bytes([0x2f, cmd, address >> 8, address & 255, len(data) & 255]) + data
            response = exchange(body + struct.pack('>H', crc16_xmodem(body)), True)
            assert response[:2] == bytes([0x2e, cmd]), response
            assert crc16_xmodem(response[:-2]) == struct.unpack('>H', response[-2:])[0], response
            assert response[-3] == 0, (cmd, response.hex())
            return response[5:-3]

        try:
            assert request(131)[6] == count
            wait_for('EDT from every ESC', lambda: all(m.edt_seen and m.volt_raw > 0 for m in fc.motors))
            # Distinct simultaneous commands; alternate outputs remain stopped.
            values = [1250 + 20 * i if i % 2 or count == 1 else 1000 for i in range(count)]
            request(214, struct.pack('<%uH' % count, *values))
            wait_for('independent motor outputs', lambda: all(
                m.rpm > 1000 if values[i] > 1000 else m.rpm == 0 for i, m in enumerate(fc.motors)))
            telemetry = request(139)
            assert telemetry[0] == count and len(telemetry) == 1 + count * 13
            for i, value in enumerate(values):
                rpm = struct.unpack_from('<I', telemetry, 1 + i * 13)[0]
                assert (rpm > 1000) if value > 1000 else (rpm == 0), (i, rpm)
            # Loss of browser polling stops every active wire.
            time.sleep(3)
            assert all(m.motor_value == 1000 for m in fc.motors)
            wait_for('all motors stop after MSP disconnect', lambda: all(m.rpm == 0 for m in fc.motors))
            if args.bootloader:
                assert request(245) == bytes([count])
                addresses = []
                for i in range(count):
                    info = fourway(0x37, data=bytes([i]))
                    assert len(info) == 4 and info[0] == 6, info
                    dev = fourway(0x3a, 0x23, bytes([27]))
                    address = struct.unpack_from('<H', dev, 23)[0]
                    settings = bytearray(fourway(0x3a, address, bytes([48])))
                    settings[26] = 20 + i
                    fourway(0x3b, address, settings)
                    addresses.append(address)
                # Revisit every target after all writes to catch cross-ESC routing.
                for i in range(count):
                    fourway(0x37, data=bytes([i]))
                    back = fourway(0x3a, addresses[i], bytes([48]))
                    assert back[26] == 20 + i, (i, back.hex())
                print('PASS: independent 4-way discovery and persistent settings for %u ESCs' % count, flush=True)
                for i in range(count): fourway(0x35, data=bytes([i]))
                fourway(0x34)
                # The CAN bootloader can retain its configuration latch.
                # Match the GUI's documented Stop/Start power-cycle workflow.
                for runner in runners: runner.stop()
                for runner, launch in zip(runners, launch_args): runner.start(**launch)
                fc.ready_at = time.monotonic() + 2.0
                wait_for('EDT returns on all ESCs after configuration and restart',
                         lambda: all(m.edt_seen for m in fc.motors))
        except Exception:
            for i, runner in enumerate(runners):
                print('ESC %u log:\n%s' % (i + 1, '\n'.join(runner.recent()[-15:])))
            raise


if __name__ == '__main__':
    main()
