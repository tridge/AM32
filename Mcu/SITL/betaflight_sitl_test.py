#!/usr/bin/env python3
"""Exercise Betaflight MSP motor commands against real AM32 SITL firmware."""
import argparse
import glob
from pathlib import Path
import queue
import socket
import struct
import subprocess
import tempfile
import time

import msp_framing as framing
import msp_stub_fc
import sitl_dshot as sd
import sitl_params
from test_msp_betaflight import Endpoint
from sitl_gui_backend import SimStream


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sitl', default=None)
    args = ap.parse_args()
    candidates = glob.glob('obj/AM32_*SITL*.elf')
    binary = args.sitl or (candidates[0] if candidates else None)
    if not binary:
        ap.error('build AM32_SITL_CAN or pass --sitl')
    with tempfile.TemporaryDirectory(prefix='am32-betaflight-') as tmp:
        ee = sitl_params.write_eeprom(Path(__file__).parent / 'data/VIMDRONES_NANO_2216/sitl.param', Path(tmp) / 'eeprom.bin')
        port = free_udp_port()
        state_port = free_udp_port()
        ep = Endpoint()
        fc = msp_stub_fc.MspStubFC(endpoint=ep, sitl_port=port,
                                  state_port=state_port, config_path=str(Path(tmp) / 'fc.json'))
        stream = SimStream(port=state_port, period_us=10000, maxlen=2)
        stream.enabled = True
        log = (Path(tmp) / 'firmware.log').open('w')
        proc = subprocess.Popen([binary, '--eeprom', str(ee), '--input-port', str(port),
            '--state-port', str(state_port), '--input-type', '1', '--can-uri', 'none', '--nosleep'],
            stdout=log, stderr=log)

        def request(cmd, payload=b'', version=1):
            ep.rx.put(framing.encode(cmd, payload, version, b'<'))
            data = ep.tx.get(timeout=3)
            assert data[2:3] == b'>', (cmd, data)
            return data[5:-1] if version == 1 else data[8:-1]

        def wait_for(description, predicate, timeout=12):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                request(150)  # normal app background polling
                if predicate():
                    print('PASS: ' + description, flush=True)
                    return
                time.sleep(0.1)
            raise AssertionError(description)

        try:
            wait_for('bidirectional DShot and EDT from firmware', lambda: fc.edt_seen and fc.temp > 0 and fc.volt_raw > 0)
            request(214, struct.pack('<8H', 1350, *([1000] * 7)))
            wait_for('MSP motor slider spins AM32', lambda: fc.rpm > 1500)
            data = request(139)
            assert data[0] == 1 and struct.unpack_from('<I', data, 1)[0] > 1500
            request(214, struct.pack('<8H', *([1000] * 8)))
            wait_for('MSP motor stop', lambda: fc.rpm == 0)
            # Observe the actual decoded wire replies, rather than accepting
            # a cached telemetry value as evidence that disable succeeded.
            replies = queue.Queue()
            original = fc.port.get_replies
            def observed():
                data = original()
                for r in data:
                    replies.put((time.monotonic(), sd.decode_reply(r[3], True)[0]))
                return data
            fc.port.get_replies = observed
            request(0x3003, bytes([1, 255, 1, 14]), 2)
            time.sleep(1)
            after = time.monotonic()
            for _ in range(20):
                request(139)
                time.sleep(0.1)
            kinds = []
            while not replies.empty():
                stamp, kind = replies.get()
                if stamp >= after:
                    kinds.append(kind)
            assert 'erpm' in kinds and not set(kinds) & {'temp', 'volt', 'current'}, set(kinds)
            print('PASS: MSPv2 EDT disable stops extended wire telemetry', flush=True)
            request(0x3003, bytes([1, 255, 1, 13]), 2)
            wait_for('MSPv2 EDT enable restores firmware telemetry', lambda: fc.temp > 0 and fc.volt_raw > 0)
            # Change DShot rate through the same fields saved by Motors.
            advanced = bytearray(request(90)[:-1]); advanced[3] = 6
            request(91, bytes(advanced)); request(250); request(68)
            assert request(90)[3] == 6
            wait_for('DShot300 after save and FC reboot', lambda: fc.edt_seen and fc.volt_raw > 0)
            request(214, struct.pack('<H', 1400))
            wait_for('motor runs after configuration reboot', lambda: fc.rpm > 1500)
            request(214, struct.pack('<H', 1000))
            wait_for('stop before bidirectional setting change', lambda: fc.rpm == 0)
            request(222, struct.pack('<HHHBB', 1070, 2000, 1000, 14, 0))
            request(250); request(68)
            for _ in range(35):
                request(150); time.sleep(0.1)
            request(214, struct.pack('<H', 1400))
            wait_for('normal DShot runs with bidirectional disabled',
                     lambda: bool(stream.samples) and abs(stream.samples[-1][1]) > 150)
            request(214, struct.pack('<H', 1000))
            request(222, struct.pack('<HHHBB', 1070, 2000, 1000, 14, 1))
            request(250); request(68)
            wait_for('bidirectional re-enable restores EDT', lambda: fc.edt_seen and fc.volt_raw > 0)
            request(214, struct.pack('<H', 1400))
            wait_for('motor runs after bidirectional re-enable', lambda: fc.rpm > 1500)
            # Simulate a vanished browser: no polling or motor requests.
            time.sleep(3)
            assert fc.motor_value == 1000
            print('PASS: lost MSP connection stops motor output', flush=True)
        except Exception:
            log.flush()
            print((Path(tmp) / 'firmware.log').read_text()[-4000:])
            raise
        finally:
            stream.close()
            fc.close()
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
            log.close()


if __name__ == '__main__':
    main()
