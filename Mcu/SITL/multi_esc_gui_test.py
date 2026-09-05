#!/usr/bin/env python3
"""Headless regression of ESC tabs, lifecycle and shared USB routing."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

import msp_stub_fc
import sitl_serial_bridge
import sitl_usbip
from sitl_gui import (QApplication, EscFleet, USB_BETAFLIGHT, USB_FOURWAY,
                      USB_SERIAL, QTimer, Qt, QFileDialog)
from PySide6.QtWidgets import QPushButton


def main():
    QApplication.setAttribute(Qt.AA_DontUseNativeDialogs)
    app = QApplication([])
    args = SimpleNamespace(host='127.0.0.1', port=18833, state_port=18834,
                           can_uri='none', poles=14, control_port=0,
                           log=None, replay=None, esc_count=1)
    with tempfile.TemporaryDirectory(prefix='am32-multi-gui-') as tmp:
        def eeprom(index=0):
            path = Path(tmp) / ('esc%u.bin' % index)
            if not path.exists(): path.write_bytes(bytes(48))
            return str(path)
        captured = []
        fc_class, serial_class = msp_stub_fc.MspStubFC, sitl_serial_bridge.SerialBridge
        def fc_factory(**kwargs):
            fc = fc_class(**kwargs)
            captured.append((kwargs, fc))
            return fc
        direct = []
        def serial_factory(**kwargs):
            direct.append(kwargs)
            return serial_class(**kwargs)
        with patch('sim_runner.bundled_eeprom', side_effect=eeprom), \
             patch.object(sitl_usbip, 'attach', return_value=3), \
             patch.object(sitl_usbip, 'detach', return_value=True), \
             patch.object(sitl_usbip, 'find_tty', return_value='test-serial'), \
             patch.object(msp_stub_fc, 'MspStubFC', side_effect=fc_factory), \
             patch.object(sitl_serial_bridge, 'SerialBridge', side_effect=serial_factory):
            fleet = EscFleet(args, app)
            try:
                assert len(fleet.panels) == fleet.count.value() == 1
                fleet.panels[0].command('esc_count 8', lambda _: None)
                assert fleet.tabs.count() == 8
                assert len({p.eeprom.text() for p in fleet.panels}) == 8
                assert len({p.args.port for p in fleet.panels}) == 8
                fleet.panels[0].command('esc_select 8', lambda _: None)
                assert fleet.tabs.currentIndex() == 7
                first = fleet.panels[0]
                with patch.object(first.runner, 'is_running', return_value=True):
                    try: fleet.set_count(1)
                    except ValueError: pass
                    else: raise AssertionError('count changed while a simulation was running')
                def usb(mode):
                    first.usb_mode.setCurrentIndex(mode)
                    deadline = time.monotonic() + 10
                    while not first.usb_mode.isEnabled() and time.monotonic() < deadline:
                        app.processEvents()
                        time.sleep(0.01)
                    assert first.usb_mode.isEnabled(), 'USB worker did not finish'
                    assert first.usb_mode.currentIndex() == mode, first.usb_status.text()
                    if mode: assert first.usb_status.text() == 'test-serial'
                for mode in (USB_BETAFLIGHT, USB_FOURWAY):
                    usb(mode)
                    kwargs, fc = captured[-1]
                    assert kwargs['esc_ports'] == [p.args.port for p in fleet.panels]
                    assert kwargs['state_ports'] == [p.args.state_port for p in fleet.panels]
                    assert fc.config.motor_count == fc.fourway.esc_count == 8
                    for panel in fleet.panels:
                        try: panel.command('ds_enable 1', lambda _: None)
                        except ValueError: pass
                        else: raise AssertionError('USB did not exclude a tab signal writer')
                    try: fleet.set_count(1)
                    except ValueError: pass
                    else: raise AssertionError('count changed with USB attached')
                    usb(0)
                    assert not fc.running
                fleet.panels[0].command('usb_target 8', lambda _: None)
                usb(USB_SERIAL)
                assert direct[-1]['sitl_port'] == fleet.panels[7].args.port
                assert direct[-1]['state_port'] == fleet.panels[7].args.state_port
                assert (direct[-1]['endpoint'].vid, direct[-1]['endpoint'].pid) == (
                    sitl_usbip.DIRECT_VENDOR_ID, sitl_usbip.DIRECT_PRODUCT_ID)
                usb(0)
                fleet.win.show()
                app.processEvents()
                # With no firmware listening, each EEPROM read takes 1.5s.
                # Exercise real event-loop time, including a modal file
                # picker and count changes while those reads are pending.
                fleet.tabs.setCurrentIndex(0)
                first.bootloader.setText(str(Path(tmp) / 'bootloader.elf'))
                browse = [b for b in first.widget.findChildren(QPushButton)
                          if b.text() == 'Browse...'][2]
                gaps, chooser_seen, edits, errors = [], [], [], []
                last = [time.monotonic()]
                def tick():
                    now = time.monotonic()
                    gaps.append(now - last[0])
                    last[0] = now
                    if any(isinstance(w, QFileDialog) and w.isVisible()
                           for w in app.topLevelWidgets()):
                        chooser_seen.append(now)
                def close_chooser():
                    for widget in app.topLevelWidgets():
                        if isinstance(widget, QFileDialog): widget.reject()
                def edit_count(count):
                    try:
                        start = time.monotonic()
                        fleet.set_count(count)
                        edits.append(time.monotonic() - start)
                    except Exception as ex:
                        errors.append(ex)
                heartbeat = QTimer()
                heartbeat.timeout.connect(tick)
                heartbeat.start(20)
                app.setQuitOnLastWindowClosed(False)
                QTimer.singleShot(200, browse.click)
                QTimer.singleShot(500, close_chooser)
                QTimer.singleShot(700, lambda: edit_count(7))
                QTimer.singleShot(900, lambda: edit_count(8))
                QTimer.singleShot(2200, app.quit)
                start = time.monotonic()
                app.exec()
                elapsed = time.monotonic() - start
                heartbeat.stop()
                assert not errors, errors
                assert chooser_seen and chooser_seen[0] - start < 0.5, 'file chooser was delayed'
                assert len(edits) == 2 and max(edits) < 0.5, edits
                assert elapsed < 4 and max(gaps) < 0.5, (elapsed, max(gaps))
                print('PASS: offline eight-ESC UI stays responsive (max event gap %.3fs, count change %.3fs)' % (max(gaps), max(edits)))
                if os.environ.get('AM32_GUI_SCREENSHOT'):
                    fleet.win.grab().save(os.environ['AM32_GUI_SCREENSHOT'])
                fleet.panels[7].eeprom.setText(first.eeprom.text())
                try: fleet.validate_eeproms()
                except ValueError: pass
                else: raise AssertionError('shared EEPROM was accepted')
                for _ in range(2):
                    fleet.set_count(1)
                    app.processEvents()
                    fleet.set_count(8)
                    app.processEvents()
                print('PASS: eight ESC tabs, independent storage, shared USB routing, writer exclusion and tab recreation')
            finally:
                fleet.close()
                fleet.win.close()
                app.processEvents()


if __name__ == '__main__':
    main()
