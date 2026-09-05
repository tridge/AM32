#!/usr/bin/env python3
"""Build the Windows GUI and the common CI/local ZIP payload (native Python)."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
USBIP_NAME = 'USBip-0.9.7.7-x64.exe'
USBIP_URL = 'https://firmware.ardupilot.org/Tools/AM32-tools/ESCSim/USBIP/' + USBIP_NAME
USBIP_SHA256 = '51620fa5f9f8be5932bc9d786deee557ce06d5407a99cab490dcfac71f185fea'


def fetch_usbip(cache):
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / USBIP_NAME
    def valid(p):
        return (p.is_file() and p.stat().st_size == 33226344
                and hashlib.sha256(p.read_bytes()).hexdigest() == USBIP_SHA256)
    if not valid(path):
        partial = path.with_suffix('.part')
        try:
            print('Downloading ' + USBIP_URL, flush=True)
            with urllib.request.urlopen(USBIP_URL, timeout=60) as src, partial.open('wb') as dst:
                shutil.copyfileobj(src, dst)
            if not valid(partial):
                raise RuntimeError('USBip installer failed size/SHA-256 verification')
            partial.replace(path)
        finally:
            partial.unlink(missing_ok=True)
    return path


def text_copy(src, dst):
    # CRLF and UTF-8: readable in Windows Notepad, regardless of checkout settings.
    dst.write_bytes(src.read_text(encoding='utf-8').replace('\r\n', '\n').replace('\n', '\r\n').encode('utf-8'))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sitl', required=True)
    ap.add_argument('--bootloader', required=True)
    ap.add_argument('--runtime', action='append', default=[])
    args = ap.parse_args()
    if sys.platform != 'win32':
        ap.error('run with native Windows Python, not Cygwin Python')
    cmd = [sys.executable, str(HERE / 'build_sitl_gui.py'),
           '--sitl', args.sitl, '--bootloader', args.bootloader]
    for runtime in args.runtime:
        cmd += ['--runtime', runtime]
    subprocess.run(cmd, check=True)
    installer = fetch_usbip(ROOT / 'build' / 'usbip')
    stage = ROOT / 'dist' / 'windows-package'
    if stage.exists():
        shutil.rmtree(stage)
    (stage / 'USBIP').mkdir(parents=True)
    shutil.copyfile(ROOT / 'dist' / 'am32-sitl-gui.exe', stage / 'am32-sitl-gui.exe')
    shutil.copyfile(installer, stage / 'USBIP' / installer.name)
    text_copy(HERE / 'windows' / 'README.txt', stage / 'README.txt')
    text_copy(ROOT / 'LICENSE', stage / 'LICENSE.txt')
    for name in ('THIRD-PARTY.txt', 'LGPL-3.0.txt'):
        text_copy(HERE / 'windows' / name, stage / name)
    text_copy(HERE / 'windows' / 'USBIP-README.txt', stage / 'USBIP' / 'README.txt')
    text_copy(HERE / 'windows' / 'usbip-win2-LICENSE.txt', stage / 'USBIP' / 'usbip-win2-LICENSE.txt')
    archive = ROOT / 'dist' / 'am32-sitl-gui-windows.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as out:
        for path in sorted(stage.rglob('*')):
            if path.is_file():
                out.write(path, path.relative_to(stage).as_posix())
    print('Built %s (%.1f MB)' % (archive, archive.stat().st_size / 1e6), flush=True)


if __name__ == '__main__':
    main()
