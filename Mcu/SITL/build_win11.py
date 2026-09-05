#!/usr/bin/env python3
"""Build dist/am32-sitl-gui-windows.zip on win11 over SSH.

From a Linux checkout: python3 Mcu/SITL/build_win11.py
On Windows/CI, in Cygwin: python3 Mcu/SITL/build_win11.py --local

Requires Cygwin gcc-core, make, python3, git, rsync and native Python 3.12
on the Windows build machine. End users only need to extract the ZIP.
Sources (including local edits) are copied into a dedicated remote directory;
no commits or pushes are needed. CI uses the same --local build and packager.
"""
import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BOOTLOADER_URL = 'https://github.com/am32-firmware/AM32-bootloader.git'
BOOTLOADER_REF = '933805415542eda3651800fba2c0a82f4574eb30'


def run(cmd, **kwargs):
    print(shlex.join(map(str, cmd)), flush=True)
    return subprocess.run(list(map(str, cmd)), check=True, **kwargs)


def output(cmd, **kwargs):
    return subprocess.check_output(list(map(str, cmd)), text=True, **kwargs).strip()


def bootloader_source(explicit):
    if explicit:
        path = Path(explicit).resolve()
        if not (path / 'sitlmakefile.mk').is_file():
            raise RuntimeError('not a bootloader SITL checkout: %s' % path)
        return path
    # Always use the pinned revision unless a developer explicitly overrides it.
    path = ROOT / 'build' / 'windows-bootloader'
    if not path.exists():
        run(['git', 'clone', BOOTLOADER_URL, path])
    current = output(['git', '-C', path, 'rev-parse', 'HEAD'])
    if current != BOOTLOADER_REF:
        if output(['git', '-C', path, 'status', '--porcelain']):
            raise RuntimeError('bootloader cache has local edits; use --bootloader-source')
        run(['git', '-C', path, 'fetch', 'origin', BOOTLOADER_REF])
        run(['git', '-C', path, 'checkout', '--detach', BOOTLOADER_REF])
    return path


def winpath(path):
    return output(['cygpath', '-aw', path])


def local_build(args):
    if sys.platform != 'cygwin':
        raise RuntimeError('--local must run under Cygwin Python')
    bootloader = bootloader_source(args.bootloader_source)
    # Separate object directory: never accidentally package Linux/MinGW objects
    # left by another build of this checkout.
    run(['make', '-j', args.jobs, 'AM32_SITL_CAN', 'OBJ=build/windows-obj'], cwd=ROOT)
    run(['make', '-j', args.jobs, 'OS=Linux', 'SHELL=/bin/bash',
         'AM32_SITL_BOOTLOADER_PB4_CAN', 'OBJ=build/windows-obj'], cwd=bootloader)
    fw = sorted((ROOT / 'build/windows-obj').glob('AM32_AM32_SITL_CAN_*.elf'))
    bl = sorted((bootloader / 'build/windows-obj').glob('AM32_SITL_BOOTLOADER_PB4_CAN_*.elf'))
    if len(fw) != 1 or len(bl) != 1:
        raise RuntimeError('expected exactly one firmware and bootloader; clean build/windows-obj')
    native = [args.python] if args.python else ['py', '-3.12']
    venv = ROOT / 'build' / 'windows-venv'
    if not (venv / 'Scripts/python.exe').exists():
        run(native + ['-m', 'venv', winpath(venv)])
    python = str(venv / 'Scripts/python.exe')
    run([python, '-m', 'pip', 'install', '-r', winpath(HERE / 'windows/requirements-build.txt')])
    run([python, winpath(HERE / 'bootloader_seed_test.py'),
         '--bootloader', winpath(bl[0])])
    run([python, winpath(HERE / 'package_windows.py'),
         '--sitl', winpath(fw[0]), '--bootloader', winpath(bl[0]),
         '--runtime', winpath('/bin/cygwin1.dll')])
    test = [python, winpath(HERE / 'windows_package_test.py'),
            '--exe', winpath(ROOT / 'dist/am32-sitl-gui.exe')]
    if args.test_usb:
        test.append('--usb')
    run(test)


def remote_build(args):
    bootloader = bootloader_source(args.bootloader_source)
    ssh = ['ssh']
    if args.ssh_config:
        ssh += ['-F', args.ssh_config]
    remote = args.remote_dir.rstrip('/')
    if not remote or remote in ('.', '/', '~'):
        raise RuntimeError('choose a dedicated --remote-dir')
    run(ssh + [args.host, 'mkdir -p ' + shlex.quote(remote)])
    # Stage only version-controlled and non-ignored new files, not .git or build
    # output. rsync --delete is scoped to our two dedicated source directories;
    # preserve build caches inside them between local test builds.
    for source, name in ((ROOT, 'AM32'), (bootloader, 'AM32-bootloader')):
        files = subprocess.check_output(['git', '-C', str(source), 'ls-files',
                                         '-z', '--cached', '--others', '--exclude-standard'])
        with tempfile.TemporaryDirectory(prefix='am32-win11-source-') as tmp:
            stage = Path(tmp)
            for raw in files.split(b'\0'):
                if not raw:
                    continue
                rel = Path(os.fsdecode(raw))
                src = source / rel
                if src.is_file():
                    dst = stage / rel
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, dst)
            run(['rsync', '-az', '--delete', '--exclude=/build/', '--exclude=/dist/',
                 '-e', shlex.join(ssh), str(stage) + '/',
                 args.host + ':' + shlex.quote(remote + '/' + name + '/')])
    command = ['python3', 'Mcu/SITL/build_win11.py', '--local',
               '--bootloader-source', '../AM32-bootloader', '--jobs', args.jobs]
    if args.python:
        command += ['--python', args.python]
    if args.test_usb:
        command.append('--test-usb')
    run(ssh + [args.host, 'cd ' + shlex.quote(remote + '/AM32') + ' && ' + shlex.join(command)])
    (ROOT / 'dist').mkdir(exist_ok=True)
    run(['rsync', '-av', '-e', shlex.join(ssh), args.host + ':' +
         shlex.quote(remote + '/AM32/dist/am32-sitl-gui-windows.zip'), str(ROOT / 'dist') + '/'])
    print('Ready: %s' % (ROOT / 'dist/am32-sitl-gui-windows.zip'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--local', action='store_true', help='build here in Cygwin (also used by CI)')
    ap.add_argument('--host', default='win11')
    ap.add_argument('--remote-dir', default='am32-sitl-gui-build')
    ap.add_argument('--ssh-config', help='optional ssh -F config path')
    ap.add_argument('--python', help='native Windows Python executable (default py -3.12)')
    ap.add_argument('--bootloader-source', help='use a local bootloader checkout including edits, instead of the pinned release')
    ap.add_argument('--jobs', default=str(min(os.cpu_count() or 2, 8)))
    ap.add_argument('--test-usb', action='store_true',
                    help='also test settings read/write over the installed USBip driver')
    args = ap.parse_args()
    local_build(args) if args.local else remote_build(args)


if __name__ == '__main__':
    main()
