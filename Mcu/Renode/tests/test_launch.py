#!/usr/bin/env python3
'''Tests for the AM32 Renode launcher's managed Renode download.'''

import hashlib
import importlib.util
import io
import tarfile

from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / 'launch.py'
SPEC = importlib.util.spec_from_file_location(
    'am32_renode_launch', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launch)

GEN_TARGET_PATH = MODULE_PATH.with_name('gen_target.py')
GEN_TARGET_SPEC = importlib.util.spec_from_file_location(
    'am32_gen_target', GEN_TARGET_PATH)
assert GEN_TARGET_SPEC is not None and GEN_TARGET_SPEC.loader is not None
gen_target = importlib.util.module_from_spec(GEN_TARGET_SPEC)
GEN_TARGET_SPEC.loader.exec_module(gen_target)


def latest_metadata(package):
    return {
        'schema_version': 1,
        'renode_version': '1.16.1',
        'source': {
            'revision': '2a060779f4e2b87d1ae7238a041d858369818805',
        },
        'artifacts': [{
            'target': {
                'platform': 'linux',
                'architecture': 'x86_64',
                'runtime_identifier': 'linux-x64',
            },
            'packages': [package],
        }],
    }


def test_select_linux_renode_package():
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': 'a' * 64,
        'size': 123,
    }

    selected = launch.select_renode_package(
        latest_metadata(package), system='linux', machine='amd64')

    assert selected['filename'] == package['filename']
    assert selected['runtime_identifier'] == 'linux-x64'


def test_select_renode_package_rejects_unsafe_runtime_identifier():
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': 'a' * 64,
        'size': 123,
    }
    latest = latest_metadata(package)
    latest['artifacts'][0]['target']['runtime_identifier'] = '../../outside'

    with pytest.raises(RuntimeError, match='runtime identifier'):
        launch.select_renode_package(latest, system='linux', machine='amd64')


def test_malformed_cache_selection_is_ignored(tmp_path):
    (tmp_path / launch.RENODE_SELECTION).write_text('[]\n')
    assert launch.cached_renode(tmp_path) is None


def test_download_cache_is_reused_only_for_current_version(tmp_path):
    executable_data = b'#!/bin/sh\nexit 0\n'
    archive_stream = io.BytesIO()
    with tarfile.open(fileobj=archive_stream, mode='w:gz') as bundle:
        info = tarfile.TarInfo('renode-test/renode')
        info.mode = 0o755
        info.size = len(executable_data)
        bundle.addfile(info, io.BytesIO(executable_data))
    archive = archive_stream.getvalue()
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': hashlib.sha256(archive).hexdigest(),
        'size': len(archive),
    }
    latest = latest_metadata(package)
    selected = launch.select_renode_package(
        latest, system='linux', machine='x86_64')
    install_key = launch.renode_cache_key(latest, selected)
    conflicting_install = tmp_path / install_key
    conflicting_install.mkdir()
    (conflicting_install / 'unexpected').write_text('preserve me\n')

    def open_archive(_request, timeout):
        assert timeout == 60
        return io.BytesIO(archive)

    executable, _metadata, downloaded = launch.install_current_renode(
        tmp_path, latest=latest, opener=open_archive)

    assert downloaded
    assert executable.read_bytes() == executable_data
    assert executable.parents[1].name.startswith(install_key + '-')
    assert (conflicting_install / 'unexpected').read_text() == 'preserve me\n'

    def no_download(_request, _timeout):
        raise AssertionError('current cache should not be downloaded again')

    cached, _metadata, downloaded = launch.install_current_renode(
        tmp_path, latest=latest, opener=no_download)
    assert not downloaded
    assert cached == executable

    newer = latest_metadata(package)
    newer['source']['revision'] = '3' * 40
    selected = launch.select_renode_package(
        newer, system='linux', machine='x86_64')
    assert launch.cached_renode(tmp_path, newer, selected) is None


def test_tar_extraction_rejects_unsafe_members(tmp_path):
    archive = tmp_path / 'renode.tar.gz'
    with tarfile.open(archive, mode='w:gz') as bundle:
        info = tarfile.TarInfo('../outside')
        info.size = 1
        bundle.addfile(info, io.BytesIO(b'x'))
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'platform': 'linux',
    }

    with pytest.raises(tarfile.TarError, match='outside'):
        launch.extract_renode(archive, tmp_path / 'payload', package)


def test_renode_environment_jits_ready_to_run_images(monkeypatch):
    monkeypatch.delenv('DOTNET_ReadyToRun', raising=False)
    assert gen_target.renode_env()['DOTNET_ReadyToRun'] == '0'

    monkeypatch.setenv('DOTNET_ReadyToRun', '1')
    assert gen_target.renode_env()['DOTNET_ReadyToRun'] == '1'
