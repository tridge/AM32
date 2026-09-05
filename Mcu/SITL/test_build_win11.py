"""Default bootloader builds must follow upstream master, even with a cache."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import build_win11


class BootloaderSourceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.upstream = self.root / 'upstream'
        self.upstream.mkdir()
        self.git(self.upstream, 'init', '-b', 'master')
        self.git(self.upstream, 'config', 'user.name', 'SITL test')
        self.git(self.upstream, 'config', 'user.email', 'sitl@example.invalid')
        self.git(self.upstream, 'config', 'commit.gpgsign', 'false')
        self.advance('first')
        self.work = self.root / 'work'
        (self.work / 'build').mkdir(parents=True)
        for target, value in (('ROOT', self.work), ('BOOTLOADER_URL', str(self.upstream))):
            override = patch.object(build_win11, target, value)
            override.start()
            self.addCleanup(override.stop)

    @staticmethod
    def git(repo, *args):
        return subprocess.check_output(['git', '-C', str(repo), *args],
                                       text=True, stderr=subprocess.PIPE).strip()

    def advance(self, value):
        (self.upstream / 'sitlmakefile.mk').write_text(value)
        self.git(self.upstream, 'add', 'sitlmakefile.mk')
        self.git(self.upstream, 'commit', '-m', value)
        return self.git(self.upstream, 'rev-parse', 'HEAD')

    def test_cached_checkout_follows_new_master_commit(self):
        source = build_win11.bootloader_source(None)
        old = self.git(source, 'rev-parse', 'HEAD')
        latest = self.advance('second')
        self.assertNotEqual(old, latest)
        self.assertEqual(build_win11.bootloader_source(None), source)
        self.assertEqual(self.git(source, 'rev-parse', 'HEAD'), latest)
        self.assertEqual((source / 'sitlmakefile.mk').read_text(), 'second')

    def test_fetch_failure_does_not_fall_back_to_cached_revision(self):
        build_win11.bootloader_source(None)
        self.upstream.rename(self.root / 'offline')
        with self.assertRaises(subprocess.CalledProcessError):
            build_win11.bootloader_source(None)

    def test_dirty_cache_is_preserved(self):
        source = build_win11.bootloader_source(None)
        (source / 'sitlmakefile.mk').write_text('local edit')
        self.advance('second')
        with self.assertRaisesRegex(RuntimeError, 'local edits'):
            build_win11.bootloader_source(None)
        self.assertEqual((source / 'sitlmakefile.mk').read_text(), 'local edit')

    def test_explicit_checkout_keeps_local_edits_without_fetching(self):
        (self.upstream / 'sitlmakefile.mk').write_text('developer edit')
        with patch.object(build_win11, 'run', side_effect=AssertionError('unexpected fetch')):
            self.assertEqual(build_win11.bootloader_source(str(self.upstream)), self.upstream)
        self.assertEqual((self.upstream / 'sitlmakefile.mk').read_text(), 'developer edit')


if __name__ == '__main__':
    unittest.main()
