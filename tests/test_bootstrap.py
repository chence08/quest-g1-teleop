"""Offline bootstrap/deployment tests; local Git fixtures, no network or DDS."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import fetch_upstream as upstream
import deploy_robot


class FetchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'project'
        self.mirror = Path(self.temp.name) / 'mirror'
        self.relative = 'repos/fixture'
        self.repo = self.mirror / self.relative
        self.repo.mkdir(parents=True)
        self.root.mkdir()
        upstream.git(self.repo, 'init', '--quiet')
        upstream.git(self.repo, 'config', 'user.email', 'test@example.invalid')
        upstream.git(self.repo, 'config', 'user.name', 'Offline test')
        (self.repo / 'value.txt').write_text('before\n')
        upstream.git(self.repo, 'add', '.')
        upstream.git(self.repo, 'commit', '--quiet', '-m', 'fixture')
        self.commit = upstream.git(self.repo, 'rev-parse', 'HEAD').decode().strip()
        self.manifest = {'repositories': [{'path': self.relative, 'url': 'https://github.com/example/fixture.git', 'commit': self.commit}]}
        (self.root / 'upstream.lock.json').write_text(json.dumps(self.manifest))
        (self.root / 'change.patch').write_text('--- a/value.txt\n+++ b/value.txt\n@@ -1 +1 @@\n-before\n+after\n')
        self.mapping = patch.dict(upstream.PATCHES, {self.relative: 'change.patch'}, clear=True)
        self.mapping.start()
        self.addCleanup(self.mapping.stop)

    def test_exact_pin_patch_and_idempotent_recheck(self):
        result = upstream.fetch(self.root, self.mirror)
        destination = self.root / self.relative
        self.assertEqual(upstream.git(destination, 'rev-parse', 'HEAD').decode().strip(), self.commit)
        self.assertEqual((destination / 'value.txt').read_text(), 'after\n')
        self.assertEqual(result['repositories'][0]['patch_status'], 'applied')
        repeated = upstream.fetch(self.root, check=True)
        self.assertEqual(repeated['repositories'][0]['patch_status'], 'already_applied')

    def test_unexpected_edit_is_preserved_and_rejected(self):
        upstream.fetch(self.root, self.mirror)
        file = self.root / self.relative / 'value.txt'
        file.write_text('after\nmy edit\n')
        with self.assertRaisesRegex(RuntimeError, 'Unexpected tracked edits'):
            upstream.fetch(self.root, self.mirror)
        self.assertEqual(file.read_text(), 'after\nmy edit\n')

    def test_wrong_pin_never_resets_existing_checkout(self):
        upstream.fetch(self.root, self.mirror)
        self.manifest['repositories'][0]['commit'] = '0' * 40
        (self.root / 'upstream.lock.json').write_text(json.dumps(self.manifest))
        with self.assertRaisesRegex(RuntimeError, 'no reset performed'):
            upstream.fetch(self.root, self.mirror)
        self.assertEqual(upstream.git(self.root / self.relative, 'rev-parse', 'HEAD').decode().strip(), self.commit)

    def test_check_missing_checkout_does_not_fetch(self):
        with self.assertRaisesRegex(RuntimeError, 'Missing checkout'):
            upstream.fetch(self.root, check=True)
        self.assertFalse((self.root / self.relative).exists())

    def test_manifest_path_cannot_escape_project(self):
        self.manifest['repositories'][0]['path'] = '../escape'
        (self.root / 'upstream.lock.json').write_text(json.dumps(self.manifest))
        with self.assertRaises(ValueError):
            upstream.fetch(self.root, self.mirror)


class DeployTests(unittest.TestCase):
    def test_archive_allowlist_includes_config_helper_and_licenses_not_secrets(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            paths = ['scripts/' + name for name in deploy_robot.HELPERS]
            paths += ['upstream.lock.json', 'patches/dfx-serial-transactions.patch', 'LICENSE', 'NOTICE',
                      'THIRD_PARTY_NOTICES.md', 'third_party/example.LICENSE',
                      'repos/DFX_inspire_service/source.cpp', 'repos/unitree_sdk2_python/source.cpp',
                      'config/local.env', 'certs/key.pem', 'reports/raw.log', 'env/secret']
            for name in paths:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(name)
            archive = root / '.cache/archive.tar.gz'
            with patch.object(deploy_robot, 'fetch') as verify, patch.object(deploy_robot, 'git', return_value=b'source.cpp\0'):
                manifest = deploy_robot.create_bundle(root, archive)
            verify.assert_called_once_with(root, check=True)
            with tarfile.open(archive) as contents:
                names = contents.getnames()
            for name in ('teleop_config.py', 'LICENSE', 'NOTICE', 'third_party/example.LICENSE'):
                self.assertIn(name, names)
            for name in ('config/local.env', 'certs/key.pem', 'reports/raw.log', 'env/secret'):
                self.assertNotIn(name, names)
            self.assertEqual(manifest['teleop_config.py'], hashlib.sha256(b'scripts/teleop_config.py').hexdigest())

    def test_remote_command_is_argv_with_quoted_shell_arguments(self):
        command = deploy_robot.ssh_command('g1', ['mkdir', '-p', '--', '/home/user/a $(touch nope)'])
        self.assertEqual(command[-2], 'g1')
        self.assertEqual(command[-1], "mkdir -p -- '/home/user/a $(touch nope)'")

    def test_plan_mode_never_opens_ssh(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / '.cache/deploy').mkdir(parents=True)
            with patch.object(deploy_robot, 'PROJECT', root), \
                 patch.object(deploy_robot, 'create_bundle', return_value={'teleop_config.py': 'hash'}), \
                 patch.object(deploy_robot.subprocess, 'run') as run:
                self.assertEqual(deploy_robot.main([]), 0)
            run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
