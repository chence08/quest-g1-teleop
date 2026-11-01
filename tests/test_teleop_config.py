"""Portable configuration tests; no SSH, DDS, camera, or robot access."""
import argparse
import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from teleop_config import DEFAULTS, load_config, remote_python_command


class ConfigurationTests(unittest.TestCase):
    def test_file_and_environment_precedence_without_shell_evaluation(self):
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'local.env'
            file.write_text('# deployment\nG1_REMOTE_ROOT="/opt/robot tools"\nG1_ROBOT_PYTHON=python3\n')
            config = load_config({'G1_ROBOT_PYTHON': '/opt/env/bin/python'}, file)
            self.assertEqual(config['G1_REMOTE_ROOT'], '/opt/robot tools')
            self.assertEqual(config['G1_ROBOT_PYTHON'], '/opt/env/bin/python')
            self.assertEqual(config['G1_DDS_NAMESPACE'], 'inspire_teleop')
            file.write_text('G1_REMOTE_ROOT="/opt/$(touch forbidden)"\n')
            self.assertEqual(load_config({}, file)['G1_REMOTE_ROOT'], '/opt/$(touch forbidden)')

    def test_unknown_settings_and_relative_remote_root_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'local.env'
            for value in ('UNKNOWN=value\n', 'G1_REMOTE_ROOT=relative/path\n', 'G1_DDS_NAMESPACE=bad-name\n'):
                file.write_text(value)
                with self.assertRaises(ValueError): load_config({}, file)

    def test_remote_process_receives_custom_config_explicitly(self):
        config = dict(DEFAULTS, G1_REMOTE_ROOT='/opt/robot tools', G1_ROBOT_PYTHON=sys.executable,
                      G1_CAMERA_PYTHON='/opt/camera/bin/python', G1_DDS_NAMESPACE='test_hands',
                      G1_HAND_INTERFACE='lo', G1_ARM_INTERFACE='eth1')
        code = f'import sys,json;sys.path.insert(0,{str(SCRIPTS)!r});from teleop_config import load_config;print(json.dumps(load_config()))'
        command = remote_python_command(args=['-c', code], config=config)
        self.assertEqual(shlex.split(shlex.join(command)), command)
        # Run the same argv locally to verify forwarded settings, without invoking SSH.
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        self.assertEqual(json.loads(result.stdout), config)

    def test_hand_driver_and_command_bridge_share_configured_namespace(self):
        from quest_dfx_sender import remote_command
        from robot_service_runner import service_command
        overrides = {'TELEOP_CONFIG': '/dev/null', 'G1_DDS_NAMESPACE': 'custom_hands',
                     'G1_REMOTE_ROOT': '/opt/robot tools', 'G1_ROBOT_PYTHON': '/opt/robot/python'}
        with patch.dict(os.environ, overrides):
            remote = shlex.split(remote_command(argparse.Namespace(ssh_host='g1', duration=30))[-1])
            service = service_command('hands', 30, root=Path('/opt/robot tools'))
        self.assertIn('/opt/robot tools/dfx_robot_bridge.py', remote)
        self.assertEqual(remote[remote.index('--namespace') + 1], 'custom_hands')
        self.assertEqual(service[service.index('--namespace') + 1], 'custom_hands')
        self.assertIn('G1_DDS_NAMESPACE=custom_hands', remote)

    def test_all_workstation_live_entrypoints_require_lan_address(self):
        from quest_upper_body import parse_args as upper_args
        from quest_dfx_sender import parse_args as hand_args
        from run_upper_body_session import parse_args as session_args
        with patch.dict(os.environ, {'TELEOP_CONFIG': '/dev/null', 'HOST_LAN_IP': '127.0.0.1'}):
            for parse in (upper_args, hand_args, session_args):
                with self.subTest(parse=parse.__module__):
                    with self.assertRaises(SystemExit), patch('sys.stderr', io.StringIO()):
                        parse(['--execute'])
                    parse(['--execute', '--host-ip', '192.0.2.10'])
            self.assertEqual(upper_args([]).host_ip, '127.0.0.1')


if __name__ == '__main__':
    unittest.main()
