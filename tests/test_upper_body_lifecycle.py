import io
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from robot_service_runner import (Heartbeat, listening_port, service_command, supervise,
                                 controller_script, controller_processes, conflict_check, require_clear_conflict_check)
from run_upper_body_session import parse_args, sender_command, remote_runner_command


class RunnerTests(unittest.TestCase):
    def run_child(self, control=None, duration=.25, heartbeat_timeout=5., emit=None, child_source='import time;time.sleep(60)'):
        readfd, writefd = os.pipe()
        if control is None:
            os.close(writefd); writefd = None
        elif control:
            os.write(writefd, control)
        events = []
        def capture(value):
            events.append(value)
            if emit: emit(value)
        try:
            with tempfile.TemporaryFile('w+b') as log:
                result = supervise([sys.executable, '-c', child_source], readfd, duration, log,
                                   heartbeat_timeout=heartbeat_timeout, emit=capture)
            with self.assertRaises(ProcessLookupError):
                os.kill(result['child_pid'], 0)
            return result, events
        finally:
            os.close(readfd)
            if writefd is not None: os.close(writefd)

    def test_eof_reaps_owned_child(self):
        result, _ = self.run_child()
        self.assertEqual(result['reason'], 'stdin_eof')

    def test_stop_message_reaps_owned_child(self):
        result, _ = self.run_child(b'stop\n')
        self.assertEqual(result['reason'], 'stop_requested')

    def test_missing_heartbeat_reaps_child(self):
        result, _ = self.run_child(b'', duration=2., heartbeat_timeout=.12)
        self.assertEqual(result['reason'], 'heartbeat_timeout')

    def test_sigterm_ignoring_child_is_killed_and_reaped(self):
        result, _ = self.run_child(b'', duration=.3, child_source='import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(60)')
        self.assertTrue(result['kill_fallback'])
        self.assertEqual(result['child_exit_code'], -signal.SIGKILL)

    def test_absolute_duration_reaps_child(self):
        result, _ = self.run_child(b'', duration=.12)
        self.assertEqual(result['reason'], 'duration_expired')

    def test_broken_stdout_still_reaps_child(self):
        events = []
        def broken(event):
            events.append(event)
            if event['event'] == 'service_started': raise BrokenPipeError()
        with self.assertRaises(BrokenPipeError):
            self.run_child(b'', emit=broken)
        with self.assertRaises(ProcessLookupError):
            os.kill(events[0]['child_pid'], 0)

    def test_stale_buffered_heartbeat_does_not_renew(self):
        beat = Heartbeat(1.)
        beat.ingest(b'ping 10\n', wall_now=20., now=2.)
        self.assertEqual(beat.last, 1.)
        beat.ingest(b'ping 20\n', wall_now=20.02, now=3.)
        self.assertEqual(beat.last, 3.)

    def test_only_named_services_and_explicit_devices(self):
        with self.assertRaises(ValueError): service_command('arbitrary', 20)
        with self.assertRaises(ValueError): service_command('camera', 20)
        hand = service_command('hands', 20)
        self.assertNotIn('sudo', hand)
        self.assertIn('inspire_teleop', hand)

    def test_proc_listener_detects_bound_nonloopback_port(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'net').mkdir()
            (root / 'net/tcp').write_text('header\n0: D364010A:15E2 00000000:0000 0A 0\n')
            (root / 'net/tcp6').write_text('header\n')
            self.assertTrue(listening_port(5602, root))
            self.assertFalse(listening_port(5603, root))

    def test_exact_executed_controller_script_only(self):
        for argv in (['/path/command_bridge.py'], ['python3', '/path/g1_controller_bridge.py'],
                     ['/env/python3.10', '-u', '-X', 'dev', '--', '/path/command_bridge.py']):
            self.assertIn(controller_script(argv), ('command_bridge.py', 'g1_controller_bridge.py'))
        for argv in (['bash', '-c', 'python3 /path/g1_controller_bridge.py'],
                     ['python3', '-c', 'g1_controller_bridge.py'],
                     ['python3', '-m', 'module', 'command_bridge.py'],
                     ['python3', '/path/robot_service_runner.py', 'command_bridge.py'],
                     ['python3', '/path/stream_g1_front.py'], ['echo', 'command_bridge.py'], []):
            self.assertIsNone(controller_script(argv))

    def test_controller_blocks_without_port_and_camera_is_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'net').mkdir()
            for name in ('tcp', 'tcp6'): (root / 'net' / name).write_text('header\n')
            for pid, argv in ((8029, ['python3', '/opt/example/g1_controller_bridge.py']),
                              (8030, ['python3', '/opt/example/stream_g1_front.py']), (20, [])):
                (root / str(pid)).mkdir()
                (root / str(pid) / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv))
            check = conflict_check(root)
            self.assertFalse(check['port_5602_listening'])
            self.assertEqual(check['known_controller_processes'], [{'pid': 8029, 'script': 'g1_controller_bridge.py'}])
            self.assertFalse(check['clear'])
            self.assertFalse(check['exclusive_ownership_established'])
            with self.assertRaises(RuntimeError): require_clear_conflict_check(json.dumps(check), 3)

    def test_unreadable_userspace_blocks_but_kernel_thread_does_not(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for pid, status in ((1, 'Kthread:\t0\n'), (2, 'Kthread:\t1\n')):
                (root / str(pid)).mkdir(); (root / str(pid) / 'status').write_text(status)
            with patch.object(Path, 'read_bytes', side_effect=PermissionError()):
                conflicts, inaccessible = controller_processes(root)
            self.assertEqual(conflicts, [])
            self.assertEqual(inaccessible, [{'pid': 1, 'error': 'PermissionError'}])

    def test_clear_check_requires_current_version_and_complete_evidence(self):
        clear = {'event': 'conflict_check', 'check_version': 2, 'port_5602_listening': False,
                 'known_controller_processes': [], 'process_scan_complete': True, 'clear': True}
        self.assertEqual(require_clear_conflict_check(json.dumps(clear), 0), clear)
        for update in ({'check_version': 1}, {'process_scan_complete': False},
                       {'known_controller_processes': [{'pid': 5}]}, {'port_5602_listening': True}):
            with self.assertRaises(RuntimeError):
                require_clear_conflict_check(json.dumps({**clear, **update}), 0)
        with self.assertRaises(RuntimeError): require_clear_conflict_check(json.dumps(clear), 255)


class LauncherTests(unittest.TestCase):
    def test_default_hardware_free_and_live_keeps_explicit_sender_arming(self):
        args, extra = parse_args([])
        command = sender_command(args, extra)
        self.assertFalse(args.execute)
        self.assertNotIn('--execute', command)
        self.assertNotIn('--robot-feedback', command)
        args, extra = parse_args(['--execute', '--host-ip', '192.0.2.10', '--arms-only', '--translation-limit', '.03'])
        command = sender_command(args, extra)
        self.assertIn('--execute', command)
        self.assertIn('--arms-only', command)
        self.assertNotIn('--auto-arm', command)
        self.assertEqual(command[-2:], ['--translation-limit', '.03'])

    def test_remote_camera_device_is_shell_quoted(self):
        args, _ = parse_args(['--camera-device', '/dev/example;touch bad'])
        command = remote_runner_command(args, 'camera')
        self.assertIn("'/dev/example;touch bad'", command[-1])

    def test_conflict_refuses_execute_before_any_service(self):
        import subprocess
        import run_upper_body_session as launch
        previous = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        try:
            check = {'event': 'conflict_check', 'check_version': 2, 'port_5602_listening': False,
                     'known_controller_processes': [{'pid': 8029, 'script': 'g1_controller_bridge.py'}],
                     'process_scan_complete': True, 'clear': False}
            with tempfile.TemporaryDirectory() as directory, patch.object(launch.subprocess, 'run', return_value=subprocess.CompletedProcess([], 3, json.dumps(check), '')), patch.object(launch, 'OwnedService') as service, patch('sys.stdout', io.StringIO()), patch('sys.stderr', io.StringIO()):
                result = launch.main(['--execute', '--host-ip', '192.0.2.10', '--report', str(Path(directory) / 'session.json')])
                self.assertEqual(result, 1)
                service.assert_not_called()
        finally:
            for number, handler in previous.items(): signal.signal(number, handler)

    def test_refuses_unowned_camera_tunnel_override(self):
        with self.assertRaises(SystemExit), patch('sys.stderr', io.StringIO()):
            parse_args(['--camera-host', '127.0.0.1'])


if __name__ == '__main__':
    unittest.main()
