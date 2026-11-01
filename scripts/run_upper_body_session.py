#!/usr/bin/env python3
"""One-command Quest teleoperation session with owned service cleanup.

No flags means hardware-free preview. --execute lets r drive the robot; it warns
about known competing controllers but does not refuse to start. An explicit
--camera-device optionally starts RGB capture and SSH forwarding.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from robot_service_runner import require_clear_conflict_check
from teleop_config import load_config, remote_python_command, validate_host_ip

ROOT = Path(__file__).resolve().parents[1]


def ssh_base(host):
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2', '-o', 'ExitOnForwardFailure=yes', host]


def remote_runner_command(args, service=None):
    command = remote_python_command('robot_service_runner.py')
    if service is None:
        command += ['--check-conflicts']
    else:
        command += ['--service', service, '--duration', str(args.session_duration)]
        if service == 'camera':
            command += ['--camera-device', args.camera_device]
    return ssh_base(args.ssh_host) + [shlex.join(command)]


class OwnedService:
    def __init__(self, args, service):
        self.service, self.messages = service, []
        self.started = threading.Event()
        self.stop = threading.Event()
        self.process = subprocess.Popen(remote_runner_command(args, service), stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True, bufsize=0)
        os.set_blocking(self.process.stdin.fileno(), False)
        self.reader = threading.Thread(target=self._read, daemon=True); self.reader.start()
        self.beater = threading.Thread(target=self._heartbeat, daemon=True); self.beater.start()

    def _read(self):
        for line in iter(self.process.stdout.readline, b''):
            text = line.decode(errors='replace').strip()
            print(f'{self.service.upper()} {text}', flush=True)
            try:
                value = json.loads(text)
                self.messages.append(value)
                if value.get('event') == 'service_started':
                    self.started.set()
            except (ValueError, AttributeError):
                pass

    def _heartbeat(self):
        while not self.stop.is_set() and self.process.poll() is None:
            try:
                os.write(self.process.stdin.fileno(), f'ping {time.time():.6f}\n'.encode())
            except BlockingIOError:
                pass
            except (BrokenPipeError, ValueError, OSError):
                return
            self.stop.wait(1.)

    def wait_started(self, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(f'{self.service} service failed to start; inspect its output')
            if self.started.wait(.1):
                # Catch immediate device/open/init failures before starting the sender.
                if self.stop.wait(.3) or self.process.poll() is not None:
                    raise RuntimeError(f'{self.service} service exited during initialization')
                return
        raise RuntimeError(f'{self.service} service startup timed out')

    def close(self):
        self.stop.set(); self.beater.join(timeout=1.2)
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=7)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=2)
        self.reader.join(timeout=1)


def check_free_local_ports():
    for port in (55555, 60000):
        sock = socket.socket()
        try:
            sock.bind(('127.0.0.1', port))
        except OSError as exc:
            raise RuntimeError(f'Local camera port {port} is already in use; do not reuse an unknown tunnel') from exc
        finally:
            sock.close()


def check_camera_frame(timeout=8.):
    """Require actual config and JPEG bytes through the tunnel, not stale config."""
    import zmq
    context = zmq.Context()
    request, images = context.socket(zmq.REQ), context.socket(zmq.SUB)
    try:
        for sock in (request, images):
            sock.setsockopt(zmq.LINGER, 0)
        images.setsockopt(zmq.SUBSCRIBE, b'')
        images.connect('tcp://127.0.0.1:55555')
        request.connect('tcp://127.0.0.1:60000')
        request.send(b'GET_DATA')
        if not request.poll(int(timeout * 1000)):
            raise RuntimeError('Camera config did not arrive through the SSH tunnel')
        config = request.recv_json()
        head = config.get('head_camera', {})
        if head.get('image_shape') != [480, 640] or head.get('binocular') is not False:
            raise RuntimeError('Unexpected camera configuration')
        if not images.poll(int(timeout * 1000)):
            raise RuntimeError('No camera image arrived; camera service is not ready')
        frame = images.recv()
        if len(frame) < 100 or not frame.startswith(b'\xff\xd8') or not frame.endswith(b'\xff\xd9'):
            raise RuntimeError('Invalid camera JPEG payload')
        return {'jpeg_bytes': len(frame), 'shape': head['image_shape']}
    finally:
        request.close(); images.close(); context.term()


def parse_args(argv=None):
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--robot-feedback', action='store_true')
    parser.add_argument('--arms-only', action='store_true')
    parser.add_argument('--ssh-host', default=config['G1_SSH_HOST'])
    parser.add_argument('--host-ip', default=config['HOST_LAN_IP'])
    parser.add_argument('--duration', type=float, default=0., help='seconds per run; 0 runs until p or q')
    parser.add_argument('--session-duration', type=float, default=4 * 3600., help='whole session/service lifetime, s')
    parser.add_argument('--camera-device', help='explicit /dev/ camera path; starts optional RGB service')
    parser.add_argument('--report', type=Path, default=ROOT / 'reports/upper-body-session.json')
    args, extra = parser.parse_known_args(argv)
    validate_host_ip(parser, args.host_ip, args.execute)
    if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', args.ssh_host) or args.ssh_host.startswith('-'):
        parser.error('Invalid SSH host')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be nonnegative')
    if not math.isfinite(args.session_duration) or args.session_duration < 30:
        parser.error('--session-duration must be at least 30 seconds')
    if args.camera_device and not args.camera_device.startswith('/dev/'):
        parser.error('--camera-device must be an explicit /dev/ path')
    args.report = args.report.expanduser().resolve()
    if '--camera-host' in extra or any(s.startswith('--camera-host=') for s in extra):
        parser.error('Use --camera-device so this launcher owns and checks the camera tunnel')
    return args, extra


def sender_command(args, extra):
    command = [str(ROOT / 'run_python.sh'), str(ROOT / 'scripts/quest_upper_body.py'),
               '--ssh-host', args.ssh_host, '--host-ip', args.host_ip,
               '--duration', str(args.duration), '--report', str(args.report)]
    if args.execute: command.append('--execute')
    if args.robot_feedback: command.append('--robot-feedback')
    if args.arms_only: command.append('--arms-only')
    if args.camera_device: command += ['--camera-host', '127.0.0.1']
    return command + extra


def stop_sender(sender):
    if sender is None or sender.poll() is not None:
        return
    # Sender catches SIGINT and closes both command stdin streams before waiting.
    sender.send_signal(signal.SIGINT)
    try:
        sender.wait(timeout=12)
    except subprocess.TimeoutExpired:
        sender.terminate()
        try:
            sender.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sender.kill(); sender.wait(timeout=2)


def main(argv=None):
    args, extra = parse_args(argv)
    services, tunnel, sender = [], None, None
    interrupted = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, lambda *_: interrupted.set())
    started = time.monotonic()
    result = 1
    lifecycle = {'started_at': time.time(), 'execute_enabled': args.execute, 'camera_requested': bool(args.camera_device),
                 'services': [], 'cleanup_order': 'sender release, service EOF/TERM, tunnel close'}
    try:
        if args.execute:
            try:
                checked = subprocess.run(remote_runner_command(args), capture_output=True, text=True, timeout=15)
                require_clear_conflict_check(checked.stdout, checked.returncode)
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                lifecycle['conflict_warning'] = str(exc)
                print('WARNING: another arm controller may be running on the robot: ' + str(exc), flush=True)
        if args.execute and not args.arms_only:
            service = OwnedService(args, 'hands'); services.append(service)
            try:
                service.wait_started()
            except RuntimeError as exc:
                lifecycle['hands_warning'] = str(exc)
                print('WARNING: ' + str(exc) + '; continuing with arms only.', flush=True)
                extra = [*extra, '--arms-only']
        if args.camera_device:
            try:
                check_free_local_ports()
                service = OwnedService(args, 'camera'); services.append(service); service.wait_started()
                command = ssh_base(args.ssh_host)
                command[-1:-1] = ['-N', '-L', '127.0.0.1:55555:127.0.0.1:55555', '-L', '127.0.0.1:60000:127.0.0.1:60000']
                tunnel = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
                if interrupted.wait(.5) or tunnel.poll() is not None:
                    raise RuntimeError('Camera SSH tunnel did not remain open')
                lifecycle['camera_verified'] = check_camera_frame()
                print('Camera frame verified through the owned SSH tunnel.', flush=True)
            except Exception as exc:
                lifecycle['camera_warning'] = str(exc)
                print('WARNING: camera unavailable (' + str(exc) + '); continuing with Pass-through.', flush=True)
                args.camera_device = None
        if interrupted.is_set():
            raise KeyboardInterrupt
        print('Starting. c calibrates, r runs, p pauses, q exits. Each key requires Enter.', flush=True)
        sender = subprocess.Popen(sender_command(args, extra), cwd=ROOT, start_new_session=True)
        while sender.poll() is None:
            if interrupted.wait(.2):
                lifecycle['stop_reason'] = 'operator_signal'; result = 130; break
            if time.monotonic() - started >= args.session_duration - 10:
                lifecycle['stop_reason'] = 'session_deadline'; result = 0; break
            for service in services:
                if service.process.poll() is not None and service.service not in lifecycle.setdefault('exited_services', []):
                    lifecycle['exited_services'].append(service.service)
                    print(f'WARNING: {service.service} service exited; teleoperation continues without it.', flush=True)
            if tunnel is not None and tunnel.poll() is not None and not lifecycle.get('tunnel_exited'):
                lifecycle['tunnel_exited'] = True
                print('WARNING: camera SSH tunnel exited; teleoperation continues without video.', flush=True)
        if sender.poll() is not None:
            result = sender.returncode
            lifecycle['stop_reason'] = 'sender_exited'
    except KeyboardInterrupt:
        lifecycle['stop_reason'] = 'operator_interrupt'; result = 130
    except Exception as exc:
        lifecycle['error'] = str(exc)
        print('Session error: ' + str(exc), file=sys.stderr, flush=True)
    finally:
        def clean(name, action):
            try:
                action()
            except Exception as exc:
                lifecycle.setdefault('cleanup_errors', []).append({'component': name, 'error': str(exc)})
        clean('sender', lambda: stop_sender(sender))
        lifecycle['sender_exit_code'] = sender.returncode if sender else None
        for service in reversed(services):
            clean(service.service, service.close)
            lifecycle['services'].append({'service': service.service, 'ssh_exit_code': service.process.returncode,
                                           'messages': service.messages})
        if tunnel is not None:
            def close_tunnel():
                if tunnel.poll() is None:
                    tunnel.terminate()
                    try: tunnel.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        tunnel.kill(); tunnel.wait(timeout=2)
            clean('tunnel', close_tunnel)
            lifecycle['tunnel_exit_code'] = tunnel.returncode
        lifecycle['ended_at'] = time.time()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        lifecycle_path = args.report.with_suffix('.lifecycle.json')
        lifecycle_path.write_text(json.dumps(lifecycle, indent=2) + '\n')
        print('Lifecycle report: ' + str(lifecycle_path), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
