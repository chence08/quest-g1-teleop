#!/usr/bin/env python3
"""Own a bounded hand or RGB service for one SSH connection.

No arbitrary command CLI and no posture command. A fresh heartbeat, connected
stdin, live parent, and unexpired deadline are all required to retain the child.
"""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import sys
import time
from teleop_config import load_config

ROOT = Path(load_config()['G1_REMOTE_ROOT'])
KNOWN_CONTROLLERS = frozenset(('g1_controller_bridge.py', 'command_bridge.py'))
CONFLICT_CHECK_VERSION = 2


def listening_port(port, proc_root=Path('/proc')):
    for name in ('tcp', 'tcp6'):
        for line in (proc_root / 'net' / name).read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) >= 4 and fields[3] == '0A' and int(fields[1].rsplit(':', 1)[1], 16) == port:
                return True
    return False


def controller_script(argv):
    """Identify an executed script, never text inside a shell/Python command."""
    if not argv:
        return None
    executable = Path(argv[0]).name
    if executable in KNOWN_CONTROLLERS:
        return executable
    if not re.fullmatch(r'(?:python(?:\d+(?:\.\d+)*)?|pypy\d*)', executable):
        return None
    index = 1
    while index < len(argv):
        argument = argv[index]
        if argument in ('-c', '-m') or argument.startswith(('-c', '-m')):
            return None
        if argument == '--':
            index += 1
            break
        if argument in ('-W', '-X'):
            index += 2
            continue
        if argument.startswith('-'):
            index += 1
            continue
        break
    return Path(argv[index]).name if index < len(argv) and Path(argv[index]).name in KNOWN_CONTROLLERS else None


def controller_processes(proc_root=Path('/proc')):
    """Read public process metadata only; an incomplete userspace scan fails closed."""
    conflicts, inaccessible = [], []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes()
        except (FileNotFoundError, ProcessLookupError):
            continue  # The process exited during this snapshot.
        except OSError as exc:
            try:
                status = (entry / 'status').read_text()
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError:
                status = ''
            if re.search(r'^Kthread:\s+1\s*$', status, re.MULTILINE):
                continue
            inaccessible.append({'pid': int(entry.name), 'error': type(exc).__name__})
            continue
        # Kernel threads and zombies normally have an empty cmdline.
        argv = [part.decode('utf-8', errors='replace') for part in raw.split(b'\0') if part]
        script = controller_script(argv)
        if script:
            conflicts.append({'pid': int(entry.name), 'script': script})
    return sorted(conflicts, key=lambda value: value['pid']), sorted(inaccessible, key=lambda value: value['pid'])


def conflict_check(proc_root=Path('/proc')):
    conflicts, inaccessible = controller_processes(proc_root)
    port_conflict = listening_port(5602, proc_root)
    return {'event': 'conflict_check', 'check_version': CONFLICT_CHECK_VERSION,
            'port_5602_listening': port_conflict, 'known_controller_processes': conflicts,
            'process_scan_complete': not inaccessible, 'inaccessible_process_count': len(inaccessible),
            'inaccessible_processes': inaccessible[:20],
            'clear': not port_conflict and not conflicts and not inaccessible,
            'exclusive_ownership_established': False}


def require_clear_conflict_check(stdout, returncode):
    """Reject failed, malformed, or outdated checks before starting a publisher."""
    checks = []
    for line in stdout.splitlines():
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and value.get('event') == 'conflict_check':
            checks.append(value)
    if len(checks) != 1:
        raise RuntimeError('Robot conflict check did not return one valid structured result')
    check = checks[0]
    if check.get('check_version') != CONFLICT_CHECK_VERSION:
        raise RuntimeError('Robot conflict checker is outdated; deploy robot_service_runner.py before arming')
    if (returncode != 0 or check.get('clear') is not True or check.get('port_5602_listening') is not False
            or check.get('process_scan_complete') is not True or check.get('known_controller_processes') != []):
        raise RuntimeError('Robot controller conflict or incomplete process scan: ' + json.dumps(check))
    return check


def service_command(service, duration, camera_device=None, root=ROOT):
    config = load_config()
    if service == 'hands':
        return [str(root / 'DFX_inspire_service/build/inspire_g1'), '--network', config['G1_HAND_INTERFACE'],
                '--namespace', config['G1_DDS_NAMESPACE']]
    if service == 'camera':
        if not camera_device or not camera_device.startswith('/dev/') or '\x00' in camera_device:
            raise ValueError('Camera service requires an explicit /dev/ device path')
        return [config['G1_CAMERA_PYTHON'], '-u', str(root / 'g1_camera_server.py'), '--device', camera_device,
                '--width', '640', '--height', '480', '--fps', '15', '--duration', str(duration),
                '--report', str(root / 'reports' / f'camera-service-{os.getpid()}.json')]
    raise ValueError('Only hands and camera services are allowed')


def busy_devices(paths):
    result = subprocess.run(['fuser', *paths], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        raise RuntimeError('Requested device is already in use: ' + result.stdout.strip() + ' ' + result.stderr.strip())
    if result.returncode != 1 or result.stderr.strip():
        raise RuntimeError('Could not establish device ownership: ' + result.stderr.strip())


class Heartbeat:
    def __init__(self, now):
        self.last = now
        self.buffer = b''

    def ingest(self, data, wall_now, now):
        self.buffer += data
        if len(self.buffer) > 4096:
            return 'oversized_control'
        parts = self.buffer.split(b'\n')
        self.buffer = parts[-1]
        for line in parts[:-1]:
            if line.strip() == b'stop':
                return 'stop_requested'
            try:
                verb, stamp = line.split()
                age = wall_now - float(stamp)
                if verb == b'ping' and math.isfinite(age) and -.2 <= age <= 2.:
                    self.last = now
            except (ValueError, TypeError):
                continue
        return None


def stop_owned_group(child, grace=2.):
    """Signal only the process group created for this child, then reap it."""
    killed = False
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        killed = True
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=2)
    # A child can exit before a descendant. They still belong to our group.
    try:
        os.killpg(child.pid, signal.SIGKILL)
        killed = True
    except ProcessLookupError:
        pass
    return killed


def supervise(command, control_fd, duration, log_file, heartbeat_timeout=5.,
              parent_pid=None, emit=lambda value: None, stop_requested=lambda: False):
    """Testable lifecycle core; the public CLI only passes service_command output."""
    environment = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1')
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log_file, stderr=subprocess.STDOUT,
                             cwd=str(ROOT) if ROOT.is_dir() else None, start_new_session=True, env=environment)
    started = time.monotonic()
    heartbeat = Heartbeat(started)
    reason, killed = 'duration_expired', False
    try:
        emit({'event': 'service_started', 'child_pid': child.pid, 'time': time.time()})
        while time.monotonic() - started < duration:
            now = time.monotonic()
            if stop_requested():
                reason = 'signal'; break
            if child.poll() is not None:
                reason = 'child_exited'; break
            if parent_pid is not None and os.getppid() != parent_pid:
                reason = 'parent_disconnected'; break
            if now - heartbeat.last > heartbeat_timeout:
                reason = 'heartbeat_timeout'; break
            if select.select([control_fd], [], [], .05)[0]:
                data = os.read(control_fd, 4096)
                if not data:
                    reason = 'stdin_eof'; break
                requested = heartbeat.ingest(data, time.time(), now)
                if requested:
                    reason = requested; break
    finally:
        killed = stop_owned_group(child)
    result = {'event': 'service_stopped', 'child_pid': child.pid, 'child_exit_code': child.returncode,
              'reason': reason, 'kill_fallback': killed, 'time': time.time()}
    try:
        emit(result)
    except (BrokenPipeError, OSError):
        pass  # The owned child has already been reaped.
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service', choices=('hands', 'camera'))
    parser.add_argument('--camera-device')
    parser.add_argument('--duration', type=float, default=600.)
    parser.add_argument('--check-conflicts', action='store_true',
                        help='read-only port 5602 and known controller process check; does not establish exclusive ownership')
    args = parser.parse_args(argv)
    if not args.check_conflicts and not args.service:
        parser.error('--service is required')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 900:
        parser.error('--duration must be in (0,900]')
    if args.service == 'camera' and not args.camera_device:
        parser.error('--camera-device is required for camera')
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.check_conflicts:
        check = conflict_check()
        print(json.dumps(check), flush=True)
        return 0 if check['clear'] else 3
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT / 'reports').mkdir(exist_ok=True)
    lock = open(ROOT / f'.{args.service}.service.lock', 'a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError(f'An owned {args.service} runner is already active')
    devices = ['/dev/ttyUSB1', '/dev/ttyUSB2'] if args.service == 'hands' else [args.camera_device]
    busy_devices(devices)
    command = service_command(args.service, args.duration, args.camera_device)
    stopped = False
    def on_signal(*_):
        nonlocal stopped
        stopped = True
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, on_signal)
    log_path = ROOT / 'reports' / f'{args.service}-{datetime.now().strftime("%Y%m%d-%H%M%S")}-{os.getpid()}.log'
    def emit(value):
        print(json.dumps({**value, 'service': args.service, 'log': str(log_path)}), flush=True)
    with open(log_path, 'ab', buffering=0) as log:
        result = supervise(command, sys.stdin.fileno(), args.duration, log, parent_pid=os.getppid(),
                           emit=emit, stop_requested=lambda: stopped)
    return 1 if result['reason'] == 'child_exited' and result['child_exit_code'] != 0 else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(json.dumps({'event': 'service_error', 'error': str(exc)}), flush=True)
        raise SystemExit(1)
