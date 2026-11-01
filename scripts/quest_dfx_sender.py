#!/usr/bin/env python3
"""Quest camera-hand tracking to a bounded, robot-side DFX-only SSH bridge.

Default is preview-only: no SSH process or robot transport is created. --execute
enables the explicitly bounded hand test. No arm controller is imported here.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import select
import shlex
import subprocess
import sys
import threading
import time

from preview_quest import PROJECT_ROOT, model_workspace, prepare_imports
from teleop_config import load_config, remote_python_command, validate_host_ip


def normalize_dfq(radians):
    """Stock xr_teleoperate Inspire DFX normalization, in hardware joint order."""
    import numpy as np
    values = np.asarray(radians, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('Expected six finite Inspire target angles')
    minimum = np.array([0., 0., 0., 0., 0., -0.1])
    maximum = np.array([1.7, 1.7, 1.7, 1.7, 0.5, 1.3])
    return np.clip((maximum - values) / (maximum - minimum), 0., 1.)


def retarget_dfq(mapping, data):
    """Return DFX wire order: right six, then left six; never instantiate hardware."""
    import numpy as np
    targets = {}
    for side in ('left', 'right'):
        positions = np.asarray(getattr(data, f'{side}_hand_pos'), dtype=float)
        if positions.shape != (25, 3) or not np.isfinite(positions).all():
            raise ValueError(f'Invalid {side} hand keypoints')
        indices = getattr(mapping, f'{side}_indices')
        sequence = getattr(mapping, f'{side}_retargeting')
        reference = positions[indices[1]] - positions[indices[0]]
        q = sequence.retarget(reference)
        if int(sequence.optimizer.opt.last_optimize_result()) <= 0:
            raise RuntimeError(f'{side} retarget optimizer failed')
        order = getattr(mapping, f'{side}_dex_retargeting_to_hardware')
        targets[side] = normalize_dfq(q[order])
    return np.concatenate((targets['right'], targets['left'])).tolist()


def tracking_eligible(snapshot, max_age=0.25, max_change_age=0.25):
    for side in ('left', 'right'):
        item = snapshot.get(side, {})
        if not item.get('valid', False):
            return False
        if not 0 <= float(item.get('event_age', math.inf)) <= max_age:
            return False
        if not 0 <= float(item.get('change_age', math.inf)) <= max_change_age:
            return False
    return True


def tracking_log_snapshot(snapshot):
    """JSON-safe per-hand receipt/change ages and sequence numbers."""
    result = {}
    for side in ('left', 'right'):
        item = snapshot.get(side, {})
        result[side] = {'valid': bool(item.get('valid', False)), 'seq': item.get('seq', 0)}
        for key in ('event_age', 'change_age'):
            value = float(item.get(key, math.inf))
            result[side][key] = value if math.isfinite(value) else None
    return result


class StdinArmingGate:
    """Accept a complete 'r' line only while tracking is stable; EOF disarms."""
    def __init__(self, stream):
        self.fd = stream.fileno()
        self.buffer = b''
        self.armed = False
        self.connected = True
        self.last_event = 'waiting_for_r'

    def poll(self, tracking_stable):
        if not self.connected:
            return False
        lines = []
        try:
            # Drain ready bytes first, so an already-closed pipe cannot arm us.
            for _ in range(16):
                if not select.select([self.fd], [], [], 0)[0]:
                    break
                data = os.read(self.fd, 4096)
                if not data:
                    self.connected = self.armed = False
                    self.last_event = 'stdin_eof'
                    self.buffer = b''
                    return False
                self.buffer += data
                parts = self.buffer.split(b'\n')
                lines.extend(parts[:-1])
                self.buffer = parts[-1]
                if len(self.buffer) > 4096:
                    self.buffer = b''
                    self.last_event = 'invalid_key'
        except (OSError, ValueError):
            self.connected = self.armed = False
            self.last_event = 'stdin_disconnected'
            return False
        for line in lines:
            if line.strip() != b'r':
                self.last_event = 'invalid_key'
            elif tracking_stable:
                self.armed = True
                self.last_event = 'armed'
            else:
                self.last_event = 'r_rejected_tracking_not_stable'
        return self.armed


def remote_command(args):
    config = load_config()
    command = remote_python_command('dfx_robot_bridge.py',
               ['--execute', '--interface', config['G1_HAND_INTERFACE'], '--namespace', config['G1_DDS_NAMESPACE'],
               '--duration', str(args.duration)], config)
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2',
            '--', args.ssh_host, shlex.join(command)]


class RemoteBridge:
    def __init__(self, args):
        if not args.execute:
            raise ValueError('RemoteBridge requires explicit --execute')
        if getattr(args, 'wait_for_arm', True) and not getattr(args, 'arm_authorized', False):
            raise ValueError('RemoteBridge requires explicit stdin arming')
        self.ready = threading.Event()
        self.messages = []
        self.last_message = None
        self.process = subprocess.Popen(remote_command(args), stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=False, bufsize=0, start_new_session=True)
        os.set_blocking(self.process.stdin.fileno(), False)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for raw in iter(self.process.stdout.readline, b''):
            line = raw.decode('utf-8', errors='replace').rstrip()
            print('ROBOT ' + line, flush=True)
            try:
                message = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                continue
            if not isinstance(message, dict):
                continue
            self.messages.append(message)
            self.last_message = message
            if message.get('type') == 'state' and not message.get('ready', False):
                self.ready.clear()
            elif message.get('ready') is True or message.get('event') == 'ready' or message.get('type') == 'ready':
                self.ready.set()

    def send(self, command):
        payload = (json.dumps(command, allow_nan=False, separators=(',', ':')) + '\n').encode()
        try:
            count = os.write(self.process.stdin.fileno(), payload)
        except BlockingIOError:
            return False
        if count != len(payload):
            raise RuntimeError('Partial SSH write; stopping instead of sending malformed commands')
        return True

    def close(self):
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.reader.join(timeout=1)


def bridge_evidence(messages):
    """Separate commands accepted by the robot process from local pipe writes."""
    baseline = None
    published = 0
    max_change = 0.
    state_count = ready_count = 0
    reasons = {}
    first_lost = last_lost = None
    for message in messages:
        if message.get('status') == 'state_qualified':
            baseline = message.get('initial_q')
        published = max(published, int(message.get('published_count', 0)))
        if message.get('type') == 'state':
            state_count += 1
            ready_count += int(message.get('ready') is True)
            if message.get('ready') is not True:
                reason = str(message.get('reason', 'unspecified'))
                reasons[reason] = reasons.get(reason, 0) + 1
            lost = message.get('lost')
            if isinstance(lost, list) and len(lost) == 12 and all(
                    isinstance(v, (int, float)) and math.isfinite(v) for v in lost):
                if first_lost is None:
                    first_lost = lost
                last_lost = lost
        values = message.get('q')
        if isinstance(baseline, list) and isinstance(values, list) and len(baseline) == len(values) == 12:
            if all(isinstance(v, (int, float)) and math.isfinite(v) for v in values + baseline):
                max_change = max(max_change, max(abs(a-b) for a,b in zip(values, baseline)))
    return {'published_count': published, 'max_observed_normalized_state_change': max_change,
            'physical_observation_confirmed': False,
            'operator_observation': {'confirmed': False, 'status': 'not_recorded'},
            'serial_quality': {
                'independently_verified': False, 'feedback_state_samples': state_count,
                'feedback_ready_samples': ready_count, 'feedback_not_ready_reasons': reasons,
                'driver_lost_counter_growth': ([max(0, b-a) for a, b in zip(first_lost, last_lost)]
                                               if first_lost is not None else None),
                'evidence_scope': 'Sampled DDS feedback; no independent serial verification'}}


def parse_args(argv=None):
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='enable bounded live DFX finger commands over SSH')
    arming = parser.add_mutually_exclusive_group()
    arming.add_argument('--wait-for-arm', dest='wait_for_arm', action='store_true', default=True,
                        help="live default: require an 'r' line on stdin while tracking is stable")
    arming.add_argument('--auto-arm', dest='wait_for_arm', action='store_false',
                        help='explicitly start live test automatically after stable tracking')
    parser.add_argument('--ssh-host', default=config['G1_SSH_HOST'])
    parser.add_argument('--host-ip', default=config['HOST_LAN_IP'])
    parser.add_argument('--duration', type=float, default=30., help='active test seconds after stable tracking; max 120')
    parser.add_argument('--wait-timeout', type=float, default=90., help='initial Quest connection deadline')
    parser.add_argument('--report', type=Path, default=PROJECT_ROOT / 'reports/live-dfx-sender.json')
    args = parser.parse_args(argv)
    validate_host_ip(parser, args.host_ip, args.execute)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 120:
        parser.error('--duration must be in (0,120]')
    if not math.isfinite(args.wait_timeout) or not 0 < args.wait_timeout <= 600:
        parser.error('--wait-timeout must be in (0,600]')
    if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', args.ssh_host) or args.ssh_host.startswith('-'):
        parser.error('--ssh-host must be an SSH hostname/alias, not options or shell text')
    args.report = args.report.expanduser().resolve()
    args.arm_authorized = False
    return args


def main(argv=None):
    args = parse_args(argv)
    report = {'started_utc': datetime.now(timezone.utc).isoformat(), 'execute': args.execute,
              'scope': 'DFX fingers only; no arm/balance commands', 'sent': 0,
              'tracking_rejections': 0, 'backpressure_drops': 0, 'retargeted': 0,
              'remote_messages': [], 'completed': False, 'transport_completed': False,
              'completion_scope': 'DDS transport only; physical observation is separate',
              'arming_mode': 'stdin_r' if args.wait_for_arm else 'explicit_auto_arm', 'armed': False}
    wrapper = remote = None
    arming_gate = None
    result = 1
    prepare_imports()
    from tracked_quest import create_tracked_wrapper
    from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
    cert = Path(os.environ.get('XR_TELEOP_CERT', PROJECT_ROOT / 'certs/cert.pem'))
    key = Path(os.environ.get('XR_TELEOP_KEY', PROJECT_ROOT / 'certs/key.pem'))
    if not cert.is_file() or not key.is_file():
        raise RuntimeError('TLS certificate/key missing')
    print('LIVE HAND TEST ENABLED' if args.execute else 'PREVIEW ONLY: no SSH or motor commands', flush=True)
    print(f'Quest: https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012', flush=True)
    print('Both hands must be visible and changing for 0.5 s before the test starts.', flush=True)
    if args.execute and args.wait_for_arm:
        print("Waiting for explicit arming: type r then Enter while both hands track stably. No SSH until armed.", flush=True)
    print('Live bridge limits: 0.15 normalized travel around measured start; 0.15 units/s.', flush=True)
    print('Loss/invalid/stale tracking stops new commands; the hand can finish its last small target. No auto-open/home.', flush=True)
    try:
        with model_workspace():
            mapping = HandRetargeting(HandType.INSPIRE_HAND)
            wrapper = create_tracked_wrapper(use_hand_tracking=True, display_mode='pass-through',
                                             zmq=False, webrtc=False, cert_file=str(cert), key_file=str(key))
            started = time.monotonic()
            eligible_since = active_since = None
            tracking_gate_seen = False
            if args.execute and args.wait_for_arm:
                arming_gate = StdinArmingGate(sys.stdin)
            last_log = 0.
            sequence = 0
            while True:
                now = time.monotonic()
                if not wrapper.tvuer.process.is_alive():
                    raise RuntimeError('Quest server process exited')
                data, snapshot = wrapper.get_tracked_data()
                eligible = tracking_eligible(snapshot)
                if not eligible:
                    eligible_since = None
                    report['tracking_rejections'] += 1
                elif eligible_since is None:
                    eligible_since = now
                stable = eligible_since is not None and now - eligible_since >= 0.5
                tracking_gate_seen = tracking_gate_seen or stable
                arm_allowed = not args.execute or not args.wait_for_arm
                if arming_gate is not None:
                    arm_allowed = arming_gate.poll(stable)
                    report['arming_status'] = arming_gate.last_event
                    if active_since is not None and not arming_gate.connected:
                        raise RuntimeError('Arming stdin disconnected; stopping live test')
                if active_since is None:
                    if now - started > args.wait_timeout and not tracking_gate_seen:
                        raise RuntimeError('No stable valid Quest tracking before connection deadline')
                    if stable and arm_allowed:
                        active_since = now
                        if args.execute:
                            args.arm_authorized = True
                            report['armed'] = True
                            report['armed_utc'] = datetime.now(timezone.utc).isoformat()
                            remote = RemoteBridge(args)
                            print('Tracking gate passed; waiting for healthy robot hand feedback.', flush=True)
                        else:
                            print('Tracking gate passed; numerical hand preview active.', flush=True)
                if remote is not None and remote.process.poll() is not None:
                    report['remote_exit_code'] = remote.process.returncode
                    if remote.process.returncode != 0:
                        raise RuntimeError(f'Robot bridge exited with code {remote.process.returncode}')
                    report['completed'] = report['sent'] > 0
                    result = 0 if report['completed'] else 1
                    break
                if active_since is not None and now - active_since >= args.duration + (12 if args.execute else 0):
                    report['completed'] = report['sent'] > 0 if args.execute else report['retargeted'] > 0
                    result = 0 if report['completed'] else 1
                    break
                q = None
                if eligible:
                    q = retarget_dfq(mapping, data)
                    report['retargeted'] += 1
                    if remote is not None and remote.ready.is_set():
                        sequence += 1
                        if remote.send({'seq': sequence, 'sent_at': time.time(), 'q': q}):
                            report['sent'] += 1
                        else:
                            report['backpressure_drops'] += 1
                if now - last_log >= 1:
                    report['last_tracking_snapshot'] = tracking_log_snapshot(snapshot)
                    print(json.dumps({'tracking_eligible': eligible, 'active': active_since is not None,
                                      'tracking_stable': stable, 'hands': report['last_tracking_snapshot'],
                                      'arming_status': arming_gate.last_event if arming_gate else 'not_required',
                                      'robot_ready': remote.ready.is_set() if remote else False,
                                      'sent': report['sent'], 'q_right_left': q}, allow_nan=False), flush=True)
                    last_log = now
                time.sleep(0.05)
    except KeyboardInterrupt:
        report['interrupted'] = True
        print('Stopping new finger commands.', flush=True)
        result = 130
    except Exception as exc:
        report['error'] = str(exc)
        print(f'Sender error: {exc}', file=sys.stderr, flush=True)
        result = 1
    finally:
        if remote is not None:
            remote.close()
            report['remote_exit_code'] = remote.process.returncode
            report['remote_messages'] = remote.messages
            report['robot_evidence'] = bridge_evidence(remote.messages)
            report['completed'] = (remote.process.returncode == 0 and
                                   report['robot_evidence']['published_count'] > 0 and
                                   'error' not in report and not report.get('interrupted', False))
            report['transport_completed'] = report['completed']
            if result != 130:
                result = 0 if report['completed'] else 1
        if wrapper is not None:
            try:
                wrapper.close()
            except Exception as exc:
                report['preview_cleanup_error'] = str(exc)
                report['completed'] = False
                result = 1
        report['ended_utc'] = datetime.now(timezone.utc).isoformat()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(f'Report: {args.report}', flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
