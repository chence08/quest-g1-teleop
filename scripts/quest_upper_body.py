#!/usr/bin/env python3
"""Calibrated Quest wrists and fingers to G1 arms plus DFX over two SSH streams.

Default is hardware-free preview. --robot-feedback opens only a read-only arm
subscriber; --execute additionally permits explicit c then r arming. No body,
waist, leg, or balance command is created on this workstation.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from multiprocessing import Array
import os
from pathlib import Path
import re
import select
import shlex
import subprocess
import sys
import threading
import time
import numpy as np

from preview_quest import PROJECT_ROOT, model_workspace, pose_is_se3, prepare_imports
from quest_dfx_sender import retarget_dfq, tracking_eligible, tracking_log_snapshot, RemoteBridge as HandBridge
from robot_service_runner import require_clear_conflict_check
from teleop_config import load_config, remote_python_command, validate_host_ip


def prearm_conflict_check(ssh_host):
    """Recheck known controllers immediately before each explicit operator run."""
    remote = remote_python_command('robot_service_runner.py', ['--check-conflicts'])
    command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
               '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2', '--', ssh_host, shlex.join(remote)]
    checked = subprocess.run(command, capture_output=True, text=True, timeout=15)
    return require_clear_conflict_check(checked.stdout, checked.returncode)


def finite_vector(value, length):
    array = np.asarray(value, dtype=float)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError(f'Expected {length} finite values')
    return array


def cap_vector(vector, maximum):
    norm = float(np.linalg.norm(vector))
    return vector * min(1., maximum / norm) if norm else vector.copy()


def rotation_vector(matrix):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(matrix).as_rotvec()


def rotation_matrix(vector):
    from scipy.spatial.transform import Rotation
    return Rotation.from_rotvec(vector).as_matrix()


class RelativeCalibration:
    """Map wrist changes in a frozen initial head-yaw frame to measured robot FK."""
    def __init__(self, head, wrists, robot_poses, q_arm, scale=.5, translation_limit=.08, rotation_limit=.25):
        if not all(pose_is_se3(p) for p in [head, *wrists, *robot_poses]):
            raise ValueError('Calibration requires valid head, wrist and robot SE(3) poses')
        self.q_arm = finite_vector(q_arm, 14).copy()
        yaw = math.atan2(head[1, 0], head[0, 0])
        self.basis = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                               [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        self.origin = head[:3, 3].copy()
        self.scale, self.translation_limit, self.rotation_limit = scale, translation_limit, rotation_limit
        self.neutral = [self._operator_pose(p) for p in wrists]
        self.robot_poses = [p.copy() for p in robot_poses]

    def _operator_pose(self, wrist):
        value = np.eye(4)
        value[:3, :3] = self.basis.T @ wrist[:3, :3]
        value[:3, 3] = self.basis.T @ (wrist[:3, 3] - self.origin)
        return value

    def targets(self, wrists):
        if not all(pose_is_se3(p) for p in wrists):
            raise ValueError('Invalid wrist transform')
        result = []
        for wrist, neutral, robot in zip(wrists, self.neutral, self.robot_poses):
            current = self._operator_pose(wrist)
            target = robot.copy()
            target[:3, 3] += cap_vector(self.scale * (current[:3, 3] - neutral[:3, 3]), self.translation_limit)
            delta = rotation_vector(current[:3, :3] @ neutral[:3, :3].T)
            target[:3, :3] = rotation_matrix(cap_vector(delta, self.rotation_limit)) @ robot[:3, :3]
            result.append(target)
        if np.linalg.norm(result[0][:3, 3] - result[1][:3, 3]) < .12:
            raise ValueError('Wrist target separation below 0.12 m')
        return result

    def record(self):
        return {'q_arm': self.q_arm.tolist(), 'head_origin': self.origin.tolist(),
                'head_yaw_basis': self.basis.tolist(), 'neutral_operator_wrists': [p.tolist() for p in self.neutral],
                'robot_wrists': [p.tolist() for p in self.robot_poses], 'translation_scale': self.scale,
                'translation_limit_m': self.translation_limit, 'rotation_limit_rad': self.rotation_limit,
                'reference': 'head yaw frozen at calibration; head gaze does not drive arms'}


class LineControls:
    """Explicit terminal controls; a disconnected terminal must never arm."""
    def __init__(self, stream):
        self.fd, self.pending, self.connected = stream.fileno(), b'', True

    def read(self):
        lines = []
        if not self.connected:
            return lines
        try:
            for _ in range(16):
                if not select.select([self.fd], [], [], 0)[0]:
                    break
                raw = os.read(self.fd, 4096)
                if not raw:
                    self.connected = False
                    self.pending = b''
                    return []
                self.pending += raw
                parts = self.pending.split(b'\n')
                lines.extend(p.decode('ascii', errors='ignore').strip().lower() for p in parts[:-1])
                self.pending = parts[-1]
                if len(self.pending) > 4096:
                    self.pending = b''
        except (ValueError, OSError):
            self.connected = False
            return []
        return lines


def create_upper_wrapper(**kwargs):
    """Local head-event instrumentation; upstream and existing finger helper stay untouched."""
    from tracked_quest import TrackedTeleVuer, TrackedWrapper
    import televuer.tv_wrapper as module

    class UpperTeleVuer(TrackedTeleVuer):
        def __init__(self, *args, **options):
            self.head_status = Array('d', 3, lock=False)
            super().__init__(*args, **options)

        async def on_cam_move(self, event, session, fps=60):
            value = event.value if isinstance(event.value, dict) else {}
            camera = value.get('camera', {})
            raw = camera.get('matrix') if isinstance(camera, dict) else None
            try:
                pose = np.asarray(raw, dtype=float).reshape(4, 4).T
                valid = pose_is_se3(pose)
            except (ValueError, TypeError):
                valid = False
            with self.hand_tracker.lock:
                self.head_status[0] = int(valid)
                if valid:
                    await super().on_cam_move(event, session, fps)
                    self.head_status[1] = time.monotonic()
                    self.head_status[2] += 1

    class UpperWrapper(TrackedWrapper):
        def snapshot(self):
            from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT,
                                              T_TO_UNITREE_HUMANOID_LEFT_ARM, T_TO_UNITREE_HUMANOID_RIGHT_ARM)
            with self.tvuer.hand_tracker.lock:
                data, hands = self.get_tracked_data()
                valid, received, seq = self.tvuer.head_status[:]
                hands['head'] = {'valid': bool(valid), 'event_age': time.monotonic() - received if received else math.inf,
                                 'seq': int(seq)}
                raw = [self.tvuer.head_pose.copy(), self.tvuer.left_arm_pose.copy(), self.tvuer.right_arm_pose.copy()]
                converted = [T_ROBOT_OPENXR @ p @ T_OPENXR_ROBOT for p in raw]
                converted[1] = converted[1] @ T_TO_UNITREE_HUMANOID_LEFT_ARM
                converted[2] = converted[2] @ T_TO_UNITREE_HUMANOID_RIGHT_ARM
                return data, hands, converted[0], converted[1:]

    original = module.TeleVuer
    module.TeleVuer = UpperTeleVuer
    try:
        return UpperWrapper(module.TeleVuerWrapper(**kwargs))
    finally:
        module.TeleVuer = original


def upper_tracking_eligible(snapshot):
    head = snapshot.get('head', {})
    return (tracking_eligible(snapshot) and head.get('valid', False)
            and 0 <= float(head.get('event_age', math.inf)) <= .3)


def tracking_diagnostics(snapshot, head=None, wrists=None):
    """JSON-safe reasons distinguish stale/unchanged events from malformed poses."""
    samples, reasons, poses = {}, [], {}
    for name in ('left', 'right', 'head'):
        item = snapshot.get(name, {})
        samples[name] = {'valid': bool(item.get('valid', False)), 'seq': item.get('seq', 0)}
        if not samples[name]['valid']:
            reasons.append(name + '.invalid_packet')
        fields = ('event_age',) if name == 'head' else ('event_age', 'change_age')
        for field in fields:
            try:
                age = float(item.get(field, math.inf))
            except (ValueError, TypeError):
                age = math.inf
            samples[name][field] = age if math.isfinite(age) else None
            if not 0 <= age <= (.3 if name == 'head' else .25):
                reasons.append(name + ('.unchanged_pose_stale' if field == 'change_age' else '.event_stale'))
    if head is not None and wrists is not None:
        for name, value in zip(('head', 'left_wrist', 'right_wrist'), (head, *wrists)):
            matrix = np.asarray(value, dtype=float)
            valid = pose_is_se3(matrix)
            poses[name] = {'valid_se3': valid, 'shape': list(matrix.shape), 'finite': bool(np.isfinite(matrix).all())}
            if matrix.shape == (4, 4) and poses[name]['finite']:
                poses[name].update(homogeneous_row=bool(np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-4)),
                                   orthonormal=bool(np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-3)),
                                   determinant=float(np.linalg.det(matrix[:3, :3])))
            if not valid:
                reasons.append(name + '.invalid_se3')
    return {'samples': samples, 'poses': poses, 'reasons': reasons}


class RunPreparation:
    """Wait before the first write; any failure after a write attempt is latched."""
    def __init__(self):
        self.ready_since = None
        self.write_attempted = False
        self.halted = False

    def invalidate(self):
        self.ready_since = None
        self.halted = self.halted or self.write_attempted

    def ready(self, tracking_valid, feedback_ready, now=None):
        now = time.monotonic() if now is None else now
        if not tracking_valid or not feedback_ready:
            self.invalidate()
            return False
        if self.halted:
            return False
        if self.write_attempted:
            return True
        if self.ready_since is None:
            self.ready_since = now
        return now - self.ready_since >= .5


class SubmissionRejected(RuntimeError):
    def __init__(self, reason, diagnostics):
        super().__init__(reason)
        self.diagnostics = diagnostics


class ArmBridge:
    def __init__(self, args, execute=False):
        self.execute, self.latest, self.received, self.messages = execute, None, 0., []
        self.lock = threading.Lock()
        remote = remote_python_command('g1_upper_body_bridge.py',
                  ['--interface', args.robot_interface, '--duration', str(args.duration if execute else 120)])
        if execute:
            remote += ['--execute', '--expected-mode-machine', str(args.expected_mode_machine)]
        command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                   '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2', '--', args.ssh_host, shlex.join(remote)]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, bufsize=0, start_new_session=True)
        os.set_blocking(self.process.stdin.fileno(), False)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for raw in iter(self.process.stdout.readline, b''):
            line = raw.decode('utf-8', errors='replace').rstrip()
            print('ARM ' + line, flush=True)
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if not isinstance(value, dict):
                continue
            with self.lock:
                self.messages.append(value)
                if value.get('type') == 'state':
                    self.latest, self.received = value, time.monotonic()
                elif value.get('ready') is False:
                    self.latest = {**(self.latest or {}), 'ready': False,
                                   'reason': value.get('status', 'bridge_not_ready')}
                    self.received = time.monotonic()

    def state(self):
        with self.lock:
            result = dict(self.latest) if self.latest else {}
            result['receive_age_s'] = time.monotonic() - self.received if self.received else math.inf
            return result

    def send(self, value):
        if not self.execute:
            raise RuntimeError('Read-only bridge cannot send commands')
        payload = (json.dumps(value, allow_nan=False, separators=(',', ':')) + '\n').encode()
        try:
            count = os.write(self.process.stdin.fileno(), payload)
        except BlockingIOError:
            return False
        if count != len(payload):
            raise RuntimeError('Partial arm SSH write')
        return True

    def close(self):
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        # Read-only ignores stdin EOF; closing its SSH ends remote stdout.
        # Execute retains time for the remote release ramp after stdin EOF.
        if not self.execute and self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=4 if self.execute else 2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.reader.join(timeout=1)


def state_eligible(state, expected_mode_machine=None):
    try:
        finite_vector(state.get('q_arm'), 14)
        finite_vector(state.get('dq_arm'), 14)
        stamp = state.get('time', state.get('timestamp'))
        wall_age = time.time() - float(stamp) if stamp is not None else math.inf
        return (state.get('ready') is True and 0 <= float(state.get('receive_age_s', math.inf)) <= .8
                and -.2 <= wall_age <= 1.0
                and state.get('fsm_id') == 500
                and (expected_mode_machine is None or state.get('mode_machine') == expected_mode_machine))
    except (ValueError, TypeError):
        return False


def hand_state_eligible(bridge):
    if bridge is None or not bridge.ready.is_set() or bridge.last_message is None:
        return False
    try:
        age = time.time() - float(bridge.last_message.get('time'))
        return -.2 <= age <= 1.0
    except (TypeError, ValueError):
        return False


def send_fresh_solution(active, hands, snapshot, current_snapshot, sample_mono, sample_wall,
                        sequence, arm_q, hand_q, expected_mode_machine, arms_only=False, preparation=None):
    """Revalidate the actual IK input ages before either transport can write."""
    elapsed = time.monotonic() - sample_mono
    aged = {name: dict(snapshot.get(name, {})) for name in ('left', 'right', 'head')}
    for name, fields in (('left', ('event_age', 'change_age')), ('right', ('event_age', 'change_age')),
                         ('head', ('event_age',))):
        for field in fields:
            aged[name][field] = float(aged[name].get(field, math.inf)) + elapsed
    if (not math.isfinite(elapsed) or elapsed < 0 or not upper_tracking_eligible(aged)
            or not upper_tracking_eligible(current_snapshot)):
        if preparation is not None:
            preparation.invalidate()
        raise SubmissionRejected('stale_solution', {'elapsed_s': elapsed if math.isfinite(elapsed) else None,
                                 'input': tracking_diagnostics(aged), 'current': tracking_diagnostics(current_snapshot)})
    if not state_eligible(active.state(), expected_mode_machine) or (not arms_only and not hand_state_eligible(hands)):
        if preparation is not None:
            preparation.invalidate()
        raise SubmissionRejected('robot_feedback_not_ready_after_IK', {'current': tracking_diagnostics(current_snapshot)})
    if preparation is not None:
        if not preparation.ready(True, True):
            return False, False
        # Latch before the first attempted write: partial writes/errors cannot resume preparation.
        preparation.write_attempted = True
    arm_sent = active.send({'seq': sequence, 'sent_at': sample_wall,
                            'q_arm': np.asarray(arm_q).tolist(), 'tau_arm': [0.] * 14})
    if not arm_sent:
        raise RuntimeError('arm_transport_backpressure')
    hand_sent = hands.send({'seq': sequence, 'sent_at': sample_wall, 'q': hand_q}) if not arms_only else False
    if not arms_only and not hand_sent:
        raise RuntimeError('hand_transport_backpressure')
    return arm_sent, hand_sent


class CheckedIK:
    def __init__(self):
        import pinocchio as pin
        from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
        self.pin, self.ik = pin, G1_29_ArmIK(Visualization=False)
        self.model = self.ik.reduced_robot.model
        self.data = self.model.createData()
        expected = [f'{side}_{joint}_joint' for side in ('left', 'right')
                    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
        actual = list(self.model.names)[1:]
        if actual != expected or self.model.nq != 14:
            raise RuntimeError('Unexpected G1 arm joint ordering')

    def fk(self, q):
        q = finite_vector(q, 14)
        self.pin.framesForwardKinematics(self.model, self.data, q)
        return [self.data.oMf[index].homogeneous.copy() for index in (self.ik.L_hand_id, self.ik.R_hand_id)]

    def reset(self, q):
        from teleop.utils.weighted_moving_filter import WeightedMovingFilter
        self.ik.init_data = finite_vector(q, 14).copy()
        self.ik.smooth_filter = WeightedMovingFilter(np.array([.4, .3, .2, .1]), 14)
        self.ik.smooth_filter.add_data(self.ik.init_data)

    def solve(self, targets, measured, initial, excursion=.15):
        measured, initial = finite_vector(measured, 14), finite_vector(initial, 14)
        actual = self.fk(measured)
        if all(np.allclose(a, b, atol=1e-8) for a, b in zip(actual, targets)):
            return measured.copy(), {'position_m': [0., 0.], 'rotation_rad': [0., 0.], 'solver': 'exact_neutral'}
        q, _unused_model_torque = self.ik.solve_ik(targets[0], targets[1], measured)
        if not self.ik.opti.stats().get('success', False):
            raise ValueError('IK solver did not converge')
        q = finite_vector(q, 14)
        q = np.clip(q, np.maximum(self.model.lowerPositionLimit, initial - excursion),
                    np.minimum(self.model.upperPositionLimit, initial + excursion))
        actual = self.fk(q)
        position = [float(np.linalg.norm(a[:3, 3] - b[:3, 3])) for a, b in zip(actual, targets)]
        rotation = [float(np.linalg.norm(rotation_vector(a[:3, :3] @ b[:3, :3].T))) for a, b in zip(actual, targets)]
        if max(position) > .04 or max(rotation) > .25:
            raise ValueError(f'IK target residual too large: position={position}, rotation={rotation}')
        if np.linalg.norm(actual[0][:3, 3] - actual[1][:3, 3]) < .12:
            raise ValueError('Robot wrist separation too small')
        return q, {'position_m': position, 'rotation_rad': rotation, 'solver': 'converged'}


class CameraPreview:
    def __init__(self, host):
        import teleimager.image_client as module
        cache = PROJECT_ROOT / '.cache' / 'upper-camera-client.yaml'
        cache.parent.mkdir(parents=True, exist_ok=True)
        module.CONFIG_CLIENT_PATH = str(cache)
        self.client = module.ImageClient(host=host, request_port=60000, request_bgr=True)
        self.configuration = self.client.get_cam_config()['head_camera']
        self.seen = False

    def options(self):
        return {'display_mode': 'ego', 'zmq': True, 'webrtc': False,
                'binocular': bool(self.configuration['binocular']), 'img_shape': tuple(self.configuration['image_shape'])}

    def render(self, wrapper):
        frame = self.client.get_head_frame()
        if bool(frame) and frame.bgr is not None:
            wrapper.tvuer.render_to_xr(frame.bgr)
            self.seen = True
            return True
        shape = self.configuration['image_shape']
        wrapper.tvuer.render_to_xr(np.zeros((shape[0], shape[1], 3), dtype=np.uint8))
        return False

    def close(self):
        self.client.close()


def parse_args(argv=None):
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--arms-only', action='store_true', help='commission arms without constructing a DFX SSH stream')
    parser.add_argument('--robot-feedback', action='store_true', help='read-only SSH feedback for calibration')
    parser.add_argument('--ssh-host', default=config['G1_SSH_HOST'])
    parser.add_argument('--host-ip', default=config['HOST_LAN_IP'])
    parser.add_argument('--robot-interface', default=config['G1_ARM_INTERFACE'])
    parser.add_argument('--expected-mode-machine', type=int, default=6)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--wait-timeout', type=float, default=600.)
    parser.add_argument('--translation-scale', type=float, default=.5)
    parser.add_argument('--translation-limit', type=float, default=.08)
    parser.add_argument('--rotation-limit', type=float, default=.25)
    parser.add_argument('--joint-excursion', type=float, default=.15)
    parser.add_argument('--camera-host', help='optional SSH-forwarded TeleImager config+head stream host')
    parser.add_argument('--report', type=Path, default=PROJECT_ROOT / 'reports/upper-body-session.json')
    args = parser.parse_args(argv)
    validate_host_ip(parser, args.host_ip, args.execute)
    for name, maximum in (('duration', 120.), ('wait_timeout', 3600.), ('translation_scale', 1.),
                           ('translation_limit', .3), ('rotation_limit', .75), ('joint_excursion', .15)):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= maximum:
            parser.error(f'{name} must be in (0,{maximum}]')
    if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', args.ssh_host) or args.ssh_host.startswith('-'):
        parser.error('Invalid SSH hostname')
    args.robot_feedback = args.robot_feedback or args.execute
    args.report = args.report.expanduser().resolve()
    return args


def main(argv=None):
    args = parse_args(argv)
    report = {'started_utc': datetime.now(timezone.utc).isoformat(), 'execute_enabled': args.execute,
              'scope': 'arms only' if args.arms_only else 'arms and DFX fingers only', 'calibrations': [], 'sessions': [], 'events': [],
              'collision_check': 'wrist separation and bounded local workspace only; no complete self/environment collision model',
              'torque_feedforward': 'zero; stock hand inertia is not calibrated for DFX'}
    wrapper = camera = feedback = active = hands = None
    result = 0
    calibration = None
    active_started = None
    preparation = None
    sequence = 0
    current_session = None
    prepare_imports()
    from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

    def log_event(kind, **fields):
        event = {'event': kind, 'time': time.time(), **fields}
        report['events'].append(event)
        print(json.dumps(event, allow_nan=False), flush=True)

    def stop_active(reason, diagnostics=None):
        nonlocal active, hands, calibration, active_started, preparation, current_session
        # Signal both remote watchdogs immediately, before waiting on either process.
        for transport in (active, hands):
            if transport is not None and transport.process.stdin and not transport.process.stdin.closed:
                transport.process.stdin.close()
        if diagnostics is not None:
            current_session['stop_diagnostics'] = diagnostics
            log_event('stopping', reason=reason, command_write_attempted=preparation.write_attempted,
                      diagnostics=diagnostics)
        if hands is not None:
            hands.close()
            current_session['hand_messages'] = hands.messages
            current_session['hand_exit_code'] = hands.process.returncode
            hands = None
        if active is not None:
            active.close()
            current_session['arm_messages'] = active.messages
            current_session['arm_exit_code'] = active.process.returncode
            active = None
        if current_session is not None:
            current_session['stop_reason'] = reason
            current_session['ended_utc'] = datetime.now(timezone.utc).isoformat()
            report['sessions'].append(current_session)
            current_session = None
        calibration, active_started, preparation = None, None, None
        log_event('released', reason=reason, next_step='fresh calibration c then explicit r required')

    print('Quest upper-body preview. Controls: c + Enter calibrate, r + Enter run, p + Enter release/clutch, q + Enter quit.', flush=True)
    print(f'Quest: https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012', flush=True)
    print('No execute publisher is started before c then r. Head yaw freezes at calibration. Zero model torque feedforward.', flush=True)
    try:
        with model_workspace():
            ik, mapping = CheckedIK(), HandRetargeting(HandType.INSPIRE_HAND)
            controls = LineControls(sys.stdin)
            options = {'display_mode': 'pass-through', 'zmq': False, 'webrtc': False}
            if args.camera_host:
                camera = CameraPreview(args.camera_host)
                options = camera.options()
            wrapper = create_upper_wrapper(use_hand_tracking=True,
                    cert_file=str(PROJECT_ROOT / 'certs/cert.pem'), key_file=str(PROJECT_ROOT / 'certs/key.pem'), **options)
            started, last_log = time.monotonic(), 0.
            eligible_since = None
            synthetic_q = np.clip(np.zeros(14), ik.model.lowerPositionLimit, ik.model.upperPositionLimit)
            if args.robot_feedback:
                feedback = ArmBridge(args)
            while True:
                now = time.monotonic()
                solution = None
                if not wrapper.tvuer.process.is_alive():
                    raise RuntimeError('Quest server exited')
                sample_mono, sample_wall = time.monotonic(), time.time()
                data, snapshot, head, wrists = wrapper.snapshot()
                valid = upper_tracking_eligible(snapshot) and all(pose_is_se3(p) for p in [head, *wrists])
                eligible_since = (eligible_since or now) if valid else None
                stable = eligible_since is not None and now - eligible_since >= .5
                if active is None and feedback is not None and feedback.process.poll() is not None:
                    feedback.close()
                    feedback = ArmBridge(args)
                    calibration = None
                    log_event('feedback_restarted', next_step='recalibrate')
                state = (active or feedback).state() if (active or feedback) else {}
                healthy = state_eligible(state, args.expected_mode_machine)
                q_measured = finite_vector(state['q_arm'], 14) if healthy else synthetic_q
                if camera is not None:
                    camera.render(wrapper)
                if active is not None:
                    reason = None
                    if preparation.write_attempted and not valid:
                        reason = 'tracking_invalid_or_stale'
                    elif active.process.poll() is not None:
                        reason = 'arm_bridge_ended'
                    elif hands is not None and hands.process.poll() is not None:
                        reason = 'hand_bridge_ended'
                    elif now - active_started > args.duration + 3:
                        reason = 'duration_limit'
                    elif preparation.write_attempted and (not healthy or (not args.arms_only and not hand_state_eligible(hands))):
                        reason = 'robot_feedback_not_ready'
                    preparation.ready(valid, healthy and (args.arms_only or hand_state_eligible(hands)), now)
                    if reason:
                        stop_active(reason, tracking_diagnostics(snapshot, head, wrists))
                        if args.robot_feedback and feedback is None:
                            feedback = ArmBridge(args)
                        state, healthy = {}, False
                for key in controls.read():
                    if key == 'q':
                        return result
                    if key == 'p':
                        if active is not None:
                            stop_active('operator_pause')
                            if args.robot_feedback and feedback is None:
                                feedback = ArmBridge(args)
                        else:
                            calibration = None
                            log_event('calibration_cleared')
                    elif key == 'c':
                        if active is not None or not stable or (args.robot_feedback and not healthy):
                            log_event('calibration_rejected', reason='need paused, stable tracking and fresh robot feedback')
                            continue
                        calibration = RelativeCalibration(head, wrists, ik.fk(q_measured), q_measured,
                                  args.translation_scale, args.translation_limit, args.rotation_limit)
                        ik.reset(q_measured)
                        report['calibrations'].append(calibration.record())
                        log_event('calibrated', hardware_feedback=healthy)
                    elif key == 'r':
                        if not args.execute:
                            log_event('run_rejected', reason='preview mode; use --execute to enable explicit arming')
                        elif active is not None or calibration is None or not stable or not healthy:
                            log_event('run_rejected', reason='need fresh c calibration, stable tracking and robot feedback')
                        else:
                            try:
                                check = prearm_conflict_check(args.ssh_host)
                            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                                calibration = None
                                log_event('run_rejected', reason=str(exc), next_step='resolve controller ownership, then new c and r')
                                continue
                            log_event('prearm_conflict_check', **{key: value for key, value in check.items() if key != 'event'})
                            # SSH can take seconds: never arm from the snapshot captured before this check.
                            sample_mono, sample_wall = time.monotonic(), time.time()
                            data, snapshot, head, wrists = wrapper.snapshot()
                            valid = upper_tracking_eligible(snapshot) and all(pose_is_se3(p) for p in [head, *wrists])
                            state = feedback.state()
                            healthy = state_eligible(state, args.expected_mode_machine)
                            if not valid or not healthy or feedback.process.poll() is not None:
                                calibration = None
                                eligible_since = None
                                log_event('run_rejected', reason='tracking or robot feedback stale after conflict check; new c and r required')
                                continue
                            q_measured = finite_vector(state['q_arm'], 14)
                            if np.max(np.abs(q_measured - calibration.q_arm)) > .05:
                                calibration = None
                                log_event('run_rejected', reason='robot moved after calibration; recalibrate')
                                continue
                            feedback.close()
                            feedback = None
                            active = ArmBridge(args, execute=True)
                            active_started, preparation = time.monotonic(), RunPreparation()
                            state, healthy = {}, False
                            sequence = 0
                            current_session = {'started_utc': datetime.now(timezone.utc).isoformat(),
                                               'arm_sent': 0, 'hand_sent': 0, 'operator_observation_confirmed': False}
                            log_event('armed', duration_s=args.duration, phase='preparing',
                                      next_step='waiting for 0.5 s of fresh tracking and ready feedback before first command')
                if not controls.connected:
                    if active is not None:
                        stop_active('operator_terminal_disconnected')
                    break
                if active is not None and healthy and hands is None and not args.arms_only:
                    hand_args = argparse.Namespace(execute=True, wait_for_arm=True, arm_authorized=True,
                                                   ssh_host=args.ssh_host, duration=args.duration)
                    hands = HandBridge(hand_args)
                can_compute = (active is None or preparation.ready(valid,
                               healthy and (args.arms_only or hand_state_eligible(hands))))
                if calibration is not None and valid and (not args.robot_feedback or healthy) and can_compute:
                    try:
                        targets = calibration.targets(wrists)
                        arm_q, residual = ik.solve(targets, q_measured, calibration.q_arm, args.joint_excursion)
                        hand_q = retarget_dfq(mapping, data) if not args.arms_only else None
                        solution = {'q_arm_requested': arm_q.tolist(), 'target_poses': [p.tolist() for p in targets],
                                    'ik_residuals': residual}
                    except (ValueError, RuntimeError) as exc:
                        if active is not None:
                            stop_active('IK_or_retarget_rejected: ' + str(exc))
                            if args.robot_feedback and feedback is None:
                                feedback = ArmBridge(args)
                        log_event('target_rejected', reason=str(exc))
                    else:
                        if active is not None and (args.arms_only or hand_state_eligible(hands)) and healthy:
                            sequence += 1
                            _, current_snapshot, _, _ = wrapper.snapshot()
                            try:
                                arm_sent, hand_sent = send_fresh_solution(active, hands, snapshot, current_snapshot,
                                          sample_mono, sample_wall, sequence, arm_q, hand_q,
                                          args.expected_mode_machine, args.arms_only, preparation)
                            except (ValueError, RuntimeError) as exc:
                                details = getattr(exc, 'diagnostics', tracking_diagnostics(snapshot, head, wrists))
                                if isinstance(exc, SubmissionRejected) and not preparation.write_attempted:
                                    preparation.invalidate()
                                    log_event('preparation_wait', reason=str(exc), diagnostics=details)
                                else:
                                    stop_active(str(exc), details)
                                    if args.robot_feedback and feedback is None:
                                        feedback = ArmBridge(args)
                            else:
                                current_session['arm_sent'] += int(arm_sent)
                                current_session['hand_sent'] += int(hand_sent)
                if now - last_log >= 1:
                    head_status = dict(snapshot['head'])
                    if not math.isfinite(head_status['event_age']):
                        head_status['event_age'] = None
                    print(json.dumps({'preview': active is None, 'tracking_stable': stable,
                          'tracking': tracking_log_snapshot(snapshot), 'head': head_status,
                          'robot_feedback_ready': healthy, 'calibrated': calibration is not None,
                          'active': active is not None, 'arm_sent': current_session['arm_sent'] if current_session else 0,
                          'phase': ('streaming' if preparation.write_attempted else 'preparing') if active else 'preview',
                          'tracking_diagnostics': tracking_diagnostics(snapshot, head, wrists) if not valid else None,
                          'camera_frame_seen': camera.seen if camera else False, 'solution': solution}, allow_nan=False), flush=True)
                    last_log = now
                if active is None and now - started > args.wait_timeout:
                    raise RuntimeError('Calibration deadline elapsed')
                time.sleep(max(0., .05 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        report['interrupted'] = True
    except Exception as exc:
        report['error'] = str(exc)
        print('Upper-body error: ' + str(exc), file=sys.stderr, flush=True)
        result = 1
    finally:
        if active is not None or hands is not None:
            stop_active('sender_exit')
        if feedback is not None:
            feedback.close()
        if camera is not None:
            camera.close()
        if wrapper is not None:
            wrapper.close()
        report['ended_utc'] = datetime.now(timezone.utc).isoformat()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print('Report: ' + str(args.report), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
