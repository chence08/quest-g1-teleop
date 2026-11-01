#!/usr/bin/env python3
"""Calibrated Quest wrists and fingers to G1 arms plus DFX over two SSH streams.

Default is hardware-free preview. --robot-feedback opens a read-only arm
subscriber; --execute lets r start streaming. Like upstream xr_teleoperate,
tracking loss, IK failure or a late packet holds the last target instead of
stopping: only p, q, a bridge exit or --duration end a run. No body, waist,
leg or balance command is created on this workstation.
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
from quest_dfx_sender import retarget_dfq, tracking_log_snapshot, RemoteBridge as HandBridge
from teleop_config import load_config, remote_python_command, validate_host_ip


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
    def __init__(self, head, wrists, robot_poses, q_arm, scale=1., translation_limit=1., rotation_limit=math.pi):
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
        return result

    def record(self):
        return {'q_arm': self.q_arm.tolist(), 'head_origin': self.origin.tolist(),
                'head_yaw_basis': self.basis.tolist(), 'neutral_operator_wrists': [p.tolist() for p in self.neutral],
                'robot_wrists': [p.tolist() for p in self.robot_poses], 'translation_scale': self.scale,
                'translation_limit_m': self.translation_limit, 'rotation_limit_rad': self.rotation_limit,
                'reference': 'head yaw frozen at calibration; head gaze does not drive arms'}


class LineControls:
    """Terminal controls; a disconnected terminal ends the session."""
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


def hands_fresh(snapshot, max_age=.5):
    """Both hands delivered a valid packet recently. A still hand counts as fresh."""
    for side in ('left', 'right'):
        item = snapshot.get(side, {})
        try:
            if not item.get('valid', False) or not 0 <= float(item.get('event_age', math.inf)) <= max_age:
                return False
        except (TypeError, ValueError):
            return False
    return True


def tracking_diagnostics(snapshot, max_age=.5):
    reasons = []
    for name in ('left', 'right', 'head'):
        item = snapshot.get(name, {})
        if not item.get('valid', False):
            reasons.append(name + '.invalid_packet')
        try:
            age = float(item.get('event_age', math.inf))
        except (TypeError, ValueError):
            age = math.inf
        if not 0 <= age <= max_age:
            reasons.append(name + '.event_stale')
    return reasons


def state_usable(state, max_age=1.5):
    """Measured arm q is present and recent; no mode, FSM or wall-clock checks."""
    try:
        finite_vector(state.get('q_arm'), 14)
        return 0 <= float(state.get('receive_age_s', math.inf)) <= max_age
    except (ValueError, TypeError):
        return False


def hands_usable(bridge):
    return bridge is not None and bridge.process.poll() is None and bridge.ready.is_set()


class ArmBridge:
    def __init__(self, args, execute=False):
        self.execute, self.latest, self.received, self.messages = execute, None, 0., []
        self.lock = threading.Lock()
        remote = remote_python_command('g1_upper_body_bridge.py',
                  ['--interface', args.robot_interface, '--duration', '0'])
        if execute:
            remote += ['--execute', '--max-velocity', str(args.max_velocity), '--speed-ramp', str(args.speed_ramp)]
        command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                   '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=3', '--', args.ssh_host, shlex.join(remote)]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, bufsize=0, start_new_session=True)
        os.set_blocking(self.process.stdin.fileno(), False)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for raw in iter(self.process.stdout.readline, b''):
            line = raw.decode('utf-8', errors='replace').rstrip()
            try:
                value = json.loads(line)
            except ValueError:
                print('ARM ' + line, flush=True)
                continue
            if not isinstance(value, dict):
                continue
            if value.get('type') != 'state':
                print('ARM ' + line, flush=True)
            with self.lock:
                if value.get('type') != 'state':
                    self.messages.append(value)
                else:
                    self.latest, self.received = value, time.monotonic()

    def state(self):
        with self.lock:
            result = dict(self.latest) if self.latest else {}
            result['receive_age_s'] = time.monotonic() - self.received if self.received else math.inf
            return result

    def send(self, value):
        """Best effort: False when SSH is backed up or closed; the robot holds the last target."""
        if not self.execute:
            raise RuntimeError('Read-only bridge cannot send commands')
        payload = (json.dumps(value, allow_nan=False, separators=(',', ':')) + '\n').encode()
        try:
            return os.write(self.process.stdin.fileno(), payload) == len(payload)
        except (BlockingIOError, BrokenPipeError, ValueError, OSError):
            return False

    def close(self):
        if self.process.stdin and not self.process.stdin.closed:
            try:
                self.process.stdin.close()
            except OSError:
                pass
        # Read-only ignores stdin EOF; closing its SSH ends remote stdout.
        # Execute retains time for the remote release ramp after stdin EOF.
        if not self.execute and self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=5 if self.execute else 2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.reader.join(timeout=1)


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

    def solve(self, targets):
        """Upstream solve_ik, warm-started from the previous solution; a failure keeps it."""
        previous = finite_vector(self.ik.init_data, 14).copy()
        q, tau = self.ik.solve_ik(targets[0], targets[1], previous)
        q = np.asarray(q, dtype=float)
        if q.shape != (14,) or not np.isfinite(q).all():
            self.ik.init_data = previous
            return previous, np.zeros(14), False
        q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        tau = np.asarray(tau, dtype=float)
        if tau.shape != (14,) or not np.isfinite(tau).all():
            tau = np.zeros(14)
        return q, tau, bool(self.ik.opti.stats().get('success', False))


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
    parser.add_argument('--arms-only', action='store_true', help='drive arms without a DFX SSH stream')
    parser.add_argument('--robot-feedback', action='store_true', help='read-only SSH feedback for calibration')
    parser.add_argument('--ssh-host', default=config['G1_SSH_HOST'])
    parser.add_argument('--host-ip', default=config['HOST_LAN_IP'])
    parser.add_argument('--robot-interface', default=config['G1_ARM_INTERFACE'])
    parser.add_argument('--duration', type=float, default=0., help='seconds per run; 0 runs until p or q')
    parser.add_argument('--frequency', type=float, default=30., help='IK and command rate, Hz')
    parser.add_argument('--translation-scale', type=float, default=1.)
    parser.add_argument('--translation-limit', type=float, default=1., help='max wrist offset from calibration, m')
    parser.add_argument('--rotation-limit', type=float, default=math.pi, help='max wrist rotation from calibration, rad')
    parser.add_argument('--tracking-timeout', type=float, default=.5,
                        help='hand packets older than this hold the last target, s')
    parser.add_argument('--max-velocity', type=float, default=10., help='robot-side joint velocity limit, rad/s')
    parser.add_argument('--speed-ramp', type=float, default=3., help='robot-side velocity ramp after engaging, s')
    parser.add_argument('--zero-feedforward', action='store_true', help='send zero instead of model gravity torque')
    parser.add_argument('--camera-host', help='optional SSH-forwarded TeleImager config+head stream host')
    parser.add_argument('--report', type=Path, default=PROJECT_ROOT / 'reports/upper-body-session.json')
    args = parser.parse_args(argv)
    validate_host_ip(parser, args.host_ip, args.execute)
    for name in ('duration', 'frequency', 'translation_scale', 'translation_limit', 'rotation_limit',
                 'tracking_timeout', 'max_velocity', 'speed_ramp'):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0:
            parser.error(f'{name} must be finite and nonnegative')
    if not 1 <= args.frequency <= 120:
        parser.error('frequency must be in [1,120]')
    if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', args.ssh_host) or args.ssh_host.startswith('-'):
        parser.error('Invalid SSH hostname')
    args.robot_feedback = args.robot_feedback or args.execute
    args.report = args.report.expanduser().resolve()
    return args


class Throttle:
    """Print a repeating event at most once per interval, with a suppressed count."""
    def __init__(self, interval=1.):
        self.interval, self.last, self.suppressed = interval, {}, {}

    def __call__(self, key, now=None):
        now = time.monotonic() if now is None else now
        if now - self.last.get(key, -math.inf) >= self.interval:
            self.last[key] = now
            return self.suppressed.pop(key, 0) + 1
        self.suppressed[key] = self.suppressed.get(key, 0) + 1
        return 0


def main(argv=None):
    args = parse_args(argv)
    report = {'started_utc': datetime.now(timezone.utc).isoformat(), 'execute_enabled': args.execute,
              'scope': 'arms only' if args.arms_only else 'arms and DFX fingers', 'calibrations': [],
              'sessions': [], 'events': []}
    wrapper = camera = feedback = active = hands = None
    result = 0
    calibration = None
    active_started = None
    sequence = 0
    current_session = None
    arm_q = arm_tau = hand_q = None
    feedback_retry_at = hands_retry_at = 0.
    throttle = Throttle()
    prepare_imports()
    from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

    def log_event(kind, **fields):
        event = {'event': kind, 'time': time.time(), **fields}
        report['events'].append(event)
        print(json.dumps(event, allow_nan=False, default=str), flush=True)

    def start_feedback():
        nonlocal feedback, feedback_retry_at
        if args.robot_feedback and feedback is None and active is None and time.monotonic() >= feedback_retry_at:
            feedback_retry_at = time.monotonic() + 3.
            try:
                feedback = ArmBridge(args)
            except OSError as exc:
                log_event('feedback_start_failed', reason=str(exc))

    def stop_hands():
        nonlocal hands
        if hands is not None:
            hands.close()
            if current_session is not None:
                current_session['hand_exit_codes'].append(hands.process.returncode)
            hands = None

    def stop_active(reason):
        nonlocal active, active_started, current_session
        for transport in (active, hands):
            if transport is not None and transport.process.stdin and not transport.process.stdin.closed:
                try:
                    transport.process.stdin.close()
                except OSError:
                    pass
        stop_hands()
        if active is not None:
            active.close()
            current_session['arm_messages'] = active.messages
            current_session['arm_exit_code'] = active.process.returncode
            active = None
        if current_session is not None:
            current_session.update(stop_reason=reason, ended_utc=datetime.now(timezone.utc).isoformat())
            report['sessions'].append(current_session)
            current_session = None
        active_started = None
        log_event('released', reason=reason, next_step='r resumes with the same calibration; c recalibrates')
        start_feedback()

    print('Controls (each + Enter): c calibrate, r run (calibrates first if needed), p pause, q quit.', flush=True)
    print(f'Quest: https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012', flush=True)
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
            last_log = 0.
            synthetic_q = np.clip(np.zeros(14), ik.model.lowerPositionLimit, ik.model.upperPositionLimit)
            start_feedback()
            period = 1. / args.frequency

            def calibrate(head, wrists, snapshot, state):
                nonlocal calibration, arm_q, arm_tau
                if not hands_fresh(snapshot, args.tracking_timeout) or not all(pose_is_se3(p) for p in [head, *wrists]):
                    log_event('calibration_rejected', reason='both hands must be visible',
                              tracking=tracking_diagnostics(snapshot, args.tracking_timeout))
                    return False
                if state_usable(state):
                    q = finite_vector(state['q_arm'], 14)
                elif args.robot_feedback:
                    log_event('calibration_rejected', reason='no arm feedback from the robot yet; wait and retry')
                    return False
                else:
                    q = synthetic_q
                q = np.clip(q, ik.model.lowerPositionLimit, ik.model.upperPositionLimit)
                calibration = RelativeCalibration(head, wrists, ik.fk(q), q, args.translation_scale,
                                                  args.translation_limit, args.rotation_limit)
                ik.reset(q)
                arm_q, arm_tau = q.copy(), np.zeros(14)
                report['calibrations'].append(calibration.record())
                log_event('calibrated', hardware_feedback=state_usable(state))
                return True

            while True:
                now = time.monotonic()
                if not wrapper.tvuer.process.is_alive():
                    raise RuntimeError('Quest server exited')
                data, snapshot, head, wrists = wrapper.snapshot()
                fresh = hands_fresh(snapshot, args.tracking_timeout) and all(pose_is_se3(p) for p in wrists)
                if feedback is not None and feedback.process.poll() is not None:
                    feedback.close()
                    feedback = None
                    log_event('feedback_ended', next_step='restarting')
                start_feedback()
                state = (active or feedback).state() if (active or feedback) else {}
                if camera is not None:
                    try:
                        camera.render(wrapper)
                    except Exception as exc:
                        if throttle('camera', now):
                            log_event('camera_frame_failed', reason=str(exc))

                if active is not None:
                    if active.process.poll() is not None:
                        stop_active('arm_bridge_ended')
                    elif args.duration and now - active_started > args.duration:
                        stop_active('duration_limit')
                if active is not None and not args.arms_only:
                    if hands is not None and hands.process.poll() is not None:
                        log_event('hand_bridge_ended', exit_code=hands.process.returncode, next_step='restarting')
                        stop_hands()
                        hands_retry_at = now + 2.
                    if hands is None and now >= hands_retry_at:
                        hands_retry_at = now + 2.
                        try:
                            hands = HandBridge(argparse.Namespace(execute=True, wait_for_arm=False,
                                                                  ssh_host=args.ssh_host, duration=0))
                        except OSError as exc:
                            log_event('hand_bridge_start_failed', reason=str(exc))

                for key in controls.read():
                    if key == 'q':
                        return result
                    if key == 'c':
                        calibrate(head, wrists, snapshot, state)
                    elif key == 'p':
                        if active is not None:
                            stop_active('operator_pause')
                        else:
                            log_event('not_running', next_step='r starts a run')
                    elif key == 'r':
                        if not args.execute:
                            log_event('run_rejected', reason='preview mode; use --execute to drive the robot')
                        elif active is not None:
                            log_event('already_running')
                        elif calibration is not None or calibrate(head, wrists, snapshot, state):
                            if feedback is not None:
                                feedback.close()
                                feedback = None
                            active = ArmBridge(args, execute=True)
                            active_started, sequence = time.monotonic(), 0
                            current_session = {'started_utc': datetime.now(timezone.utc).isoformat(),
                                               'arm_sent': 0, 'arm_dropped': 0, 'hand_sent': 0,
                                               'held_frames': 0, 'hand_exit_codes': []}
                            log_event('running', duration_s=args.duration or None)
                    elif key:
                        log_event('unknown_key', key=key)
                if not controls.connected:
                    if active is not None:
                        stop_active('operator_terminal_disconnected')
                    break

                solved = False
                if calibration is not None and fresh:
                    try:
                        q, tau, converged = ik.solve(calibration.targets(wrists))
                        arm_q, arm_tau, solved = q, tau, True
                        if not converged and throttle('ik', now):
                            log_event('ik_not_converged', note='using best effort solution')
                    except Exception as exc:
                        if throttle('target', now):
                            log_event('target_skipped', reason=str(exc))
                    if not args.arms_only:
                        try:
                            hand_q = retarget_dfq(mapping, data)
                        except Exception as exc:
                            if throttle('hand', now):
                                log_event('hand_retarget_skipped', reason=str(exc))

                if active is not None and arm_q is not None:
                    sequence += 1
                    current_session['held_frames'] += int(not solved)
                    tau = np.zeros(14) if args.zero_feedforward else arm_tau
                    if active.send({'seq': sequence, 'sent_at': time.time(), 'q_arm': arm_q.tolist(),
                                    'tau_arm': tau.tolist()}):
                        current_session['arm_sent'] += 1
                    else:
                        current_session['arm_dropped'] += 1
                    if hands_usable(hands) and hand_q is not None:
                        if hands.send({'seq': sequence, 'sent_at': time.time(), 'q': hand_q}):
                            current_session['hand_sent'] += 1

                if now - last_log >= 1:
                    print(json.dumps({'phase': 'running' if active else 'preview', 'tracking_fresh': fresh,
                          'tracking': tracking_log_snapshot(snapshot),
                          'tracking_problems': tracking_diagnostics(snapshot, args.tracking_timeout) if not fresh else None,
                          'robot_feedback': state_usable(state), 'calibrated': calibration is not None,
                          'hands_connected': hands_usable(hands),
                          'session': {k: v for k, v in current_session.items() if k != 'hand_exit_codes'}
                          if current_session else None,
                          'q_arm': None if arm_q is None else np.round(arm_q, 3).tolist()},
                          allow_nan=False, default=str), flush=True)
                    last_log = now
                time.sleep(max(0., period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        report['interrupted'] = True
    except Exception as exc:
        report['error'] = str(exc)
        print('Upper-body error: ' + str(exc), file=sys.stderr, flush=True)
        result = 1
    finally:
        args.robot_feedback = False  # Do not restart feedback while shutting down.
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
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False, default=str) + '\n')
        print('Report: ' + str(args.report), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
