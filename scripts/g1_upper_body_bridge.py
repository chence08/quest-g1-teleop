#!/usr/bin/env python3
"""Bounded G1_29 arm_sdk bridge over newline JSON; read-only unless --execute.

Only arm motors 15..28 and overlay weight slot29 are populated. Other motor
fields stay zero. Native firmware decides overlay behavior; waist/IMU drift is
monitored, not a guarantee that the body cannot move. No mode-setting RPCs,
lowcmd publisher, home motion, or nonzero feedforward torque are used.
"""
import argparse
from dataclasses import dataclass
import fcntl
import json
import math
import os
import select
import signal
import sys
import threading
import time

ARM_INDICES = tuple(range(15, 29))
WEIGHT_INDEX = 29
JOINT_NAMES = tuple(f"{side}_{joint}_joint" for side in ("left", "right") for joint in
                    ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                     "wrist_roll", "wrist_pitch", "wrist_yaw"))
# Pinned xr_teleoperate assets/g1/g1_body29_hand14.urdf. Verified by offline test.
LOWER = (-3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558,
         -3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558)
UPPER = (2.6704, 2.2515, 2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558,
         2.6704, 1.5882, 2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558)
EXPECTED_FSM = 500
PD_LIMIT = tuple(3. if i % 7 >= 5 else 15. for i in range(14))
MEASURED_TAU_LIMIT = tuple(4. if i % 7 >= 5 else 20. for i in range(14))


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def vector(values, size, name):
    if not isinstance(values, (tuple, list)) or len(values) != size or not all(finite(x) for x in values):
        raise ValueError(f"{name} must contain exactly {size} finite numbers")
    return tuple(float(x) for x in values)


def arm_positions(values):
    q = vector(values, 14, "q_arm")
    if any(not low <= value <= high for value, low, high in zip(q, LOWER, UPPER)):
        raise ValueError("q_arm outside pinned G1 URDF limits")
    return q


def wrapped_delta(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


@dataclass(frozen=True)
class Command:
    seq: int
    sent_at: float
    q_arm: tuple
    received_at: float
    initial_age: float

    def age(self, now):
        return self.initial_age + max(0., now - self.received_at)


def validate_command(packet, wall_now, now, last_seq, max_age=0.3):
    if not isinstance(packet, dict):
        raise ValueError("command must be an object")
    seq, stamp = packet.get("seq"), packet.get("sent_at")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0 or seq <= last_seq:
        raise ValueError("seq must increase strictly")
    if not finite(stamp) or not -0.05 <= wall_now - stamp <= max_age:
        raise ValueError("stale/future command timestamp")
    tau = vector(packet.get("tau_arm", [0.] * 14), 14, "tau_arm")
    if any(value != 0 for value in tau):
        raise ValueError("initial commissioning accepts zero feedforward torque only")
    return Command(seq, float(stamp), arm_positions(packet.get("q_arm")), now, max(0., wall_now - stamp))


class Mailbox:
    """Latest only; malformed input and EOF latch a fault and cannot re-enable."""
    def __init__(self, max_age=0.3):
        self.lock = threading.Lock()
        self.max_age = max_age
        self.command = None
        self.last_seq = -1
        self.fault = None

    def fail(self, reason):
        with self.lock:
            self.fault = self.fault or reason
            self.command = None

    def ingest(self, packet, wall_now, now):
        with self.lock:
            if self.fault:
                return False
            try:
                self.command = validate_command(packet, wall_now, now, self.last_seq, self.max_age)
            except ValueError as exc:
                self.fault = "invalid_command: " + str(exc)
                self.command = None
                return False
            self.last_seq = self.command.seq
            return True

    def snapshot(self, now):
        with self.lock:
            age = None if self.command is None else self.command.age(now)
            command = self.command if age is not None and age <= self.max_age and not self.fault else None
            return command, age, self.fault


def read_commands(stream, mailbox, stop):
    pending = b""
    try:
        fd = stream.fileno()
        while not stop.is_set():
            if not select.select([fd], [], [], .05)[0]:
                continue
            chunk = os.read(fd, 16384)
            if not chunk:
                mailbox.fail("stdin_eof")
                return
            pending += chunk
            if len(pending) > 65536:
                mailbox.fail("input_record_too_large")
                return
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                try:
                    packet = json.loads(line)
                except (ValueError, UnicodeError):
                    mailbox.fail("invalid_json")
                    return
                if not mailbox.ingest(packet, time.time(), time.monotonic()):
                    return
    except Exception as exc:
        mailbox.fail("stdin_error: " + str(exc))


class StateGate:
    """Duplicate ticks never refresh receipt age; active faults are latched."""
    def __init__(self, expected_mode=6, state_max_age=.1, quiet_speed=.05, body_limit=.05):
        self.expected_mode = expected_mode
        self.state_max_age = state_max_age
        self.quiet_speed = quiet_speed
        self.body_limit = body_limit
        self.state = None
        self.received_at = None
        self.stable_frames = 0
        self.duplicates = 0
        self.fsm_id = None
        self.fsm_received = None
        self.fsm_error = None
        self.engaged = False
        self.fault = None
        self.immediate = False
        self.reason = "waiting_for_state"
        self.baseline = None
        self.initial_arm = self.target = None

    def fail(self, reason, immediate=True):
        self.stable_frames = 0
        self.reason = reason
        if self.engaged:
            self.fault = self.fault or reason
            self.immediate = self.immediate or immediate

    def update_fsm(self, code, fsm_id, now):
        self.fsm_id = fsm_id if isinstance(fsm_id, int) and not isinstance(fsm_id, bool) else None
        self.fsm_received = now
        self.fsm_error = None if code == 0 and self.fsm_id == EXPECTED_FSM else "native_fsm_not_500"
        if self.fsm_error and self.engaged:
            self.fail(self.fsm_error)

    def update(self, packet, now, wall_now):
        try:
            q = arm_positions(packet["q_arm"])
            dq = vector(packet["dq_arm"], 14, "dq_arm")
            tau = vector(packet.get("tau_arm", [0.] * 14), 14, "tau_arm")
            waist = vector(packet["q_waist"], 3, "q_waist")
            imu = vector(packet["imu_rpy"], 3, "imu_rpy")
            tick = packet["tick"]
            if not isinstance(tick, int) or isinstance(tick, bool) or not 0 <= tick < 2**32:
                raise ValueError("invalid tick")
            if packet["mode_machine"] != self.expected_mode or packet["mode_pr"] != 0:
                raise ValueError("mode_machine/mode_pr changed")
            if list(packet["arm_mode"]) != [1] * 14 or list(packet["arm_status"]) != [0] * 14:
                raise ValueError("arm motor mode/status invalid")
            if self.state is not None:
                advance = (tick - self.state["tick"]) & 0xFFFFFFFF
                if advance == 0:
                    self.duplicates += 1
                    return False
                if advance >= 2**31:
                    raise ValueError("lowstate tick moved backwards")
            if self.received_at is not None and now - self.received_at > self.state_max_age:
                self.fail("stale_lowstate_gap", immediate=False)
            if self.engaged and self.baseline is not None:
                if any(wrapped_delta(a, b) > self.body_limit for a, b in zip(waist + imu, self.baseline)):
                    raise ValueError("waist_or_imu_drift")
                if max(abs(a - b) for a, b in zip(q, self.initial_arm)) > .25:
                    raise ValueError("measured_arm_excursion")
                if max(abs(a - b) for a, b in zip(q, self.target)) > .2:
                    raise ValueError("arm_tracking_error")
                if max(abs(value) for value in dq) > .75:
                    raise ValueError("excessive_arm_velocity")
            if any(abs(value) > limit for value, limit in zip(tau, MEASURED_TAU_LIMIT)):
                raise ValueError("excessive_measured_arm_torque")
        except (ValueError, KeyError, TypeError) as exc:
            self.fail("invalid_state: " + str(exc))
            return False
        self.received_at = now
        self.state = {"q_arm": q, "dq_arm": dq, "tau_arm": tau, "q_waist": waist, "imu_rpy": imu,
                      "mode_machine": packet["mode_machine"], "mode_pr": 0, "tick": tick,
                      "timestamp": wall_now}
        if not self.engaged and max(abs(x) for x in dq) > self.quiet_speed:
            self.fail("arms_not_quiet")
        elif not self.fault:
            self.stable_frames += 1
            self.reason = "ready" if self.stable_frames >= 10 else "qualifying_state"
        return self.stable_frames >= 10 and not self.fault

    def engage(self):
        self.engaged = True
        self.baseline = self.state["q_waist"] + self.state["imu_rpy"]
        self.initial_arm = self.target = self.state["q_arm"]

    def snapshot(self, now):
        age = None if self.received_at is None else max(0., now - self.received_at)
        fsm_age = None if self.fsm_received is None else max(0., now - self.fsm_received)
        if age is None or age > self.state_max_age:
            self.fail("waiting_for_state" if age is None else "stale_lowstate", immediate=False)
        fsm_ok = self.fsm_error is None and self.fsm_id == EXPECTED_FSM and fsm_age is not None and fsm_age <= .4
        if self.engaged and not fsm_ok:
            self.fail(self.fsm_error or "stale_native_fsm")
        ready = self.stable_frames >= 10 and not self.fault and fsm_ok
        result = dict(self.state or {"q_arm": None, "dq_arm": None, "tau_arm": None, "q_waist": None, "imu_rpy": None,
                                     "mode_machine": None, "mode_pr": None, "tick": None, "timestamp": None})
        result.update(ready=ready, fsm_id=self.fsm_id, state_age_s=age, fsm_age_s=fsm_age,
                      stable_frames=self.stable_frames, duplicate_ticks=self.duplicates,
                      reason=self.fault or (self.reason if fsm_ok else self.fsm_error or "waiting_for_native_fsm"),
                      fault=self.fault, immediate_release=self.immediate)
        return result


def bounded_step(initial, previous, requested, dt, slew=.15, excursion=.15):
    initial, previous, requested = map(arm_positions, (initial, previous, requested))
    if not all(finite(x) and x >= 0 for x in (dt, slew, excursion)):
        raise ValueError("invalid step parameters")
    output = []
    for origin, old, target, lower, upper in zip(initial, previous, requested, LOWER, UPPER):
        low, high = max(lower, origin - excursion), min(upper, origin + excursion)
        if not low - 1e-9 <= old <= high + 1e-9:
            raise ValueError("previous target outside excursion envelope")
        target = min(high, max(low, target))
        output.append(min(high, max(low, old + max(-slew * dt, min(slew * dt, target - old)))))
    return tuple(output)


class ArmMotion:
    def __init__(self, q, now, period=.01, slew=.15, excursion=.15, ramp_up=2., release_time=.5):
        self.initial = self.target = arm_positions(q)
        self.started = self.last_step = now
        self.period, self.slew, self.excursion = period, slew, excursion
        self.ramp_up, self.release_time = ramp_up, release_time
        self.weight = 0.
        self.phase = "ramp_up_hold"
        self.release_at = None
        self.release_weight = 0.
        self.reason = None

    def release(self, now, reason, immediate=False):
        self.reason = self.reason or reason
        if self.phase == "released":
            return  # Irrevocable: finally/repeated requests must never restore weight.
        if self.release_at is None:
            self.release_at, self.release_weight = now, self.weight
        self.phase = "releasing"
        if immediate:
            self.weight = 0.
            self.phase = "released"

    def step(self, now, requested=None):
        dt = min(self.period, max(0., now - self.last_step))
        self.last_step = now
        if self.phase == "released":
            return
        if self.release_at is not None:
            self.weight = min(self.weight, max(0., self.release_weight * (1. - max(0., now - self.release_at) / self.release_time)))
            if self.weight <= 0:
                self.phase = "released"
            return
        if now - self.started < self.ramp_up:
            self.weight = min(1., max(0., (now - self.started) / self.ramp_up))
            return  # Keep the measured startup posture during weight ramp.
        self.weight = 1.
        self.phase = "tracking"
        if requested is not None:
            self.target = bounded_step(self.initial, self.target, requested, dt, self.slew, self.excursion)


def build_message(factory, crc, q, weight, mode_machine=6, feedback=None):
    q = arm_positions(q)
    if not finite(weight) or not 0 <= weight <= 1:
        raise ValueError("invalid overlay weight")
    if weight > 0:
        if feedback is None:
            raise ValueError("positive weight requires fresh feedback for PD estimate")
        observed = arm_positions(feedback["q_arm"])
        velocity = vector(feedback["dq_arm"], 14, "dq_arm")
        for i, (target, actual, speed, limit) in enumerate(zip(q, observed, velocity, PD_LIMIT)):
            kp = 40. if i % 7 >= 4 else 60.
            if abs(kp * (target - actual) - 1.5 * speed) > limit:
                raise ValueError("estimated_PD_effort_exceeded at arm joint " + str(i))
    message = factory()  # fresh all-zero legs, waist and unused slots each tick
    message.mode_pr, message.mode_machine = 0, mode_machine
    for index, value in zip(ARM_INDICES, q):
        motor = message.motor_cmd[index]
        motor.mode, motor.q, motor.dq, motor.tau = 1, value, 0., 0.
        motor.kp = 40. if (index - 15) % 7 >= 4 else 60.
        motor.kd = 1.5
    message.motor_cmd[WEIGHT_INDEX].q = weight
    message.crc = crc.Crc(message)
    return message


class JsonOutput:
    """Nonblocking telemetry: a stalled SSH reader cannot stall control/release."""
    def __init__(self, fd=1):
        self.fd, self.pending, self.closed = fd, b"", False
        os.set_blocking(fd, False)

    def flush(self):
        if self.pending and not self.closed:
            try:
                count = os.write(self.fd, self.pending)
                self.pending = self.pending[count:]
            except BlockingIOError:
                pass
            except OSError:
                self.closed = True

    def emit(self, kind, **fields):
        self.flush()
        if not self.pending and not self.closed:
            self.pending = (json.dumps({"type": kind, "time": time.time(), **fields}, allow_nan=False) + "\n").encode()
            self.flush()


def parse_args(argv=None):
    from teleop_config import load_config
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--expected-mode-machine", type=int)
    parser.add_argument("--interface", default=load_config()['G1_ARM_INTERFACE'])
    parser.add_argument("--duration", type=float)
    parser.add_argument("--rate", type=int, choices=(100, 250), default=100)
    parser.add_argument("--max-age", type=float, default=.3)
    parser.add_argument("--state-max-age", type=float, default=.1)
    parser.add_argument("--slew-rate", type=float, default=.15)
    parser.add_argument("--max-excursion", type=float, default=.15)
    parser.add_argument("--ramp-up", type=float, default=2.)
    parser.add_argument("--release-time", type=float, default=.5)
    parser.add_argument("--startup-timeout", type=float, default=10.)
    args = parser.parse_args(argv)
    if args.execute and args.expected_mode_machine != 6:
        parser.error("--execute requires explicit --expected-mode-machine 6")
    if args.expected_mode_machine is None:
        args.expected_mode_machine = 6
    if args.expected_mode_machine != 6:
        parser.error("only the verified mode_machine 6 is supported")
    if args.duration is None:
        args.duration = 30. if args.execute else 15.
    for key in ("duration", "max_age", "state_max_age", "slew_rate", "max_excursion", "ramp_up", "release_time", "startup_timeout"):
        if not finite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(key + " must be finite and positive")
    if args.duration > 120 or args.release_time > 1 or args.release_time >= args.duration:
        parser.error("duration <=120s, release-time <=1s and shorter than duration are required")
    if args.max_age > .3 or args.state_max_age > .1 or args.max_excursion > .15 or args.slew_rate > .15:
        parser.error("initial commissioning limits cannot be increased")
    return args


def release_transport(publisher, factory, crc, motion, mode, period, output, safety_snapshot=None):
    """Best-effort bounded release in finally, including after publish exceptions."""
    if publisher is None or motion is None:
        return 0
    now = time.monotonic()
    motion.release(now, motion.reason or "cleanup")
    deadline = now + motion.release_time
    writes = 0
    while time.monotonic() <= deadline and motion.phase != "released":
        feedback = safety_snapshot() if safety_snapshot is not None else None
        if feedback is None or feedback["immediate_release"] or not feedback.get("ready", False):
            motion.release(time.monotonic(), motion.reason or "state_changed_during_release", immediate=True)
        motion.step(time.monotonic())
        try:
            if publisher.Write(build_message(factory, crc, motion.target, motion.weight, mode, feedback)) is not False:
                writes += 1
        except Exception:
            motion.release(time.monotonic(), motion.reason or "release_write_error", immediate=True)
        time.sleep(period)
    motion.release(time.monotonic(), motion.reason or "cleanup", immediate=True)
    zero_writes = 0
    for _ in range(3):
        try:
            if publisher.Write(build_message(factory, crc, motion.target, 0., mode)) is not False:
                writes += 1
                zero_writes += 1
        except Exception:
            pass
        time.sleep(period)
    output.emit("status", status="released", ready=False, weight=0., reason=motion.reason,
                release_write_successes=writes, zero_weight_writes=zero_writes,
                note="weight is commanded; DDS write success does not acknowledge native handoff")
    return writes


def acquire_execute_lock(path="/tmp/g1-upper-body-bridge.execute.lock"):
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.ftruncate(descriptor, 0)
        os.write(descriptor, (str(os.getpid()) + "\n").encode())
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise RuntimeError("another instance owns the G1 arm execute lock")


def main(argv=None):
    args = parse_args(argv)
    execute_lock = acquire_execute_lock() if args.execute else None
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    ChannelFactoryInitialize(0, networkInterface=args.interface)
    output = JsonOutput()
    gate = StateGate(args.expected_mode_machine, args.state_max_age)
    lock = threading.Lock()
    mailbox = Mailbox(args.max_age)
    stop = threading.Event()
    requested_stop = threading.Event()
    subscriber = publisher = motion = reader = fsm_thread = None
    factory = crc = None
    published = 0
    result = 0
    started = time.monotonic()
    period = 1. / args.rate
    previous_handlers = {}

    def signal_stop(signum, frame):
        requested_stop.set()

    def state_callback(message):
        with lock:
            try:
                motors = message.motor_state
                if len(motors) != 35:
                    raise ValueError("expected 35 lowstate motors")
                packet = {"q_arm": [motors[i].q for i in ARM_INDICES],
                          "dq_arm": [motors[i].dq for i in ARM_INDICES],
                          "tau_arm": [motors[i].tau_est for i in ARM_INDICES],
                          "arm_mode": [motors[i].mode for i in ARM_INDICES],
                          "arm_status": [motors[i].motorstate for i in ARM_INDICES],
                          "q_waist": [motors[i].q for i in (12, 13, 14)],
                          "imu_rpy": list(message.imu_state.rpy), "tick": message.tick,
                          "mode_machine": message.mode_machine, "mode_pr": message.mode_pr}
                gate.update(packet, time.monotonic(), time.time())
            except Exception as exc:
                gate.fail("invalid_state: " + str(exc))

    def poll_fsm(client):
        while not stop.is_set():
            try:
                code, fsm_id = client.GetFsmId()  # Read-only API; no SetFsmId calls.
            except Exception:
                code, fsm_id = -1, None
            with lock:
                gate.update_fsm(code, fsm_id, time.monotonic())
            stop.wait(.1)

    def safety_snapshot():
        with lock:
            return gate.snapshot(time.monotonic())

    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous_handlers[sig] = signal.signal(sig, signal_stop)
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(state_callback)
        client = LocoClient()
        client.SetTimeout(.15)
        client.Init()
        fsm_thread = threading.Thread(target=poll_fsm, args=(client,), daemon=True)
        fsm_thread.start()
        if args.execute:
            reader = threading.Thread(target=read_commands, args=(sys.stdin, mailbox, stop), daemon=True)
            reader.start()
        output.emit("status", status="started", mode="execute" if args.execute else "read_only",
                    duration_s=args.duration, rate_hz=args.rate, expected_mode_machine=6, expected_fsm=500,
                    max_excursion_rad=args.max_excursion, slew_rad_s=args.slew_rate,
                    actual_excursion_limit_rad=.25, tracking_error_limit_rad=.2, velocity_limit_rad_s=.75,
                    estimated_pd_limits_nm=PD_LIMIT, measured_tau_limits_nm=MEASURED_TAU_LIMIT,
                    effort_note="software estimate/feedback gates; no physical torque guarantee",
                    body_drift_limit_rad=.05, nonarm_fields="all zero; body immobility is not guaranteed")
        next_report = started
        while True:
            now = time.monotonic()
            with lock:
                state = gate.snapshot(now)
            command, command_age, input_fault = mailbox.snapshot(now)
            duration_done = now >= started + args.duration - (args.release_time if args.execute else 0.)
            reason = None
            immediate = False
            if requested_stop.is_set():
                reason = "requested_stop"
            elif output.closed:
                reason = "stdout_disconnected"
            elif args.execute and input_fault:
                reason = input_fault
            elif motion is not None and state["fault"]:
                reason, immediate = state["fault"], state["immediate_release"]
            elif motion is not None and command is None:
                reason = "stale_command"
            elif duration_done:
                reason = "duration_complete"
            elif motion is None and args.execute and now - started > args.startup_timeout:
                reason = "startup_timeout"
            if reason:
                if reason not in ("duration_complete", "requested_stop"):
                    result = 2
                if motion is not None:
                    motion.release(now, reason, immediate)
                output.emit("status", status="release_requested" if motion else "stopped",
                            ready=False, reason=reason, weight=motion.weight if motion else 0.)
                break
            if args.execute and motion is None and state["ready"] and command is not None:
                if publisher is None:
                    from unitree_sdk2py.core.channel import ChannelPublisher
                    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
                    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
                    from unitree_sdk2py.utils.crc import CRC
                    factory, crc = unitree_hg_msg_dds__LowCmd_, CRC()
                    publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
                    publisher.Init()
                    continue  # Init waits; recheck both sources before any write.
                with lock:
                    state = gate.snapshot(time.monotonic())
                    if state["ready"]:
                        gate.engage()
                        motion = ArmMotion(state["q_arm"], time.monotonic(), period, args.slew_rate,
                                           args.max_excursion, args.ramp_up, args.release_time)
                if motion is not None:
                    output.emit("status", status="engaged_measured_hold", ready=True, weight=0.,
                                initial_q=motion.initial, q_waist=state["q_waist"], imu_rpy=state["imu_rpy"])
            if motion is not None:
                # Final checks before positive-weight writes; callback faults latch.
                with lock:
                    checked = gate.snapshot(time.monotonic())
                latest, _, fault = mailbox.snapshot(time.monotonic())
                if checked["fault"] or not checked["ready"] or latest is None or fault:
                    motion.release(time.monotonic(), checked["fault"] or fault or "stale_before_write",
                                   checked["immediate_release"])
                    result = 2
                    break
                motion.step(time.monotonic(), latest.q_arm)
                if publisher.Write(build_message(factory, crc, motion.target, motion.weight, args.expected_mode_machine, checked)) is False:
                    raise RuntimeError("arm_sdk DDS write failed")
                published += 1
                with lock:
                    gate.target = motion.target
            if now >= next_report:
                output.emit("state", **state, mode="execute" if args.execute else "read_only",
                            weight=motion.weight if motion else 0., phase=motion.phase if motion else "waiting",
                            published_count=published, command_age_s=command_age,
                            command_seq=command.seq if command else None,
                            q_command=motion.target if motion else None)
                next_report = now + .5
            output.flush()
            time.sleep(max(0., period - (time.monotonic() - now)))
    except BaseException as exc:
        result = 130 if isinstance(exc, KeyboardInterrupt) else 1
        if motion is not None:
            motion.release(time.monotonic(), "exception: " + str(exc))
        output.emit("error", error=str(exc))
    finally:
        # Release before stopping state/FSM reception or closing the publisher.
        try:
            published += release_transport(publisher, factory, crc, motion,
                                           args.expected_mode_machine, period, output, safety_snapshot)
        finally:
            stop.set()
            for thread in (reader, fsm_thread):
                if thread is not None:
                    thread.join(timeout=.3)
            for channel in (publisher, subscriber):
                if channel is not None:
                    try:
                        channel.Close()
                    except Exception:
                        pass
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
            output.emit("status", status="finished", ready=False, weight=0., published_count=published,
                        release_reason=motion.reason if motion else None, exit_code=result)
            output.flush()
            if execute_lock is not None:
                os.close(execute_lock)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
