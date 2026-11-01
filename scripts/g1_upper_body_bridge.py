#!/usr/bin/env python3
"""G1_29 arm_sdk bridge over newline JSON; read-only unless --execute.

Follows the upstream xr_teleoperate motion-mode arm controller: arm motors
15..28 track the latest received target through a joint velocity limit that
ramps up after engagement, and the overlay weight in slot 29 ramps in at start
and out on exit. A late, missing or malformed command never stops the bridge;
it holds the last good target. Only stdin EOF, a signal or --duration end it.
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
# Upstream G1_29_ArmController gains: kp_low/kd_low for shoulder+elbow, kp_wrist/kd_wrist for wrists.
KP = tuple(40. if i % 7 >= 4 else 80. for i in range(14))
KD = tuple(1.5 if i % 7 >= 4 else 3. for i in range(14))
TAU_LIMIT = 25.  # Clip only garbage feedforward; model gravity torque is far below this.


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def vector(values, size, name):
    if not isinstance(values, (tuple, list)) or len(values) != size or not all(finite(x) for x in values):
        raise ValueError(f"{name} must contain exactly {size} finite numbers")
    return tuple(float(x) for x in values)


def arm_positions(values):
    """Clip, never reject, a finite 14-vector to the URDF joint limits."""
    q = vector(values, 14, "q_arm")
    return tuple(min(high, max(low, value)) for value, low, high in zip(q, LOWER, UPPER))


@dataclass(frozen=True)
class Command:
    seq: int
    q_arm: tuple
    tau_arm: tuple


def parse_command(packet):
    if not isinstance(packet, dict):
        raise ValueError("command must be an object")
    seq = packet.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool):
        raise ValueError("seq must be an integer")
    tau = vector(packet.get("tau_arm", [0.] * 14), 14, "tau_arm")
    tau = tuple(min(TAU_LIMIT, max(-TAU_LIMIT, value)) for value in tau)
    return Command(seq, arm_positions(packet.get("q_arm")), tau)


class Mailbox:
    """Latest good command wins; bad or out-of-order packets are counted and skipped."""
    def __init__(self):
        self.lock = threading.Lock()
        self.command = None
        self.last_seq = -1
        self.rejected = 0
        self.last_error = None
        self.eof = False

    def ingest(self, packet):
        try:
            command = parse_command(packet)
        except ValueError as exc:
            with self.lock:
                self.rejected += 1
                self.last_error = str(exc)
            return False
        with self.lock:
            if command.seq <= self.last_seq:
                return False
            self.command, self.last_seq = command, command.seq
        return True

    def mark_eof(self):
        with self.lock:
            self.eof = True

    def snapshot(self):
        with self.lock:
            return self.command, self.eof


def read_commands(stream, mailbox, stop):
    pending = b""
    try:
        fd = stream.fileno()
        while not stop.is_set():
            if not select.select([fd], [], [], .05)[0]:
                continue
            chunk = os.read(fd, 16384)
            if not chunk:
                return
            pending += chunk
            if len(pending) > 65536:
                pending = pending[pending.rfind(b"\n") + 1:] if b"\n" in pending else b""
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                try:
                    packet = json.loads(line)
                except (ValueError, UnicodeError):
                    mailbox.ingest(None)
                    continue
                mailbox.ingest(packet)
    except Exception:
        pass
    finally:
        mailbox.mark_eof()


class StateGate:
    """Latest lowstate for telemetry and engagement; duplicate ticks are skipped."""
    def __init__(self):
        self.state = None
        self.received_at = None
        self.duplicates = 0
        self.invalid = 0
        self.fsm_id = None

    def update_fsm(self, code, fsm_id):
        self.fsm_id = fsm_id if code == 0 and isinstance(fsm_id, int) and not isinstance(fsm_id, bool) else None

    def update(self, packet, now, wall_now):
        try:
            q = vector(packet["q_arm"], 14, "q_arm")
            dq = vector(packet["dq_arm"], 14, "dq_arm")
            tau = vector(packet.get("tau_arm", [0.] * 14), 14, "tau_arm")
            waist = vector(packet["q_waist"], 3, "q_waist")
            imu = vector(packet["imu_rpy"], 3, "imu_rpy")
            tick = packet["tick"]
        except (ValueError, KeyError, TypeError):
            self.invalid += 1
            return False
        if self.state is not None and tick == self.state["tick"]:
            self.duplicates += 1
            return False
        self.received_at = now
        self.state = {"q_arm": q, "dq_arm": dq, "tau_arm": tau, "q_waist": waist, "imu_rpy": imu,
                      "mode_machine": packet.get("mode_machine"), "mode_pr": packet.get("mode_pr"),
                      "tick": tick, "timestamp": wall_now}
        return True

    def snapshot(self, now, max_age=.5):
        age = None if self.received_at is None else max(0., now - self.received_at)
        result = dict(self.state or {"q_arm": None, "dq_arm": None, "tau_arm": None, "q_waist": None,
                                     "imu_rpy": None, "mode_machine": None, "mode_pr": None,
                                     "tick": None, "timestamp": None})
        ready = age is not None and age <= max_age
        result.update(ready=ready, fsm_id=self.fsm_id, state_age_s=age, duplicate_ticks=self.duplicates,
                      invalid_states=self.invalid, reason="ready" if ready else
                      ("waiting_for_state" if age is None else "stale_lowstate"))
        return result


def velocity_step(previous, requested, max_step):
    """Upstream clip_arm_q_target: scale the whole move so no joint exceeds max_step."""
    delta = [target - old for target, old in zip(requested, previous)]
    scale = max(1., max(abs(value) for value in delta) / max_step) if max_step > 0 else math.inf
    return tuple(old + value / scale for old, value in zip(previous, delta))


class ArmMotion:
    def __init__(self, q, now, ramp_up=1., speed_ramp=3., min_velocity=1., max_velocity=10., release_time=1.):
        self.initial = self.target = arm_positions(q)
        self.started = self.last_step = now
        self.ramp_up, self.speed_ramp, self.release_time = ramp_up, speed_ramp, release_time
        self.min_velocity, self.max_velocity = min_velocity, max_velocity
        self.weight = 0.
        self.phase = "ramp_up_hold"
        self.release_at = None
        self.release_weight = 0.
        self.reason = None

    def velocity_limit(self, now):
        tracking = now - self.started - self.ramp_up
        fraction = min(1., max(0., tracking / self.speed_ramp)) if self.speed_ramp > 0 else 1.
        return self.min_velocity + (self.max_velocity - self.min_velocity) * fraction

    def release(self, now, reason, immediate=False):
        self.reason = self.reason or reason
        if self.phase == "released":
            return
        if self.release_at is None:
            self.release_at, self.release_weight = now, self.weight
        self.phase = "releasing"
        if immediate:
            self.weight, self.phase = 0., "released"

    def step(self, now, requested=None):
        dt = min(.02, max(0., now - self.last_step))
        self.last_step = now
        if self.phase == "released":
            return
        if self.release_at is not None:
            elapsed = max(0., now - self.release_at)
            self.weight = max(0., self.release_weight * (1. - elapsed / self.release_time)) if self.release_time > 0 else 0.
            if self.weight <= 0:
                self.phase = "released"
            return
        if now - self.started < self.ramp_up:
            self.weight = min(1., max(0., (now - self.started) / self.ramp_up))
            return  # Hold the measured startup posture while the overlay weight ramps in.
        self.weight, self.phase = 1., "tracking"
        if requested is not None:
            self.target = velocity_step(self.target, arm_positions(requested), self.velocity_limit(now) * dt)


def build_message(factory, crc, q, weight, mode_machine=0, tau=None):
    q = arm_positions(q)
    tau = (0.,) * 14 if tau is None else tau
    message = factory()  # fresh all-zero legs, waist and unused slots each tick
    message.mode_pr, message.mode_machine = 0, int(mode_machine or 0)
    for index, value, effort, kp, kd in zip(ARM_INDICES, q, tau, KP, KD):
        motor = message.motor_cmd[index]
        motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd = 1, value, 0., float(effort), kp, kd
    message.motor_cmd[WEIGHT_INDEX].q = min(1., max(0., float(weight)))
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
    parser.add_argument("--interface", default=load_config()['G1_ARM_INTERFACE'])
    parser.add_argument("--duration", type=float, default=0., help="seconds; 0 runs until stdin EOF or a signal")
    parser.add_argument("--rate", type=int, choices=(100, 250), default=250)
    parser.add_argument("--state-max-age", type=float, default=.5)
    parser.add_argument("--min-velocity", type=float, default=1., help="rad/s velocity limit right after engagement")
    parser.add_argument("--max-velocity", type=float, default=10., help="rad/s velocity limit after --speed-ramp")
    parser.add_argument("--speed-ramp", type=float, default=3.)
    parser.add_argument("--ramp-up", type=float, default=1., help="overlay weight ramp-in seconds")
    parser.add_argument("--release-time", type=float, default=1., help="overlay weight ramp-out seconds")
    args = parser.parse_args(argv)
    for key in ("duration", "state_max_age", "min_velocity", "max_velocity", "speed_ramp", "ramp_up", "release_time"):
        if not finite(getattr(args, key)) or getattr(args, key) < 0:
            parser.error(key + " must be finite and nonnegative")
    return args


def release_transport(publisher, factory, crc, motion, mode_machine, period, output):
    """Ramp the overlay weight out at the last target, then send zero weight."""
    if publisher is None or motion is None:
        return 0
    motion.release(time.monotonic(), motion.reason or "cleanup")
    deadline = time.monotonic() + motion.release_time + .5
    writes = 0
    while motion.phase != "released" and time.monotonic() <= deadline:
        motion.step(time.monotonic())
        try:
            if publisher.Write(build_message(factory, crc, motion.target, motion.weight, mode_machine())) is not False:
                writes += 1
        except Exception:
            pass
        time.sleep(period)
    motion.release(time.monotonic(), motion.reason or "cleanup", immediate=True)
    for _ in range(3):
        try:
            if publisher.Write(build_message(factory, crc, motion.target, 0., mode_machine())) is not False:
                writes += 1
        except Exception:
            pass
        time.sleep(period)
    output.emit("status", status="released", ready=False, weight=0., reason=motion.reason,
                release_writes=writes)
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
    ChannelFactoryInitialize(0, networkInterface=args.interface)
    output = JsonOutput()
    gate = StateGate()
    lock = threading.Lock()
    mailbox = Mailbox()
    stop = threading.Event()
    requested_stop = threading.Event()
    subscriber = publisher = motion = reader = fsm_thread = None
    factory = crc = None
    published = write_failures = 0
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
                gate.update({"q_arm": [motors[i].q for i in ARM_INDICES],
                             "dq_arm": [motors[i].dq for i in ARM_INDICES],
                             "tau_arm": [motors[i].tau_est for i in ARM_INDICES],
                             "q_waist": [motors[i].q for i in (12, 13, 14)],
                             "imu_rpy": list(message.imu_state.rpy), "tick": message.tick,
                             "mode_machine": message.mode_machine, "mode_pr": message.mode_pr},
                            time.monotonic(), time.time())
            except Exception:
                gate.invalid += 1

    def poll_fsm():
        """Telemetry only; the loco service may be absent in debug mode."""
        try:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            client = LocoClient()
            client.SetTimeout(.15)
            client.Init()
        except Exception:
            return
        while not stop.is_set():
            try:
                code, fsm_id = client.GetFsmId()
            except Exception:
                code, fsm_id = -1, None
            with lock:
                gate.update_fsm(code, fsm_id)
            stop.wait(.5)

    def mode_machine():
        with lock:
            return (gate.state or {}).get("mode_machine") or 0

    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous_handlers[sig] = signal.signal(sig, signal_stop)
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(state_callback)
        fsm_thread = threading.Thread(target=poll_fsm, daemon=True)
        fsm_thread.start()
        if args.execute:
            reader = threading.Thread(target=read_commands, args=(sys.stdin, mailbox, stop), daemon=True)
            reader.start()
        output.emit("status", status="started", mode="execute" if args.execute else "read_only",
                    duration_s=args.duration, rate_hz=args.rate, min_velocity_rad_s=args.min_velocity,
                    max_velocity_rad_s=args.max_velocity, speed_ramp_s=args.speed_ramp, kp=KP, kd=KD)
        next_report = started
        while True:
            now = time.monotonic()
            with lock:
                state = gate.snapshot(now, args.state_max_age)
            command, eof = mailbox.snapshot()
            reason = None
            if requested_stop.is_set():
                reason = "requested_stop"
            elif args.execute and eof:
                reason = "stdin_eof"
            elif output.closed:
                reason = "stdout_disconnected"
            elif args.duration and now - started >= args.duration:
                reason = "duration_complete"
            if reason:
                if motion is not None:
                    motion.release(now, reason)
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
                motion = ArmMotion(state["q_arm"], time.monotonic(), args.ramp_up, args.speed_ramp,
                                   args.min_velocity, args.max_velocity, args.release_time)
                output.emit("status", status="engaged", ready=True, weight=0., initial_q=motion.initial)
            if motion is not None:
                motion.step(time.monotonic(), command.q_arm if command else None)
                tau = command.tau_arm if command and motion.phase == "tracking" else None
                try:
                    written = publisher.Write(build_message(factory, crc, motion.target, motion.weight,
                                                            state["mode_machine"], tau))
                except Exception:
                    written = False
                if written is False:
                    write_failures += 1
                else:
                    published += 1
            if now >= next_report:
                output.emit("state", **state, mode="execute" if args.execute else "read_only",
                            weight=motion.weight if motion else 0., phase=motion.phase if motion else "waiting",
                            published_count=published, write_failures=write_failures,
                            command_seq=command.seq if command else None,
                            rejected_commands=mailbox.rejected, last_command_error=mailbox.last_error,
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
        try:
            published += release_transport(publisher, factory, crc, motion, mode_machine, period, output)
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
