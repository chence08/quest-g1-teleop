#!/usr/bin/env python3
"""Bounded DFX-only DDS shim for newline JSON over SSH stdin/stdout.

Default mode only subscribes to hand state. --execute enables a bounded hand
publisher. No arm/body controllers, force settings, or return-home commands.
Stopping publication does not disable the hand's onboard position controller.
"""

import argparse
from dataclasses import dataclass
import json
import math
import os
import re
import select
import sys
import threading
import time
from teleop_config import load_config


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_positions(values):
    if not isinstance(values, (list, tuple)) or len(values) != 12:
        raise ValueError("q must contain exactly 12 values (right six, then left six)")
    if not all(finite_number(value) and 0.0 <= value <= 1.0 for value in values):
        raise ValueError("all q values must be finite numbers in [0,1]")
    return tuple(float(value) for value in values)


def validate_lost(values):
    if not isinstance(values, (list, tuple)) or len(values) != 12:
        raise ValueError("lost must contain exactly 12 counters")
    if not all(isinstance(value, int) and not isinstance(value, bool) and value >= 0 for value in values):
        raise ValueError("lost counters must be nonnegative integers")
    return tuple(values)


@dataclass(frozen=True)
class Command:
    seq: int
    sent_at: float
    q: tuple
    received_monotonic: float
    age_at_receipt: float


def validate_command(packet, wall_now, monotonic_now, last_seq, max_age=0.3, future_tolerance=0.05):
    if not isinstance(packet, dict):
        raise ValueError("command must be a JSON object")
    seq = packet.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0 or seq <= last_seq:
        raise ValueError("seq must be a strictly increasing nonnegative integer")
    sent_at = packet.get("sent_at")
    if not finite_number(sent_at):
        raise ValueError("sent_at must be a finite Unix timestamp in seconds")
    age = wall_now - sent_at
    if age < -future_tolerance or age > max_age:
        raise ValueError("command timestamp is stale or too far in the future")
    return Command(seq, float(sent_at), validate_positions(packet.get("q")), monotonic_now, max(0.0, age))


def command_age(command, monotonic_now):
    return command.age_at_receipt + max(0.0, monotonic_now - command.received_monotonic)


class StateGate:
    """Qualify fresh, consecutive state frames without rising serial-loss counters."""

    def __init__(self, minimum_frames=10):
        self.minimum_frames = minimum_frames
        self.q = None
        self.lost = None
        self.received_at = None
        self.stable_frames = 0
        self.reason = "waiting_for_state"

    def invalidate(self, reason):
        self.stable_frames = 0
        self.reason = reason

    def update(self, q, lost, now):
        try:
            checked_q, checked_lost = validate_positions(q), validate_lost(lost)
        except ValueError as exc:
            self.invalidate("invalid_state: " + str(exc))
            return False
        increased = self.lost is not None and any(new > old for new, old in zip(checked_lost, self.lost))
        self.q, self.lost, self.received_at = checked_q, checked_lost, now
        if increased:
            self.invalidate("serial_loss_counter_increased")
            return False
        self.stable_frames += 1
        self.reason = "ready" if self.stable_frames >= self.minimum_frames else "qualifying_state"
        return self.stable_frames >= self.minimum_frames

    def snapshot(self, now, max_age):
        age = None if self.received_at is None else max(0.0, now - self.received_at)
        fresh = age is not None and age <= max_age
        if not fresh and self.stable_frames:
            # Fresh samples must qualify again following a stream interruption.
            self.invalidate("stale_state")
        return {"ready": fresh and self.stable_frames >= self.minimum_frames,
                "q": self.q, "lost": self.lost, "stable_frames": self.stable_frames,
                "state_age_s": age, "reason": self.reason}


class CommandMailbox:
    """Single-slot command mailbox; invalid input clears the previous target."""

    def __init__(self, max_age=0.3):
        self.max_age = max_age
        self.lock = threading.Lock()
        self.latest = None
        self.last_seq = -1
        self.eof = False
        self.reason = "waiting_for_command"

    def ingest(self, packet, wall_now, monotonic_now):
        with self.lock:
            try:
                command = validate_command(packet, wall_now, monotonic_now, self.last_seq, self.max_age)
            except ValueError as exc:
                self.latest = None
                self.reason = "invalid_command: " + str(exc)
                return False
            self.latest = command
            self.last_seq = command.seq
            self.reason = "command_received"
            return True

    def invalidate(self, reason):
        with self.lock:
            self.latest = None
            self.reason = reason

    def mark_eof(self):
        with self.lock:
            self.eof = True
            self.latest = None
            self.reason = "stdin_eof"

    def snapshot(self, now):
        with self.lock:
            age = None if self.latest is None else command_age(self.latest, now)
            command = self.latest if not self.eof and age is not None and age <= self.max_age else None
            reason = self.reason
            if age is not None and age > self.max_age:
                reason = "stale_command"
            return command, age, reason, self.eof


def bounded_step(initial, previous, requested, dt, slew_rate=0.15, max_excursion=0.15):
    initial, previous, requested = map(validate_positions, (initial, previous, requested))
    if not all(finite_number(value) and value >= 0 for value in (dt, slew_rate, max_excursion)):
        raise ValueError("rate, excursion, and timestep must be finite and nonnegative")
    result = []
    maximum_step = slew_rate * dt
    for origin, old, target in zip(initial, previous, requested):
        low, high = max(0.0, origin - max_excursion), min(1.0, origin + max_excursion)
        if not low - 1e-9 <= old <= high + 1e-9:
            raise ValueError("previous command is outside the initial excursion envelope")
        target = min(high, max(low, target))
        result.append(min(high, max(low, old + min(maximum_step, max(-maximum_step, target - old)))))
    return tuple(result)


def read_commands(stream, mailbox, stop_event):
    """Read bounded newline records without holding stdin's buffered lock at exit."""
    pending = b""
    try:
        descriptor = stream.fileno()
        while not stop_event.is_set():
            if not select.select([descriptor], [], [], 0.05)[0]:
                continue
            data = os.read(descriptor, 16384)
            if not data:
                break
            pending += data
            if len(pending) > 65536:
                mailbox.invalidate("input_record_too_large")
                break
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                try:
                    packet = json.loads(line)
                except (ValueError, TypeError, UnicodeError) as exc:
                    mailbox.invalidate("invalid_json: " + str(exc))
                    continue
                mailbox.ingest(packet, time.time(), time.monotonic())
    except Exception as exc:
        mailbox.invalidate("stdin_error: " + str(exc))
    finally:
        mailbox.mark_eof()


def emit(kind, **fields):
    print(json.dumps({"type": kind, "time": time.time(), **fields}, allow_nan=False), flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default=load_config()['G1_HAND_INTERFACE'])
    parser.add_argument("--namespace", default=load_config()['G1_DDS_NAMESPACE'])
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float, help="default 15 seconds read-only, 30 execute; maximum 120")
    parser.add_argument("--slew-rate", type=float, default=0.15, help="maximum normalized target change per second")
    parser.add_argument("--max-excursion", type=float, default=0.15, help="per-joint range on either side of initial measured position")
    parser.add_argument("--max-age", type=float, default=0.3, help="maximum command age in seconds")
    parser.add_argument("--state-max-age", type=float, default=0.3)
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.duration is None:
        args.duration = 30.0 if args.execute else 15.0
    if not re.fullmatch(r"[A-Za-z0-9_]+", args.namespace):
        parser.error("--namespace must contain only letters, digits and underscores")
    for name in ("duration", "slew_rate", "max_excursion", "max_age", "state_max_age", "startup_timeout"):
        if not finite_number(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.duration > 120:
        parser.error("--duration cannot exceed 120 seconds")
    if args.max_excursion > 1:
        parser.error("--max-excursion cannot exceed 1 normalized unit")
    return args


def main(argv=None):
    args = parse_args(argv)
    # Keep all DDS imports here: importing this module for unit tests is offline.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_

    ChannelFactoryInitialize(0, networkInterface=args.interface)
    gate = StateGate(minimum_frames=10)
    state_lock = threading.Lock()
    mailbox = CommandMailbox(args.max_age)
    reader_stop = threading.Event()
    reader = None
    publisher = None
    subscriber = None
    initial = previous = None
    last_published_seq = None
    published_count = 0
    exit_code = 0
    started = time.monotonic()

    def on_state(message):
        with state_lock:
            try:
                states = message.states
                if len(states) != 12:
                    raise ValueError("state message must contain exactly 12 motors")
                gate.update([motor.q for motor in states], [motor.lost for motor in states], time.monotonic())
            except Exception as exc:
                gate.invalidate("invalid_state: " + str(exc))

    try:
        subscriber = ChannelSubscriber(f"rt/{args.namespace}/state", MotorStates_)
        subscriber.Init(on_state)
        if args.execute:
            reader = threading.Thread(target=read_commands, args=(sys.stdin, mailbox, reader_stop), daemon=True)
            reader.start()
        emit("status", status="started", mode="execute" if args.execute else "read_only",
             interface=args.interface, namespace=args.namespace, duration_s=args.duration,
             command_max_age_s=args.max_age, state_max_age_s=args.state_max_age,
             slew_rate=args.slew_rate, max_excursion=args.max_excursion,
             stop_behavior="stop publishing; no open, home, force change or actuator disable")
        next_report, previous_tick = started, time.monotonic()
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            # Never accumulate a catch-up motion after a stall or pause.
            dt = min(0.05, max(0.0, now - previous_tick))
            previous_tick = now
            with state_lock:
                state = gate.snapshot(now, args.state_max_age)
            command, age, command_reason, eof = mailbox.snapshot(now)
            status = "read_only" if not args.execute else command_reason
            if state["ready"] and initial is None:
                initial = previous = state["q"]
                emit("status", status="state_qualified", initial_q=initial, stable_frames=state["stable_frames"])
            if initial is None and now - started >= min(args.startup_timeout, args.duration):
                emit("status", status="startup_timeout", reason=state["reason"])
                exit_code = 2
                break
            if args.execute and state["ready"] and command is not None:
                if publisher is None:
                    from unitree_sdk2py.core.channel import ChannelPublisher
                    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
                    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
                    publisher = ChannelPublisher(f"rt/{args.namespace}/cmd", MotorCmds_)
                    publisher.Init()
                    # Init may block briefly; recheck both sources before writing.
                    continue
                target = bounded_step(initial, previous, command.q, dt, args.slew_rate, args.max_excursion)
                msg = MotorCmds_()
                msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(12)]
                for motor, value in zip(msg.cmds, target):
                    motor.q = value
                # Recheck immediately before a write, including stdin EOF.
                latest, _, _, _ = mailbox.snapshot(time.monotonic())
                with state_lock:
                    state_still_ready = gate.snapshot(time.monotonic(), args.state_max_age)["ready"]
                if latest is not None and latest.seq == command.seq and state_still_ready:
                    if publisher.Write(msg) is False:
                        raise RuntimeError("DDS hand command write failed")
                    previous = target
                    last_published_seq = command.seq
                    published_count += 1
                    status = "publishing"
            elif args.execute and not state["ready"]:
                status = "paused_" + state["reason"]
            if now >= next_report:
                emit("state", mode="execute" if args.execute else "read_only", status=status,
                     **state, command_age_s=age, command_seq=None if command is None else command.seq,
                     last_published_seq=last_published_seq, last_command_q=previous if published_count else None,
                     published_count=published_count, stdin_eof=eof)
                next_report = now + 0.5
            if args.execute and eof:
                emit("status", status="stdin_eof_no_more_commands", published_count=published_count)
                break
            time.sleep(max(0.0, 0.05 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        emit("status", status="interrupted_no_more_commands", published_count=published_count)
    finally:
        reader_stop.set()
        if reader is not None:
            reader.join(timeout=0.2)
        # Closing transports sends no final posture command.
        if publisher is not None:
            publisher.Close()
        if subscriber is not None:
            subscriber.Close()
        emit("status", status="finished", published_count=published_count)
    return exit_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        emit("error", error=str(exc))
        raise SystemExit(1)
