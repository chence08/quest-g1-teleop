#!/usr/bin/env python3
"""DFX-only DDS shim for newline JSON over SSH stdin/stdout.

Default mode only subscribes to hand state. --execute republishes the latest
received finger target at 20 Hz through a slew limit, like the upstream
xr_teleoperate Inspire DFX controller. Late or malformed commands are skipped
and the last good target is held; only stdin EOF, a signal or --duration end it.
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
    """Clip a finite 12-vector (right six, then left six) to normalized [0,1]."""
    if not isinstance(values, (list, tuple)) or len(values) != 12:
        raise ValueError("q must contain exactly 12 values (right six, then left six)")
    if not all(finite_number(value) for value in values):
        raise ValueError("all q values must be finite numbers")
    return tuple(min(1.0, max(0.0, float(value))) for value in values)


@dataclass(frozen=True)
class Command:
    seq: int
    q: tuple


def validate_command(packet):
    if not isinstance(packet, dict):
        raise ValueError("command must be a JSON object")
    seq = packet.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool):
        raise ValueError("seq must be an integer")
    return Command(seq, validate_positions(packet.get("q")))


class StateGate:
    """Latest hand state for telemetry; serial-loss counters are reported, not enforced."""

    def __init__(self):
        self.q = None
        self.lost = None
        self.received_at = None
        self.invalid = 0

    def update(self, q, lost, now):
        try:
            self.q = validate_positions(q)
        except ValueError:
            self.invalid += 1
            return False
        self.lost = list(lost) if isinstance(lost, (list, tuple)) else None
        self.received_at = now
        return True

    def snapshot(self, now, max_age):
        age = None if self.received_at is None else max(0.0, now - self.received_at)
        ready = age is not None and age <= max_age
        return {"ready": ready, "q": self.q, "lost": self.lost, "state_age_s": age,
                "invalid_states": self.invalid,
                "reason": "ready" if ready else ("waiting_for_state" if age is None else "stale_state")}


class CommandMailbox:
    """Latest good command wins; bad or out-of-order packets are counted and skipped."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.last_seq = -1
        self.rejected = 0
        self.last_error = None
        self.eof = False

    def ingest(self, packet):
        try:
            command = validate_command(packet)
        except ValueError as exc:
            with self.lock:
                self.rejected += 1
                self.last_error = str(exc)
            return False
        with self.lock:
            if command.seq <= self.last_seq:
                return False
            self.latest, self.last_seq = command, command.seq
        return True

    def mark_eof(self):
        with self.lock:
            self.eof = True

    def snapshot(self):
        with self.lock:
            return self.latest, self.eof


def slew_step(previous, requested, dt, slew_rate=5.0):
    maximum_step = slew_rate * dt
    return tuple(min(1.0, max(0.0, old + min(maximum_step, max(-maximum_step, target - old))))
                 for old, target in zip(validate_positions(previous), validate_positions(requested)))


def read_commands(stream, mailbox, stop_event):
    """Read newline records without holding stdin's buffered lock at exit."""
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
                pending = pending[pending.rfind(b"\n") + 1:] if b"\n" in pending else b""
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                try:
                    packet = json.loads(line)
                except (ValueError, TypeError, UnicodeError):
                    mailbox.ingest(None)
                    continue
                mailbox.ingest(packet)
    except Exception:
        pass
    finally:
        mailbox.mark_eof()


def emit(kind, **fields):
    print(json.dumps({"type": kind, "time": time.time(), **fields}, allow_nan=False), flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default=load_config()['G1_HAND_INTERFACE'])
    parser.add_argument("--namespace", default=load_config()['G1_DDS_NAMESPACE'])
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stdin EOF or a signal")
    parser.add_argument("--slew-rate", type=float, default=5.0, help="maximum normalized target change per second")
    parser.add_argument("--state-max-age", type=float, default=0.5)
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_]+", args.namespace):
        parser.error("--namespace must contain only letters, digits and underscores")
    for name in ("duration", "slew_rate", "state_max_age"):
        if not finite_number(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and nonnegative")
    return args


def main(argv=None):
    args = parse_args(argv)
    # Keep all DDS imports here: importing this module for unit tests is offline.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_

    ChannelFactoryInitialize(0, networkInterface=args.interface)
    gate = StateGate()
    state_lock = threading.Lock()
    mailbox = CommandMailbox()
    reader_stop = threading.Event()
    reader = None
    publisher = None
    subscriber = None
    previous = None
    published_count = write_failures = 0
    started = time.monotonic()

    def on_state(message):
        with state_lock:
            try:
                states = message.states
                gate.update([motor.q for motor in states], [motor.lost for motor in states], time.monotonic())
            except Exception:
                gate.invalid += 1

    try:
        subscriber = ChannelSubscriber(f"rt/{args.namespace}/state", MotorStates_)
        subscriber.Init(on_state)
        if args.execute:
            reader = threading.Thread(target=read_commands, args=(sys.stdin, mailbox, reader_stop), daemon=True)
            reader.start()
        emit("status", status="started", mode="execute" if args.execute else "read_only",
             interface=args.interface, namespace=args.namespace, duration_s=args.duration,
             slew_rate=args.slew_rate, ready=True)
        next_report, previous_tick = started, time.monotonic()
        while not args.duration or time.monotonic() - started < args.duration:
            now = time.monotonic()
            # Never accumulate a catch-up motion after a stall or pause.
            dt = min(0.05, max(0.0, now - previous_tick))
            previous_tick = now
            with state_lock:
                state = gate.snapshot(now, args.state_max_age)
            command, eof = mailbox.snapshot()
            if args.execute and eof:
                emit("status", status="stdin_eof", published_count=published_count)
                break
            if args.execute and command is not None:
                if publisher is None:
                    from unitree_sdk2py.core.channel import ChannelPublisher
                    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
                    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
                    publisher = ChannelPublisher(f"rt/{args.namespace}/cmd", MotorCmds_)
                    publisher.Init()
                if previous is None:
                    previous = state["q"] if state["q"] is not None else command.q
                previous = slew_step(previous, command.q, dt, args.slew_rate)
                msg = MotorCmds_()
                msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(12)]
                for motor, value in zip(msg.cmds, previous):
                    motor.q = value
                try:
                    written = publisher.Write(msg)
                except Exception:
                    written = False
                if written is False:
                    write_failures += 1
                else:
                    published_count += 1
            if now >= next_report:
                # ready=True keeps the workstation sending even while hand feedback is late.
                emit("state", mode="execute" if args.execute else "read_only", **{**state, "ready": True},
                     state_ready=state["ready"], command_seq=None if command is None else command.seq,
                     last_command_q=previous, published_count=published_count, write_failures=write_failures,
                     rejected_commands=mailbox.rejected, last_command_error=mailbox.last_error)
                next_report = now + 0.5
            time.sleep(max(0.0, 0.05 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        emit("status", status="interrupted", published_count=published_count)
    finally:
        reader_stop.set()
        if reader is not None:
            reader.join(timeout=0.2)
        # Closing transports sends no final posture command; the hand holds its last target.
        for channel in (publisher, subscriber):
            if channel is not None:
                try:
                    channel.Close()
                except Exception:
                    pass
        emit("status", status="finished", published_count=published_count)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        emit("error", error=str(exc))
        raise SystemExit(1)
