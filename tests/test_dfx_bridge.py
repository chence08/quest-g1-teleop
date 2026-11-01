"""Pure tests for DFX stream validation and limits. No SDK imports or hardware."""

import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "dfx_robot_bridge.py"
sys.path.insert(0, str(SCRIPT.parent))
spec = importlib.util.spec_from_file_location("dfx_robot_bridge", SCRIPT)
bridge = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = bridge
spec.loader.exec_module(bridge)


class DfxValidationTests(unittest.TestCase):
    def packet(self, seq=1, sent_at=100.0, q=None):
        return {"seq": seq, "sent_at": sent_at, "q": [0.5] * 12 if q is None else q}

    def test_positions_reject_nonfinite_wrong_size_and_sentinels(self):
        for values in ([0.5] * 11, [-1] * 12, [1.1] * 12, [math.nan] * 12,
                       [math.inf] * 12, [True] * 12, ["0.5"] * 12, None):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    bridge.validate_positions(values)

    def test_commands_require_fresh_epoch_and_increasing_sequence(self):
        valid = bridge.validate_command(self.packet(), 100.1, 8.0, 0)
        self.assertEqual(valid.seq, 1)
        self.assertAlmostEqual(bridge.command_age(valid, 8.1), 0.2)
        for packet in (self.packet(seq=0), self.packet(seq=True), self.packet(sent_at=99.0),
                       self.packet(sent_at=101.0), self.packet(sent_at=math.nan)):
            with self.subTest(packet=packet):
                with self.assertRaises(ValueError):
                    bridge.validate_command(packet, 100.1, 8.0, 0)

    def test_mailbox_latest_only_stale_invalid_and_eof(self):
        box = bridge.CommandMailbox()
        self.assertTrue(box.ingest(self.packet(1), 100.0, 10.0))
        self.assertTrue(box.ingest(self.packet(2, q=[0.6] * 12), 100.0, 10.0))
        self.assertEqual(box.snapshot(10.1)[0].seq, 2)
        self.assertIsNone(box.snapshot(10.31)[0])
        self.assertFalse(box.ingest(self.packet(1), 100.0, 10.4))
        self.assertIsNone(box.snapshot(10.4)[0])
        self.assertTrue(box.ingest(self.packet(3), 100.0, 10.5))
        box.mark_eof()
        self.assertIsNone(box.snapshot(10.5)[0])
        self.assertTrue(box.snapshot(10.5)[3])

    def test_state_requires_ten_consecutive_valid_frames(self):
        gate = bridge.StateGate()
        for index in range(9):
            self.assertFalse(gate.update([0.4] * 12, [0] * 12, index * 0.05))
        self.assertTrue(gate.update([0.4] * 12, [0] * 12, 0.45))
        self.assertTrue(gate.snapshot(0.5, 0.3)["ready"])
        self.assertFalse(gate.snapshot(0.9, 0.3)["ready"])
        self.assertFalse(gate.update([0.4] * 12, [0] * 12, 0.91))
        self.assertEqual(gate.stable_frames, 1)

    def test_loss_counter_or_invalid_state_requalifies(self):
        gate = bridge.StateGate()
        for index in range(10):
            gate.update([0.4] * 12, [2] * 12, index * 0.05)
        self.assertFalse(gate.update([0.4] * 12, [3] * 12, 0.5))
        self.assertEqual(gate.stable_frames, 0)
        for index in range(10):
            gate.update([0.4] * 12, [3] * 12, 0.55 + index * 0.05)
        self.assertTrue(gate.snapshot(1.0, 0.3)["ready"])
        self.assertFalse(gate.update([math.nan] * 12, [3] * 12, 1.01))
        self.assertFalse(gate.snapshot(1.01, 0.3)["ready"])

    def test_step_limits_initial_excursion_and_slew(self):
        initial = previous = (0.5,) * 12
        target = (1.0,) * 6 + (0.0,) * 6
        first = bridge.bounded_step(initial, previous, target, 0.05)
        self.assertAlmostEqual(first[0], 0.5075)
        self.assertAlmostEqual(first[6], 0.4925)
        for _ in range(100):
            updated = bridge.bounded_step(initial, previous, target, 0.05)
            self.assertLessEqual(max(abs(a - b) for a, b in zip(updated, previous)), 0.0075 + 1e-12)
            previous = updated
        self.assertAlmostEqual(previous[0], 0.65)
        self.assertAlmostEqual(previous[6], 0.35)
        self.assertEqual(bridge.bounded_step(initial, initial, target, 0), initial)

    def test_cli_defaults_are_read_only_bounded(self):
        args = bridge.parse_args([])
        self.assertFalse(args.execute)
        self.assertEqual(args.duration, 15)
        self.assertEqual(args.interface, "lo")
        self.assertEqual(bridge.parse_args(["--execute"]).duration, 30)

    def test_reader_can_stop_with_open_stdin(self):
        read_fd, write_fd = os.pipe()
        box = bridge.CommandMailbox()
        stop = threading.Event()
        with os.fdopen(read_fd, "rb", buffering=0) as stream:
            thread = threading.Thread(target=bridge.read_commands, args=(stream, box, stop), daemon=True)
            thread.start()
            try:
                os.write(write_fd, (json.dumps(self.packet(4, time.time())) + "\n").encode())
                deadline = time.monotonic() + 1
                while box.snapshot(time.monotonic())[0] is None and time.monotonic() < deadline:
                    time.sleep(0.005)
                self.assertEqual(box.snapshot(time.monotonic())[0].seq, 4)
                stop.set()
                thread.join(timeout=0.2)
                self.assertFalse(thread.is_alive())
                self.assertTrue(box.snapshot(time.monotonic())[3])
            finally:
                stop.set()
                os.close(write_fd)
                thread.join(timeout=0.2)

    def test_module_import_does_not_load_robot_transports(self):
        self.assertFalse(any(name.startswith(("unitree_sdk2py", "cyclonedds", "teleop.robot_control")) for name in sys.modules))


if __name__ == "__main__":
    unittest.main()
