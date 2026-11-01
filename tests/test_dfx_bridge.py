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

    def test_positions_clip_range_and_reject_malformed(self):
        self.assertEqual(bridge.validate_positions([-1] * 6 + [2] * 6), (0.0,) * 6 + (1.0,) * 6)
        for values in ([0.5] * 11, [math.nan] * 12, [math.inf] * 12, [True] * 12, ["0.5"] * 12, None):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    bridge.validate_positions(values)

    def test_mailbox_skips_bad_and_old_packets_and_holds_latest(self):
        box = bridge.CommandMailbox()
        self.assertTrue(box.ingest(self.packet(1, sent_at=0.0)))  # Clock skew is irrelevant.
        self.assertTrue(box.ingest(self.packet(2, q=[0.6] * 12)))
        self.assertFalse(box.ingest(self.packet(1)))
        self.assertFalse(box.ingest(self.packet(3, q=[math.nan] * 12)))
        self.assertFalse(box.ingest(None))
        command, eof = box.snapshot()
        self.assertEqual((command.seq, eof, box.rejected), (2, False, 2))
        box.mark_eof()
        self.assertEqual(box.snapshot()[0].seq, 2)
        self.assertTrue(box.snapshot()[1])

    def test_state_reports_loss_counters_without_blocking(self):
        gate = bridge.StateGate()
        self.assertFalse(gate.snapshot(0.0, 0.5)["ready"])
        self.assertTrue(gate.update([0.4] * 12, [2] * 12, 0.0))
        self.assertTrue(gate.update([0.4] * 12, [9] * 12, 0.1))
        self.assertTrue(gate.snapshot(0.2, 0.5)["ready"])
        self.assertEqual(gate.snapshot(0.2, 0.5)["lost"], [9] * 12)
        self.assertFalse(gate.snapshot(1.0, 0.5)["ready"])
        self.assertFalse(gate.update([math.nan] * 12, [9] * 12, 1.0))
        self.assertEqual(gate.invalid, 1)

    def test_slew_reaches_full_range(self):
        previous = (0.5,) * 12
        target = (1.0,) * 6 + (0.0,) * 6
        first = bridge.slew_step(previous, target, 0.05)
        self.assertAlmostEqual(first[0], 0.75)
        self.assertAlmostEqual(first[6], 0.25)
        for _ in range(3):
            previous = bridge.slew_step(previous, target, 0.05)
        self.assertEqual(previous, target)
        self.assertEqual(bridge.slew_step(target, target, 0), target)

    def test_cli_defaults_run_until_stopped(self):
        args = bridge.parse_args([])
        self.assertFalse(args.execute)
        self.assertEqual((args.duration, args.slew_rate, args.interface), (0.0, 5.0, "lo"))
        self.assertEqual(bridge.parse_args(["--execute", "--duration", "600"]).duration, 600)

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
                while box.snapshot()[0] is None and time.monotonic() < deadline:
                    time.sleep(0.005)
                self.assertEqual(box.snapshot()[0].seq, 4)
                stop.set()
                thread.join(timeout=0.2)
                self.assertFalse(thread.is_alive())
                self.assertTrue(box.snapshot()[1])
            finally:
                stop.set()
                os.close(write_fd)
                thread.join(timeout=0.2)

    def test_module_import_does_not_load_robot_transports(self):
        self.assertFalse(any(name.startswith(("unitree_sdk2py", "cyclonedds", "teleop.robot_control")) for name in sys.modules))


if __name__ == "__main__":
    unittest.main()
