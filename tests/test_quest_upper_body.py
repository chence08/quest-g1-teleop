import argparse
import io
import json
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from quest_upper_body import (RelativeCalibration, LineControls, state_usable, hands_fresh, hands_usable,
                             parse_args, ArmBridge, finite_vector, tracking_diagnostics, Throttle)


def pose(position):
    value = np.eye(4)
    value[:3, 3] = position
    return value


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.head = pose([1., 2., 1.6])
        self.wrists = [pose([1.4, 2.2, 1.2]), pose([1.4, 1.8, 1.2])]
        self.robot = [pose([.2, .2, .3]), pose([.2, -.2, .3])]
        self.calibration = RelativeCalibration(self.head, self.wrists, self.robot, np.zeros(14))

    def test_neutral_is_exact_robot_pose(self):
        for actual, expected in zip(self.calibration.targets(self.wrists), self.robot):
            np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_translation_is_one_to_one_and_large_moves_pass(self):
        wrists = [p.copy() for p in self.wrists]
        wrists[0][:3, 3] += [.3, .2, .1]
        target = self.calibration.targets(wrists)[0]
        np.testing.assert_allclose(target[:3, 3] - self.robot[0][:3, 3], [.3, .2, .1], atol=1e-12)
        wrists[0][:3, 3] += [5., 0., 0.]
        self.assertAlmostEqual(np.linalg.norm(self.calibration.targets(wrists)[0][:3, 3] - self.robot[0][:3, 3]), 1.)

    def test_invalid_transform_rejected_but_close_wrists_allowed(self):
        bad = self.wrists[0].copy(); bad[0, 0] = 2
        with self.assertRaises(ValueError):
            self.calibration.targets([bad, self.wrists[1]])
        RelativeCalibration(self.head, self.wrists, [pose([0, 0, 0]), pose([0, .01, 0])], np.zeros(14)).targets(self.wrists)

    def test_common_world_yaw_preserves_mapping(self):
        theta = .8
        world = np.eye(4); world[:2, :2] = [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        world[:3, 3] = [.8, -.5, .2]
        transformed = RelativeCalibration(world @ self.head, [world @ p for p in self.wrists], self.robot, np.zeros(14))
        moved = [p.copy() for p in self.wrists]; moved[0][0, 3] += .03
        for a, b in zip(self.calibration.targets(moved), transformed.targets([world @ p for p in moved])):
            np.testing.assert_allclose(a, b, atol=1e-12)


class GateTests(unittest.TestCase):
    def test_defaults_run_until_stopped(self):
        args = parse_args([])
        self.assertFalse(args.execute)
        self.assertFalse(args.robot_feedback)
        self.assertEqual((args.duration, args.translation_scale, args.frequency), (0., 1., 30.))
        self.assertEqual(parse_args(['--duration', '3600']).duration, 3600.)
        with self.assertRaises(SystemExit), patch('sys.stderr', io.StringIO()):
            parse_args(['--duration', '-1'])

    def test_still_hands_stay_fresh_and_head_is_not_required(self):
        hand = {'valid': True, 'event_age': .02, 'change_age': 30.}
        snapshot = {'left': hand, 'right': dict(hand), 'head': {'valid': False, 'event_age': float('inf')}}
        self.assertTrue(hands_fresh(snapshot))
        snapshot['right']['event_age'] = .6
        self.assertFalse(hands_fresh(snapshot))
        self.assertTrue(hands_fresh(snapshot, max_age=1.))
        self.assertFalse(hands_fresh({'left': hand}))

    def test_robot_feedback_needs_only_recent_finite_q(self):
        state = {'q_arm': [0.] * 14, 'receive_age_s': .02, 'fsm_id': 1, 'mode_machine': 5, 'time': 0.}
        self.assertTrue(state_usable(state))
        self.assertFalse(state_usable({**state, 'receive_age_s': 2.}))
        self.assertFalse(state_usable({**state, 'q_arm': [float('nan')] * 14}))
        self.assertFalse(state_usable({}))

    def test_hands_usable_requires_live_ready_bridge(self):
        bridge = Mock(); bridge.process.poll.return_value = None; bridge.ready.is_set.return_value = True
        self.assertTrue(hands_usable(bridge))
        bridge.process.poll.return_value = 1
        self.assertFalse(hands_usable(bridge))
        self.assertFalse(hands_usable(None))

    def test_terminal_eof_cannot_deliver_arm_line(self):
        readfd, writefd = os.pipe()
        stream = os.fdopen(readfd, 'rb', buffering=0)
        control = LineControls(stream)
        os.write(writefd, b'r\n'); os.close(writefd)
        self.assertEqual(control.read(), [])
        self.assertFalse(control.connected)
        stream.close()

    def test_readonly_bridge_rejects_writes(self):
        bridge = object.__new__(ArmBridge); bridge.execute = False
        with self.assertRaises(RuntimeError):
            bridge.send({'q_arm': [0.] * 14})

    def test_execute_send_reports_backpressure_and_broken_pipe_as_false(self):
        bridge = object.__new__(ArmBridge); bridge.execute = True
        bridge.process = Mock(); bridge.process.stdin.fileno.return_value = 99
        for error in (BlockingIOError(), BrokenPipeError(), OSError()):
            with patch('quest_upper_body.os.write', side_effect=error):
                self.assertFalse(bridge.send({'seq': 1}))
        with patch('quest_upper_body.os.write', side_effect=lambda fd, data: len(data)):
            self.assertTrue(bridge.send({'seq': 1}))

    def test_nonfinite_joint_feedback_rejected(self):
        with self.assertRaises(ValueError):
            finite_vector([float('inf')] * 14, 14)

    def test_tracking_diagnostics_name_problems(self):
        snapshot = {'left': {'valid': True, 'event_age': .8}, 'right': {'valid': True, 'event_age': .01},
                    'head': {'valid': False, 'event_age': float('inf')}}
        self.assertEqual(tracking_diagnostics(snapshot), ['left.event_stale', 'head.invalid_packet', 'head.event_stale'])

    def test_throttle_counts_suppressed_events(self):
        throttle = Throttle(1.)
        self.assertEqual(throttle('ik', 0.), 1)
        self.assertEqual(throttle('ik', .5), 0)
        self.assertEqual(throttle('ik', .6), 0)
        self.assertEqual(throttle('ik', 1.1), 3)

    def test_readonly_close_terminates_ssh_without_execute_release_wait(self):
        bridge = object.__new__(ArmBridge); bridge.execute = False
        bridge.process, bridge.reader = Mock(), Mock()
        bridge.process.stdin.closed = False; bridge.process.poll.return_value = None
        events = []
        bridge.process.stdin.close.side_effect = lambda: events.append('eof')
        bridge.process.terminate.side_effect = lambda: events.append('terminate')
        bridge.process.wait.side_effect = lambda **kwargs: events.append(('wait', kwargs['timeout']))
        bridge.close()
        self.assertEqual(events, ['eof', 'terminate', ('wait', 2)])

    def test_execute_close_preserves_eof_release_grace(self):
        bridge = object.__new__(ArmBridge); bridge.execute = True
        bridge.process, bridge.reader = Mock(), Mock(); bridge.process.stdin.closed = False
        bridge.close()
        bridge.process.stdin.close.assert_called_once()
        bridge.process.wait.assert_called_once_with(timeout=5)
        bridge.process.terminate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
