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
from quest_upper_body import (RelativeCalibration, LineControls, state_eligible,
                             upper_tracking_eligible, parse_args, ArmBridge, finite_vector,
                             prearm_conflict_check, send_fresh_solution, RunPreparation, tracking_diagnostics)


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

    def test_translation_is_scaled_and_norm_bounded(self):
        wrists = [p.copy() for p in self.wrists]
        wrists[0][:3, 3] += [1., 1., 1.]
        target = self.calibration.targets(wrists)[0]
        self.assertAlmostEqual(np.linalg.norm(target[:3, 3] - self.robot[0][:3, 3]), .08)
        wrists = [p.copy() for p in self.wrists]
        wrists[0][0, 3] += .02
        self.assertAlmostEqual(self.calibration.targets(wrists)[0][0, 3] - .2, .01)

    def test_invalid_transform_and_crossed_target_rejected(self):
        bad = self.wrists[0].copy(); bad[0, 0] = 2
        with self.assertRaises(ValueError):
            self.calibration.targets([bad, self.wrists[1]])
        with self.assertRaises(ValueError):
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
    def test_preview_default_and_no_invalid_duration(self):
        args = parse_args([])
        self.assertFalse(args.execute)
        self.assertFalse(args.robot_feedback)
        self.assertEqual(args.expected_mode_machine, 6)
        with self.assertRaises(SystemExit), patch('sys.stderr', io.StringIO()):
            parse_args(['--duration', '121'])

    def test_head_and_both_hands_must_be_fresh(self):
        hand = {'valid': True, 'event_age': .02, 'change_age': .02}
        snapshot = {'left': hand, 'right': hand, 'head': {'valid': True, 'event_age': .02}}
        self.assertTrue(upper_tracking_eligible(snapshot))
        snapshot['head']['event_age'] = .5
        self.assertFalse(upper_tracking_eligible(snapshot))

    def test_robot_feedback_mode_and_freshness(self):
        state = {'ready': True, 'q_arm': [0.] * 14, 'dq_arm': [0.] * 14,
                 'receive_age_s': .02, 'fsm_id': 500, 'mode_machine': 6, 'time': time.time()}
        self.assertTrue(state_eligible(state, 6))
        for key, value in [('fsm_id', 1), ('mode_machine', 5), ('receive_age_s', 1.1), ('q_arm', [float('nan')] * 14), ('time', time.time() - 2.)]:
            self.assertFalse(state_eligible({**state, key: value}, 6))

    def test_terminal_eof_cannot_deliver_arm_line(self):
        readfd, writefd = os.pipe()
        stream = os.fdopen(readfd, 'rb', buffering=0)
        control = LineControls(stream)
        os.write(writefd, b'r\n'); os.close(writefd)
        self.assertEqual(control.read(), [])
        self.assertFalse(control.connected)
        stream.close()

    def test_readonly_bridge_rejects_writes_before_transport_use(self):
        bridge = object.__new__(ArmBridge); bridge.execute = False
        with self.assertRaises(RuntimeError):
            bridge.send({'q_arm': [0.] * 14})

    def test_nonfinite_joint_feedback_rejected(self):
        with self.assertRaises(ValueError):
            finite_vector([float('inf')] * 14, 14)

    def test_clock_lead_within_allowance_passes(self):
        state = {'ready': True, 'q_arm': [0.] * 14, 'dq_arm': [0.] * 14,
                 'receive_age_s': .02, 'fsm_id': 500, 'mode_machine': 6, 'time': 1000.16}
        with patch('quest_upper_body.time.time', return_value=1000.):
            self.assertTrue(state_eligible(state, 6))
            self.assertFalse(state_eligible({**state, 'time': 1000.25}, 6))

    def test_prearm_recheck_is_readonly_and_rejects_old_remote_checker(self):
        import subprocess
        clear = {'event': 'conflict_check', 'check_version': 2, 'port_5602_listening': False,
                 'known_controller_processes': [], 'process_scan_complete': True, 'clear': True}
        with patch('quest_upper_body.subprocess.run', return_value=subprocess.CompletedProcess([], 0, json.dumps(clear), '')) as run:
            self.assertEqual(prearm_conflict_check('g1'), clear)
            command = run.call_args.args[0]
            self.assertIn('--check-conflicts', command[-1])
            self.assertNotIn('--execute', command[-1])
        with patch('quest_upper_body.subprocess.run', return_value=subprocess.CompletedProcess([], 0, '{"event":"conflict_check","port_5602_listening":false}', '')):
            with self.assertRaises(RuntimeError): prearm_conflict_check('g1')

    def test_tracking_diagnostics_distinguish_event_change_and_pose_failures(self):
        snapshot = {'left': {'valid': True, 'event_age': .3, 'change_age': .01},
                    'right': {'valid': True, 'event_age': .01, 'change_age': .3},
                    'head': {'valid': False, 'event_age': float('inf')}}
        bad = np.eye(4); bad[0, 0] = 2.
        result = tracking_diagnostics(snapshot, np.eye(4), [bad, np.eye(4)])
        for reason in ('left.event_stale', 'right.unchanged_pose_stale', 'head.invalid_packet', 'left_wrist.invalid_se3'):
            self.assertIn(reason, result['reasons'])
        self.assertFalse(result['poses']['left_wrist']['orthonormal'])
        self.assertIsNone(result['samples']['head']['event_age'])
        json.dumps(result, allow_nan=False)

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
        bridge.process.wait.assert_called_once_with(timeout=4)
        bridge.process.terminate.assert_not_called()


class SubmissionTests(unittest.TestCase):
    def setUp(self):
        self.active, self.hands = Mock(), Mock()
        self.active.send.return_value = self.hands.send.return_value = True
        self.active.state.return_value = {'ready': True, 'q_arm': [0.] * 14, 'dq_arm': [0.] * 14,
                 'receive_age_s': .02, 'fsm_id': 500, 'mode_machine': 6, 'time': 1000.}
        self.hands.ready.is_set.return_value = True
        self.hands.last_message = {'time': 1000.}

    def snapshot(self, age):
        return {'left': {'valid': True, 'event_age': age, 'change_age': age},
                'right': {'valid': True, 'event_age': age, 'change_age': age},
                'head': {'valid': True, 'event_age': age}}

    def submit(self, age, elapsed, current=None):
        with patch('quest_upper_body.time.monotonic', return_value=elapsed), patch('quest_upper_body.time.time', return_value=1000.):
            return send_fresh_solution(self.active, self.hands, self.snapshot(age), current or self.snapshot(0.),
                                       0., 999.9, 1, np.zeros(14), [0.] * 12, 6)

    def test_slow_ik_or_old_input_sends_neither_stream(self):
        for age, elapsed in ((.02, .24), (.24, .02), (.25, .000001)):
            with self.subTest(age=age, elapsed=elapsed), self.assertRaisesRegex(RuntimeError, 'stale_solution'):
                self.submit(age, elapsed)
            self.active.send.assert_not_called(); self.hands.send.assert_not_called()

    def test_fresh_solution_preserves_capture_timestamp(self):
        self.assertEqual(self.submit(.02, .22), (True, True))
        self.assertEqual(self.active.send.call_args.args[0]['sent_at'], 999.9)
        self.assertEqual(self.hands.send.call_args.args[0]['sent_at'], 999.9)

    def test_exact_age_limit_passes(self):
        self.assertEqual(self.submit(.25, 0.), (True, True))

    def test_new_tracking_failure_or_robot_failure_prevents_both_writes(self):
        current = self.snapshot(0.); current['head']['valid'] = False
        with self.assertRaisesRegex(RuntimeError, 'stale_solution'): self.submit(.01, .01, current)
        self.active.state.return_value['ready'] = False
        with self.assertRaisesRegex(RuntimeError, 'robot_feedback'): self.submit(.01, .01)
        self.active.send.assert_not_called(); self.hands.send.assert_not_called()

    def prepared_submit(self, preparation, now, age=.01):
        with patch('quest_upper_body.time.monotonic', return_value=now), patch('quest_upper_body.time.time', return_value=1000.):
            return send_fresh_solution(self.active, self.hands, self.snapshot(age), self.snapshot(age),
                                       now, 999.99, 1, np.zeros(14), [0.] * 12, 6, preparation=preparation)

    def test_preparation_waits_for_stable_fresh_interval_before_first_write(self):
        preparation = RunPreparation()
        with self.assertRaisesRegex(RuntimeError, 'stale_solution'):
            self.prepared_submit(preparation, 0., age=.3)
        self.assertFalse(preparation.write_attempted)
        self.assertEqual(self.prepared_submit(preparation, 1.), (False, False))
        self.assertEqual(self.prepared_submit(preparation, 1.49), (False, False))
        self.active.send.assert_not_called(); self.hands.send.assert_not_called()
        self.assertEqual(self.prepared_submit(preparation, 1.5), (True, True))
        self.assertTrue(preparation.write_attempted)

    def test_preparation_feedback_loss_resets_interval_and_postwrite_loss_latches(self):
        preparation = RunPreparation()
        self.prepared_submit(preparation, 0.)
        preparation.ready(True, False, .4)
        self.assertEqual(self.prepared_submit(preparation, .5), (False, False))
        self.assertEqual(self.prepared_submit(preparation, 1.), (True, True))
        self.active.send.reset_mock(); self.hands.send.reset_mock()
        with self.assertRaisesRegex(RuntimeError, 'stale_solution'):
            self.prepared_submit(preparation, 1.1, age=.3)
        self.assertTrue(preparation.halted)
        self.assertEqual(self.prepared_submit(preparation, 2.), (False, False))
        self.active.send.assert_not_called(); self.hands.send.assert_not_called()

    def test_partial_transport_failure_cannot_return_to_preparation(self):
        preparation = RunPreparation(); self.prepared_submit(preparation, 0.)
        self.hands.send.side_effect = RuntimeError('Partial hand SSH write')
        with self.assertRaisesRegex(RuntimeError, 'Partial hand'):
            self.prepared_submit(preparation, .5)
        self.active.send.assert_called_once()
        self.assertTrue(preparation.write_attempted)
        preparation.invalidate()
        self.assertTrue(preparation.halted)
        self.assertFalse(preparation.ready(True, True, 10.))

    def test_arm_backpressure_faults_before_any_hand_write(self):
        preparation = RunPreparation(); self.prepared_submit(preparation, 0.)
        self.active.send.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'arm_transport_backpressure'):
            self.prepared_submit(preparation, .5)
        self.hands.send.assert_not_called()
        self.assertTrue(preparation.write_attempted)

    def test_hand_backpressure_faults_after_arm_write(self):
        preparation = RunPreparation(); self.prepared_submit(preparation, 0.)
        self.hands.send.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'hand_transport_backpressure'):
            self.prepared_submit(preparation, .5)
        self.active.send.assert_called_once()
        self.assertTrue(preparation.write_attempted)


if __name__ == '__main__':
    unittest.main()
