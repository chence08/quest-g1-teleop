"""Offline bridge tests: no real DDS factory, robot transport, or hardware."""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
MODEL_REPO = Path(os.environ.get('XR_TELEOP_TEST_REPO', ROOT / 'repos/xr_teleoperate'))
SPEC = importlib.util.spec_from_file_location("g1_upper_body_bridge", ROOT / "scripts/g1_upper_body_bridge.py")
bridge = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bridge
SPEC.loader.exec_module(bridge)
Q = (.29, .2, 0., .98, .15, 0., 0., .29, -.2, 0., .98, -.15, 0., 0.)


def packet(tick=1, **changes):
    result = {"q_arm": Q, "dq_arm": [.014] * 14, "q_waist": [0.] * 3, "imu_rpy": [0.] * 3,
              "mode_machine": 6, "mode_pr": 0, "tick": tick, "arm_mode": [1] * 14,
              "arm_status": [0] * 14}
    result.update(changes)
    return result


def qualify(gate):
    gate.update_fsm(0, 500, 0.)
    for index in range(10):
        gate.update(packet(index + 1), index * .001, 100. + index * .001)
    return gate.snapshot(.01)


def factory():
    return types.SimpleNamespace(mode_machine=0, mode_pr=0, crc=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0., dq=0., tau=0., kp=0., kd=0., reserve=0) for _ in range(35)])


class Crc:
    def Crc(self, msg):
        return 123456


class UpperBodyTests(unittest.TestCase):
    def test_pinned_limits_and_order_match_urdf(self):
        joints = {node.attrib['name']: node for node in ET.parse(MODEL_REPO / 'assets/g1/g1_body29_hand14.urdf').getroot().findall('joint')}
        for name, low, high in zip(bridge.JOINT_NAMES, bridge.LOWER, bridge.UPPER):
            limit = joints[name].find('limit')
            self.assertEqual((low, high), (float(limit.attrib['lower']), float(limit.attrib['upper'])))
        self.assertEqual(bridge.ARM_INDICES, tuple(range(15, 29)))

    def test_commands_validate_sequence_age_bounds_and_zero_torque(self):
        good = {'seq': 1, 'sent_at': 100., 'q_arm': Q}
        command = bridge.validate_command(good, 100.05, 1., 0)
        self.assertAlmostEqual(command.age(1.1), .15)
        bad = [dict(good, seq=0), dict(good, seq=True), dict(good, sent_at=99.),
               dict(good, sent_at=101.), dict(good, q_arm=[math.nan] * 14),
               dict(good, q_arm=[0.] * 13), dict(good, q_arm=[10.] * 14),
               dict(good, tau_arm=[.1] * 14), dict(good, tau_arm=[math.inf] * 14)]
        for item in bad:
            with self.subTest(item=item), self.assertRaises(ValueError):
                bridge.validate_command(item, 100., 1., 0)

    def test_mailbox_invalid_is_latched_latest_only(self):
        box = bridge.Mailbox()
        for seq in (1, 2):
            self.assertTrue(box.ingest({'seq': seq, 'sent_at': 100., 'q_arm': Q}, 100., 1.))
        self.assertEqual(box.snapshot(1.1)[0].seq, 2)
        self.assertIsNone(box.snapshot(1.31)[0])
        self.assertFalse(box.ingest({'seq': 2, 'sent_at': 100., 'q_arm': Q}, 100., 1.4))
        self.assertFalse(box.ingest({'seq': 3, 'sent_at': 100., 'q_arm': Q}, 100., 1.4))
        self.assertIsNotNone(box.snapshot(1.4)[2])

    def test_ten_unique_ticks_and_duplicates_do_not_refresh_age(self):
        gate = bridge.StateGate()
        self.assertTrue(qualify(gate)['ready'])
        for now in (.02, .04, .08, .11, .15):
            self.assertFalse(gate.update(packet(10), now, 100. + now))
        self.assertAlmostEqual(gate.received_at, .009)
        self.assertEqual(gate.duplicates, 5)
        self.assertFalse(gate.snapshot(.15)['ready'])

    def test_fresh_arrival_after_gap_cannot_hide_stale_active_state(self):
        gate = bridge.StateGate()
        qualify(gate)
        gate.engage()
        gate.update(packet(11), .2, 100.2)
        self.assertEqual(gate.snapshot(.2)['fault'], 'stale_lowstate_gap')
        for tick in range(12, 30):
            gate.update(packet(tick), .201 + tick * .001, 100.2)
        self.assertFalse(gate.snapshot(.25)['ready'])

    def test_quiet_start_and_motor_mode_status_checks(self):
        for changed in ({'dq_arm': [.1] * 14}, {'arm_mode': [0] * 14},
                        {'arm_status': [1] * 14}, {'q_arm': [math.nan] * 14}):
            gate = bridge.StateGate()
            gate.update_fsm(0, 500, 0.)
            for tick in range(10):
                gate.update(packet(tick, **changed), tick * .001, 100.)
            self.assertFalse(gate.snapshot(.01)['ready'])

    def test_mode_change_duplicate_tick_immediately_latches(self):
        gate = bridge.StateGate()
        qualify(gate)
        gate.engage()
        gate.update(packet(10, mode_machine=7), .02, 100.02)
        self.assertTrue(gate.snapshot(.02)['immediate_release'])
        self.assertIsNotNone(gate.fault)
        gate.update(packet(11), .03, 100.03)
        self.assertFalse(gate.snapshot(.03)['ready'])

    def test_fsm_change_or_failure_latches_without_mode_setting(self):
        for code, fsm in ((0, 1), (-1, None)):
            gate = bridge.StateGate()
            qualify(gate)
            gate.engage()
            gate.update_fsm(code, fsm, .02)
            self.assertTrue(gate.snapshot(.02)['immediate_release'])
            gate.update_fsm(0, 500, .03)
            self.assertFalse(gate.snapshot(.03)['ready'])

    def test_body_drift_trips_and_yaw_wrap_is_handled(self):
        self.assertAlmostEqual(bridge.wrapped_delta(math.pi - .01, -math.pi + .01), .02)
        for changed in ({'q_waist': [.051, 0., 0.]}, {'imu_rpy': [0., .051, 0.]}):
            gate = bridge.StateGate()
            qualify(gate)
            gate.engage()
            gate.update(packet(11, **changed), .02, 100.02)
            self.assertIn('waist_or_imu_drift', gate.fault)
            self.assertTrue(gate.immediate)

    def test_ramp_holds_measured_target_then_slew_and_excursion(self):
        motion = bridge.ArmMotion(Q, 0.)
        requested = tuple(q + .5 for q in Q)
        motion.step(1., requested)
        self.assertEqual(motion.target, Q)
        self.assertAlmostEqual(motion.weight, .5)
        motion.step(10., requested)  # long pause never produces a target catch-up
        self.assertLessEqual(max(abs(a - b) for a, b in zip(motion.target, Q)), .0015 + 1e-12)
        for index in range(200):
            motion.step(10.01 + index * .01, requested)
        self.assertAlmostEqual(max(abs(a - b) for a, b in zip(motion.target, Q)), .15)
        target = motion.target
        motion.release(13., 'stale_command')
        motion.step(13.5, tuple(Q))
        self.assertEqual(motion.weight, 0.)
        self.assertEqual(motion.phase, 'released')
        self.assertEqual(motion.target, target)
        motion.step(15., Q)
        self.assertEqual(motion.weight, 0.)

    def test_message_populates_only_arm_and_weight_and_crc(self):
        msg = bridge.build_message(factory, Crc(), Q, .4, feedback=packet())
        self.assertEqual((msg.mode_pr, msg.mode_machine, msg.crc), (0, 6, 123456))
        for index, motor in enumerate(msg.motor_cmd):
            if index in bridge.ARM_INDICES:
                self.assertEqual(motor.q, Q[index - 15])
                self.assertEqual(motor.tau, 0.)
                self.assertEqual(motor.kp, 40. if (index - 15) % 7 >= 4 else 60.)
            elif index == 29:
                self.assertEqual(motor.q, .4)
                self.assertEqual((motor.mode, motor.dq, motor.tau, motor.kp, motor.kd), (0, 0., 0., 0., 0.))
            else:
                self.assertTrue(all(value == 0 for value in vars(motor).values()), f'non-arm slot {index} changed')

    def test_release_retries_zero_after_write_exception(self):
        class Publisher:
            def __init__(self):
                self.attempts = []
            def Write(self, msg):
                self.attempts.append(msg.motor_cmd[29].q)
                if len(self.attempts) == 1:
                    raise RuntimeError('simulated DDS error')
                return True
        pub = Publisher()
        motion = bridge.ArmMotion(Q, time.monotonic() - 3., release_time=.02)
        motion.step(time.monotonic(), Q)
        output = mock.Mock()
        bridge.release_transport(pub, factory, Crc(), motion, 6, .001, output)
        self.assertEqual(pub.attempts[-3:], [0., 0., 0.])
        self.assertEqual(motion.target, Q)

    def test_release_mode_change_goes_directly_zero(self):
        motion = bridge.ArmMotion(Q, time.monotonic() - 3.)
        motion.step(time.monotonic(), Q)
        pub = mock.Mock()
        bridge.release_transport(pub, factory, Crc(), motion, 6, .001, mock.Mock(),
                                 lambda: {'immediate_release': True})
        self.assertTrue(all(call.args[0].motor_cmd[29].q == 0 for call in pub.Write.call_args_list))

    def test_immediate_release_then_finally_cannot_restore_positive_weight(self):
        motion = bridge.ArmMotion(Q, time.monotonic() - 3.)
        motion.step(time.monotonic(), Q)
        self.assertEqual(motion.weight, 1.)
        motion.release(time.monotonic(), 'mode_changed', immediate=True)
        motion.release(time.monotonic(), 'cleanup')
        motion.step(time.monotonic())
        self.assertEqual(motion.phase, 'released')
        pub = mock.Mock()
        bridge.release_transport(pub, factory, Crc(), motion, 6, .001, mock.Mock())
        self.assertTrue(all(call.args[0].motor_cmd[29].q == 0 for call in pub.Write.call_args_list))

    def test_actual_arm_runaway_and_measured_torque_latch(self):
        changed_q = list(Q)
        changed_q[0] += .251
        tracking_q = list(Q)
        tracking_q[1] += .201
        torque = [0.] * 14
        torque[5] = 4.01
        for changes, reason in (({'q_arm': changed_q}, 'measured_arm_excursion'),
                                ({'q_arm': tracking_q}, 'arm_tracking_error'),
                                ({'dq_arm': [.751] * 14}, 'excessive_arm_velocity'),
                                ({'tau_arm': torque}, 'excessive_measured_arm_torque')):
            gate = bridge.StateGate()
            qualify(gate)
            gate.engage()
            gate.update(packet(11, **changes), .02, 100.02)
            self.assertIn(reason, gate.fault)
            self.assertTrue(gate.immediate)

    def test_estimated_pd_effort_requires_feedback_and_rejects_excess(self):
        with self.assertRaises(ValueError):
            bridge.build_message(factory, Crc(), Q, .5)
        for joint, delta in ((0, .251), (5, .076)):
            target = list(Q)
            target[joint] += delta
            with self.assertRaisesRegex(ValueError, 'estimated_PD_effort'):
                bridge.build_message(factory, Crc(), target, .5, feedback=packet(dq_arm=[0.] * 14))
        bridge.build_message(factory, Crc(), Q, 0.)

    def test_execute_lock_rejects_second_owner(self):
        with tempfile.TemporaryDirectory() as temp:
            path = str(Path(temp) / 'bridge.lock')
            descriptor = bridge.acquire_execute_lock(path)
            try:
                with self.assertRaises(RuntimeError):
                    bridge.acquire_execute_lock(path)
            finally:
                os.close(descriptor)
            descriptor = bridge.acquire_execute_lock(path)
            os.close(descriptor)

    def test_cli_readonly_default_explicit_mode_and_caps(self):
        args = bridge.parse_args([])
        self.assertFalse(args.execute)
        self.assertEqual(args.duration, 15)
        self.assertEqual(bridge.parse_args(['--duration', '120']).duration, 120)
        with mock.patch('sys.stderr'):
            for argv in (['--execute'], ['--execute', '--expected-mode-machine', '7'],
                         ['--duration', '121'], ['--release-time', '2'], ['--max-excursion', '.3']):
                with self.assertRaises(SystemExit):
                    bridge.parse_args(argv)

    def test_import_is_offline(self):
        self.assertFalse(any(name.startswith(('unitree_sdk2py', 'cyclonedds')) for name in sys.modules))


class MockRuntimeTests(unittest.TestCase):
    def run_mock(self, execute, fail_write=False, disconnect_output=False):
        emitted, publishers, topics = [], [], []
        real_output = bridge.JsonOutput
        output_pipe = os.pipe() if disconnect_output else None
        output_reader_closed = False
        class Output:
            def __init__(self):
                self.transport = real_output(output_pipe[1]) if output_pipe else None
            @property
            def closed(self):
                return self.transport.closed if self.transport else False
            def emit(self, kind, **fields):
                nonlocal output_reader_closed
                emitted.append({'type': kind, **fields})
                if self.transport is not None:
                    if kind == 'state' and not output_reader_closed:
                        # Exercise the production BrokenPipe path after startup,
                        # as when the caller terminates its read-only SSH stream.
                        os.close(output_pipe[0])
                        output_reader_closed = True
                    self.transport.emit(kind, **fields)
            def flush(self):
                if self.transport is not None:
                    self.transport.flush()
        class Publisher:
            def __init__(self, topic, kind):
                topics.append(topic)
                publishers.append(self)
                self.messages, self.closed, self.failed = [], False, False
            def Init(self):
                pass
            def Write(self, msg):
                self.messages.append(copy.deepcopy(msg))
                if fail_write and msg.motor_cmd[29].q > 0 and not self.failed:
                    self.failed = True
                    raise RuntimeError('injected publication failure')
                return True
            def Close(self):
                self.closed = True
        class Subscriber:
            def __init__(self, topic, kind):
                topics.append(topic)
            def Init(self, callback):
                for tick in range(1, 11):
                    motors = [types.SimpleNamespace(q=0., dq=0., tau_est=0., mode=1, motorstate=0) for _ in range(35)]
                    for index, q in zip(bridge.ARM_INDICES, Q):
                        motors[index].q = q
                    callback(types.SimpleNamespace(motor_state=motors, imu_state=types.SimpleNamespace(rpy=[0.]*3),
                                                   tick=tick, mode_machine=6, mode_pr=0))
            def Close(self):
                pass
        class Loco:
            def Init(self):
                pass
            def SetTimeout(self, value):
                pass
            def GetFsmId(self):
                return 0, 500
        modules = {}
        contents = {'unitree_sdk2py.core.channel': {'ChannelFactoryInitialize': lambda *a, **kw: None,
                    'ChannelSubscriber': Subscriber, 'ChannelPublisher': Publisher},
                    'unitree_sdk2py.idl.unitree_hg.msg.dds_': {'LowState_': object, 'LowCmd_': object},
                    'unitree_sdk2py.g1.loco.g1_loco_client': {'LocoClient': Loco},
                    'unitree_sdk2py.idl.default': {'unitree_hg_msg_dds__LowCmd_': factory},
                    'unitree_sdk2py.utils.crc': {'CRC': Crc}}
        for name, values in contents.items():
            module = types.ModuleType(name)
            module.__dict__.update(values)
            modules[name] = module
        def input_stream(stream, box, stop):
            box.ingest({'seq': 1, 'sent_at': time.time(), 'q_arm': Q}, time.time(), time.monotonic())
            stop.wait(1)
        argv = ['--duration', '.08', '--release-time', '.02', '--ramp-up', '.02']
        if execute:
            argv += ['--execute', '--expected-mode-machine', '6']
        try:
            with tempfile.TemporaryDirectory() as temp:
                original_acquire = bridge.acquire_execute_lock
                with mock.patch.dict(sys.modules, modules), mock.patch.object(bridge, 'JsonOutput', Output), \
                        mock.patch.object(bridge, 'read_commands', input_stream), \
                        mock.patch.object(bridge, 'acquire_execute_lock', lambda: original_acquire(str(Path(temp) / 'lock'))):
                    result = bridge.main(argv)
        finally:
            if output_pipe:
                if not output_reader_closed:
                    os.close(output_pipe[0])
                os.close(output_pipe[1])
        return result, publishers, emitted, topics

    def test_readonly_runtime_never_constructs_arm_publisher(self):
        result, publishers, messages, topics = self.run_mock(False)
        self.assertEqual(result, 0)
        self.assertEqual(publishers, [])
        self.assertEqual(topics, ['rt/lowstate'])
        self.assertTrue(any(m.get('ready') for m in messages))

    def test_execute_exception_finally_releases_and_closes(self):
        result, publishers, messages, topics = self.run_mock(True, True)
        self.assertEqual(result, 1)
        self.assertEqual(topics, ['rt/lowstate', 'rt/arm_sdk'])
        self.assertTrue(publishers[0].closed)
        self.assertEqual([m.motor_cmd[29].q for m in publishers[0].messages[-3:]], [0., 0., 0.])
        self.assertTrue(any(m.get('status') == 'released' for m in messages))
        self.assertEqual(messages[-1]['weight'], 0.)

    def test_readonly_broken_output_exits_without_arm_publisher(self):
        result, publishers, messages, topics = self.run_mock(False, disconnect_output=True)
        self.assertEqual(result, 2)
        self.assertEqual(publishers, [])
        self.assertEqual(topics, ['rt/lowstate'])
        self.assertTrue(any(m.get('status') == 'stopped' and m.get('reason') == 'stdout_disconnected'
                            for m in messages))
        self.assertFalse(any(m.get('reason') == 'duration_complete' for m in messages))
        self.assertEqual(messages[-1]['status'], 'finished')
        self.assertEqual(messages[-1]['published_count'], 0)


if __name__ == '__main__':
    unittest.main()
