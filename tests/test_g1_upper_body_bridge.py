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


def factory():
    return types.SimpleNamespace(mode_machine=0, mode_pr=0, crc=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0., dq=0., tau=0., kp=0., kd=0., reserve=0) for _ in range(35)])


class Crc:
    def Crc(self, msg):
        return 123456


class UpperBodyTests(unittest.TestCase):
    def test_pinned_limits_and_order_match_urdf(self):
        urdf = MODEL_REPO / 'assets/g1/g1_body29_hand14.urdf'
        if not urdf.is_file():
            self.skipTest('upstream checkout missing; run scripts/fetch_upstream.py')
        joints = {node.attrib['name']: node for node in ET.parse(urdf).getroot().findall('joint')}
        for name, low, high in zip(bridge.JOINT_NAMES, bridge.LOWER, bridge.UPPER):
            limit = joints[name].find('limit')
            self.assertEqual((low, high), (float(limit.attrib['lower']), float(limit.attrib['upper'])))
        self.assertEqual(bridge.ARM_INDICES, tuple(range(15, 29)))

    def test_targets_are_clipped_not_rejected(self):
        clipped = bridge.arm_positions([10.] * 14)
        self.assertEqual(clipped, bridge.UPPER)
        command = bridge.parse_command({'seq': 1, 'q_arm': [-10.] * 14, 'tau_arm': [100.] * 14})
        self.assertEqual(command.q_arm, bridge.LOWER)
        self.assertEqual(command.tau_arm, (bridge.TAU_LIMIT,) * 14)
        for bad in (None, {'seq': True, 'q_arm': Q}, {'seq': 1, 'q_arm': [math.nan] * 14}, {'seq': 1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                bridge.parse_command(bad)

    def test_mailbox_skips_bad_and_old_packets_and_holds_latest(self):
        box = bridge.Mailbox()
        self.assertTrue(box.ingest({'seq': 2, 'q_arm': Q}))
        self.assertFalse(box.ingest({'seq': 1, 'q_arm': [0.] * 14}))
        self.assertFalse(box.ingest({'seq': 3, 'q_arm': [math.nan] * 14}))
        self.assertFalse(box.ingest(None))
        command, eof = box.snapshot()
        self.assertEqual((command.seq, eof, box.rejected), (2, False, 2))
        self.assertTrue(box.ingest({'seq': 4, 'q_arm': Q, 'sent_at': 0.}))  # Any clock skew is accepted.
        box.mark_eof()
        self.assertEqual(box.snapshot()[0].seq, 4)
        self.assertTrue(box.snapshot()[1])

    def test_state_gate_skips_duplicates_and_reports_age(self):
        gate = bridge.StateGate()
        self.assertFalse(gate.snapshot(0.)['ready'])
        self.assertTrue(gate.update(packet(1, mode_machine=5, dq_arm=[3.] * 14), 1., 100.))
        self.assertFalse(gate.update(packet(1), 1.1, 100.1))
        self.assertTrue(gate.snapshot(1.2)['ready'])
        self.assertEqual(gate.snapshot(1.2)['duplicate_ticks'], 1)
        self.assertFalse(gate.snapshot(2.)['ready'])
        self.assertFalse(gate.update(packet(2, q_arm=[math.nan] * 14), 2., 101.))
        self.assertEqual(gate.invalid, 1)

    def test_velocity_step_scales_whole_move(self):
        step = bridge.velocity_step((0.,) * 14, (1.,) + (.5,) * 13, .1)
        self.assertAlmostEqual(step[0], .1)
        self.assertAlmostEqual(step[1], .05)
        self.assertEqual(bridge.velocity_step((0.,) * 14, (.05,) * 14, .1), (.05,) * 14)

    def test_motion_ramps_weight_then_velocity_then_holds_and_releases(self):
        motion = bridge.ArmMotion(Q, 0., ramp_up=1., speed_ramp=2., min_velocity=1., max_velocity=5., release_time=.5)
        far = tuple(value + 1. for value in Q)
        motion.step(.5, far)
        self.assertEqual((motion.weight, motion.target), (.5, motion.initial))
        motion.step(1.01, far)
        self.assertEqual(motion.phase, 'tracking')
        self.assertAlmostEqual(motion.velocity_limit(2.), 3.)
        self.assertAlmostEqual(motion.velocity_limit(9.), 5.)
        before = motion.target
        motion.step(1.02, None)
        self.assertEqual(motion.target, before)  # No command holds the last target.
        motion.release(2., 'test')
        motion.step(2.25)
        self.assertAlmostEqual(motion.weight, .5)
        motion.step(2.6)
        self.assertEqual((motion.weight, motion.phase), (0., 'released'))
        motion.release(3., 'again')
        self.assertEqual((motion.weight, motion.reason), (0., 'test'))

    def test_message_populates_only_arm_weight_gains_and_torque(self):
        message = bridge.build_message(factory, Crc(), Q, .4, 6, tuple(float(i) for i in range(14)))
        self.assertEqual((message.mode_machine, message.crc, message.motor_cmd[29].q), (6, 123456, .4))
        for slot, index in enumerate(bridge.ARM_INDICES):
            motor = message.motor_cmd[index]
            self.assertEqual((motor.mode, motor.q, motor.tau), (1, Q[slot], float(slot)))
            self.assertEqual((motor.kp, motor.kd), (40., 1.5) if slot % 7 >= 4 else (80., 3.))
        self.assertTrue(all(message.motor_cmd[i].kp == 0 for i in range(15)))

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

    def test_cli_defaults_run_until_stopped(self):
        args = bridge.parse_args([])
        self.assertEqual((args.execute, args.duration, args.rate, args.max_velocity), (False, 0., 250, 10.))
        self.assertEqual(bridge.parse_args(['--execute', '--max-velocity', '30']).max_velocity, 30.)
        with mock.patch('sys.stderr'), self.assertRaises(SystemExit):
            bridge.parse_args(['--max-velocity', '-1'])

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
            box.ingest({'seq': 1, 'sent_at': time.time(), 'q_arm': Q})
            stop.wait(1)
        argv = ['--duration', '.6' if execute else '.08', '--release-time', '.02', '--ramp-up', '.02']
        if execute:
            argv += ['--execute']
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

    def test_execute_write_failure_continues_then_releases_and_closes(self):
        result, publishers, messages, topics = self.run_mock(True, True)
        self.assertEqual(result, 0)
        self.assertEqual(topics, ['rt/lowstate', 'rt/arm_sdk'])
        self.assertTrue(publishers[0].closed)
        self.assertTrue(any(m.motor_cmd[29].q == 1. for m in publishers[0].messages))
        self.assertEqual([m.motor_cmd[29].q for m in publishers[0].messages[-3:]], [0., 0., 0.])
        self.assertTrue(any(m.get('status') == 'released' for m in messages))
        self.assertTrue(any(m.get('write_failures') == 1 for m in messages if m['type'] == 'state'))
        self.assertEqual(messages[-1]['weight'], 0.)

    def test_readonly_broken_output_exits_without_arm_publisher(self):
        result, publishers, messages, topics = self.run_mock(False, disconnect_output=True)
        self.assertEqual(publishers, [])
        self.assertEqual(topics, ['rt/lowstate'])
        self.assertTrue(any(m.get('status') == 'stopped' and m.get('reason') == 'stdout_disconnected'
                            for m in messages))
        self.assertEqual(messages[-1]['status'], 'finished')
        self.assertEqual(messages[-1]['published_count'], 0)


if __name__ == '__main__':
    unittest.main()
