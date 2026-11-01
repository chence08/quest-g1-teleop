#!/usr/bin/env python3
"""Read native G1 lowstate and FSM ID; never publish motor commands."""
import argparse
import json
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    from teleop_config import load_config
    parser.add_argument('--interface', default=load_config()['G1_ARM_INTERFACE'])
    parser.add_argument('--duration', type=float, default=5)
    args = parser.parse_args()
    if not 1 <= args.duration <= 15:
        parser.error('duration must be between 1 and 15 seconds')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    ChannelFactoryInitialize(0, networkInterface=args.interface)
    lock = threading.Lock()
    received = {'count': 0, 'distinct_ticks': 0, 'last': None}
    def on_state(msg):
        with lock:
            previous = received['last']
            received['count'] += 1
            received['distinct_ticks'] += int(previous is None or previous['tick'] != msg.tick)
            received['last'] = {
                'received_at': time.time(), 'tick': msg.tick,
                'mode_machine': msg.mode_machine, 'mode_pr': msg.mode_pr,
                'version': list(msg.version),
                'q_all': [m.q for m in msg.motor_state],
                'q_arm': [m.q for m in msg.motor_state[15:29]],
                'dq_arm': [m.dq for m in msg.motor_state[15:29]],
                'tau_arm': [m.tau_est for m in msg.motor_state[15:29]],
                'arm_mode': [m.mode for m in msg.motor_state[15:29]],
                'arm_status': [m.motorstate for m in msg.motor_state[15:29]],
                'imu_rpy': list(msg.imu_state.rpy)}
    subscriber = ChannelSubscriber('rt/lowstate', LowState_)
    subscriber.Init(on_state, 10)
    try:
        time.sleep(args.duration)
        try:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            client = LocoClient()
            client.SetTimeout(2)
            client.Init()
            fsm_result = client.GetFsmId()
        except Exception as exc:
            fsm_result = {'error': str(exc)}
        with lock:
            report = {'mode': 'read_only', 'interface': args.interface,
                      'fsm_result': fsm_result, **received}
        print(json.dumps(report, allow_nan=False), flush=True)
        return 0 if report['count'] else 2
    finally:
        subscriber.Close()


if __name__ == '__main__':
    raise SystemExit(main())
