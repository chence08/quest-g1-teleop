#!/usr/bin/env python3
"""Read-only clock/round-trip probe over one persistent SSH connection."""
import argparse
import json
import select
import shlex
import subprocess
import time
from pathlib import Path
from teleop_config import load_config, remote_python_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=load_config()['G1_SSH_HOST'])
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.host.startswith('-'):
        parser.error('host cannot be an option')
    script = 'import sys,time,json\nfor line in sys.stdin:\n print(json.dumps({"robot_time":time.time()}),flush=True)'
    remote = shlex.join(remote_python_command(args=['-c', script]))
    process = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
        '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2', '--', args.host, remote],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    samples = []
    try:
        for index in range(8):
            before = time.time()
            process.stdin.write('probe\n'); process.stdin.flush()
            if not select.select([process.stdout], [], [], 8)[0]:
                raise RuntimeError('SSH clock response timed out')
            line = process.stdout.readline()
            after = time.time()
            data = json.loads(line)
            if index:
                samples.append({'rtt_s': after-before,
                    'robot_minus_workstation_s': data['robot_time']-(before+after)/2})
            time.sleep(.05)
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.terminate(); process.wait(timeout=2)
    best = min(samples, key=lambda s: s['rtt_s'])
    report = {'scope':'read-only SSH clock check; not a latency guarantee', 'checked_at':time.time(),
        'samples':samples, 'best_rtt_sample':best,
        'small_clock_offset':
            -.04 < best['robot_minus_workstation_s'] < .2 and best['rtt_s'] < .1}
    output = json.dumps(report, indent=2)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output+'\n')
    print(output)
    return 0 if report['small_clock_offset'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
