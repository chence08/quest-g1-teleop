#!/usr/bin/env python3
"""Run real InspireHand C++ methods against a local PTY hand emulator only."""
import argparse
import errno
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import threading
import time


def frame(operation, address, payload, device=1):
    data = bytes([0x90, 0xEB, device, len(payload) + 3, operation,
                  address & 255, address >> 8]) + payload
    return data + bytes([sum(data[2:]) & 255])


def run_case(binary, name, mode, cycles):
    master, slave = os.openpty()
    stop = threading.Event()
    requests = []
    errors = []

    def emulate():
        pending = b''
        read_index = 0
        try:
            while not stop.is_set():
                if not select.select([master], [], [], 0.01)[0]:
                    continue
                try:
                    data = os.read(master, 4096)
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        continue
                    raise
                if not data:
                    continue
                pending += data
                while len(pending) >= 4:
                    if pending[:2] != b'\xeb\x90':
                        raise AssertionError('request framing corrupted')
                    length = pending[3] + 5
                    if len(pending) < length:
                        break
                    request, pending = pending[:length], pending[length:]
                    if sum(request[2:-1]) & 255 != request[-1]:
                        raise AssertionError('request checksum incorrect')
                    requests.append(request.hex())
                    operation, address = request[4], request[5] | request[6] << 8
                    if operation == 0x12:
                        if address != 0x05CE or len(request) != 20:
                            raise AssertionError('unexpected write register or length')
                        reply = frame(0x12, address, b'\x01')
                    else:
                        if operation != 0x11 or address != 0x060A or request[7] != 12:
                            raise AssertionError('unexpected read register or length')
                        reply = frame(0x11, address, struct.pack('<6H', *(300 + read_index + j * 10 for j in range(6))))
                        read_index += 1
                    if name == 'no_response':
                        continue
                    if name == 'bad_checksum_only':
                        os.write(master, reply[:-1] + bytes([reply[-1] ^ 1]))
                        continue
                    if name == 'wrong_frame_only':
                        os.write(master, frame(0x11, 0x062E, b'\x00' * 12))
                        continue
                    if name == 'negative_ack':
                        os.write(master, frame(0x12, address, b'\x00'))
                        continue
                    # Simulate measured USB latency on every transaction, then
                    # alternate fragmentation and coalesced unrelated frames.
                    time.sleep(0.016)
                    pattern = (len(requests) - 1) % 4
                    if pattern == 0:
                        os.write(master, reply)
                    elif pattern == 1:
                        for piece in (reply[:1], reply[1:4], reply[4:8], reply[8:]):
                            if piece:
                                os.write(master, piece)
                                time.sleep(0.0006)
                    elif pattern == 2:
                        unrelated = (frame(0x12, 0x05F2, b'\x01') +
                                     frame(0x11, 0x060A, b'\x00' * 12, device=2) +
                                     frame(0x11, 0x062E, b'\x00' * 12))
                        os.write(master, unrelated + reply)
                    else:
                        corrupted = reply[:-1] + bytes([reply[-1] ^ 1])
                        os.write(master, b'\x00\x90\x00garbage' + corrupted + reply)
        except Exception as exc:
            errors.append(repr(exc))

    process = subprocess.Popen([str(binary), os.ttyname(slave), mode, str(cycles)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    worker = threading.Thread(target=emulate, daemon=True)
    worker.start()
    try:
        stdout, stderr = process.communicate(timeout=15)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        stop.set()
        worker.join(timeout=1)
        os.close(master)
        os.close(slave)
    expected = cycles * 2 if mode == 'success' else (0 if mode == 'invalid_target' else cycles)
    passed = process.returncode == 0 and not errors and len(requests) == expected and not worker.is_alive()
    return {'case': name, 'passed': passed, 'request_count': len(requests),
            'expected_requests': expected, 'stdout': stdout.strip(), 'stderr': stderr.strip(),
            'emulator_errors': errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    cases = [('delayed_fragmented_coalesced_100_cycles', 'success', 100),
             ('no_response', 'read_failure', 3),
             ('bad_checksum_only', 'read_failure', 3),
             ('wrong_frame_only', 'read_failure', 3),
             ('negative_ack', 'bad_ack', 3),
             ('invalid_target', 'invalid_target', 1)]
    results = [run_case(args.binary.resolve(), *case) for case in cases]
    report = {'passed': all(item['passed'] for item in results), 'hardware': 'none; PTY emulator only',
              'transaction_deadline_ms': 40, 'simulated_usb_latency_ms': 16, 'cases': results}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
