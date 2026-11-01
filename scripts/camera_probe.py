#!/usr/bin/env python3
"""Read camera inventory and Python availability without opening a video device.

Run on the robot, or pipe this file to its Python interpreter over SSH. This does
not import native camera libraries, start capture, install packages, or change
device permissions. Missing entries are evidence gaps, not camera failures.
"""
import argparse
from datetime import datetime, timezone
import glob
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, UnicodeError):
        return None


def command(argv, timeout=5):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return {"argv": argv, "returncode": result.returncode,
                "stdout": result.stdout[-24000:], "stderr": result.stderr[-4000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"argv": argv, "error": str(exc)}


def video_inventory():
    devices = []
    for item in sorted(Path('/sys/class/video4linux').glob('video*')):
        device = Path('/dev') / item.name
        resolved = item.resolve()
        usb = {}
        for ancestor in [resolved] + list(resolved.parents):
            if (ancestor / 'idVendor').is_file():
                usb = {key: read_text(ancestor / key) for key in
                       ('idVendor', 'idProduct', 'manufacturer', 'product', 'serial', 'speed')}
                usb['sysfs_path'] = str(ancestor)
                break
        devices.append({"device": str(device), "name": read_text(item / 'name'),
                        "index": read_text(item / 'index'), "sysfs_path": str(resolved),
                        "readable": os.access(device, os.R_OK),
                        "writable": os.access(device, os.W_OK), "usb": usb})
    return devices


RUNTIME_PROBE = r'''
import importlib.util, importlib.metadata, json, sys
modules = ['teleimager', 'pyrealsense2', 'cv2', 'uvc', 'zmq', 'aiohttp', 'aiortc', 'av', 'numpy', 'yaml', 'logging_mp']
found = {}
for module in modules:
    try:
        spec = importlib.util.find_spec(module)
        found[module] = {'available': spec is not None, 'origin': spec.origin if spec else None}
    except Exception as exc:
        found[module] = {'available': False, 'error': str(exc)}
versions = {}
for package in ['teleimager','pyrealsense2','opencv-python','pupil-labs-uvc','pyzmq','aiohttp','aiortc','av','numpy','PyYAML','logging-mp']:
    try:
        versions[package] = importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
        pass
print(json.dumps({'executable':sys.executable,'python':sys.version,'modules':found,'packages':versions}))
'''


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', action='append', dest='pythons', help='runtime to inspect; repeatable')
    parser.add_argument('--report', type=Path, help='optional JSON output file')
    args = parser.parse_args(argv)
    if args.pythons is None:
        candidates = [sys.executable] + glob.glob(str(Path.home() / 'miniconda3/envs/*/bin/python'))
        args.pythons = list(dict.fromkeys(candidates))[:8]
    devices = video_inventory()
    report = {'checked_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'Read-only metadata; no video device opened and no camera capture started',
              'host': platform.node(), 'platform': platform.platform(),
              'os_release': read_text('/etc/os-release'),
              'jetson_release': read_text('/etc/nv_tegra_release'),
              'video_devices': devices, 'runtimes': []}
    for executable in args.pythons:
        record = command([executable, '-c', RUNTIME_PROBE])
        if record.get('returncode') == 0:
            try:
                record['result'] = json.loads(record.pop('stdout'))
            except (ValueError, KeyError):
                pass
        report['runtimes'].append(record)
    if shutil.which('lsusb'):
        report['usb_inventory'] = command(['lsusb'])
    if devices and shutil.which('fuser'):
        report['video_owners'] = command(['fuser', '-v'] + [d['device'] for d in devices])
    text = json.dumps(report, indent=2)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
