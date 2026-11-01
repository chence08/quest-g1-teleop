#!/usr/bin/env python3
"""Bounded RGB-only camera server compatible with pinned TeleImager ImageClient.

Uses existing OpenCV/NumPy/ZeroMQ; does not load TeleImager, RealSense pipelines,
robot SDKs, or motor controllers. The explicit RGB interface is separate from
the already running depth bridge. Ports bind loopback for SSH forwarding.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import threading
import time


def camera_config(args):
    head = {'enable_zmq': True, 'zmq_port': args.image_port,
            'enable_webrtc': False, 'webrtc_port': 60001, 'type': 'opencv',
            'image_shape': [args.height, args.width], 'binocular': False,
            'fps': args.fps, 'video_id': None, 'serial_number': None,
            'physical_path': args.device}
    return {'head_camera': head,
            'left_wrist_camera': {'enable_zmq': False, 'enable_webrtc': False},
            'right_wrist_camera': {'enable_zmq': False, 'enable_webrtc': False}}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, help='explicit /dev/ RGB device; no camera is selected automatically')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--quality', type=int, default=75)
    parser.add_argument('--duration', type=float, default=30., help='capture seconds; at most 3600')
    parser.add_argument('--image-port', type=int, default=55555)
    parser.add_argument('--config-port', type=int, default=60000)
    parser.add_argument('--snapshot', type=Path, help='save first valid RGB frame as a JPEG')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error('--duration must be in (0, 3600]')
    if not 1 <= args.fps <= 30 or not 1 <= args.quality <= 95:
        parser.error('fps must be 1..30 and quality must be 1..95')
    if not 1 <= args.width <= 1920 or not 1 <= args.height <= 1080:
        parser.error('unsupported image size')
    if any(not 1024 <= p <= 65535 for p in (args.image_port, args.config_port)):
        parser.error('ports must be in 1024..65535')
    if args.image_port == args.config_port:
        parser.error('image and config ports must differ')
    if not args.device.startswith('/dev/'):
        parser.error('--device must be an explicit /dev/ video device path')
    return args


def serve(args, stop, capture_factory=None):
    # Imports remain lazy so configuration/protocol tests do not open devices.
    import cv2
    import zmq
    report = {'started_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'RGB camera capture only; no robot or depth commands',
              'device': args.device, 'requested': {'width': args.width, 'height': args.height, 'fps': args.fps},
              'frames': 0, 'config_requests': 0, 'completed': False}
    context = zmq.Context()
    images = context.socket(zmq.PUB)
    images.setsockopt(zmq.SNDHWM, 1)
    images.setsockopt(zmq.LINGER, 0)
    config = context.socket(zmq.REP)
    config.setsockopt(zmq.LINGER, 0)
    camera = None
    started = time.monotonic()
    try:
        images.bind(f'tcp://127.0.0.1:{args.image_port}')
        config.bind(f'tcp://127.0.0.1:{args.config_port}')
        camera = (capture_factory or cv2.VideoCapture)(args.device, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError(f'Cannot open the explicit RGB device {args.device}')
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        camera.set(cv2.CAP_PROP_FPS, args.fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        report['driver_reported'] = {'width': camera.get(cv2.CAP_PROP_FRAME_WIDTH),
                                     'height': camera.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                     'fps': camera.get(cv2.CAP_PROP_FPS)}
        print(json.dumps({'event': 'camera_opened', **report['driver_reported']}), flush=True)
        config_value = camera_config(args)
        failures = 0
        last_log = started
        while not stop.is_set() and time.monotonic() - started < args.duration:
            tick = time.monotonic()
            if config.poll(0):
                request = config.recv()
                config.send_json(config_value if request == b'GET_DATA' else {'error': 'expected GET_DATA'})
                report['config_requests'] += 1
            ok, frame = camera.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 5:
                    raise RuntimeError('Five consecutive RGB frame reads failed')
                stop.wait(0.02)
                continue
            failures = 0
            if frame.shape != (args.height, args.width, 3):
                raise RuntimeError(f'RGB frame shape {frame.shape} does not match advertised shape')
            encoded_ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            if not encoded_ok:
                raise RuntimeError('JPEG encoding failed')
            payload = encoded.tobytes()
            try:
                images.send(payload, flags=zmq.DONTWAIT)
            except zmq.Again:
                report['send_drops'] = report.get('send_drops', 0) + 1
            report['frames'] += 1
            report['last_jpeg_bytes'] = len(payload)
            if report['frames'] == 1:
                report['first_frame_utc'] = datetime.now(timezone.utc).isoformat()
                report['first_frame_shape'] = list(frame.shape)
                if args.snapshot:
                    args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                    args.snapshot.write_bytes(payload)
            if time.monotonic() - last_log >= 5:
                print(json.dumps({'event': 'streaming', 'frames': report['frames'],
                                  'average_fps': report['frames'] / (time.monotonic() - started)}), flush=True)
                last_log = time.monotonic()
            stop.wait(max(0., 1. / args.fps - (time.monotonic() - tick)))
        report['completed'] = report['frames'] > 0
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if camera is not None:
            camera.release()
        config.close()
        images.close()
        context.term()
        report['ended_utc'] = datetime.now(timezone.utc).isoformat()
        report['elapsed_s'] = time.monotonic() - started
        report['average_fps'] = report['frames'] / max(report['elapsed_s'], 1e-9)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report), flush=True)
    return report


def main(argv=None):
    args = parse_args(argv)
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    result = serve(args, stop)
    return 0 if result['completed'] and 'error' not in result else 1


if __name__ == '__main__':
    raise SystemExit(main())
