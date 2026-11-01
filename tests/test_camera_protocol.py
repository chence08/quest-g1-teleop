"""Real ZeroMQ/pinned-client integration with synthetic frames, no camera device."""
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from g1_camera_server import camera_config, parse_args, serve


def free_port():
    with socket.socket() as candidate:
        candidate.bind(('127.0.0.1', 0))
        return candidate.getsockname()[1]


class SyntheticCapture:
    released = False

    def __init__(self, *_):
        self.frame = np.full((480, 640, 3), [20, 100, 220], dtype=np.uint8)

    def isOpened(self):
        return True

    def set(self, *_):
        return True

    def get(self, *_):
        return 0.

    def read(self):
        return True, self.frame.copy()

    def release(self):
        SyntheticCapture.released = True


class CameraProtocolTests(unittest.TestCase):
    def test_monocular_config_and_disabled_wrists(self):
        config = camera_config(parse_args(['--device', '/dev/synthetic']))
        self.assertFalse(config['head_camera']['binocular'])
        self.assertEqual(config['head_camera']['image_shape'], [480, 640])
        for key in ('left_wrist_camera', 'right_wrist_camera'):
            self.assertFalse(config[key]['enable_zmq'])
            self.assertFalse(config[key]['enable_webrtc'])

    def test_pinned_client_receives_jpeg_and_detects_stream_stop(self):
        import teleimager.image_client as image_client
        ports = {free_port(), free_port()}
        while len(ports) < 2:
            ports.add(free_port())
        image_port, config_port = sorted(ports)
        args = parse_args(['--device', '/dev/synthetic', '--image-port', str(image_port), '--config-port', str(config_port), '--duration', '3'])
        stop = threading.Event()
        outcome = []
        worker = threading.Thread(target=lambda: outcome.append(serve(args, stop, SyntheticCapture)))
        client = None
        original = image_client.CONFIG_CLIENT_PATH
        try:
            with tempfile.TemporaryDirectory() as temp:
                image_client.CONFIG_CLIENT_PATH = str(Path(temp) / 'client.yaml')
                worker.start()
                time.sleep(0.15)
                client = image_client.ImageClient(host='127.0.0.1', request_port=config_port, request_bgr=True)
                self.assertEqual(client.get_cam_config()['head_camera']['zmq_port'], image_port)
                deadline = time.monotonic() + 2
                frame = None
                while time.monotonic() < deadline:
                    frame = client.get_head_frame()
                    if frame and frame.bgr is not None:
                        break
                    time.sleep(0.02)
                self.assertTrue(frame)
                self.assertEqual(frame.bgr.shape, (480, 640, 3))
                self.assertTrue(frame.jpg.startswith(b'\xff\xd8'))
                stop.set()
                worker.join(timeout=2)
                self.assertFalse(worker.is_alive())
                time.sleep(0.3)
                self.assertFalse(client.get_head_frame())
        finally:
            stop.set()
            worker.join(timeout=2)
            if client:
                client.close()
            image_client.CONFIG_CLIENT_PATH = original
        self.assertTrue(SyntheticCapture.released)
        self.assertTrue(outcome[0]['completed'])
        self.assertNotIn('error', outcome[0])


if __name__ == '__main__':
    unittest.main()
