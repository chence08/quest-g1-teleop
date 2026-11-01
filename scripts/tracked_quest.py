"""Instrument the pinned TeleVuer without changing upstream files.

Receipt/change ages are conservative gates, not sensor capture timestamps.
Identical cached poses expire, and a genuinely stationary hand can also expire.
"""
from multiprocessing import Array, RLock
import time
import numpy as np
from televuer.televuer import TeleVuer
import televuer.tv_wrapper as wrapper_module


def valid_hand(value):
    try:
        raw = np.asarray(value, dtype=float)
        if raw.shape != (400,) or not np.isfinite(raw).all():
            return False
        matrices = raw.reshape(25, 4, 4).transpose(0, 2, 1)
        rotations = matrices[:, :3, :3]
        if not np.allclose(matrices[:, 3, :], [0, 0, 0, 1], atol=1e-3):
            return False
        if not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=0.03):
            return False
        if not np.allclose(np.linalg.det(rotations), 1.0, atol=0.03):
            return False
        spread = np.linalg.norm(matrices[:, :3, 3] - matrices[0, :3, 3], axis=1).max()
        return bool(0.02 < spread < 0.4)
    except (ValueError, TypeError):
        return False


def valid_hand_state(value):
    """Validate every conversion used by the pinned upstream hand handler."""
    if not isinstance(value, dict):
        return False
    try:
        # Missing members use the same defaults as TeleVuer.extract_hands.
        bool(value.get('pinch', False))
        bool(value.get('squeeze', False))
        return all(np.isfinite(float(value.get(key, 0.0)))
                   for key in ('pinchValue', 'squeezeValue'))
    except (ValueError, TypeError, OverflowError):
        return False


class HandSampleTracker:
    def __init__(self):
        self.lock = RLock()
        self.values = Array('d', 8, lock=False)
        self.last = {}

    def record(self, left, right, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            for index, (side, raw) in enumerate((('left', left), ('right', right))):
                offset = index * 4
                if not valid_hand(raw):
                    self.values[offset] = 0
                    self.last.pop(side, None)
                    continue
                raw = np.asarray(raw, dtype=float)
                changed = side not in self.last or np.max(np.abs(raw - self.last[side])) > 1e-7
                self.values[offset] = 1
                self.values[offset + 1] = now
                if changed:
                    self.values[offset + 2] = now
                    self.last[side] = raw.copy()
                self.values[offset + 3] += 1

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            result = {}
            for index, side in enumerate(('left', 'right')):
                offset = index * 4
                valid, received, changed, seq = self.values[offset:offset + 4]
                result[side] = {'valid': bool(valid),
                                'event_age': max(0., now - received) if received else float('inf'),
                                'change_age': max(0., now - changed) if changed else float('inf'),
                                'seq': int(seq)}
            return result


class TrackedTeleVuer(TeleVuer):
    def __init__(self, *args, **kwargs):
        # Must exist before TeleVuer forks its server process.
        self.hand_tracker = HandSampleTracker()
        super().__init__(*args, **kwargs)

    async def on_hand_move(self, event, session, fps=60):
        value = event.value if isinstance(event.value, dict) else {}
        left, right = value.get('left'), value.get('right')
        with self.hand_tracker.lock:
            if not (valid_hand(left) and valid_hand(right)
                    and valid_hand_state(value.get('leftState'))
                    and valid_hand_state(value.get('rightState'))):
                self.hand_tracker.record(None, None)
                return
            await super().on_hand_move(event, session, fps)
            self.hand_tracker.record(left, right)


class TrackedWrapper:
    def __init__(self, wrapped):
        self.wrapped = wrapped
        self.tvuer = wrapped.tvuer

    def get_tracked_data(self):
        with self.tvuer.hand_tracker.lock:
            return self.wrapped.get_tele_data(), self.tvuer.hand_tracker.snapshot()

    def close(self):
        self.wrapped.close()


def create_tracked_wrapper(**kwargs):
    original = wrapper_module.TeleVuer
    wrapper_module.TeleVuer = TrackedTeleVuer
    try:
        return TrackedWrapper(wrapper_module.TeleVuerWrapper(**kwargs))
    finally:
        wrapper_module.TeleVuer = original
