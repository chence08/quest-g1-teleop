import sys
import asyncio
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from tracked_quest import HandSampleTracker, TrackedTeleVuer, TeleVuer, valid_hand


def sample():
    poses = np.tile(np.eye(4), (25, 1, 1))
    poses[:, 0, 3] = np.linspace(0., 0.15, 25)
    return poses.transpose(0, 2, 1).reshape(-1).tolist()


class TrackingTests(unittest.TestCase):
    def test_valid_and_invalid_geometry(self):
        self.assertTrue(valid_hand(sample()))
        self.assertFalse(valid_hand([0.] * 400))
        self.assertFalse(valid_hand(None))
        bad = sample(); bad[14] = float('nan')
        self.assertFalse(valid_hand(bad))

    def test_cached_frames_do_not_refresh_change_age(self):
        tracker = HandSampleTracker(); hand = sample()
        tracker.record(hand, hand, 10.)
        tracker.record(hand, hand, 11.)
        status = tracker.snapshot(11.1)
        self.assertAlmostEqual(status['left']['event_age'], 0.1)
        self.assertAlmostEqual(status['left']['change_age'], 1.1)
        self.assertEqual(status['right']['seq'], 2)

    def test_invalid_side_disables_it_and_new_sample_recovers(self):
        tracker = HandSampleTracker(); hand = sample()
        tracker.record(hand, hand, 10.)
        tracker.record(None, hand, 11.)
        self.assertFalse(tracker.snapshot(11.)['left']['valid'])
        tracker.record(hand, hand, 12.)
        self.assertTrue(tracker.snapshot(12.)['left']['valid'])
        self.assertEqual(tracker.snapshot(12.)['left']['change_age'], 0.)

    def test_malformed_state_invalidates_both_hands_before_upstream(self):
        hand = sample()
        for side in ('leftState', 'rightState'):
            for bad_state in (None, [], {'pinchValue': 'bad'},
                              {'squeezeValue': float('nan')},
                              {'pinchValue': float('inf')}):
                with self.subTest(side=side, state=bad_state):
                    tracked = TrackedTeleVuer.__new__(TrackedTeleVuer)
                    tracked.hand_tracker = HandSampleTracker()
                    tracked.hand_tracker.record(hand, hand)
                    value = {'left': hand, 'right': hand,
                             'leftState': {}, 'rightState': {}, side: bad_state}
                    with patch.object(TeleVuer, 'on_hand_move', new_callable=AsyncMock) as upstream:
                        asyncio.run(tracked.on_hand_move(SimpleNamespace(value=value), None))
                        upstream.assert_not_awaited()
                    status = tracked.hand_tracker.snapshot()
                    self.assertFalse(status['left']['valid'])
                    self.assertFalse(status['right']['valid'])

    def test_empty_states_use_upstream_defaults(self):
        tracked = TrackedTeleVuer.__new__(TrackedTeleVuer)
        tracked.hand_tracker = HandSampleTracker()
        hand = sample()
        event = SimpleNamespace(value={'left': hand, 'right': hand,
                                       'leftState': {}, 'rightState': {}})
        with patch.object(TeleVuer, 'on_hand_move', new_callable=AsyncMock) as upstream:
            asyncio.run(tracked.on_hand_move(event, None))
            upstream.assert_awaited_once()
        status = tracked.hand_tracker.snapshot()
        self.assertTrue(status['left']['valid'])
        self.assertTrue(status['right']['valid'])


if __name__ == '__main__':
    unittest.main()
