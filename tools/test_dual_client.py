"""Exercise prompt isolation, simultaneous calls, arm routing, and paired failure handling."""
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np
from robots import bodies
from dual_client import DualClient, PROMPTS


class FakeClient:
    def __init__(self, side, gate):
        self.side, self.gate = side, gate
        self.session_id = side + "-session"
        self.last_timing = {}
        self.observation = self.commanded = None
        self.closed = False
        self.reset = 0
        self.failure = None
        self.shape = (48, 7)
        self.finished = False

    def set_observation(self, frames, observation, *, timestamp):
        self.observation = dict(observation)
        self.timestamp = timestamp

    def get_action(self, *, commanded):
        self.commanded = commanded
        self.gate.wait(timeout=2)
        if self.side == "right":
            time.sleep(0.03)
        self.finished = True
        if self.failure:
            raise self.failure
        return {"q_l_chunk": np.full(self.shape, 0.1 if self.side == "left" else 0.9),
                "q_r_chunk": np.full(self.shape, 0.2 if self.side == "right" else 0.8)}

    def reset_episode(self):
        self.reset += 1

    def close(self):
        self.closed = True


class DualTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        gate = threading.Barrier(2)
        self.clients = {s: FakeClient(s, gate) for s in ("left", "right")}
        wire = bodies.METAL.wire().for_arms({"left": 6, "right": 6})
        self.dual = DualClient(self.clients, {"action_hz": 20, "sync_above_ms": 900},
                               Path(self.temp.name), wire, 48)

    def tearDown(self):
        self.dual.close()
        self.temp.cleanup()

    def test_separate_prompts_keep_same_scene_and_timestamp(self):
        obs = {"prompt": "shared placeholder", "state.left_arm": np.zeros(6)}
        self.dual.set_observation({"video.overhead": "frame"}, obs, timestamp=1.5)
        for side, client in self.clients.items():
            self.assertEqual(client.observation["prompt"], PROMPTS[side])
            self.assertEqual(client.timestamp, 1.5)
        self.assertEqual(obs["prompt"], "shared placeholder")
        self.assertNotEqual(PROMPTS["left"], PROMPTS["right"])

    def test_simultaneous_requests_select_only_assigned_arm(self):
        anchor = {"q_l_chunk": np.zeros(6), "q_r_chunk": np.zeros(6)}
        reply = self.dual.get_action(commanded=anchor)
        np.testing.assert_array_equal(reply["q_l_chunk"], np.full((48, 7), 0.1))
        np.testing.assert_array_equal(reply["q_r_chunk"], np.full((48, 7), 0.2))
        for client in self.clients.values():
            self.assertIs(client.commanded, anchor)
        events = self.dual.events
        self.assertLess(max(e["started_monotonic_s"] for e in events),
                        min(e["finished_monotonic_s"] for e in events))
        self.assertEqual(self.dual.stats()["api_calls_by_arm"], {"left": 1, "right": 1})

    def test_one_side_failure_drains_other_and_returns_no_pair(self):
        self.clients["left"].failure = RuntimeError("left API failed")
        with self.assertRaisesRegex(RuntimeError, "left API failed"):
            self.dual.get_action()
        self.assertTrue(self.clients["right"].finished)
        self.assertFalse(list(Path(self.temp.name).glob("selected-*.npz")))
        self.dual.reset_episode()
        self.assertTrue(all(c.reset == 1 for c in self.clients.values()))

    def test_malformed_selected_chunk_is_refused(self):
        self.clients["right"].shape = (47, 7)
        with self.assertRaises(Exception):
            self.dual.get_action()
        self.assertFalse(list(Path(self.temp.name).glob("selected-*.npz")))

    def test_close_releases_both_sessions_and_is_idempotent(self):
        self.dual.close()
        self.dual.close()
        self.assertTrue(all(c.closed for c in self.clients.values()))
        with self.assertRaisesRegex(RuntimeError, "closed"):
            self.dual.get_action()


if __name__ == "__main__":
    unittest.main()
