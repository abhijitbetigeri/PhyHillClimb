import math
import unittest

from dimos.autoresearch.judge import BodyPose, Snapshot
from sim.judges import CableSet, PlugJudge, UnplugJudge, UnplugPlugJudge

CABLES = CableSet(hub="hub", sockets=(("plug_a", (-0.025, 0.0, 0.0225)), ("plug_b", (0.025, 0.0, 0.0225))))
UP = (0, 0, 0, 1)


def scene(hub=(0.26, 0.0, 0.0), a=(0.235, 0.0, 0.0225), b=(0.285, 0.0, 0.0225), a_q=UP, hub_q=UP):
    return Snapshot(bodies={"hub": BodyPose(position=hub, orientation=hub_q),
                            "plug_a": BodyPose(position=a, orientation=a_q),
                            "plug_b": BodyPose(position=b, orientation=UP)})


class UnplugJudgeTests(unittest.TestCase):
    def setUp(self):
        self.judge = UnplugJudge(CABLES, ("plug_a", "plug_b"))

    def test_untouched_fails_at_reach(self):
        v = self.judge.verdict(scene(), scene())
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "reach")

    def test_lifted_plug_passes(self):
        v = self.judge.verdict(scene(), scene(a=(0.235, 0.0, 0.0225 + 0.05)))
        self.assertTrue(v.success, v.note)

    def test_partial_pull_fails(self):
        v = self.judge.verdict(scene(), scene(a=(0.235, 0.0, 0.0225 + 0.02)))
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "pull")

    def test_dropped_beside_hub_passes(self):
        v = self.judge.verdict(scene(), scene(a=(0.235, 0.10, 0.006)))
        self.assertTrue(v.success, v.note)

    def test_fallen_off_bench_fails(self):
        v = self.judge.verdict(scene(), scene(a=(0.235, 0.40, -0.70)))
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "drop")

    def test_displaced_hub_fails(self):
        v = self.judge.verdict(scene(), scene(hub=(0.28, 0.0, 0.0), a=(0.255, 0.0, 0.08)))
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "hub")

    def test_rotated_hub_sockets_follow(self):
        yaw = 0.3
        q = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
        a = (0.26 - 0.025 * math.cos(yaw), -0.025 * math.sin(yaw), 0.0225)
        before = scene(a=a, hub_q=q, a_q=q)
        self.assertEqual(self.judge.verdict(before, before).stage, "reach")
        after = scene(a=(a[0], a[1], a[2] + 0.045), hub_q=q, a_q=q)
        self.assertTrue(self.judge.verdict(before, after).success)


class PlugJudgeTests(unittest.TestCase):
    def setUp(self):
        self.judge = PlugJudge(CABLES, "plug_a")

    def test_seated_passes(self):
        before = scene(a=(0.235, 0.10, 0.006))
        self.assertTrue(self.judge.verdict(before, scene(a=(0.2372, 0.002, 0.024))).success)

    def test_high_fails_at_insert(self):
        before = scene(a=(0.235, 0.10, 0.006))
        v = self.judge.verdict(before, scene(a=(0.235, 0.0, 0.0225 + 0.01)))
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "insert")

    def test_tilted_fails(self):
        before = scene(a=(0.235, 0.10, 0.006))
        tilt = math.radians(15)
        v = self.judge.verdict(before, scene(a=(0.235, 0.0, 0.0225), a_q=(math.sin(tilt / 2), 0, 0, math.cos(tilt / 2))))
        self.assertFalse(v.success)

    def test_far_away_fails_at_carry(self):
        before = scene(a=(0.235, 0.10, 0.006))
        v = self.judge.verdict(before, scene(a=(0.235, 0.05, 0.006)))
        self.assertEqual(v.stage, "carry")


class CycleJudgeTests(unittest.TestCase):
    def test_needs_a_rise(self):
        judge = UnplugPlugJudge(CABLES, ("plug_a", "plug_b"), peak_z=lambda name: 0.03)
        v = judge.verdict(scene(), scene())
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "unplug")

    def test_rise_then_seated_passes(self):
        judge = UnplugPlugJudge(CABLES, ("plug_a", "plug_b"), peak_z=lambda name: 0.09 if name == "plug_a" else 0.0225)
        self.assertTrue(judge.verdict(scene(), scene(a=(0.236, 0.001, 0.0235))).success)

    def test_rise_then_dropped_fails(self):
        judge = UnplugPlugJudge(CABLES, ("plug_a", "plug_b"), peak_z=lambda name: 0.09)
        v = judge.verdict(scene(), scene(a=(0.235, 0.10, 0.006)))
        self.assertFalse(v.success)
        self.assertEqual(v.stage, "carry")


if __name__ == "__main__":
    unittest.main()
