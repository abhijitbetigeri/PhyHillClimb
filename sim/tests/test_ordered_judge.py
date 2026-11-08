import math
import unittest

from dimos.autoresearch.judge import BodyPose, Snapshot
from sim.judges import OrderedTowerJudge, StableTowerJudge

ORDER = ('cube_yellow', 'cube_red', 'cube_blue')


def tower():
    positions = dict(zip(ORDER, ((.26, 0, .025), (.26, 0, .075), (.26, 0, .125))))
    positions['peg_red'] = (.26, 0, .155)
    return Snapshot(bodies={n: BodyPose(position=p, orientation=(0, 0, 0, 1)) for n, p in positions.items()})


class OrderedJudgeTests(unittest.TestCase):
    def setUp(self):
        self.judge = OrderedTowerJudge(ORDER, 'peg_red')

    def test_valid_tower(self):
        self.assertTrue(self.judge.verdict(tower(), tower()).success)

    def test_horizontal_stick_fails(self):
        s = tower()
        s.bodies['peg_red'].orientation = (math.sqrt(.5), 0, 0, math.sqrt(.5))
        self.assertFalse(self.judge.verdict(tower(), s).success)

    def test_tilted_base_fails(self):
        s = tower()
        s.bodies[ORDER[0]].orientation = (math.sin(math.pi/8), 0, 0, math.cos(math.pi/8))
        self.assertFalse(self.judge.verdict(tower(), s).success)

    def test_invalid_telemetry_fails(self):
        for name in (*ORDER, 'peg_red'):
            for bad in (math.nan, math.inf, -math.inf):
                for field in ('position', 'orientation'):
                    with self.subTest(name=name, bad=bad, field=field):
                        s = tower()
                        values = list(getattr(s.bodies[name], field))
                        values[0] = bad
                        setattr(s.bodies[name], field, tuple(values))
                        self.assertFalse(self.judge.verdict(tower(), s).success)

    def test_zero_quaternion_fails(self):
        s = tower()
        s.bodies['peg_red'].orientation = (0, 0, 0, 0)
        self.assertFalse(self.judge.verdict(tower(), s).success)

    def test_wrong_order_fails(self):
        s = tower()
        s.bodies[ORDER[0]], s.bodies[ORDER[1]] = s.bodies[ORDER[1]], s.bodies[ORDER[0]]
        self.assertFalse(self.judge.verdict(tower(), s).success)

    def test_invalid_order_rejected(self):
        for order in ((), ('cube_red',), ('cube_red',)*3):
            with self.assertRaises(ValueError):
                OrderedTowerJudge(order)

    def test_sideways_cube_still_passes(self):
        s = tower()
        s.bodies[ORDER[0]].orientation = (math.sqrt(.5), 0, 0, math.sqrt(.5))
        self.assertTrue(self.judge.verdict(tower(), s).success)


class StabilityTests(unittest.TestCase):
    def run_judge(self, alter=None, tool=(0, 0, .4), interval=.2):
        self.now, self.samples = 0., 0
        def observe():
            self.now += interval
            self.samples += 1
            s = tower()
            if alter:
                alter(s, self.samples)
            return s, tool
        judge = StableTowerJudge(OrderedTowerJudge(ORDER, 'peg_red'), observe,
                                clock=lambda: self.now, wait=lambda seconds: None)
        return judge.verdict(tower(), tower())

    def test_stable_released_tower_passes_after_dwell(self):
        self.assertTrue(self.run_judge().success)
        self.assertGreaterEqual(self.now, 3.2)

    def test_gripper_support_fails(self):
        self.assertEqual(self.run_judge(tool=(.26, 0, .19)).stage, 'release')

    def test_invalid_tool_fails(self):
        self.assertFalse(self.run_judge(tool=(math.nan, 0, .4)).success)

    def test_collapse_mid_dwell_fails(self):
        def collapse(s, i):
            if i == 5:
                s.bodies[ORDER[-1]].position = (.3, 0, .025)
        self.assertFalse(self.run_judge(alter=collapse).success)

    def test_drift_with_valid_geometry_fails(self):
        def drift(s, i):
            if i == 5:
                for pose in s.bodies.values():
                    x, y, z = pose.position
                    pose.position = (x+.005, y, z)
        self.assertFalse(self.run_judge(alter=drift).success)

    def test_observation_gap_fails(self):
        self.assertEqual(self.run_judge(interval=1.1).stage, 'perception')

    def test_signed_quaternion_is_same_rotation(self):
        def flip(s, i):
            if i % 2:
                for pose in s.bodies.values():
                    pose.orientation = (0, 0, 0, -1)
        self.assertTrue(self.run_judge(alter=flip).success)
