import math
import unittest

from dimos.autoresearch.judge import BodyPose, Snapshot
from sim.judges import ThreeCubeJudge


def scene(heights=(0.025, 0.075, 0.125), xs=(0.3, 0.3, 0.3)):
    return Snapshot(bodies={name: BodyPose(position=(x, 0.0, z), orientation=(0, 0, 0, 1))
                           for name, x, z in zip(ThreeCubeJudge().cube_names, xs, heights)})


class StackJudgeTests(unittest.TestCase):
    def test_three_layers(self):
        self.assertTrue(ThreeCubeJudge().verdict(scene(), scene()).success)

    def test_two_layers_fail(self):
        self.assertFalse(ThreeCubeJudge().verdict(scene(), scene((.025, .075, .025))).success)

    def test_floating_tower_fails(self):
        self.assertFalse(ThreeCubeJudge().verdict(scene(), scene((.125, .175, .225))).success)

    def test_offset_top_fails(self):
        self.assertFalse(ThreeCubeJudge().verdict(scene(), scene(xs=(.3, .3, .33))).success)

    def test_tilted_top_fails(self):
        after = scene()
        after.bodies['cube_yellow'].orientation = (math.sin(math.pi/8), 0, 0, math.cos(math.pi/8))
        self.assertFalse(ThreeCubeJudge().verdict(scene(), after).success)

    def test_cube_on_another_face_passes(self):
        after = scene()
        after.bodies['cube_yellow'].orientation = (math.sqrt(.5), 0, 0, math.sqrt(.5))
        self.assertTrue(ThreeCubeJudge().verdict(scene(), after).success)

    def test_missing_cube_fails(self):
        after = scene()
        del after.bodies['cube_yellow']
        self.assertFalse(ThreeCubeJudge().verdict(scene(), after).success)


if __name__ == '__main__':
    unittest.main()
