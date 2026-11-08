"""Geometry checks for the three-cube task; temporal stability is a separate check.

These truth-pose checks do not prove release from the gripper or physical contact.
"""
from dataclasses import dataclass
import math

from dimos.autoresearch.judge import Snapshot, Verdict


@dataclass(frozen=True)
class ThreeCubeJudge:
    """Require three upright cubes in a vertical tower, with the base on the bench."""

    cube_names: tuple[str, ...] = ("cube_blue", "cube_red", "cube_yellow")
    cube_m: float = 0.05
    xy_m: float = 0.015
    z_m: float = 0.008
    max_tilt_rad: float = math.radians(15)

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        missing = set(self.cube_names) - after.bodies.keys()
        if missing:
            return Verdict(success=False, stage="perception", note=f"missing bodies: {sorted(missing)}")
        cubes = sorted(((name, after.bodies[name]) for name in self.cube_names),
                       key=lambda item: item[1].position[2])
        if abs(cubes[0][1].position[2] - self.cube_m / 2) > self.z_m:
            return Verdict(success=False, stage="base", note="bottom cube is not at bench height")
        for name, pose in cubes:
            x, y, z, w = pose.orientation
            norm = x*x + y*y + z*z + w*w
            if not math.isfinite(norm) or norm < 1e-12:
                return Verdict(success=False, stage="perception", note=f"invalid orientation: {name}")
            # A cube can rest on any face, so any local axis may be vertical.
            vertical = max(abs(2*(x*z-w*y)/norm), abs(2*(y*z+w*x)/norm),
                           abs(1-2*(x*x+y*y)/norm))
            if vertical < math.cos(self.max_tilt_rad):
                return Verdict(success=False, stage="place", note=f"{name} is tilted")
            if not all(math.isfinite(v) for v in pose.position):
                return Verdict(success=False, stage="perception", note=f"invalid position: {name}")
        for (lower_name, lower), (upper_name, upper) in zip(cubes, cubes[1:]):
            dx, dy = upper.position[0]-lower.position[0], upper.position[1]-lower.position[1]
            dz = upper.position[2]-lower.position[2]
            if math.hypot(dx, dy) > self.xy_m or abs(dz-self.cube_m) > self.z_m:
                return Verdict(success=False, stage="place",
                               note=f"{upper_name} is not centred one cube above {lower_name}")
        return Verdict(success=True, stage="done", note="three-cube tower geometry passes; release and stability unverified")
