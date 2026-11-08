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


@dataclass(frozen=True)
class OrderedTowerJudge:
    """The tower the task asked for: ``order`` is the cube bodies bottom to top, ``peg`` (optional) must end
    standing in the top cube's hole. Geometry from true poses only; release and the 3 s dwell are not verified."""

    order: tuple[str, ...]
    peg: str | None = None
    peg_half_m: float = 0.03  # half the stick's length
    cube_m: float = 0.05
    xy_m: float = 0.015
    z_m: float = 0.008
    peg_xy_m: float = 0.012
    max_tilt_rad: float = math.radians(15)

    def _upright(self, pose) -> bool:
        x, y, z, w = pose.orientation
        norm = x * x + y * y + z * z + w * w
        if not math.isfinite(norm) or norm < 1e-12:
            return False
        vertical = max(abs(2 * (x * z - w * y) / norm), abs(2 * (y * z + w * x) / norm), abs(1 - 2 * (x * x + y * y) / norm))
        return vertical >= math.cos(self.max_tilt_rad)

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        names = list(self.order) + ([self.peg] if self.peg else [])
        missing = [n for n in names if n not in after.bodies]
        if missing:
            return Verdict(success=False, stage="perception", note=f"missing bodies: {missing}")
        base = after.bodies[self.order[0]]
        if abs(base.position[2] - self.cube_m / 2) > self.z_m:
            return Verdict(success=False, stage="base", note=f"{self.order[0]} is not on the bench")
        for lower_name, upper_name in zip(self.order, self.order[1:]):
            lower, upper = after.bodies[lower_name], after.bodies[upper_name]
            if not self._upright(upper):
                return Verdict(success=False, stage="place", note=f"{upper_name} is tilted")
            dxy = math.hypot(upper.position[0] - lower.position[0], upper.position[1] - lower.position[1])
            dz = upper.position[2] - lower.position[2]
            if dxy > self.xy_m or abs(dz - self.cube_m) > self.z_m:
                return Verdict(success=False, stage="place",
                               note=f"{upper_name} is not centred one cube above {lower_name} ({dxy * 100:.1f} cm off, {dz * 100:.1f} cm up)")
        if self.peg:
            top, peg = after.bodies[self.order[-1]], after.bodies[self.peg]
            dxy = math.hypot(peg.position[0] - top.position[0], peg.position[1] - top.position[1])
            bottom = peg.position[2] - self.peg_half_m
            top_z = top.position[2]
            if not self._upright(peg):
                return Verdict(success=False, stage="stick", note=f"{self.peg} is not standing")
            if dxy > self.peg_xy_m or not (top_z - self.cube_m / 2 - 0.005 <= bottom <= top_z + self.cube_m / 2):
                return Verdict(success=False, stage="stick",
                               note=f"{self.peg} is not in {self.order[-1]}'s hole ({dxy * 100:.1f} cm off, bottom {bottom * 100:.1f} cm)")
        return Verdict(success=True, stage="done", note="tower in the requested order" + (", stick standing in the top cube" if self.peg else "") + "; release and 3 s stability unverified")
