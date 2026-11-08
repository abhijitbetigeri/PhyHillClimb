"""Geometry checks for the three-cube task; temporal stability is a separate check.

These truth-pose checks do not prove release from the gripper or physical contact.
"""
from dataclasses import dataclass
from collections.abc import Callable
import time
import math

from dimos.autoresearch.judge import Snapshot, Verdict


def valid_pose(pose) -> bool:
    values = (*pose.position, *pose.orientation)
    return all(math.isfinite(v) for v in values) and 1e-6 < math.hypot(*pose.orientation) < math.inf


def vertical_components(pose) -> tuple[float, float, float]:
    scale = math.hypot(*pose.orientation)
    x, y, z, w = (v / scale for v in pose.orientation)
    return (2 * (x*z-w*y), 2 * (y*z+w*x), 1-2*(x*x+y*y))


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
        for name in self.cube_names:
            if not valid_pose(after.bodies[name]):
                return Verdict(success=False, stage="perception", note=f"invalid pose: {name}")
        cubes = sorted(((name, after.bodies[name]) for name in self.cube_names),
                       key=lambda item: item[1].position[2])
        if abs(cubes[0][1].position[2] - self.cube_m / 2) > self.z_m:
            return Verdict(success=False, stage="base", note="bottom cube is not at bench height")
        for name, pose in cubes:
            # A cube may rest on any face, so any local axis may be vertical.
            vertical = max(map(abs, vertical_components(pose)))
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

    def __post_init__(self) -> None:
        if len(self.order) != 3 or len(set(self.order)) != 3 or self.peg in self.order:
            raise ValueError("tower requires three distinct cubes and a separate optional peg")
        if any(not math.isfinite(v) or v <= 0 for v in (
            self.peg_half_m, self.cube_m, self.xy_m, self.z_m, self.peg_xy_m, self.max_tilt_rad
        )) or self.max_tilt_rad >= math.pi / 2:
            raise ValueError("dimensions and tolerances must be finite and positive; tilt must be below pi/2")

    def _upright(self, pose, *, peg: bool = False) -> bool:
        components = vertical_components(pose)
        # A cube may rest on any face; a cylinder must stand on its local Z axis.
        vertical = abs(components[2]) if peg else max(map(abs, components))
        return vertical >= math.cos(self.max_tilt_rad)

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        names = list(self.order) + ([self.peg] if self.peg else [])
        missing = [n for n in names if n not in after.bodies]
        if missing:
            return Verdict(success=False, stage="perception", note=f"missing bodies: {missing}")
        for name in names:
            if not valid_pose(after.bodies[name]):
                return Verdict(success=False, stage="perception", note=f"invalid pose: {name}")
        for name in self.order:
            if not self._upright(after.bodies[name]):
                return Verdict(success=False, stage="place", note=f"{name} is tilted")
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
            if not self._upright(peg, peg=True):
                return Verdict(success=False, stage="stick", note=f"{self.peg} is not standing")
            if dxy > self.peg_xy_m or not (top_z - self.cube_m / 2 - 0.005 <= bottom <= top_z + self.cube_m / 2):
                return Verdict(success=False, stage="stick",
                               note=f"{self.peg} is not in {self.order[-1]}'s hole ({dxy * 100:.1f} cm off, bottom {bottom * 100:.1f} cm)")
        return Verdict(success=True, stage="done", note="tower in the requested order" + (", stick standing in the top cube" if self.peg else "") + "; release and 3 s stability unverified")


@dataclass
class StableTowerJudge:
    """Verify a released tower over a wall-clock dwell using fresh sim observations.

    ``observe`` must wait for a fresh physics pose and return (snapshot, tool xyz).
    The tool clearance sphere conservatively excludes gripper support. This does
    not certify contact with other arm links, foam behaviour, or simulator time.
    """

    geometry: OrderedTowerJudge
    observe: Callable
    dwell_s: float = 3.0
    max_gap_s: float = 1.0
    drift_m: float = 0.003
    rotation_rad: float = math.radians(5)
    gripper_radius_m: float = 0.12  # exceeds the sim's 9.2 cm fingertip reach
    clock: Callable = time.monotonic
    wait: Callable = time.sleep
    record: Callable = lambda event: None

    def __post_init__(self) -> None:
        if any(not math.isfinite(v) or v <= 0 for v in (
            self.dwell_s, self.max_gap_s, self.drift_m, self.rotation_rad, self.gripper_radius_m
        )):
            raise ValueError("verification durations, clearance and tolerances must be finite and positive")

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        self.record({"event": "tower_verification_start", "time": self.clock()})

        def finish(verdict):
            self.record({"event": "tower_verdict", "time": self.clock(), **verdict.model_dump()})
            return verdict

        initial = self.geometry.verdict(before, after)
        if not initial.success:
            return finish(initial)
        names = (*self.geometry.order, *((self.geometry.peg,) if self.geometry.peg else ()))
        reference = None
        started = previous = None
        while True:
            snapshot, tool = self.observe()
            now = self.clock()
            verdict = self.geometry.verdict(before, snapshot)
            if not verdict.success:
                return finish(Verdict(success=False, stage="stability", note=verdict.note))
            if previous is not None and now - previous > self.max_gap_s:
                return finish(Verdict(success=False, stage="perception", note="gap in fresh tower observations"))
            if len(tool) != 3 or not all(math.isfinite(v) for v in tool):
                return finish(Verdict(success=False, stage="perception", note="invalid tool position"))
            self.record({"event": "tower_sample", "time": now,
                         "snapshot": snapshot.model_dump(), "tool_xyz": tool})
            for name in names:
                pose = snapshot.bodies[name]
                radius = (self.geometry.peg_half_m**2 + 0.01**2)**0.5 if name == self.geometry.peg else math.sqrt(3)*self.geometry.cube_m/2
                if math.dist(tool, pose.position) <= self.gripper_radius_m + radius:
                    return finish(Verdict(success=False, stage="release", note=f"gripper not clear of {name}"))
                if reference is not None:
                    old = reference.bodies[name]
                    a, b = old.orientation, pose.orientation
                    an, bn = math.hypot(*a), math.hypot(*b)
                    dot = abs(sum((x/an)*(y/bn) for x, y in zip(a, b)))
                    angle = 2 * math.acos(min(1.0, dot))
                    if math.dist(old.position, pose.position) > self.drift_m or angle > self.rotation_rad:
                        return finish(Verdict(success=False, stage="stability", note=f"{name} moved during dwell"))
            if reference is None:
                reference, started = snapshot, now
            if now - started >= self.dwell_s:
                return finish(Verdict(success=True, stage="done", note="tower geometry, gripper clearance and 3 s observed stability verified"))
            previous = now
            self.wait(0.1)


# --- Stage 1: the cable set (sim/cable_bench.xml, sim/lab_cable.py) ---------------------------------------------
# A plug is a free body whose origin is the centre of its blade; its socket is a slot in the hub body. ``CableSet``
# says where each plug sits when fully inserted, in the hub's own frame, so the judges follow the hub wherever a
# trial's reset placed it. The slot axis is the hub's +z (the hub lies flat), an unplug is a pull along it.


def moved(before, after) -> float:
    """How far a body's centre travelled between two poses, metres."""
    return math.dist(before.position, after.position)


def yaw_of(pose) -> float:
    """Rotation about world z of a pose's xyzw quaternion, radians."""
    x, y, z, w = pose.orientation
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class CableSet:
    """The hub body and, for each plug, where its origin sits when fully inserted, relative to the hub origin in
    the hub frame (metres). ``seated`` turns that into a world position for the hub's current pose."""

    hub: str
    sockets: tuple[tuple[str, tuple[float, float, float]], ...]

    def socket(self, plug: str) -> tuple[float, float, float]:
        for name, offset in self.sockets:
            if name == plug:
                return offset
        raise KeyError(f"{plug} has no socket in the cable set")

    def seated(self, hub_pose, plug: str) -> tuple[float, float, float]:
        ox, oy, oz = self.socket(plug)
        yaw = yaw_of(hub_pose)
        hx, hy, hz = hub_pose.position
        return (hx + ox * math.cos(yaw) - oy * math.sin(yaw), hy + ox * math.sin(yaw) + oy * math.cos(yaw), hz + oz)


def _hub_displaced(cables: CableSet, before: Snapshot, after: Snapshot, limit_m: float) -> Verdict | None:
    if cables.hub not in after.bodies or cables.hub not in before.bodies:
        return Verdict(success=False, stage="perception", note=f"missing body: {cables.hub}")
    shift = math.dist(before.bodies[cables.hub].position, after.bodies[cables.hub].position)
    if shift > limit_m:
        return Verdict(success=False, stage="hub", note=f"the hub was displaced {shift * 100:.1f} cm")
    return None


@dataclass(frozen=True)
class UnplugJudge:
    """Success when at least ``min_count`` of ``plugs`` are out of their sockets: at least ``out_m`` up along the
    slot axis (held in the air counts), or clear of the slot (more than ``clear_xy_m`` off its axis) and still on
    the bench. A plug below the bench top fails the trial (it fell on the floor), and so does a hub that moved
    more than ``hub_move_m``. Truth poses only: the grasp itself is not inspected."""

    cables: CableSet
    plugs: tuple[str, ...]
    out_m: float = 0.04
    clear_xy_m: float = 0.02
    min_count: int = 1
    hub_move_m: float = 0.01
    bench_z: float = 0.0

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        missing = [n for n in self.plugs if n not in after.bodies or n not in before.bodies]
        if missing:
            return Verdict(success=False, stage="perception", note=f"missing bodies: {missing}")
        hub = _hub_displaced(self.cables, before, after, self.hub_move_m)
        if hub is not None:
            return hub
        out, partial, touched = [], [], []
        for name in self.plugs:
            start, end = before.bodies[name], after.bodies[name]
            if end.position[2] < self.bench_z - 0.02:
                return Verdict(success=False, stage="drop", note=f"{name} fell off the bench")
            sx, sy, sz = self.cables.seated(after.bodies[self.cables.hub], name)
            dz = end.position[2] - sz
            lateral = math.hypot(end.position[0] - sx, end.position[1] - sy)
            if dz >= self.out_m:
                out.append(f"{name} is {dz * 100:.1f} cm out of its socket")
            elif lateral > self.clear_xy_m:
                out.append(f"{name} is clear of its socket, {lateral * 100:.1f} cm away")
            elif moved(start, end) >= 0.01:
                partial.append((dz, name))
            elif moved(start, end) >= 0.003:
                touched.append(name)
        if len(out) >= self.min_count:
            return Verdict(success=True, stage="done", note="; ".join(out))
        if partial:
            dz, name = max(partial)
            return Verdict(success=False, stage="pull", note=f"{name} rose only {dz * 100:.1f} cm of {self.out_m * 100:.0f} cm")
        if touched:
            return Verdict(success=False, stage="grasp", note=f"{', '.join(touched)} moved but stayed in the socket")
        return Verdict(success=False, stage="reach", note="no plug was touched")


@dataclass(frozen=True)
class PlugJudge:
    """Success when ``plug`` is fully inserted in its socket: within ``xy_m`` of the socket axis, within ``z_m`` of
    full insertion depth and its slot axis (body axis ``axis``: 0 = x, 2 = z) within ``max_tilt_rad`` of vertical,
    with the hub not displaced. Truth poses only: contact and release are not inspected."""

    cables: CableSet
    plug: str
    xy_m: float = 0.004
    z_m: float = 0.003
    max_tilt_rad: float = math.radians(10)
    hub_move_m: float = 0.01
    axis: int = 2

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        if self.plug not in after.bodies or self.plug not in before.bodies:
            return Verdict(success=False, stage="perception", note=f"missing body: {self.plug}")
        hub = _hub_displaced(self.cables, before, after, self.hub_move_m)
        if hub is not None:
            return hub
        start, end = before.bodies[self.plug], after.bodies[self.plug]
        if not valid_pose(end):
            return Verdict(success=False, stage="perception", note=f"invalid pose: {self.plug}")
        sx, sy, sz = self.cables.seated(after.bodies[self.cables.hub], self.plug)
        lateral = math.hypot(end.position[0] - sx, end.position[1] - sy)
        dz = end.position[2] - sz
        upright = vertical_components(end)[self.axis] >= math.cos(self.max_tilt_rad)
        if lateral <= self.xy_m and abs(dz) <= self.z_m and upright:
            return Verdict(success=True, stage="done", note=f"{self.plug} is seated ({lateral * 1000:.1f} mm off axis, {dz * 1000:+.1f} mm depth)")
        if lateral > 0.03:
            if moved(start, end) < 0.01:
                return Verdict(success=False, stage="reach", note=f"{self.plug} was not moved")
            return Verdict(success=False, stage="carry", note=f"{self.plug} is {lateral * 100:.1f} cm from the socket axis")
        if not upright:
            return Verdict(success=False, stage="insert", note=f"{self.plug} is at the socket but tilted ({lateral * 1000:.0f} mm off axis, {dz * 1000:+.0f} mm)")
        return Verdict(success=False, stage="insert", note=f"{self.plug} is {lateral * 1000:.0f} mm off the socket axis and {dz * 1000:+.0f} mm from full insertion")


@dataclass(frozen=True)
class UnplugPlugJudge:
    """An unplug followed by a re-plug of the same plug: ``peak_z(name)`` is the highest z the world saw for a body
    since the trial's reset (``-inf`` if none), so a plug that never rose ``out_m`` above its start fails at stage
    ``unplug``; otherwise the best-lifted plug must pass ``PlugJudge`` at the end."""

    cables: CableSet
    plugs: tuple[str, ...]
    peak_z: Callable[[str], float]
    out_m: float = 0.04
    xy_m: float = 0.004
    z_m: float = 0.003
    max_tilt_rad: float = math.radians(10)
    hub_move_m: float = 0.01

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        missing = [n for n in self.plugs if n not in after.bodies or n not in before.bodies]
        if missing:
            return Verdict(success=False, stage="perception", note=f"missing bodies: {missing}")
        hub = _hub_displaced(self.cables, before, after, self.hub_move_m)
        if hub is not None:
            return hub
        rises = {name: self.peak_z(name) - before.bodies[name].position[2] for name in self.plugs}
        best = max(rises, key=rises.get)
        if not math.isfinite(rises[best]) or rises[best] < self.out_m:
            seen = ", ".join(f"{n} {r * 100:.1f} cm" for n, r in rises.items() if math.isfinite(r))
            return Verdict(success=False, stage="unplug", note=f"no plug rose {self.out_m * 100:.0f} cm during the trial ({seen or 'no poses'})")
        verdict = PlugJudge(self.cables, best, self.xy_m, self.z_m, self.max_tilt_rad, self.hub_move_m).verdict(before, after)
        return Verdict(success=verdict.success, stage=verdict.stage,
                       note=f"{best} rose {rises[best] * 100:.1f} cm, then: {verdict.note}")
