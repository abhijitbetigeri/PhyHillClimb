"""Scene scans answered from a YAML file of measured object positions, for the real Metal with no detector.

The file names each object, its box centre in the arm's base frame (``world`` on this stack: the base_link
origin, x forward, y to the arm's left, z up, metres) and its size. The operator edits it between trials and calls
the ``reload`` RPC (``dimos mcp`` is not needed: ``python -m real.scene_ctl reload``); nothing restarts.

A measured scene is static, but the sim-found programs read the scene after every pick and place (did the cube
come up with the jaw, is it seated on the one below, how tall is the tower). So the module also keeps a belief of
where each object is now, from the arm's own telemetry: when the jaw closes by ``close_m`` with the finger pads
within reach of an object, that object rides with the tool; when the jaw opens by ``open_m`` it is let go and
settles on whatever is under it (another object's top or the bench). This is dead reckoning from the arm's
encoders, not perception: a grasp that missed still reads as held. The operator watches and corrects the file.

The same module stands in for the obstacle map the sim stacks built from a depth camera. It publishes
``scene_cloud`` at ``map_hz`` with fresh stamps: the bench plane just below z = 0 (outside a disc around the base),
plus every object not in the jaw as a filled box of points, depending on ``map_mode``. dimos's own
``PointCloudBoxFilter`` takes that cloud, leaves out the boxes a program clears, and feeds the planner's
``voxel_map``; ``scene.clear``/``restore`` return once a map stamped after the change arrives, which the fixed rate
guarantees. Modelled on ``dimos.autoresearch.truth_scene.TruthSceneModule``.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import Any, Literal

from reactivex.disposable import Disposable

import numpy as np
from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
from pydantic import BaseModel, Field
import yaml

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray

FRAME = "world"
DEFAULT_SCENE_FILE = Path(__file__).resolve().parent / "scene.yaml"


class MeasuredObject(BaseModel):
    """One object as the operator measured it: ``xyz`` the box centre (``z`` may be left out: then the object
    stands on the bench, centre half its height up), ``size`` its extents, ``yaw`` about z in radians."""

    xyz: tuple[float, float] | tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0


class SceneFile(BaseModel):
    frame: str = FRAME
    bench_z: float = 0.0
    objects: dict[str, MeasuredObject]


def load_scene_file(path: Path) -> SceneFile:
    data = yaml.safe_load(Path(path).read_text()) or {}
    scene = SceneFile.model_validate(data)
    if scene.frame != FRAME:
        raise ValueError(f"{path}: positions must be in the arm base frame 'world', not {scene.frame!r}")
    for name, obj in scene.objects.items():
        if any(not math.isfinite(v) or v <= 0 for v in obj.size):
            raise ValueError(f"{path}: {name} needs three positive sizes")
        if any(not math.isfinite(v) for v in obj.xyz):
            raise ValueError(f"{path}: {name} has a non-finite position")
    return scene


class MeasuredSceneModuleConfig(ModuleConfig):
    """``scene_file`` is the YAML of measured objects. ``pad_m`` is how far ahead of the planning tip
    (gripper_base, along its local +z) the finger pads sit (METAL_HEURISTIC_AIM). A close attaches the nearest
    object whose centre is within ``grasp_xy_m`` of the pads in x and y and within its half height plus
    ``grasp_z_m`` in z."""

    scene_file: str = str(DEFAULT_SCENE_FILE)
    tool_frame: str = "gripper_base"
    jaw_joint: str = "arm/joint7"
    # joint7, metres: 0 closed, -0.05 fully open (METAL_GRIPPER_RANGE)
    jaw_open_m: float = -0.05
    close_m: float = 0.008
    # a release is the jaw ``open_m`` more open than its most closed reading, for ``release_dwell_s``: a jaw that
    # closes on a cube rebounds a few millimetres, which must not count as letting go
    open_m: float = 0.010
    release_dwell_s: float = 0.25
    # the grasp check from the encoder: a jaw that closed on an object stalls open by the object's width; one that
    # ends within ``empty_jaw_m`` of fully closed (joint7 = 0) for ``empty_dwell_s`` holds nothing, so the object
    # stays where it is (a missed grasp) or drops below the jaw (a slip). This is what lets a program's "did it lift"
    # check fail on a missed grasp, as it did against the simulator's true poses.
    # (sim: joint7 stalls at -17.7 mm on a 5 cm cube). Only for objects at least ``empty_check_min_width_m`` wide:
    # a thin plug housing may let the jaw close almost fully while still holding it.
    empty_check: bool = True  # off on the echo mock, whose jaw closes fully on anything
    empty_jaw_m: float = 0.004
    empty_dwell_s: float = 0.3
    empty_check_min_width_m: float = 0.03
    pad_m: float = 0.078
    grasp_xy_m: float = 0.03
    grasp_z_m: float = 0.03
    track_grasps: bool = True
    # a closing jaw pulls the object onto the pads' axis: True takes the held object as centred between the pads
    # (x, y offset 0), False keeps the offset it had from the pad point when the jaw closed
    centre_on_grasp: bool = True
    # False: the pads are straight below the mount (pad_m down), as the programs assume. True: along the tool's
    # measured local +z. In the sim the measured tilt pointed the pad estimate the wrong way (belief 2-3 cm behind
    # where the cube really landed), so the default is the programs' own assumption.
    use_tool_tilt: bool = False
    # where a released object ends relative to the jaw mount, tool frame (x, y), metres: the opening jaw drags it.
    # The sim's pick check (verify_measured_sim.py --pick-check: three cubes picked and put back) measured
    # +0.8 to +1.5 cm along tool x and -0.4 to +0.05 cm along y; with grippers that wait for the jaw (live_real),
    # v24's own set-downs landed -0.1 to +1.6 cm along x. tower_order_v24 compensates (0.9, -0.3) cm (`drag`).
    # On the real arm measure it in the single-pick test (RUNBOOK step 9) and pass
    # `--measuredscenemodule.release-offset '[dx,dy]'`.
    release_offset: tuple[float, float] = (0.008, -0.002)
    map_mode: Literal["bench+objects", "bench", "objects", "empty"] = "bench+objects"
    map_hz: float = Field(default=4.0, gt=0.0)
    # the bench plane: one voxel below the top so the bench cells end at z = 0
    bench_cloud_dz: float = -0.0125
    bench_x: tuple[float, float] = (-0.15, 0.55)
    bench_y: tuple[float, float] = (-0.45, 0.45)
    base_clearance_m: float = 0.12
    point_spacing_m: float = 0.0125


class _Body:
    """An object's current belief: position (box centre), yaw, size."""

    def __init__(self, name: str, position: tuple[float, float, float], size: tuple[float, float, float], yaw: float):
        self.name = name
        self.position = position
        self.size = size
        self.yaw = yaw


class MeasuredSceneModule(Module):
    """Answers ``scan_scene`` from measured positions plus the grasp belief, and publishes the stand-in map."""

    config: MeasuredSceneModuleConfig

    coordinator_joint_state: In[JointState]
    tf: In[TFMessage]
    scene_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._bodies: dict[str, _Body] = {}
        self._bench_z = 0.0
        self._prompts: list[str] | None = None
        # grasp tracking
        self._jaw_closed: bool | None = None
        self._jaw_extreme = 0.0
        self._opening_since: float | None = None
        self._empty_since: float | None = None
        self._held: str | None = None
        # the held object's offset from the pads in the tool's yaw frame, and its yaw relative to the tool's
        self._held_offset = (0.0, 0.0, 0.0)
        self._held_dyaw = 0.0
        self._attach_position = (0.0, 0.0, 0.0)
        self._attach_pads = (0.0, 0.0, 0.0)
        self._events: list[str] = []
        self._stop_map = threading.Event()
        self._map_thread: threading.Thread | None = None
        self._scene_file = self.config.scene_file
        self._reload_locked()

    # ---- lifecycle ----

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - subscribe to tf now, so the tool pose is known by the first grasp
        super().start()
        if self.config.track_grasps:
            self.register_disposable(Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state)))
        self._stop_map.clear()
        self._map_thread = threading.Thread(target=self._map_loop, daemon=True, name="measured-scene-map")
        self._map_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_map.set()
        if self._map_thread is not None and self._map_thread is not threading.current_thread():
            self._map_thread.join(timeout=2.0)
        super().stop()

    # ---- operator RPCs ----

    @rpc
    def reload(self, path: str | None = None) -> str:
        """Re-read the scene file (or switch to ``path``): every object goes back to its measured position and
        the jaw holds nothing."""
        with self._lock:
            if path:
                load_scene_file(Path(path))  # refuse a bad file before switching
                self._scene_file = str(path)
            self._reload_locked()
            return self.describe()

    @rpc
    def set_object(self, name: str, x: float, y: float, z: float | None = None, yaw: float | None = None) -> str:
        """Put one object at a new believed position (z None: standing on the bench) without editing the file."""
        with self._lock:
            body = self._bodies[name]
            z_val = self._bench_z + body.size[2] / 2 if z is None else float(z)
            body.position = (float(x), float(y), z_val)
            if yaw is not None:
                body.yaw = float(yaw)
            if self._held == name:
                self._held = None
            self._events.append(f"{time.time():.2f} set {name} to ({x:.3f}, {y:.3f}, {z_val:.3f})")
            return self.describe()

    @rpc
    def describe(self) -> str:
        """Every object's believed position, what the jaw holds, and the last grasp events, as text."""
        with self._lock:
            lines = [f"scene file: {self._scene_file}", f"held: {self._held or 'nothing'}"]
            for name, body in self._bodies.items():
                x, y, z = self._current(body)
                lines.append(f"  {name}: ({x:.3f}, {y:.3f}, {z:.3f}) yaw {body.yaw:.2f} size {body.size}")
            lines += ["events:", *("  " + e for e in self._events[-40:])]
            return "\n".join(lines)

    @rpc
    def held(self) -> str | None:
        """The object the belief has in the jaw, or None."""
        return self._held

    # ---- ObjectSceneRegistrationSpec ----

    @rpc
    def set_prompts(self, text: list[str] | None = None) -> None:
        """Remember the words later scans without a query use."""
        self._prompts = text

    @rpc
    def scan_scene(self, text: list[str] | None = None) -> Detection3DArray:
        """World-frame boxes for the objects ``text`` names (an object matches a query that contains its name
        as a word, or its colour/kind word for names like ``cube_red``); every object when no words."""
        words = self._prompts if text is None else text
        with self._lock:
            found = [
                (name, self._current(body), body)
                for name, body in self._bodies.items()
                if _named(name, words)
            ]
        # exact name matches first: programs take objects(["cube_red"])[0]
        found.sort(key=lambda item: not _exact(item[0], words))
        now = time.time()
        detections = [_detection(name, pos, body.size, self._yaw_now(body), now) for name, pos, body in found]
        return Detection3DArray(detections_length=len(detections), header=Header(now, FRAME), detections=detections)

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        """None: a measured scene has no camera, so no point clouds."""
        return None

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """None: a measured scene has no camera, so no point clouds."""
        return None

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """None: a measured scene has no camera, so no point clouds."""
        return None

    # ---- belief ----

    def _reload_locked(self) -> None:
        scene = load_scene_file(Path(self._scene_file))
        self._bench_z = scene.bench_z
        self._bodies = {}
        for name, obj in scene.objects.items():
            if len(obj.xyz) == 2:
                position = (float(obj.xyz[0]), float(obj.xyz[1]), scene.bench_z + obj.size[2] / 2)
            else:
                position = (float(obj.xyz[0]), float(obj.xyz[1]), float(obj.xyz[2]))
            self._bodies[name] = _Body(name, position, tuple(obj.size), obj.yaw)  # type: ignore[arg-type]
        self._held = None
        self._events.append(f"{time.time():.2f} loaded {len(self._bodies)} objects")

    def _tool(self) -> tuple[np.ndarray, Quaternion] | None:
        """The planning tip's world position and orientation from tf, or None while unknown."""
        transform = self.tfbuffer.get(FRAME, self.config.tool_frame, warn=False)
        if transform is None:
            return None
        t = transform.translation
        return np.array([t.x, t.y, t.z]), transform.rotation

    def _pads(self) -> tuple[np.ndarray, float] | None:
        tool = self._tool()
        if tool is None:
            return None
        position, rotation = tool
        if self.config.use_tool_tilt:
            axis = rotation.rotate_vector(Vector3(0.0, 0.0, 1.0))
            pads = position + self.config.pad_m * np.array([axis.x, axis.y, axis.z])
        else:
            pads = position - np.array([0.0, 0.0, self.config.pad_m])
        return pads, float(rotation.to_euler().z)

    def _current(self, body: _Body) -> tuple[float, float, float]:
        if body.name != self._held:
            return body.position
        pads = self._pads()
        if pads is None:
            return body.position
        p, tool_yaw = pads
        ox, oy, oz = self._held_offset
        c, s = math.cos(tool_yaw), math.sin(tool_yaw)
        return (float(p[0] + c * ox - s * oy), float(p[1] + s * ox + c * oy), float(p[2] + oz))

    def _yaw_now(self, body: _Body) -> float:
        if body.name != self._held:
            return body.yaw
        pads = self._pads()
        return body.yaw if pads is None else pads[1] + self._held_dyaw

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            jaw = float(msg.position[list(msg.name).index(self.config.jaw_joint)])
        except (ValueError, IndexError):
            return
        with self._lock:
            if self._jaw_closed is None:
                self._jaw_closed = jaw > self.config.jaw_open_m / 2
                self._jaw_extreme = jaw
                return
            if not self._jaw_closed:
                self._jaw_extreme = min(self._jaw_extreme, jaw)
                if jaw - self._jaw_extreme >= self.config.close_m:
                    self._jaw_closed = True
                    self._jaw_extreme = jaw
                    self._attach()
            else:
                self._jaw_extreme = max(self._jaw_extreme, jaw)
                if (
                    self.config.empty_check
                    and self._held is not None
                    and min(self._bodies[self._held].size[:2]) >= self.config.empty_check_min_width_m
                    and jaw > -self.config.empty_jaw_m
                ):
                    now = time.monotonic()
                    if self._empty_since is None:
                        self._empty_since = now
                    elif now - self._empty_since >= self.config.empty_dwell_s:
                        self._empty_since = None
                        self._events.append(f"{time.time():.2f} jaw closed on nothing (joint7 {jaw * 1000:.1f} mm)")
                        self._release(empty=True)
                        return
                else:
                    self._empty_since = None
                if self._jaw_extreme - jaw < self.config.open_m:
                    self._opening_since = None
                    return
                now = time.monotonic()
                if self._opening_since is None:
                    self._opening_since = now
                if now - self._opening_since >= self.config.release_dwell_s:
                    if self._held is not None:
                        self._events.append(f"{time.time():.2f} jaw opening from {self._jaw_extreme * 1000:.1f} mm")
                    self._jaw_closed = False
                    self._jaw_extreme = jaw
                    self._opening_since = None
                    self._release()

    def _attach(self) -> None:
        pads = self._pads()
        if pads is None:
            self._events.append(f"{time.time():.2f} jaw closed, tool pose unknown: nothing attached")
            return
        p, tool_yaw = pads
        best, best_d = None, math.inf
        for name, body in self._bodies.items():
            bx, by, bz = body.position
            dxy = math.hypot(bx - p[0], by - p[1])
            if dxy <= self.config.grasp_xy_m and abs(bz - p[2]) <= body.size[2] / 2 + self.config.grasp_z_m:
                if dxy < best_d:
                    best, best_d = name, dxy
        if best is None:
            self._events.append(f"{time.time():.2f} jaw closed at pads ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}): nothing in reach")
            return
        body = self._bodies[best]
        dx, dy, dz = body.position[0] - p[0], body.position[1] - p[1], body.position[2] - p[2]
        c, s = math.cos(tool_yaw), math.sin(tool_yaw)
        local = (c * dx + s * dy, -s * dx + c * dy)
        self._held_offset = (0.0, 0.0, dz) if self.config.centre_on_grasp else (*local, dz)
        self._held_dyaw = body.yaw - tool_yaw
        self._held = best
        self._attach_position = body.position
        self._attach_pads = (float(p[0]), float(p[1]), float(p[2]))
        self._events.append(
            f"{time.time():.2f} grasped {best} at pads ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}); it was "
            f"({dx * 100:+.1f}, {dy * 100:+.1f}, {dz * 100:+.1f}) cm from them"
        )

    def _release(self, empty: bool = False) -> None:
        name = self._held
        if name is None:
            return
        body = self._bodies[name]
        x, y, z = self._current(body)
        pads = self._pads()
        if empty and pads is not None and math.dist(tuple(pads[0]), self._attach_pads) < 0.02:
            # closed on nothing where it grasped: the object never moved
            self._held = None
            body.position = self._attach_position
            self._events.append(f"{time.time():.2f} missed {name}: it stays at {tuple(round(v, 3) for v in body.position)}")
            return
        if pads is not None and not empty:
            c, s = math.cos(pads[1]), math.sin(pads[1])
            dx, dy = self.config.release_offset
            x, y = x + c * dx - s * dy, y + s * dx + c * dy
        body.yaw = self._yaw_now(body)
        self._held = None
        support, below = self._bench_z, "the bench"
        for other_name, other in self._bodies.items():
            if other_name == name:
                continue
            ox, oy, oz = other.position
            reach_x = (body.size[0] + other.size[0]) / 2 * 0.8
            reach_y = (body.size[1] + other.size[1]) / 2 * 0.8
            top = oz + other.size[2] / 2
            if abs(x - ox) < reach_x and abs(y - oy) < reach_y and top <= z + 0.01 and top > support:
                support, below = top, other_name
        body.position = (x, y, support + body.size[2] / 2)
        self._events.append(
            f"{time.time():.2f} released {name} at ({x:.3f}, {y:.3f}) onto {below}"
            + ("" if pads is None else f"; pads at ({pads[0][0]:.3f}, {pads[0][1]:.3f}), tool yaw {pads[1]:.2f}")
        )

    # ---- the stand-in map ----

    def _map_loop(self) -> None:
        period = 1.0 / self.config.map_hz
        while not self._stop_map.wait(period):
            try:
                self.scene_cloud.publish(self.map_cloud())
            except Exception as error:  # a publish error must not kill the map: clears would time out
                self._events.append(f"{time.time():.2f} map publish failed: {error}")

    def map_cloud(self) -> PointCloud2:
        """The bench plane and the objects not in the jaw, as one world-frame cloud stamped now."""
        cfg = self.config
        parts: list[np.ndarray] = []
        if cfg.map_mode in ("bench+objects", "bench"):
            xs = np.arange(cfg.bench_x[0], cfg.bench_x[1], cfg.point_spacing_m)
            ys = np.arange(cfg.bench_y[0], cfg.bench_y[1], cfg.point_spacing_m)
            gx, gy = np.meshgrid(xs, ys)
            keep = np.hypot(gx, gy) > cfg.base_clearance_m
            plane = np.stack([gx[keep], gy[keep], np.full(keep.sum(), self._bench_z + cfg.bench_cloud_dz)], axis=1)
            parts.append(plane)
        if cfg.map_mode in ("bench+objects", "objects"):
            with self._lock:
                bodies = [(b.position, b.size, b.yaw) for n, b in self._bodies.items() if n != self._held]
            for position, size, yaw in bodies:
                parts.append(_box_points(position, size, yaw, cfg.point_spacing_m))
        points = np.concatenate(parts).astype(np.float32) if parts else np.zeros((0, 3), dtype=np.float32)
        return PointCloud2.from_numpy(points, frame_id=FRAME, timestamp=time.time())


def _box_points(position: tuple[float, float, float], size: tuple[float, float, float], yaw: float, spacing: float) -> np.ndarray:
    axes = [np.linspace(-s / 2, s / 2, max(2, int(math.ceil(s / spacing)) + 1)) for s in size]
    gx, gy, gz = np.meshgrid(*axes, indexing="ij")
    local = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return local @ rot.T + np.asarray(position)


def _exact(body: str, words: list[str] | None) -> bool:
    return words is not None and any(body.lower() in word.lower().split() for word in words)


def _named(body: str, words: list[str] | None) -> bool:
    """``cube_red`` answers "cube_red", "red cube", "the red cube" or "cube"; ``plug_a`` answers "plug_a" or
    "plug" but not "plug_b"."""
    if words is None:
        return True
    if _exact(body, words):
        return True
    parts = body.lower().split("_")
    for word in words:
        tokens = [t for t in word.lower().replace("_", " ").split() if t != "the"]
        if tokens and all(t in parts for t in tokens):
            return True
    return False


def _detection(name: str, position: tuple[float, float, float], size: tuple[float, float, float], yaw: float, ts: float) -> Detection3D:
    q = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    detection = Detection3D()
    detection.header = Header(ts, FRAME)
    detection.id = name
    detection.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=name, score=1.0))]
    detection.results_length = 1
    detection.bbox = BoundingBox3D(
        center=Pose(position=Vector3(*position), orientation=Quaternion(q.x, q.y, q.z, q.w)),
        size=Vector3(*size),
    )
    return detection
