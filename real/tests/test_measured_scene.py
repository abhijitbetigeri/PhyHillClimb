"""MeasuredSceneModule: scans from the YAML, reload, the grasp belief and the stand-in map (dimos's test style).

    ~/projects/dimos/.venv/bin/python -m pytest real/tests -q
"""

from __future__ import annotations

import asyncio
import math
from pathlib import Path
import time
from typing import Any, cast

import numpy as np
import pytest

from dimos.autoresearch.program import MapStamps, program_box
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.manipulators.damiao._damiao_test_helpers import LocalTransport

from real.measured_scene import MeasuredSceneModule, load_scene_file

SCENE = """
frame: world
objects:
  cube_yellow: {xyz: [0.30, 0.11], size: [0.05, 0.05, 0.05]}
  cube_red:    {xyz: [0.22, 0.12, 0.025], size: [0.05, 0.05, 0.05], yaw: 0.3}
  cube_blue:   {xyz: [0.20, 0.00], size: [0.05, 0.05, 0.05]}
  plug_a:      {xyz: [0.235, 0.0, 0.055], size: [0.018, 0.012, 0.04]}
  plug_b:      {xyz: [0.285, 0.0, 0.055], size: [0.018, 0.012, 0.04]}
"""
DOWN = Quaternion.from_euler(Vector3(math.pi, 0.0, 0.0))


class Harness:
    def __init__(self, tmp_path: Path, **config) -> None:
        self.file = tmp_path / "scene.yaml"
        self.file.write_text(SCENE)
        config.setdefault("release_offset", (0.0, 0.0))
        self.module = MeasuredSceneModule(scene_file=str(self.file), map_hz=50.0, release_dwell_s=0.05, **config)
        self.joints: LocalTransport[JointState] = LocalTransport()
        self.clouds: LocalTransport[PointCloud2] = LocalTransport()
        self.module.coordinator_joint_state.transport = self.joints
        self.module.tf.transport = LocalTransport[TFMessage]()
        self.module.scene_cloud.transport = self.clouds
        self.tool = (np.array([0.3, 0.0, 0.3]), DOWN)
        self.module._tool = lambda: self.tool  # type: ignore[method-assign]
        self.module.start()

    def jaw(self, value: float, hold_s: float = 0.06) -> None:
        """The jaw reading ``value``, repeated for ``hold_s`` as the 100 Hz joint state would."""
        end = time.monotonic() + hold_s
        while True:
            self.joints.publish(JointState(name=["arm/joint1", "arm/joint7"], position=[0.0, value]))
            if time.monotonic() >= end:
                return
            time.sleep(0.01)

    def at_pads(self, x: float, y: float, z: float) -> None:
        self.tool = (np.array([x, y, z + 0.078]), DOWN)

    def where(self, name: str) -> tuple[float, float, float]:
        d = self.module.scan_scene([name]).detections[0]
        p = d.bbox.center.position
        return (p.x, p.y, p.z)


@pytest.fixture
def scene(tmp_path: Path):
    harness = Harness(tmp_path)
    yield harness
    harness.module.stop()


def test_scan_scene_answers_measured_world_boxes(scene: Harness) -> None:
    found = scene.module.scan_scene(["cube_red"])
    assert found.header.frame_id == "world"
    assert [d.id for d in found.detections] == ["cube_red"]
    red = found.detections[0]
    assert (red.bbox.center.position.x, red.bbox.center.position.z) == pytest.approx((0.22, 0.025))
    assert (red.bbox.size.x, red.bbox.size.y, red.bbox.size.z) == pytest.approx((0.05, 0.05, 0.05))
    # yaw about z, as DimosBackend reads it back
    q = red.bbox.center.orientation
    assert math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) == pytest.approx(0.3)
    # z left out: standing on the bench, centre half its height up
    assert scene.where("cube_yellow")[2] == pytest.approx(0.025)
    # names match as words; plug_a never answers for plug_b; an exact match comes first
    assert [d.id for d in scene.module.scan_scene(["plug_b"]).detections] == ["plug_b"]
    assert [d.id for d in scene.module.scan_scene(["the red cube"]).detections] == ["cube_red"]
    assert len(scene.module.scan_scene(None).detections) == 5
    assert scene.module.scan_scene(["spoon"]).detections_length == 0


def test_reload_rereads_the_file_without_a_restart(scene: Harness) -> None:
    scene.file.write_text(SCENE.replace("[0.22, 0.12, 0.025]", "[0.25, 0.10, 0.025]"))
    text = scene.module.reload()
    assert "cube_red: (0.250, 0.100, 0.025)" in text
    assert scene.where("cube_red")[:2] == pytest.approx((0.25, 0.10))
    scene.module.set_object("cube_blue", 0.18, -0.02)
    assert scene.where("cube_blue") == pytest.approx((0.18, -0.02, 0.025))


def test_a_closed_jaw_carries_the_cube_and_an_open_one_sets_it_on_the_one_below(scene: Harness) -> None:
    scene.jaw(-0.05)  # opened
    scene.at_pads(0.302, 0.108, 0.025)  # pads at the yellow cube, 2 mm off
    scene.jaw(-0.02)  # closed on it
    assert scene.module.held() == "cube_yellow"
    scene.jaw(-0.032, hold_s=0.0)  # the jaw rebounds 12 mm off the cube for an instant: still held
    scene.jaw(-0.021)
    assert scene.module.held() == "cube_yellow"
    scene.at_pads(0.302, 0.108, 0.125)  # lifted 10 cm
    assert scene.where("cube_yellow") == pytest.approx((0.302, 0.108, 0.125))  # centred on the pads
    # over the blue cube, released 3 mm above its top
    scene.at_pads(0.202, -0.002, 0.078)
    assert scene.where("cube_yellow")[2] == pytest.approx(0.078)
    scene.jaw(-0.05)
    assert scene.module.held() is None
    assert scene.where("cube_yellow") == pytest.approx((0.202, -0.002, 0.075))  # on blue's top (0.05) + half
    # a close with nothing within reach holds nothing
    scene.at_pads(0.40, -0.20, 0.025)
    scene.jaw(0.0)
    assert scene.module.held() is None
    assert "nothing in reach" in scene.module.describe()


def test_a_jaw_that_closes_on_nothing_holds_nothing(tmp_path: Path) -> None:
    harness = Harness(tmp_path, empty_dwell_s=0.05)
    try:
        harness.jaw(-0.05)
        harness.at_pads(0.30, 0.11, 0.025)
        harness.jaw(-0.01, hold_s=0.0)  # closing: attached for now
        assert harness.module.held() == "cube_yellow"
        harness.jaw(0.0, hold_s=0.1)  # ...but it ends fully closed: missed
        assert harness.module.held() is None
        assert harness.where("cube_yellow") == pytest.approx((0.30, 0.11, 0.025))
        assert "missed cube_yellow" in harness.module.describe()
        # a slip in the air: held, lifted, then the jaw closes the rest of the way
        harness.jaw(-0.05)
        harness.jaw(-0.025)
        assert harness.module.held() == "cube_yellow"
        harness.at_pads(0.30, -0.05, 0.10)
        harness.jaw(0.0, hold_s=0.1)
        assert harness.module.held() is None
        assert harness.where("cube_yellow") == pytest.approx((0.30, -0.05, 0.025))  # fell to the bench below
    finally:
        harness.module.stop()


def test_released_away_from_everything_it_lands_on_the_bench(scene: Harness) -> None:
    scene.jaw(-0.05)
    scene.at_pads(0.235, 0.0, 0.055)
    scene.jaw(-0.04)  # stepped closes still count
    assert scene.module.held() == "plug_a"
    scene.at_pads(0.235, -0.10, 0.07)
    scene.jaw(-0.05)
    assert scene.where("plug_a") == pytest.approx((0.235, -0.10, 0.02))


def test_a_release_drags_the_object_by_the_release_offset_in_the_tool_frame(tmp_path: Path) -> None:
    harness = Harness(tmp_path, release_offset=(0.009, -0.003))
    try:
        harness.jaw(-0.05)
        harness.tool = (np.array([0.20, 0.0, 0.103]), Quaternion.from_euler(Vector3(math.pi, 0.0, math.pi / 2)))
        harness.module.set_object("cube_blue", 0.40, -0.30)  # out of the way
        harness.module.set_object("cube_yellow", 0.20, 0.0)
        harness.jaw(-0.02)
        assert harness.module.held() == "cube_yellow"
        harness.jaw(-0.05)
        x, y, _ = harness.where("cube_yellow")
        # tool yaw 90 degrees: tool x is world y, tool y is world -x
        assert (x, y) == pytest.approx((0.203, 0.009), abs=1e-3)
    finally:
        harness.module.stop()


def test_the_map_is_fresh_has_the_bench_and_objects_and_drops_the_held_one(scene: Harness) -> None:
    since = time.time()
    deadline = since + 3.0
    while time.time() < deadline and not scene.clouds._messages:
        time.sleep(0.05)
    time.sleep(0.1)
    cloud = scene.clouds.latest
    assert cloud.frame_id == "world" and cloud.ts > since
    points = cloud.points_f32()
    bench = points[np.isclose(points[:, 2], -0.0125)]
    assert len(bench) > 1000
    assert np.all(np.hypot(bench[:, 0], bench[:, 1]) > 0.12)  # nothing around the base
    near_red = np.linalg.norm(points - np.array([0.22, 0.12, 0.025]), axis=1) < 0.03
    assert near_red.any()
    scene.jaw(-0.05)
    scene.at_pads(0.22, 0.12, 0.025)
    scene.jaw(-0.02)
    held = scene.module.map_cloud().points_f32()
    assert not (np.linalg.norm(held - np.array([0.22, 0.12, 0.025]), axis=1) < 0.03).any()


def test_box_filter_clears_reach_the_planner_map_within_the_map_timeout(scene: Harness) -> None:
    box_filter = PointCloudBoxFilter()
    cast("dict[str, Any]", box_filter.__dict__)["_tf"] = MultiTBuffer()
    maps = MapStamps()
    sent: list[PointCloud2] = []
    box_filter.filtered_cloud.subscribe(sent.append)
    box_filter.filtered_cloud.subscribe(maps.on_map)
    scene.clouds.subscribe(lambda cloud: asyncio.run(box_filter.handle_cloud(cloud)))
    try:
        box_filter.set_box(program_box("cube_red"), Pose(Vector3(0.22, 0.12, 0.025), Quaternion()), Vector3(0.11, 0.11, 0.11), "world")
        changed = time.time()
        assert maps.wait_newer(changed, 2.0)  # DimosBackend._wait_for_map's condition
        points = sent[-1].points_f32()
        assert len(points) > 1000
        assert not (np.linalg.norm(points - np.array([0.22, 0.12, 0.025]), axis=1) < 0.03).any()
    finally:
        cast("dict[str, Any]", box_filter.__dict__)["_tf"] = None
        box_filter.dispose()


def test_the_shipped_templates_load() -> None:
    here = Path(__file__).resolve().parent.parent
    assert set(load_scene_file(here / "scene.yaml").objects) == {"cube_yellow", "cube_red", "cube_blue"}
    assert set(load_scene_file(here / "scene_cable.yaml").objects) == {"hub", "plug_a", "plug_b"}
