"""The real-arm blueprint: the CAN port flag, the wiring the live backend needs, the speed limit."""

from __future__ import annotations

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection

from real.blueprint_real import DEFAULT_CAMERA, metal_real_program, metal_real_program_testcam
from real.measured_scene import MeasuredSceneModule
from real.pattern_camera import PatternCamera


def _topics(blueprint: Blueprint) -> dict[tuple[type, str], str]:
    return {
        (atom.module, stream.name): blueprint.remapping_map.get((atom.name, stream.name), stream.name)
        for atom in blueprint.active_blueprints
        for stream in atom.streams
    }


def test_the_can_port_defaults_to_the_right_follower_and_takes_the_flag() -> None:
    [connection] = [a for a in metal_real_program.active_blueprints if a.module is MetalConnection]
    assert connection.kwargs["can_port"] == "can_follower_r"
    parsed = BlueprintConfigParser(metal_real_program).parse(["--can-port", "can_test"], environ={})
    assert parsed.module_kwargs("metalconnection")["can_port"] == "can_test"


def test_the_live_backend_finds_one_camera_the_scene_and_the_box_filter_map() -> None:
    for blueprint in (metal_real_program, metal_real_program_testcam):
        topics = _topics(blueprint)
        cameras = [k for k in topics if k[1] == "color_image"]
        assert len(cameras) == 1
        assert topics[MeasuredSceneModule, "scene_cloud"] == topics[PointCloudBoxFilter, "cloud"]
        assert topics[PointCloudBoxFilter, "filtered_cloud"] == topics[ManipulationModule, "voxel_map"]
        assert topics[MeasuredSceneModule, "coordinator_joint_state"] == topics[ManipulationModule, "coordinator_joint_state"]
    assert _topics(metal_real_program_testcam)[PatternCamera, "color_image"] == "color_image"
    [webcam] = [a for a in metal_real_program.active_blueprints if a.module is WebcamModule]
    assert webcam.kwargs["hardware"].camera_index == DEFAULT_CAMERA


def test_the_grasp_check_is_on_for_the_real_arm() -> None:
    [scene] = [a for a in metal_real_program.active_blueprints if a.module is MeasuredSceneModule]
    assert scene.kwargs["empty_check"] is True


def test_the_planner_moves_slowly_and_has_a_home_preset() -> None:
    [planner] = [a for a in metal_real_program.active_blueprints if a.module is ManipulationModule]
    config = ManipulationModuleConfig.model_validate(planner.kwargs)
    assert config.default_speed_scale <= 0.1 and config.linear_speed_scale <= 0.1
    assert config.model.home_joints is not None
    assert config.voxel_map_resolution == 0.025
