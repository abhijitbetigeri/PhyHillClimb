"""The real right Metal behind dimos's Autoresearch live backend, with a measured scene instead of a detector.

    dimos run phyhillclimb.metal-real-program --can-port can_follower_r            # the real arm, wrist_r camera
    dimos --simulation run phyhillclimb.metal-real-program-testcam                  # echo mock, generated frames

then, in another shell, `python real/run_program_real.py --program sim/candidates/tower_order_v24.py`.

Modules (the same planner and coordinator as dimos's `makermods-metal-planner-coordinator`):

| Module | Does |
|---|---|
| `DamiaoCoordinator` + `MetalConnection` | trajectories and the jaw over SocketCAN (`--can-port`, default `can_follower_r`). Physical start-up ENABLES THE MOTORS (they hold where they are) and CALIBRATES THE JAW by driving it to both ends. `--simulation` swaps in the echo mock |
| `ManipulationModule` | the planner; `default_speed_scale` and `linear_speed_scale` both REAL_SPEED_SCALE (default 0.1; dimos ships 0.2); a `home` preset (the sim's home); bench + objects map from the scene |
| `WebcamModule` "wrist" | `color_image` from `--wrist.hardware.camera-index` (default the wrist_r by-path device), for the keyframes |
| `MeasuredSceneModule` | `scan_scene` from `real/scene.yaml` (REAL_SCENE or `--measuredscenemodule.scene-file`), the grasp belief, and the stand-in map cloud |
| `PointCloudBoxFilter` | dimos's box filter: `scene.clear`/`restore` boxes, its `filtered_cloud` is the planner's `voxel_map` |

No AutoresearchModule: its world is MujocoWorld, and a plain stack plus `run_program` is all a real trial needs.
"""

from __future__ import annotations

import os
from pathlib import Path

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator, mock_initial_positions
from dimos.robot.manipulators.makermods_metal.config import METAL_SIM_HOME, make_metal_model_config
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection

from real.measured_scene import DEFAULT_SCENE_FILE, MeasuredSceneModule
from real.pattern_camera import PatternCamera

#: The station's right follower arm (notes/station-backup/.../physical-ai-hacks_configs_station.yaml).
DEFAULT_CAN_PORT = "can_follower_r"
#: The right wrist camera by USB path (the same file); overhead is ...usb-0:1.4:1.0-video-index0.
DEFAULT_CAMERA = "/dev/v4l/by-path/pci-0000:67:00.3-usb-0:1.3:1.0-video-index0"
#: Planned and straight moves both run at this fraction of the joint limits. dimos's real default is 0.2.
SPEED_SCALE = float(os.environ.get("REAL_SPEED_SCALE", "0.1"))
SCENE_FILE = os.environ.get("REAL_SCENE", str(DEFAULT_SCENE_FILE))
#: The planner's octree cell; the scene map's bench plane sits half a cell below z = 0.
VOXEL = 0.025
#: Planning and execution limits, seconds; slow moves need more than the 60 s default only at speed 0.05.
EXECUTION_TIMEOUT_S = float(os.environ.get("REAL_EXECUTION_TIMEOUT_S", "90"))

MODEL = make_metal_model_config(robot="arm", home_joints=METAL_SIM_HOME)


def connection(can_port: str = DEFAULT_CAN_PORT) -> Blueprint:
    """The Metal on ``can_port``, or the echo mock under ``--simulation`` (no dynamics, the jaw follows commands)."""
    if global_config.simulation:
        return MockJointConnection.blueprint(
            source="arm",
            model=MetalConnection.robot.model,
            initial_positions=mock_initial_positions(MetalConnection.robot),
        )
    return MetalConnection.blueprint(source="arm", can_port=can_port)


def _remaps() -> list:
    module = MockJointConnection if global_config.simulation else MetalConnection
    return [
        (module, "position_command", "arm_position_command"),
        (DamiaoCoordinator, "arm_joint_state", "joint_state"),
        (PointCloudBoxFilter, "cloud", "scene_cloud"),
        (PointCloudBoxFilter, "filtered_cloud", "voxel_map"),
    ]


def planner() -> Blueprint:
    return ManipulationModule.blueprint(
        model=MODEL,
        gripper_tasks={"arm": "metal_gripper"},
        voxel_map_resolution=VOXEL,
        default_speed_scale=SPEED_SCALE,
        linear_speed_scale=SPEED_SCALE,
        execution_timeout=EXECUTION_TIMEOUT_S,
        visualization={"backend": "viser"},
    )


def scene(scene_file: str = SCENE_FILE, *, mock_jaw: bool = False) -> Blueprint:
    """The measured scene and dimos's box filter. ``mock_jaw``: the echo mock's jaw closes fully on anything, so
    the scene's "closed on nothing" grasp check is off."""
    return autoconnect(
        MeasuredSceneModule.blueprint(scene_file=scene_file, empty_check=not mock_jaw),
        PointCloudBoxFilter.blueprint(),
    )


def build(*, test_camera: bool = False) -> Blueprint:
    camera = (
        PatternCamera.blueprint()
        if test_camera
        else WebcamModule.blueprint(
            instance_name="wrist",
            hardware=WebcamConfig(camera_index=DEFAULT_CAMERA, width=640, height=480, fps=15),
            frame_id="wrist_camera_link",
            transform=None,
        )
    )
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=MODEL.model,
            tasks=[
                joint_trajectory_task(list(MODEL.joint_names)),
                # joint7 is open at its lower end.
                TaskConfig(
                    name="metal_gripper",
                    type="gripper",
                    joint_names=["arm/joint7"],
                    priority=20,
                    params={"open_at": "lower"},
                ),
            ],
            zero_stiffness_joints=[],
        ),
        connection(),
        planner(),
        camera,
        scene(mock_jaw=global_config.simulation),
    ).remappings(_remaps())


#: `dimos run phyhillclimb.metal-real-program --can-port can_follower_r`
metal_real_program = build()
#: the same with generated frames instead of the webcam (rehearsal with `--simulation`, or no camera free)
metal_real_program_testcam = build(test_camera=True)

if __name__ == "__main__":
    raise SystemExit(f"run it with the dimos CLI: dimos run phyhillclimb.metal-real-program ({Path(__file__).name})")
