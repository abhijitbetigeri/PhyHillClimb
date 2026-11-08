"""The real-arm configuration on the MuJoCo Metal: the same planner, measured scene and stand-in map as
`blueprint_real.py`, but the simulated arm and our bench instead of the CAN connection and the webcam.

No TruthSceneModule and no camera-built voxel map: the program only knows what the scene YAML says plus the grasp
belief, exactly as on the station. The simulator still reports true poses on `body_poses`, which only
`real/verify_measured_sim.py` reads, to fill the YAML with a seeded layout and to judge the result.

    MEASURED_BENCH=stack dimos --simulation --n-workers 8 run phyhillclimb.metal-measured-sim --mujocosimmodule.headless true
"""

from __future__ import annotations

import os
from pathlib import Path

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.robot.manipulators.common.blueprints import coordinator
from dimos.robot.manipulators.makermods_metal.config import make_metal_sim_hardware
from dimos.robot.manipulators.makermods_metal.mjcf import make_metal_sim_module_kwargs
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

from real.blueprint_real import MODEL, planner, scene

SIM = Path(__file__).resolve().parent.parent / "sim"
BENCHES = {
    "stack": (SIM / "stack_bench.xml", ["cube_blue", "cube_red", "cube_yellow", "peg_yellow", "peg_red"]),
    "cable": (SIM / "cable_bench.xml", ["hub", "plug_a", "plug_b", "plug_loose"]),
}
BENCH = os.environ.get("MEASURED_BENCH", "stack")
#: verify_measured_sim.py writes the seeded layout here before every trial and calls reload
SIM_SCENE_FILE = os.environ.get("REAL_SCENE", str(Path(__file__).resolve().parent / "runs" / "sim_scene.yaml"))


def build():
    xml, bodies = BENCHES[BENCH]
    sim_kwargs = make_metal_sim_module_kwargs()
    sim_kwargs.update(scene_xml=str(xml), tracked_bodies=list(bodies))
    Path(SIM_SCENE_FILE).parent.mkdir(parents=True, exist_ok=True)
    if not Path(SIM_SCENE_FILE).exists():
        default = Path(__file__).resolve().parent / ("scene.yaml" if BENCH == "stack" else "scene_cable.yaml")
        Path(SIM_SCENE_FILE).write_text(default.read_text())
    return autoconnect(
        MujocoSimModule.blueprint(
            **sim_kwargs, headless=True, fps=5, scene_camera_fps=5, pointcloud_fps=1, width=480, height=360
        ),
        coordinator(
            hardware=[make_metal_sim_hardware()],
            tasks=[
                joint_trajectory_task(list(MODEL.joint_names)),
                TaskConfig(name="metal_gripper", type="gripper", joint_names=["arm/joint7"], priority=20,
                           params={"open_at": "lower"}),
            ],
        ),
        planner(),
        scene(SIM_SCENE_FILE),
    ).remappings(
        [
            (PointCloudBoxFilter, "cloud", "scene_cloud"),
            (PointCloudBoxFilter, "filtered_cloud", "voxel_map"),
        ]
    )


metal_measured_sim = build()
