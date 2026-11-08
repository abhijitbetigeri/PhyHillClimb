"""The Autoresearch lab on the Hill Climb stage 2 toy: a Metal in MuJoCo, our peg-stack bench, a stack judge.

Install once into the dimos venv, then run it with the dimos CLI (flags like --mujocosimmodule.fps work):

    uv pip install --python ~/projects/dimos/.venv/bin/python -e ~/projects/PhyHillClimb
    cd ~/projects/dimos && .venv/bin/dimos --simulation run phyhillclimb.autoresearch-stack \
        --mujocosimmodule.headless true   # the camera rates are already lowered in build() for macOS

Then drive it as the actor: `dimos mcp call task`, `scene`, `submit`, `run`, `results`, `leaderboard`, `freeze`.
The bench is `sim/stack_bench.xml`: three hollow 5 cm cubes (blue, red, yellow) and two pegs, as on the station.
Level 1 of the stack is this session's judge: cube_red ends on cube_blue (within 2 cm in x and y, 5 cm above).
Programs move gripper_base, the jaw's mount; the finger pads are ~7.8 cm further along the approach (see stack_v4).
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
DIMOS = Path(os.environ.get("DIMOS_ROOT", Path.home() / "projects" / "dimos"))
sys.path.insert(0, str(DIMOS))

from sim.judges import OrderedTowerJudge, ThreeCubeJudge  # noqa: E402
from sim.scene_video import SceneVideoModule  # noqa: E402

from dimos.agents.mcp.mcp_server import McpServer  # noqa: E402
from dimos.autoresearch.judge import Judge, Snapshot, Verdict, moved  # noqa: E402
from dimos.autoresearch.module import AutoresearchModule  # noqa: E402
from dimos.autoresearch.session import Budget  # noqa: E402
from dimos.autoresearch.truth_scene import TruthObject, TruthSceneModule  # noqa: E402
from dimos.autoresearch.world import PlacementBounds  # noqa: E402
from dimos.constants import RECORDINGS_DIR  # noqa: E402
from dimos.control.coordinator import TaskConfig  # noqa: E402
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task  # noqa: E402
from dimos.core.coordination.blueprints import autoconnect  # noqa: E402
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule  # noqa: E402
from dimos.imitation.collection.recorder import collect  # noqa: E402
from dimos.manipulation.manipulation_module import ManipulationModule  # noqa: E402
from dimos.robot.manipulators.common.blueprints import coordinator, manipulation_mapping  # noqa: E402
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection import METAL_QUEST_PROFILE  # noqa: E402
from dimos.robot.manipulators.makermods_metal.config import (  # noqa: E402
    METAL_SIM_HOME,
    METAL_SIM_MAP_GRACE,
    METAL_SIM_SELF_FILTER_PADDING,
    METAL_SIM_VOXEL_SIZE,
    METAL_TF_LINKS,
    make_metal_model_config,
    make_metal_sim_hardware,
)
from dimos.robot.manipulators.makermods_metal.mjcf import make_metal_sim_module_kwargs  # noqa: E402
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule  # noqa: E402

STACK_BENCH = HERE / "stack_bench.xml"
BODIES = ("cube_blue", "cube_red", "cube_yellow", "peg_yellow", "peg_red")
TRUTH = {
    "cube_blue": TruthObject(size=(0.05, 0.05, 0.05)),
    "cube_red": TruthObject(size=(0.05, 0.05, 0.05)),
    "cube_yellow": TruthObject(size=(0.05, 0.05, 0.05)),
    "peg_yellow": TruthObject(size=(0.02, 0.02, 0.10)),
    "peg_red": TruthObject(size=(0.02, 0.02, 0.06)),
}
PEG_HALF = {"peg_yellow": 0.05, "peg_red": 0.03}
TASK = "stack the red cube on top of the blue cube"
#: The tower the three-cube lab builds, bottom to top, then the stick: any permutation of the cubes, e.g.
#: TOWER_ORDER="blue,yellow,red,yellow stick". Set it in the environment before launching the lab.
TOWER_ORDER = os.environ.get("TOWER_ORDER", "yellow,red,blue,red stick")


def parse_order(text: str) -> tuple[list[str], str | None]:
    """'yellow,red,blue,red stick' -> (['cube_yellow', 'cube_red', 'cube_blue'], 'peg_red')."""
    cubes, peg = [], None
    for item in [i.strip().lower() for i in text.split(",") if i.strip()]:
        colour = item.split()[0]
        if "stick" in item or "peg" in item:
            peg = f"peg_{colour}"
        else:
            cubes.append(f"cube_{colour}")
    return cubes, peg


def tower_task_text(cubes: list[str], peg: str | None) -> str:
    names = [c.replace("cube_", "") for c in cubes]
    text = f"stack the {names[0]} cube first"
    for n in names[1:]:
        text += f", then the {n} cube on top"
    if peg:
        text += f", then stand the {peg.replace('peg_', '')} stick in the top cube's hole"
    return text


@dataclass(frozen=True)
class PlaceOnJudge:
    """Success when ``body`` ends centred on ``target`` (within ``xy_m`` in x and y, as it stood before the trial)
    and ``dz_m`` above it (within ``z_m``). Stage names say how far the trial got."""

    body: str
    target: str
    dz_m: float = 0.04
    xy_m: float = 0.02  # a 5 cm cube overhanging by 2 cm still stands; the rubric only asks for free-standing
    z_m: float = 0.012

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        tx, ty, tz = before.bodies[self.target].position
        start, end = before.bodies[self.body], after.bodies[self.body]
        x, y, z = end.position
        dxy = math.hypot(x - tx, y - ty)
        dz = z - (tz + self.dz_m)
        if dxy <= self.xy_m and abs(dz) <= self.z_m:
            return Verdict(success=True, stage="done", note=f"{self.body} is on the peg ({dxy * 100:.1f} cm off centre)")
        if moved(start, end) < 0.01:
            return Verdict(success=False, stage="reach", note=f"{self.body} was not touched")
        if z - start.position[2] < 0.03 and dxy > 0.05:
            return Verdict(success=False, stage="grasp", note=f"{self.body} moved but was not lifted")
        return Verdict(
            success=False,
            stage="place",
            note=f"{self.body} ended {dxy * 100:.1f} cm from the peg axis and {dz * 100:+.1f} cm from the seat height",
        )


class StackLab(AutoresearchModule):
    """dimos's lab with the stack judge and its wording."""

    def _judge(self) -> Judge:
        return PlaceOnJudge("cube_red", "cube_blue", dz_m=0.05)

    def _success_rule(self) -> str:
        return "the red cube ends on top of the blue cube: within 2 cm of it in x and y, 5 cm above it"


class ThreeCubeLab(StackLab):
    """The tower in the order TOWER_ORDER asks for, with the stick in the top cube; release and the 3 s dwell are
    not certified."""

    def _judge(self) -> Judge:
        cubes, peg = parse_order(TOWER_ORDER)
        return OrderedTowerJudge(tuple(cubes), peg, peg_half_m=PEG_HALF.get(peg or "", 0.03))

    def _success_rule(self) -> str:
        cubes, peg = parse_order(TOWER_ORDER)
        rule = f"cubes stacked bottom to top in the order {[c.replace('cube_', '') for c in cubes]}, each within 1.5 cm of the one below and 5 cm above it, upright"
        if peg:
            rule += f"; the {peg.replace('peg_', '')} stick standing in the top cube's hole (within 1.2 cm of its centre)"
        return rule + "; release and 3 s stability unverified"


def build(full_stack: bool = False):
    sim_kwargs = make_metal_sim_module_kwargs()
    sim_kwargs.update(scene_xml=str(STACK_BENCH), tracked_bodies=list(BODIES))
    model = make_metal_model_config(robot="arm", home_joints=METAL_SIM_HOME, tf_extra_links=METAL_TF_LINKS)
    label = "three_cube" if full_stack else "stack"
    session = RECORDINGS_DIR / f"autoresearch_{label}_{datetime.now():%Y%m%d_%H%M%S}"
    cubes, peg = parse_order(TOWER_ORDER)
    task = tower_task_text(cubes, peg) if full_stack else TASK
    lab_type = ThreeCubeLab if full_stack else StackLab
    judge_body = cubes[1] if full_stack and len(cubes) > 1 else "cube_red"  # the first cube that moves is randomised
    metal_sim = autoconnect(
        MujocoSimModule.blueprint(
            **sim_kwargs, headless=True, fps=5, scene_camera_fps=10, pointcloud_fps=2, width=640, height=480
        ),  # override from the CLI on a slow machine: --mujocosimmodule.width 320 --mujocosimmodule.scene-camera-fps 2
        coordinator(
            hardware=[make_metal_sim_hardware()],
            tasks=[
                joint_trajectory_task(list(model.joint_names)),
                TaskConfig(name="metal_gripper", type="gripper", joint_names=["arm/joint7"], priority=20,
                           params={"open_at": "lower"}),
            ],
        ),
        ManipulationModule.blueprint(model=model, gripper_tasks={"arm": "metal_gripper"},
                                     voxel_map_resolution=METAL_SIM_VOXEL_SIZE),
        manipulation_mapping(self_filter_model=model.model, voxel_size=METAL_SIM_VOXEL_SIZE,
                             grace_depth=METAL_SIM_MAP_GRACE, self_filter_padding=METAL_SIM_SELF_FILTER_PADDING),
    )
    return autoconnect(
        collect(profile=METAL_QUEST_PROFILE, recording=session),
        EpisodeMonitorModule.blueprint(instance_name="episodes", task=task),
        metal_sim,
        TruthSceneModule.blueprint(objects=TRUTH),
        SceneVideoModule.blueprint(out_dir=str(session / "scene"), fps=10.0),
        lab_type.blueprint(
            session_dir=session,
            task=task,
            judge_body=judge_body,
            program_timeout_s=300.0,  # a four-level tower with a relocation and retries needs more than the 120 s default
            placement_bounds=PlacementBounds(x=(0.18, 0.28), y=(0.08, 0.18), z=0.025, yaw=(-0.5, 0.5)),
            budget=Budget(iterations=5, candidates_per_round=3, dev_trials=5, heldout_trials=10, threshold=0.8),
        ),
        McpServer.blueprint(),
    ).remappings([("recorder", "wrist_image", "color_image")])


#: The blueprint `dimos run` finds through the `dimos.blueprints` entry point (pyproject.toml).
lab = build()
three_cube_lab = build(full_stack=True)

if __name__ == "__main__":
    raise SystemExit("run it with the dimos CLI: dimos --simulation run phyhillclimb.autoresearch-stack")
