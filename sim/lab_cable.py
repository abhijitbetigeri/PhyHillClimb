"""The Autoresearch lab on the Hill Climb stage 1 object: a Metal in MuJoCo, a cable set (USB hub) with plugs
standing in its sockets, and judges for unplugging, plugging in and the unplug-then-replug cycle.

Install once into the dimos venv, then run one of the three blueprints with the dimos CLI:

    uv pip install --python ~/projects/dimos/.venv/bin/python -e ~/projects/PhyHillClimb
    cd ~/projects/dimos && .venv/bin/dimos --simulation --n-workers 8 run phyhillclimb.autoresearch-cable-unplug \
        --mujocosimmodule.headless true

| Blueprint | Task | Judge |
|---|---|---|
| `phyhillclimb.autoresearch-cable-unplug` | unplug the cables | `UnplugJudge`: a plug 4 cm out of its socket or clear of it, hub still |
| `phyhillclimb.autoresearch-cable-plug` | plug the loose cable into the free socket | `PlugJudge`: 4 mm / 3 mm / 10 degrees |
| `phyhillclimb.autoresearch-cable-cycle` | unplug one and plug it back | `UnplugPlugJudge`: rose 4 cm during the trial, then seated |

CABLE_TASK in the environment overrides the task text (like TOWER_ORDER for the tower lab). The bench is
`sim/cable_bench.xml`: the hub lies along the arm's x axis about 0.23-0.28 m out, its two sockets 5 cm apart along
it; every trial's seed places the hub (x, y, yaw) and the plugs with it (``CableWorld``), so one lab reset does not
tear a plug out of its socket the way the base lab's single-body placement would. The loose plug of the plug-in
task lies on its side beside the free socket; the unplug labs park it at the far corner. Programs move
``gripper_base``; the finger pads are 7.8 cm further along the approach (see stack_v4) and the fingers are 4 cm wide
along the tool's x axis, so the jaw squeezes a plug across the strip (tool yaw = hub yaw)."""

from __future__ import annotations

import math
import os
import random
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
DIMOS = Path(os.environ.get("DIMOS_ROOT", Path.home() / "projects" / "dimos"))
sys.path.insert(0, str(DIMOS))

from sim.judges import CableSet, PlugJudge, UnplugJudge, UnplugPlugJudge  # noqa: E402
from sim.motion_capture import MotionCaptureModule  # noqa: E402
from sim.scene_video import SceneVideoModule  # noqa: E402

from reactivex.disposable import Disposable  # noqa: E402

from dimos.agents.mcp.mcp_server import McpServer  # noqa: E402
from dimos.autoresearch.judge import Judge  # noqa: E402
from dimos.autoresearch.module import (  # noqa: E402
    PLANNER_TIMEOUT_S,
    AutoresearchModule,
    _home_preset,
    _wait_for_arm,
    bus_env,
)
from dimos.autoresearch.session import Budget, Session  # noqa: E402
from dimos.autoresearch.truth_scene import TruthObject, TruthSceneModule  # noqa: E402
from dimos.autoresearch.world import MujocoWorld, PlacementBounds  # noqa: E402
from dimos.autoresearch.program import program_box  # noqa: E402
from dimos.constants import RECORDINGS_DIR  # noqa: E402
from dimos.control.coordinator import TaskConfig  # noqa: E402
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task  # noqa: E402
from dimos.core.coordination.blueprints import autoconnect  # noqa: E402
from dimos.core.core import rpc  # noqa: E402
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

CABLE_BENCH = HERE / "cable_bench.xml"
HUB = "hub"
PLUGS = ("plug_a", "plug_b")
LOOSE = "plug_loose"
BODIES = (HUB, *PLUGS, LOOSE)
#: Where a fully inserted plug's origin (its blade centre) sits relative to the hub origin (bottom-face centre),
#: hub frame, metres: the sockets are 5 cm apart along the hub, 2.5 cm deep over a 1 cm floor. The loose plug
#: belongs in the near socket, which the plug-in task leaves free.
SOCKET_A = (-0.025, 0.0, 0.0225)
SOCKET_B = (0.025, 0.0, 0.0225)
CABLES = CableSet(hub=HUB, sockets=(("plug_a", SOCKET_A), ("plug_b", SOCKET_B), (LOOSE, SOCKET_A)))
#: What scene.objects reports: the hub's box, and for each plug its housing (the part the jaw takes), whose centre
#: is 3.25 cm up the standing plug's axis; the loose plug's housing offset is along its body x (reported unrotated,
#: so its yaw is kept small).
TRUTH = {
    HUB: TruthObject(size=(0.26, 0.05, 0.035), centre=(0.0, 0.0, 0.0175)),
    "plug_a": TruthObject(size=(0.018, 0.012, 0.040), centre=(0.0, 0.0, 0.0325)),
    "plug_b": TruthObject(size=(0.018, 0.012, 0.040), centre=(0.0, 0.0, 0.0325)),
    LOOSE: TruthObject(size=(0.040, 0.018, 0.012), centre=(0.0325, 0.0, 0.0)),
}
TASKS = {
    "unplug": "Unplug all the cables from the cable set.",
    "plug": "Plug the loose cable into the free socket of the cable set.",
    "cycle": "Unplug a cable from the cable set, then plug it back into its socket.",
}
#: Where the seeded hub may lie: centre x and y on the bench, yaw about z. Both sockets stay within 0.21-0.31 m.
HUB_X = (0.23, 0.28)
HUB_Y = (-0.05, 0.05)
HUB_YAW = (-0.3, 0.3)
PARKED = {LOOSE: (0.55, -0.27, 0.006, 0.0), "plug_a": (0.52, 0.26, 0.0225, 0.0)}


def layout(seed: int, variant: str) -> dict[str, tuple[float, float, float, float]]:
    """Every free body's (x, y, z, yaw) for a trial seed: the hub, the plugs seated in it (or one parked away for
    the plug-in task), and the loose plug beside the free socket or parked at the far corner."""
    rng = random.Random(seed)
    hx, hy, yaw = rng.uniform(*HUB_X), rng.uniform(*HUB_Y), rng.uniform(*HUB_YAW)
    c, s = math.cos(yaw), math.sin(yaw)

    def on_hub(ox: float, oy: float, oz: float) -> tuple[float, float, float]:
        return (hx + ox * c - oy * s, hy + ox * s + oy * c, oz)

    out = {HUB: (hx, hy, 0.0, yaw)}
    for name, offset in (("plug_a", SOCKET_A), ("plug_b", SOCKET_B)):
        out[name] = (*on_hub(*offset), yaw)
    if variant == "plug":
        out["plug_a"] = PARKED["plug_a"]  # the near socket is free; the loose plug lies beside it
        side = rng.choice((-1.0, 1.0))
        lx, ly, _ = on_hub(SOCKET_A[0] + rng.uniform(-0.02, 0.02), side * rng.uniform(0.07, 0.09), 0.0)
        out[LOOSE] = (lx, ly, 0.006, yaw + rng.uniform(-0.15, 0.15))
    else:
        out[LOOSE] = PARKED[LOOSE]
    return out


class CableWorld(MujocoWorld):
    """The base world with a reset that places the hub and its plugs together, and a record of the highest z each
    body reached since the last reset (for the cycle judge)."""

    def __init__(self, *args, variant: str = "unplug", **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.variant = variant
        self._peak: dict[str, float] = {}

    def on_body_poses(self, msg) -> None:
        super().on_body_poses(msg)
        with self._changed:
            for transform in msg.transforms:
                name, z = transform.child_frame_id, transform.translation.z
                if z > self._peak.get(name, -math.inf):
                    self._peak[name] = z

    def peak_z(self, body: str) -> float:
        with self._changed:
            return self._peak.get(body, -math.inf)

    def reset(self, seed: int) -> None:
        # MujocoWorld.reset with the whole cable set placed at once instead of the one seeded body.
        self._arm.cancel()
        with self._changed:
            pose_counts = dict(self._pose_counts)
            joint_count = self._joint_count
        if not self._sim.reset(placements=layout(seed, self.variant)):
            raise RuntimeError("the simulation did not apply the reset")
        self._drive_home()
        self._wait(lambda: self._joint_count > joint_count, "a joint state after the reset", 10.0)
        self.wait_for_rest()
        self._wait_for_map()
        with self._changed:
            bodies = sorted(set(self._poses) | {self._body})
        for body in bodies:
            self._box_filter.clear_box(program_box(body))
        self._wait_for_poses(pose_counts)
        with self._changed:
            self._peak = {}


class CableLab(AutoresearchModule):
    """dimos's lab on the cable bench: the same session, skills and trial loop, with ``CableWorld`` for the resets."""

    VARIANT = "unplug"

    @rpc
    def start(self) -> None:
        super(AutoresearchModule, self).start()
        self._session = Session.create(self.config.session_dir, self.config.task, self.config.budget, self.config.model)
        self._env = bus_env(self.config.g)
        arm = _wait_for_arm(self._arm, PLANNER_TIMEOUT_S)
        self._world = CableWorld(
            self._sim, arm, self._control, self._box_filter, _home_preset(arm), self.config.judge_body,
            self.config.placement_bounds, self._stop, variant=self.VARIANT,
        )
        self.register_disposable(Disposable(self.body_poses.subscribe(self._world.on_body_poses)))
        self.register_disposable(Disposable(self.coordinator_joint_state.subscribe(self._world.on_joint_state)))
        self.register_disposable(Disposable(self.voxel_map.subscribe(self._world.on_voxel_map)))
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_frame)))
        self._world.home()
        self._emit(self.status())

    def _judge(self) -> Judge:
        return UnplugJudge(CABLES, PLUGS)

    def _success_rule(self) -> str:
        return ("at least one plug ends 4 cm or more out of its socket along the socket axis (held up counts) or"
                " clear of the socket and still on the bench; no plug on the floor; the hub moved under 1 cm."
                " The plugs are plug_a (nearer the arm) and plug_b, 5 cm apart along the hub; scene.objects"
                " reports each plug's housing box (1.8 x 1.2 x 4 cm, standing on the hub) and the hub's box.")


class PlugLab(CableLab):
    VARIANT = "plug"

    def _judge(self) -> Judge:
        return PlugJudge(CABLES, LOOSE, axis=0)

    def _success_rule(self) -> str:
        return ("plug_loose (lying on its side beside the hub, blade toward the arm's -x side of its housing) ends"
                " fully inserted in the hub's free near socket: within 4 mm of the socket axis, within 3 mm of full"
                " depth, within 10 degrees of vertical; the hub moved under 1 cm")


class CycleLab(CableLab):
    VARIANT = "cycle"

    def _judge(self) -> Judge:
        return UnplugPlugJudge(CABLES, PLUGS, self._world.peak_z)

    def _success_rule(self) -> str:
        return ("one plug rises at least 4 cm out of its socket during the trial and ends fully inserted back in"
                " it: within 4 mm of the socket axis, within 3 mm of full depth, within 10 degrees of vertical;"
                " the hub moved under 1 cm")


LABS = {"unplug": CableLab, "plug": PlugLab, "cycle": CycleLab}


def capture_blueprints(session: Path) -> list:
    """The motion recorder for offline renders (sim/render_replay.py), only when CAPTURE_MOTION=1."""
    if os.environ.get("CAPTURE_MOTION", "0") != "1":
        return []
    return [MotionCaptureModule.blueprint(out_path=str(session / "motion.jsonl"))]


def build(variant: str = "unplug"):
    sim_kwargs = make_metal_sim_module_kwargs()
    sim_kwargs.update(scene_xml=str(CABLE_BENCH), tracked_bodies=list(BODIES))
    model = make_metal_model_config(robot="arm", home_joints=METAL_SIM_HOME, tf_extra_links=METAL_TF_LINKS)
    session = RECORDINGS_DIR / f"autoresearch_cable_{variant}_{datetime.now():%Y%m%d_%H%M%S}"
    task = os.environ.get("CABLE_TASK") or TASKS[variant]
    metal_sim = autoconnect(
        MujocoSimModule.blueprint(
            **sim_kwargs, headless=True, fps=5, scene_camera_fps=10, pointcloud_fps=2, width=640, height=480
        ),
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
        *capture_blueprints(session),
        LABS[variant].blueprint(
            session_dir=session,
            task=task,
            judge_body="plug_a",  # the base world waits for this body's poses; CableWorld places the whole set
            program_timeout_s=300.0,
            placement_bounds=PlacementBounds(x=HUB_X, y=HUB_Y, z=0.0, yaw=HUB_YAW),  # documented; layout() decides
            budget=Budget(iterations=5, candidates_per_round=3, dev_trials=5, heldout_trials=10, threshold=0.8),
        ),
        McpServer.blueprint(),
    ).remappings([("recorder", "wrist_image", "color_image")])


#: The blueprints `dimos run` finds through the `dimos.blueprints` entry points (pyproject.toml).
unplug_lab = build("unplug")
plug_lab = build("plug")
cycle_lab = build("cycle")

if __name__ == "__main__":
    raise SystemExit("run it with the dimos CLI: dimos --simulation run phyhillclimb.autoresearch-cable-unplug")
