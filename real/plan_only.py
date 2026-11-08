"""A backend that plans but never moves: ``run_program_real.py --plan-only``.

What the Arm API allows without executing: ``plan_to_poses`` and ``plan_to_joints`` are separate RPCs from
``execute``, so ``move_pose``, ``move_joints`` and ``home`` are planned from the arm's real current state, the
result checked (a refusal raises ``RobotCallError("PLANNING_FAILED")`` as on a live run) and the pending plan
cleared, never executed. ``move_linear`` has no plan-only form (the RPC plans and executes in one call, even with
``blocking=False``), so it is skipped and only logged; the gripper calls are skipped too.

Because nothing moves, every move is planned from the resting arm, and the program's own feedback (pose reads,
scene re-reads) sees the arm where it started. So this checks the wiring on the real stack (the arm's state,
the measured scene, map clears, keyframes) and whether the FIRST target of each kind is reachable and plans from
rest; it says nothing about the sequence. For that rehearse on the echo mock (`--simulation`) or the MuJoCo
measured-scene stack.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from dimos.autoresearch.live import backend as live_backend
from dimos.autoresearch.program import DimosBackend, RobotBackend, RobotCallError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.JointState import JointState


class PlanOnlyBackend:
    """Wraps the live ``DimosBackend``: reads, scene and keyframes pass through; motion is planned only."""

    def __init__(self, inner: DimosBackend) -> None:
        self._inner = inner
        self._arm = inner._arm
        self.planned = 0
        self.skipped = 0

    def __getattr__(self, name: str):  # joints, pose, objects, clear_object, restore, capture, gripper_range
        return getattr(self._inner, name)

    def _check(self, what: str, plan) -> None:
        try:
            self._arm.rpc.clear_planned_path()
        except Exception:
            pass
        if not plan.succeeded or plan.plan is None:
            raise RobotCallError("PLANNING_FAILED", f"plan-only: {what} refused: {plan}")
        self.planned += 1
        print(f"plan-only: {what} planned, not executed", flush=True)

    def move_pose(self, position: tuple[float, float, float], orientation: tuple[float, float, float, float] | None) -> None:
        rotation = self._arm.pose().orientation if orientation is None else Quaternion(*orientation)
        target = PoseStamped(frame_id="world", position=list(position), orientation=rotation)
        self._check(f"move_pose {tuple(round(v, 3) for v in position)}", self._arm.rpc.plan_to_poses({self._arm.info.id: target}))

    def move_joints(self, positions: Sequence[float]) -> None:
        target = JointState(name=list(self._arm.info.joint_names), position=[float(v) for v in positions])
        self._check("move_joints", self._arm.rpc.plan_to_joints({self._arm.info.id: target}))

    def home(self) -> None:
        presets = self._arm.state().joint_presets
        if "home" not in presets:
            raise RobotCallError("PLANNING_FAILED", "plan-only: no home preset")
        self._check("home", self._arm.rpc.plan_to_joints({self._arm.info.id: presets["home"]}))

    def move_linear(self, dx: float, dy: float, dz: float) -> None:
        self.skipped += 1
        print(f"plan-only: move_linear ({dx:.3f}, {dy:.3f}, {dz:.3f}) skipped (no plan-only form)", flush=True)

    def open_gripper(self) -> None:
        print("plan-only: gripper open skipped", flush=True)

    def close_gripper(self) -> None:
        print("plan-only: gripper close skipped", flush=True)

    def set_gripper(self, joint_value: float) -> None:
        print(f"plan-only: gripper set {joint_value:.3f} skipped", flush=True)


def backend(trial_dir: Path) -> RobotBackend:
    """The factory ``run_program`` names: the live backend, wrapped."""
    return PlanOnlyBackend(live_backend(trial_dir))  # type: ignore[return-value]
