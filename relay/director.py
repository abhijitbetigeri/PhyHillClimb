"""The Relay Director: a Portal operator that runs the Hill Climb relay stage by stage.

    python -m relay.director --seconds 20            # against relay/robot_sim.py or the station's robot

It claims control of the robot, runs the stage sequencer (unplug -> stack -> stop), asks a Skill
for the next action each tick from the latest observation, asks a Verifier whether the stage's
pass bar is met, enforces the clock and the DQ guard, and publishes a status document over an
RPC (`status`) that dashboards and the voice agent read. Skills and verifiers are pluggable; the
ones here are placeholders that exercise the loop (hold pose, pass after N seconds).

Rubric constants (AGENTS.md §1): 300 s clock, time bonus 1 point per 2 s left once stages 1 and 2
are passed and a teammate calls stop. The Director never moves the robot on a later stage's
objects before the current pass bar: the sequencer is the guard.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
from livekit.portal import Observation, Operator, OperatorConfig, frame_bytes_to_numpy_rgb

from relay.common import (
    CONFIG_PATH,
    DIRECTOR_IDENTITY,
    STATE_NAMES,
    config_fps,
    livekit_url,
    load_env,
    mint_token,
    pace,
    room_name,
)

CLOCK_S = 300.0
POINTS = {"unplug_pass": 10, "unplug_extra": 5, "plug_in": 20, "stack_pass": 20, "stack_extra": 10}


@dataclass
class Stage:
    name: str
    prompt: str
    pass_points: int
    max_s: float  # give up on the stage after this long (the relay continues only if passed)


RELAY = [
    Stage("unplug", "unplug one USB cable from the hub and lift it clear", POINTS["unplug_pass"], 90.0),
    Stage("stack", "stack the blocks three layers tall and let go", POINTS["stack_pass"], 150.0),
]


class Skill(Protocol):
    def reset(self, stage: Stage) -> None: ...

    def act(self, obs: Observation, frames: dict[str, np.ndarray]) -> dict[str, float] | None:
        """The next action values (portal.yaml names), or None to hold."""


class Verifier(Protocol):
    def reset(self, stage: Stage) -> None: ...

    def verdict(self, obs: Observation, frames: dict[str, np.ndarray]) -> str:
        """`passed`, `progress` or `stuck`."""


class HoldSkill:
    """Placeholder: hold the measured pose. Replace with the Dream-1 loop or a planner program."""

    def reset(self, stage: Stage) -> None:
        self._stage = stage

    def act(self, obs: Observation, frames: dict[str, np.ndarray]) -> dict[str, float] | None:
        return {name: float(obs.state[name]) for name in STATE_NAMES if name in obs.state}


class TimerVerifier:
    """Placeholder: the stage passes after `seconds`. Replace with the overhead-camera VLM check."""

    def __init__(self, seconds: float = 5.0) -> None:
        self._seconds = seconds

    def reset(self, stage: Stage) -> None:
        self._t0 = time.monotonic()

    def verdict(self, obs: Observation, frames: dict[str, np.ndarray]) -> str:
        return "passed" if time.monotonic() - self._t0 >= self._seconds else "progress"


@dataclass
class Status:
    phase: str = "idle"  # idle, running, done, stopped
    stage: str = ""
    stage_index: int = 0
    stages_passed: list[str] = field(default_factory=list)
    points: int = 0
    started_at: float = 0.0
    observations: int = 0
    actions: int = 0
    last_verdict: str = ""
    advice: str = ""

    def clock_left(self) -> float:
        return max(0.0, CLOCK_S - (time.monotonic() - self.started_at)) if self.started_at else CLOCK_S

    def projected(self) -> int:
        """Points if a teammate called stop right now."""
        bonus = int(self.clock_left() // 2) if {"unplug", "stack"} <= set(self.stages_passed) else 0
        return self.points + bonus

    def document(self) -> dict:
        return {
            "phase": self.phase,
            "stage": self.stage,
            "stages_passed": self.stages_passed,
            "points": self.points,
            "clock_left_s": round(self.clock_left(), 1),
            "projected_if_stop_now": self.projected(),
            "advice": self.advice,
            "observations": self.observations,
            "actions": self.actions,
            "last_verdict": self.last_verdict,
        }


def decode(obs: Observation) -> dict[str, np.ndarray]:
    out = {}
    for name, frame in obs.frames.items():
        try:
            out[name] = frame_bytes_to_numpy_rgb(frame.data, frame.width, frame.height)
        except Exception:  # noqa: BLE001 - a frame we cannot decode is skipped, the state still drives
            continue
    return out


async def run(op: Operator, skill: Skill, verifier: Verifier, status: Status, fps: float, seconds: float) -> None:
    latest: Observation | None = None

    def on_observation(obs: Observation) -> None:
        nonlocal latest
        latest = obs
        status.observations += 1

    op.on_observation(on_observation)
    me = op.local_identity()
    await op.set_active_operator(me)
    status.phase, status.started_at = "running", time.monotonic()
    stage_i = 0
    stage = RELAY[0]
    skill.reset(stage)
    verifier.reset(stage)
    status.stage = stage.name
    stage_t0 = time.monotonic()
    deadline = time.monotonic() + seconds if seconds else None
    last_print = 0.0

    async for _ in pace(fps):
        if (deadline and time.monotonic() > deadline) or status.clock_left() <= 0:
            status.phase, status.advice = "stopped", "clock or run limit reached"
            break
        if op.active_operator() != me:
            status.advice = "not the active operator: holding"
            continue
        obs = latest
        if obs is None:
            continue
        frames = decode(obs)
        action = skill.act(obs, frames)
        if action:
            op.send_action(action, in_reply_to_ts_us=obs.timestamp_us)
            status.actions += 1
        verdict = verifier.verdict(obs, frames)
        status.last_verdict = verdict
        if verdict == "passed":
            status.stages_passed.append(stage.name)
            status.points += stage.pass_points
            stage_i += 1
            if stage_i >= len(RELAY):
                status.phase = "done"
                status.advice = f"CALL STOP NOW: {status.projected()} points if stopped now"
                break
            stage = RELAY[stage_i]
            skill.reset(stage)
            verifier.reset(stage)
            status.stage, stage_t0 = stage.name, time.monotonic()
        elif time.monotonic() - stage_t0 > stage.max_s:
            status.phase, status.advice = "stopped", f"stage {stage.name} timed out; call stop"
            break
        if time.monotonic() - last_print > 2.0:
            last_print = time.monotonic()
            print("[director]", json.dumps(status.document()), flush=True)
        if hasattr(skill, "stats"):
            status.advice = status.advice or skill.stats()
    print("[director] final", json.dumps(status.document()), flush=True)


async def main() -> None:
    load_env()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--room", default=room_name())
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = the 300 s clock)")
    ap.add_argument("--pass-after", type=float, default=5.0, help="placeholder verifier: seconds per stage")
    ap.add_argument("--skill", choices=("hold", "dream"), default="hold")
    ap.add_argument("--checkpoint", default="dreamlabs/dream-1-beta")
    ap.add_argument("--dry-run", action="store_true", help="dream: call the model, send nothing to the robot")
    args = ap.parse_args()

    cfg = OperatorConfig.from_yaml_file(CONFIG_PATH, args.room)
    op = Operator(cfg)
    status = Status()
    op.register_rpc_method("status", lambda data: json.dumps(status.document()))
    await op.connect(livekit_url(), mint_token(DIRECTOR_IDENTITY, args.room))
    print(f"[director] in room {args.room!r}; robot={op.robot_identity()}", flush=True)
    fps = config_fps()
    if args.skill == "dream":
        from relay.skills.dream import DreamSkill

        skill = DreamSkill(args.checkpoint, fps=fps, dry_run=args.dry_run)
    else:
        skill = HoldSkill()
    try:
        await run(op, skill, TimerVerifier(args.pass_after), status, fps, args.seconds)
    finally:
        if hasattr(skill, "close"):
            skill.close()
        await op.disconnect()
        op.close()


if __name__ == "__main__":
    asyncio.run(main())
