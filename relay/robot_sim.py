"""A Portal robot with no hardware: the room's robot side, fed by a backend.

    python -m relay.robot_sim --backend fake            # synthetic frames and a tracking arm
    python -m relay.robot_sim --backend dimos           # the dimos Metal sim (TODO: wire the streams)

The fake backend is enough to develop and test the Relay Director end to end: it publishes
three camera frames (a moving marker on a grey bench) and a 14-value state, and its "arm" tracks
the active operator's actions with a little lag, like a position-controlled arm. On the real
station this process is replaced by Portal's own `uv run robot` from portal-makermods.
"""

from __future__ import annotations

import argparse
import asyncio
import time

import numpy as np
from livekit.portal import Action, Robot, RobotConfig

from relay.common import (
    CONFIG_PATH,
    GRIPPER_MAX_MM,
    ROBOT_IDENTITY,
    STATE_NAMES,
    config_fps,
    livekit_url,
    load_env,
    mint_token,
    pace,
    room_name,
)

WIDTH, HEIGHT = 640, 360


class FakeBackend:
    """Synthetic cameras and a tracking arm."""

    def __init__(self, gain: float = 0.5) -> None:
        self.gain = gain  # 1.0 tracks instantly; lower values mimic a position-controlled arm's lag
        self.state = {name: 0.0 for name in STATE_NAMES}
        self.state["left_gripper.pos"] = GRIPPER_MAX_MM
        self.state["right_gripper.pos"] = GRIPPER_MAX_MM
        self.target = dict(self.state)
        self._t0 = time.monotonic()

    def read(self) -> dict[str, float]:
        for k in self.state:
            self.state[k] += self.gain * (self.target[k] - self.state[k])
        return dict(self.state)

    def apply(self, action: dict[str, float]) -> None:
        for k, v in action.items():
            if k in self.target:
                self.target[k] = float(v)

    def frames(self) -> dict[str, np.ndarray]:
        t = time.monotonic() - self._t0
        base = np.full((HEIGHT, WIDTH, 3), 110, dtype=np.uint8)
        x = int(WIDTH / 2 + 120 * np.sin(t))
        y = int(HEIGHT / 2 + 60 * np.cos(0.7 * t))
        base[max(0, y - 15) : y + 15, max(0, x - 15) : x + 15] = (220, 40, 40)
        out = {}
        for i, name in enumerate(("overhead_camera", "left_wrist_camera", "right_wrist_camera")):
            frame = base.copy()
            frame[: 20 + 10 * i, :40] = (40, 200, 40)  # a per-camera tag so tracks are told apart
            out[name] = frame
        return out


async def main() -> None:
    load_env()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--backend", choices=("fake", "dimos"), default="fake")
    ap.add_argument("--room", default=room_name())
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = run until Ctrl-C)")
    ap.add_argument("--gain", type=float, default=0.5, help="fake arm tracking gain per tick (1.0 = instant)")
    args = ap.parse_args()

    if args.backend == "dimos":
        raise SystemExit("the dimos backend is not wired yet: run the fake backend, or Portal's robot on the station")
    backend = FakeBackend(gain=args.gain)

    cfg = RobotConfig.from_yaml_file(CONFIG_PATH, args.room)
    fps = config_fps()
    robot = Robot(cfg)
    latest: dict[str, float] = {}

    def on_action(action: Action) -> None:
        nonlocal latest
        latest = {k: float(v) for k, v in action.values.items()}

    def on_operator_changed(identity: str | None) -> None:
        nonlocal latest
        latest = {}
        print(f"[robot] active operator: {identity}", flush=True)

    robot.on_action(on_action)
    robot.on_active_operator_changed(on_operator_changed)
    await robot.connect(livekit_url(), mint_token(ROBOT_IDENTITY, args.room))
    print(f"[robot] in room {args.room!r} at {livekit_url()} with backend {args.backend}", flush=True)

    deadline = time.monotonic() + args.seconds if args.seconds else None
    try:
        async for tick in pace(fps):
            if deadline and time.monotonic() > deadline:
                break
            ts = int(time.time() * 1e6)
            if latest:
                backend.apply(latest)
            robot.send_state(backend.read(), timestamp_us=ts)
            for name, frame in backend.frames().items():
                robot.send_video_frame(name, frame, timestamp_us=ts)
            if tick % int(fps * 5) == 0:
                ops = robot.operators()
                print(f"[robot] t={tick / fps:.0f}s operators={ops} active={robot.active_operator()}", flush=True)
    finally:
        await robot.disconnect()
        robot.close()


if __name__ == "__main__":
    asyncio.run(main())
