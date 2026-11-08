"""The station bridge: publish the real arm into the LiveKit relay room as the Portal robot.

    python -m relay.station_bridge                      # reads /dev/shm/phyhill-relay (or $RELAY_TAP_DIR)
    python -m relay.station_bridge --room relay-nuck4 --seconds 600

It reads what `tools/relay_tap.py` writes from inside the arm runner (JPEG per camera plus `state.json` with the
14-value state and the runner's status) and publishes it through `livekit.portal.Robot`: three VP8 camera tracks
and the state at portal.yaml's rate, exactly what `relay/robot_sim.py` publishes for the fake robot, so the Relay
Director, `web/viewer.html` and `relay.peek` see the real station and the fake one the same way. It answers the
dashboard's `status` RPC with the runner's status mapped to the Director's document (phase, stage, points,
clock, advice).

Safety: the bridge is publish-only. It never commands the arm: actions from an operator in the room are counted and
ignored, the runner keeps driving the motors itself. The bridge runs in its own venv (`~/relay-env` on the station)
and its own process, so it cannot stall or crash the control loop.
"""

from __future__ import annotations

import argparse
import asyncio
import io
import json
import pathlib
import time

import numpy as np
from livekit.portal import Action, Robot, RobotConfig
from PIL import Image

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

TRACKS = ("overhead_camera", "left_wrist_camera", "right_wrist_camera")
STALE_S = 2.0
CLOCK_S = 300.0
POINTS = {"unplug_pass": 10, "unplug_extra": 5, "stack_pass": 20}


def default_tap_dir() -> pathlib.Path:
    import os
    import tempfile

    env = os.environ.get("RELAY_TAP_DIR")
    if env:
        return pathlib.Path(env)
    shm = pathlib.Path("/dev/shm")
    return (shm if shm.is_dir() else pathlib.Path(tempfile.gettempdir())) / "phyhill-relay"


class TapReader:
    """Newest sample from the tap folder; each file is re-read only when its mtime changes."""

    def __init__(self, directory: pathlib.Path) -> None:
        self.dir = directory
        self._mtimes: dict[str, float] = {}
        self.frames: dict[str, np.ndarray] = {}
        self.state: dict[str, float] = {name: 0.0 for name in STATE_NAMES}
        self.state["left_gripper.pos"] = self.state["right_gripper.pos"] = GRIPPER_MAX_MM
        self.status: dict = {}
        self.sample_t = 0.0
        self.frames_read = 0

    def poll(self) -> None:
        path = self.dir / "state.json"
        if self._changed(path):
            try:
                doc = json.loads(path.read_bytes())
                self.state.update({k: float(v) for k, v in doc.get("state", {}).items() if k in self.state})
                self.status = doc.get("status", {}) or {}
                self.sample_t = float(doc.get("t", 0.0))
            except (OSError, ValueError):
                pass
        for track in TRACKS:
            jpg = self.dir / f"{track}.jpg"
            if self._changed(jpg):
                try:
                    with Image.open(io.BytesIO(jpg.read_bytes())) as im:
                        self.frames[track] = np.ascontiguousarray(np.asarray(im.convert("RGB")))
                    self.frames_read += 1
                except (OSError, ValueError):
                    pass

    def _changed(self, path: pathlib.Path) -> bool:
        try:
            mtime = path.stat().st_mtime_ns
        except OSError:
            return False
        if self._mtimes.get(path.name) == mtime:
            return False
        self._mtimes[path.name] = mtime
        return True

    def stale(self) -> bool:
        return time.time() - self.sample_t > STALE_S


def placeholder(track: str, size=(640, 360)) -> np.ndarray:
    """A dark frame for a camera the runner does not open (the left wrist in single-arm runs)."""
    img = np.full((size[1], size[0], 3), 24, dtype=np.uint8)
    img[: 12, : 12 * (TRACKS.index(track) + 1)] = (60, 60, 60)
    return img


def status_document(reader: TapReader, ignored_actions: int) -> dict:
    """The runner's status in the Director's vocabulary, which web/viewer.html renders."""
    s = reader.status
    stale = reader.stale()
    task = str(s.get("task", "")).lower()
    stage = s.get("stage") or ("stack" if "stack" in task else "unplug" if task else "")
    unplugged = int(s.get("unplugged", 0) or 0)
    placed = int(s.get("placed", 0) or 0)
    points = 0
    passed = []
    if unplugged:
        points += POINTS["unplug_pass"] + POINTS["unplug_extra"] * (unplugged - 1)
        passed.append("unplug")
    if placed >= 2:  # three layers = two placements on the base block
        points += POINTS["stack_pass"]
        passed.append("stack")
    elapsed = float(s.get("elapsed_s", 0.0) or 0.0)
    clock_left = max(0.0, CLOCK_S - elapsed)
    bonus = int(clock_left // 2) if {"unplug", "stack"} <= set(passed) else 0
    if stale:
        phase, advice = "stale", "runner not publishing (bridge sees no fresh sample)"
    else:
        phase = f"{s.get('mode', 'live')}:{s.get('phase', 'running')}"
        advice = s.get("event") or ""
        if {"unplug", "stack"} <= set(passed):
            advice = f"CALL STOP NOW: {points + bonus} points if stopped now"
    return {
        "phase": phase,
        "stage": stage,
        "stages_passed": passed,
        "points": points,
        "clock_left_s": round(clock_left, 1),
        "projected_if_stop_now": points + bonus,
        "advice": advice,
        "observations": reader.frames_read,
        "actions": ignored_actions,
        "last_verdict": f"unplugged {unplugged}, placed {placed}",
        "source": "station-bridge",
        "runner": s,
        "sample_age_s": round(time.time() - reader.sample_t, 2) if reader.sample_t else None,
    }


async def main() -> None:
    load_env()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--room", default=room_name())
    ap.add_argument("--tap-dir", default=str(default_tap_dir()))
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--identity", default=ROBOT_IDENTITY)
    args = ap.parse_args()

    reader = TapReader(pathlib.Path(args.tap_dir))
    cfg = RobotConfig.from_yaml_file(CONFIG_PATH, args.room)
    fps = config_fps()
    robot = Robot(cfg)
    ignored = {"actions": 0}

    def on_action(action: Action) -> None:  # publish-only: the runner owns the motors
        ignored["actions"] += 1

    def on_operator_changed(identity: str | None) -> None:
        print(f"[bridge] active operator {identity!r}: actions are ignored, the station runner drives the arm", flush=True)

    robot.on_action(on_action)
    robot.on_active_operator_changed(on_operator_changed)
    robot.register_rpc_method("status", lambda data: json.dumps(status_document(reader, ignored["actions"])))
    # On the shared station only a pre-minted robot token is present (LIVEKIT_ROBOT_TOKEN), not the API secret.
    import os

    token = os.environ.get("LIVEKIT_ROBOT_TOKEN") or mint_token(args.identity, args.room)
    await robot.connect(livekit_url(), token)
    print(f"[bridge] in room {args.room!r} at {livekit_url()} as {args.identity!r}; tap {reader.dir}", flush=True)

    deadline = time.monotonic() + args.seconds if args.seconds else None
    placeholders = {t: placeholder(t) for t in TRACKS}
    sent = 0
    try:
        async for tick in pace(fps):
            if deadline and time.monotonic() > deadline:
                break
            reader.poll()
            ts = int(time.time() * 1e6)
            robot.send_state(dict(reader.state), timestamp_us=ts)
            for track in TRACKS:
                robot.send_video_frame(track, reader.frames.get(track, placeholders[track]), timestamp_us=ts)
            sent += 1
            if tick % int(fps * 5) == 0:
                doc = status_document(reader, ignored["actions"])
                print(f"[bridge] t={tick / fps:.0f}s sent={sent} frames_read={reader.frames_read} "
                      f"phase={doc['phase']} stage={doc['stage']} points={doc['points']} "
                      f"age={doc['sample_age_s']} operators={robot.operators()}", flush=True)
    finally:
        await robot.disconnect()
        robot.close()
        print(f"[bridge] done: {sent} ticks, {reader.frames_read} frames, {ignored['actions']} actions ignored", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
