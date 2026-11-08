"""Dream-1 control harness skeleton: loop + verifier + recovery + stopping rule.

Written before the MakerMods Physical AI Hackathon (2026-11-07) against dreamlabs-client 1.1.3.
Station specifics (arm driver, cameras, embodiment spans) are the TODOs; the loop mechanics are the part
that is easy to get subtly wrong and is checked offline with `python -m dream.check_loop`.

Run against the stand-in (no arm, no model):

    python -m dream.check_loop --rate 20          # terminal 1, answer "the workspace is clear"
    DREAM_GATEWAY_URL=http://127.0.0.1:8642 DREAM_API_KEY=x \
      python harness/dream_harness.py --fake --seconds 20   # terminal 2
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import numpy as np

import dream

# ---------------------------------------------------------------------------------------------------------------
# Embodiment. The YAM-shaped parts are what dreamlabs/dream-1-beta and check_loop understand. For a fine-tune on
# the station's arms, replace with the spans used in add_embodiment() and the roles used in its `video` map.
# ---------------------------------------------------------------------------------------------------------------
PARTS = {"right_arm": [0, 6], "right_gripper": [6, 7], "left_arm": [7, 13], "left_gripper": [13, 14]}
ROLES = ("overhead", "wrist_l", "wrist_r")
EMBODIMENT = {"state": PARTS, "action": PARTS, "video": {r: f"observation.images.{r}" for r in ROLES}}


def metal_embodiment() -> dict:
    """Portal's layout for dream-1-beta on a Metal station. For a hackathon fine-tune on the station, use the kit's
    `src/dream_hackathon/embodiments/makermods.json` instead (dreamlabs-co/physical-ai-hack-2026): left arm first,
    eef parts filled by tools/add_eef.py, video roles scene / wrist_l / wrist_r; it is what `hackathon-maker` registers.

    The MakerMods Metal station body, as livekit-examples/portal-makermods sends it to dream-1-beta:
    per side (right, then left) arm(6) grip(1) eef_xyz(3) eef_rot6(6); the client's IK fills the eef parts
    with `policy({..., "ik": {"arm": "metal", "head": "eef"}})`. Grippers are 0..1 (mm / GRIPPER_MAX_MM)."""
    spans, start = {}, 0
    for side in ("right", "left"):
        for part, width in (("arm", 6), ("grip", 1), ("eef_xyz", 3), ("eef_rot6", 6)):
            spans[f"{side}_{part}"] = [start, start + width]
            start += width
    return {"state": spans, "action": spans, "video": {r: f"observation.images.{r}" for r in ROLES}}
FRAME_SIZE = (320, 176)  # what the model reads; resize before encoding
JPEG_QUALITY = 85


class Robot(Protocol):
    def read_state(self) -> dict[str, np.ndarray]:
        """`state.<part>` arrays: joints in radians, grippers 0..1."""

    def command(self, targets: dict[str, np.ndarray]) -> None:
        """One row of targets, same keys as read_state() but under `action.<part>`."""

    def safe_pose(self) -> None:
        """Lift clear of the scene, jaws open. Used by recovery."""


class Cameras(Protocol):
    def frames(self) -> dict[str, str]:
        """`video.<role>` -> base64 JPEG (320x176, q85, no data: prefix)."""


# ---------------------------------------------------------------------------------------------------------------
# Fakes, so the loop runs against check_loop with nothing plugged in.
# ---------------------------------------------------------------------------------------------------------------
class FakeRobot:
    """Tracks commands with a little lag, like a position-controlled arm."""

    def __init__(self) -> None:
        self.q = {p: np.zeros(e - s, dtype=np.float32) for p, (s, e) in PARTS.items()}
        for g in ("right_gripper", "left_gripper"):
            self.q[g][:] = 1.0  # open
        self.target = {p: v.copy() for p, v in self.q.items()}

    def read_state(self) -> dict[str, np.ndarray]:
        for p in self.q:
            self.q[p] += 0.9 * (self.target[p] - self.q[p])
        return {f"state.{p}": self.q[p].copy() for p in self.q}

    def command(self, targets: dict[str, np.ndarray]) -> None:
        for k, v in targets.items():
            self.target[k.removeprefix("action.")] = np.asarray(v, dtype=np.float32)

    def safe_pose(self) -> None:
        for p in self.target:
            self.target[p][:] = 1.0 if "gripper" in p else 0.0


class FakeCameras:
    def __init__(self) -> None:
        from PIL import Image

        buf = io.BytesIO()
        Image.new("RGB", FRAME_SIZE, (90, 90, 90)).save(buf, format="JPEG", quality=JPEG_QUALITY)
        self._jpeg = base64.b64encode(buf.getvalue()).decode()

    def frames(self) -> dict[str, str]:
        return {f"video.{r}": self._jpeg for r in ROLES}


def encode_frame(rgb: np.ndarray) -> str:
    """Resize an RGB array to the model's frame and return base64 JPEG."""
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(rgb).resize(FRAME_SIZE).save(buf, format="JPEG", quality=JPEG_QUALITY)
    return base64.b64encode(buf.getvalue()).decode()


# ---------------------------------------------------------------------------------------------------------------
# Harness proper: subgoals, verifier, recovery, stopping rule.
# ---------------------------------------------------------------------------------------------------------------
@dataclass
class Plan:
    """A ladder of prompts from the rubric: the full task first, simpler sub-prompts as fallbacks."""

    prompts: list[str]
    index: int = 0

    @property
    def prompt(self) -> str:
        return self.prompts[self.index]

    def simplify(self) -> bool:
        if self.index + 1 < len(self.prompts):
            self.index += 1
            return True
        return False


class Verifier:
    """Decides done / progress / stuck between chunks. v0 is a stall detector.

    TODO v1: ask a VLM (Claude or Gemini) "Is <rubric level> true in this image?" on the overhead frame, off the
    control thread, and cache the verdict per chunk. Never block the loop on it.
    """

    def __init__(self, stall_eps: float = 0.01, stall_chunks: int = 2) -> None:
        self.stall_eps, self.stall_chunks = stall_eps, stall_chunks
        self._last: np.ndarray | None = None
        self._still = 0

    def verdict(self, state: dict[str, np.ndarray], frames: dict[str, str]) -> str:
        q = np.concatenate([state[f"state.{p}"] for p in PARTS])
        if self._last is not None and float(np.abs(q - self._last).max()) < self.stall_eps:
            self._still += 1
        else:
            self._still = 0
        self._last = q
        if self._still >= self.stall_chunks:
            return "stuck"
        return "progress"  # "done" comes from the VLM in v1


@dataclass
class Episode:
    prompt_strikes: int = 0
    chunks: int = 0
    recoveries: int = 0
    log: list[dict] = field(default_factory=list)


# ---------------------------------------------------------------------------------------------------------------
# The loop.
# ---------------------------------------------------------------------------------------------------------------
def chunk_rows(action: dict) -> list[dict[str, np.ndarray]]:
    """Turn a get_action() reply into rows of `action.<part>` targets, whatever the model's reply shape."""
    if "per_key" in action and action["per_key"]:  # fine-tuned model
        per = {k: np.asarray(v) for k, v in action["per_key"].items() if k.startswith("action.")}
        n = min(len(v) for v in per.values())
        return [{k: v[i] for k, v in per.items()} for i in range(n)]
    rows = []
    r, l = np.asarray(action["q_r_chunk"]), np.asarray(action["q_l_chunk"])
    for i in range(min(len(r), len(l))):
        rows.append(
            {
                "action.right_arm": r[i, :-1],
                "action.right_gripper": r[i, -1:],
                "action.left_arm": l[i, :-1],
                "action.left_gripper": l[i, -1:],
            }
        )
    return rows


def run_episode(
    client: dream.DreamClient,
    robot: Robot,
    cams: Cameras,
    plan: Plan,
    verifier: Verifier,
    *,
    action_hz: int = 20,
    camera_hz: int = 10,
    seconds: float = 60.0,
    max_recoveries: int = 3,
    log_path: Path | None = None,
) -> Episode:
    ep = Episode()
    stop = threading.Event()
    latest_frames: dict[str, str] = {}
    lock = threading.Lock()

    def camera_loop() -> None:
        t0 = time.monotonic()
        while not stop.is_set():
            frames = cams.frames()
            state = robot.read_state()
            with lock:
                latest_frames.update(frames)
            client.set_observation(frames, {**state, "prompt": plan.prompt}, timestamp=time.monotonic() - t0)
            time.sleep(1.0 / camera_hz)

    def jsonable(o: object) -> object:
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        return str(o)

    def log(**kw: object) -> None:
        rec = {"t": round(time.monotonic(), 3), **kw}
        ep.log.append(rec)
        if log_path:
            with log_path.open("a") as f:
                f.write(json.dumps(rec, default=jsonable) + "\n")

    client.on_round_trip(lambda rt: log(event="round_trip", **{k: v for k, v in vars(rt).items() if not k.startswith("_")}))

    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()
    time.sleep(0.3)  # one tick in the window before the first call

    pool = ThreadPoolExecutor(max_workers=1)

    def ask() -> tuple[float, list[dict[str, np.ndarray]]]:
        asked_at = time.monotonic()  # row k of the reply is due k / action_hz after this
        return asked_at, chunk_rows(client.get_action())

    pending: Future | None = pool.submit(ask)
    chunk: tuple[float, list[dict[str, np.ndarray]], int] | None = None  # (asked_at, rows, first row played)
    deadline = time.monotonic() + seconds
    tick = 1.0 / action_hz

    def restart() -> None:
        nonlocal pending, chunk
        client.reset_episode()
        time.sleep(0.5)
        chunk, pending = None, pool.submit(ask)

    try:
        while time.monotonic() < deadline:
            t_tick = time.monotonic()
            # 1. Install a reply when it lands, past the rows the round trip used up (time-indexed, as the
            #    official portal-makermods loop does it).
            if pending is not None and pending.done():
                try:
                    asked_at, rows = pending.result()
                except dream.DreamError as e:
                    code = getattr(e, "code", None) or getattr(e, "reason", None)
                    log(event="refused", error=str(e), code=str(code))
                    time.sleep(float(getattr(e, "retry_after_s", 0.7) or 0.7))
                    pending = pool.submit(ask)
                    continue
                pending = None
                ep.chunks += 1
                first = min(len(rows) - 1, int((time.monotonic() - asked_at) * action_hz))
                chunk = (asked_at, rows, first)
                # 2. Harness: verify between chunks, recover or stop.
                with lock:
                    frames = dict(latest_frames)
                verdict = verifier.verdict(robot.read_state(), frames)
                log(event="verdict", verdict=verdict, chunk=ep.chunks, prompt=plan.prompt)
                if verdict == "done":
                    break  # hold where we are; the stopping rule
                if verdict == "stuck":
                    ep.recoveries += 1
                    ep.prompt_strikes += 1
                    robot.safe_pose()
                    if ep.prompt_strikes >= 2 and plan.simplify():
                        ep.prompt_strikes = 0
                        log(event="simplify", prompt=plan.prompt)
                    if ep.recoveries > max_recoveries:
                        log(event="give_up")
                        break
                    restart()
                    continue
            # 3. Play the row that is due now.
            played = remaining = 0
            if chunk is not None:
                asked_at, rows, first = chunk
                due = int((time.monotonic() - asked_at) * action_hz)
                if due < len(rows):
                    robot.command(rows[due])
                    played, remaining = due - first, len(rows) - due
            # 4. Ask for the next chunk when the pacer says so (async: early, so it lands before this runs out).
            if pending is None and chunk is not None and client.refill_due(executed=played, remaining=remaining):
                pending = pool.submit(ask)
            time.sleep(max(0.0, tick - (time.monotonic() - t_tick)))
    finally:
        stop.set()
        pool.shutdown(wait=False, cancel_futures=True)
        client.reset_episode()
    return ep


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default="dreamlabs/dream-1-beta")
    ap.add_argument("--prompt", default="pick up the cube and place it in the bowl")
    ap.add_argument("--fallback", action="append", default=[], help="simpler prompt(s) to fall back to")
    ap.add_argument("--fake", action="store_true", help="fake robot and cameras (for check_loop)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--log", default="runs/episode.jsonl")
    ap.add_argument("--no-recover", action="store_true", help="disable the stall verifier (check_loop holds on purpose)")
    args = ap.parse_args(argv)

    if not args.fake:
        print("TODO: plug in the station's Robot and Cameras implementations", file=sys.stderr)
        return 2
    robot, cams = FakeRobot(), FakeCameras()

    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with dream.DreamClient() as client:
        client.policy(
            {
                "checkpoint": args.checkpoint,
                "embodiment": EMBODIMENT,
                "strict": False,  # check_loop answers a YAM; turn strict on for a real fine-tune
                "action_hz": 20,
                "execution_horizon": 30,
                "timeout_ms": 45000,
            }
        )
        ep = run_episode(
            client,
            robot,
            cams,
            Plan([args.prompt, *args.fallback]),
            Verifier(stall_chunks=10**9) if args.no_recover else Verifier(),
            seconds=args.seconds,
            log_path=log_path,
        )
    print(f"chunks={ep.chunks} recoveries={ep.recoveries} log={log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
