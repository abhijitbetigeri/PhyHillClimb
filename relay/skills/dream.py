"""Dream-1 as a Relay Director skill: the official Portal loop behind the Skill interface.

Ported from livekit-examples/portal-makermods `src/policies/dream_1/run.py`: the Metal embodiment
(per side arm, grip, eef_xyz, eef_rot6), `ik: {arm: metal, head: eef}` so the client's IK turns the
model's tool poses into joints, three camera roles cropped 16:9 and resized to 320x176 JPEG, rows
indexed by time since the request, `refill_due` pacing, and a per-tick ramp on the commanded target.

Check it with no model and no arm, through the room:

    python -m dream.check_loop --rate 20                                   # terminal 1
    python -m relay.robot_sim --backend fake                                # terminal 2
    DREAM_GATEWAY_URL=http://127.0.0.1:8642 DREAM_API_KEY=x \\
      python -m relay.director --skill dream --seconds 40 --pass-after 999  # terminal 3
"""

from __future__ import annotations

import base64
import io
import threading
import time
import warnings
from concurrent.futures import Future, ThreadPoolExecutor

import numpy as np
from PIL import Image

from relay.common import GRIPPER_MAX_MM

ROW_HZ = 20  # chunk rows played per second
CAMERAS = {"overhead": "overhead_camera", "wrist_l": "left_wrist_camera", "wrist_r": "right_wrist_camera"}
SIDES = ("left", "right")
JOINTS = 6


def embodiment() -> dict:
    """Where each part sits in a state or action row; the client checks every request and reply against it."""
    spans, start = {}, 0
    for side in ("right", "left"):
        for part, width in (("arm", 6), ("grip", 1), ("eef_xyz", 3), ("eef_rot6", 6)):
            spans[f"{side}_{part}"] = [start, start + width]
            start += width
    return {"state": spans, "action": spans, "video": {role: f"observation.images.{role}" for role in CAMERAS}}


def encode_frame(rgb: np.ndarray) -> str:
    """Center-crop to 16:9, resize to 320x176, base64 JPEG q85 (what the model reads)."""
    h, w = rgb.shape[:2]
    if h * 16 > w * 9:
        top = (h - w * 9 // 16) // 2
        rgb = rgb[top : top + w * 9 // 16]
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb)).resize((320, 176), Image.BILINEAR).save(buf, "JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def arm_fields(side: str) -> list[str]:
    return [f"{side}_joint{i}.pos" for i in range(1, JOINTS + 1)] + [f"{side}_gripper.pos"]


def to_model(state: dict[str, float]) -> np.ndarray:
    """Portal state -> 14 model values: per arm (left, right) six joints in radians, then grip 0..1."""
    row = []
    for side in SIDES:
        *joints, gripper = arm_fields(side)
        row += [float(state[n]) for n in joints] + [min(max(float(state[gripper]) / GRIPPER_MAX_MM, 0.0), 1.0)]
    return np.array(row)


def to_robot(row: np.ndarray) -> dict[str, float]:
    """14 model values -> a Portal action (joints rad, gripper mm)."""
    action = dict(zip(arm_fields("left") + arm_fields("right"), map(float, row), strict=True))
    for side in SIDES:
        action[f"{side}_gripper.pos"] *= GRIPPER_MAX_MM
    return action


def chunk_rows(reply: dict) -> np.ndarray:
    """The reply's two chunks side by side, left then right: [rows, 14]."""
    import dream

    left, right = (np.asarray(reply[k], dtype=np.float64) for k in ("q_l_chunk", "q_r_chunk"))
    if left.shape != right.shape or left.ndim != 2 or left.shape[1] != 7 or not np.isfinite([left, right]).all():
        raise dream.DreamActionError(f"chunks of shape {left.shape} and {right.shape}; expected [rows, 7] each")
    return np.concatenate([left, right], axis=1)


class DreamSkill:
    """Runs Dream-1 for the current stage's prompt. `act` is called every room tick (~30 Hz)."""

    def __init__(
        self,
        checkpoint: str = "dreamlabs/dream-1-beta",
        *,
        fps: float = 30.0,
        max_joint_rad_s: float | None = None,
        dry_run: bool = False,
        log=print,
    ) -> None:
        import dream

        self._dream = dream
        self._fps = fps
        self._dry = dry_run
        self._log = log
        import os

        cap = max_joint_rad_s if max_joint_rad_s is not None else float(os.environ.get("DREAM_MAX_JOINT_RAD_S", 0.5))
        self._max_step = np.tile([cap / fps] * JOINTS + [0.05 * ROW_HZ / fps], 2)
        warnings.filterwarnings("ignore", category=dream.DreamSlowNetworkWarning)
        self._client = dream.DreamClient()
        self._client.policy(
            {"checkpoint": checkpoint, "embodiment": embodiment(), "ik": {"arm": "metal", "head": "eef"},
             "action_hz": ROW_HZ, "timeout_ms": 45000}
        )
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="dream")
        self._request: Future | None = None
        self._chunk: tuple[float, np.ndarray] | None = None  # (asked_at, rows); row k is due k / ROW_HZ after asked_at
        self._first = 0
        self._target: np.ndarray | None = None
        self._retry_at = 0.0
        self._prompt = ""
        self.calls = 0
        self.refusals = 0
        self._sent_ts = -1
        self._rtts: list[float] = []
        self._last_call_at = 0.0
        self.intervals: list[float] = []

    # Skill protocol -----------------------------------------------------------------------------------------
    def reset(self, stage) -> None:
        """A new stage: new prompt, fresh episode, ramp in from wherever the arm is."""
        self._prompt = stage.prompt
        self._cancel_request()
        self._client.reset_episode()
        self._chunk, self._target, self._retry_at = None, None, 0.0
        self._log(f"[dream] stage {stage.name}: prompt {stage.prompt!r}")

    def act(self, obs, frames: dict[str, np.ndarray]) -> dict[str, float] | None:
        if any(name not in frames for name in CAMERAS.values()):
            return None  # no complete view: send nothing, the arm holds
        if self._target is None:
            self._target = to_model(obs.state)
        if obs.timestamp_us != self._sent_ts:  # encode and send only a new observation, as the Portal loop does
            self._send_observation(obs, frames)
            self._sent_ts = obs.timestamp_us
        now = time.monotonic()
        self._collect_reply(now)
        row, played, remaining = None, 0, 0
        if self._chunk is not None:
            asked_at, rows = self._chunk
            due = int((now - asked_at) * ROW_HZ)
            if due < len(rows):
                row, played, remaining = rows[due], due - self._first, len(rows) - due
        if self._request is None and now >= self._retry_at and self._client.refill_due(played, remaining):
            if self._last_call_at:
                self.intervals.append(now - self._last_call_at)
            self._last_call_at = now
            self._request = self._pool.submit(self._ask)
        if row is None:
            return None
        self._target = self._target + np.clip(row - self._target, -self._max_step, self._max_step)
        action = to_robot(self._target)
        return None if self._dry else action

    def stats(self) -> str:
        rtt = sum(self._rtts) / len(self._rtts) if self._rtts else 0.0
        gap = sum(self.intervals) / len(self.intervals) if self.intervals else 0.0
        return f"dream calls {self.calls}, refusals {self.refusals}, mean rtt {rtt:.2f} s, mean call gap {gap:.2f} s"

    def close(self) -> None:
        self._cancel_request()
        self._pool.shutdown(wait=False, cancel_futures=True)
        self._client.close()

    # internals ----------------------------------------------------------------------------------------------
    def _send_observation(self, obs, frames: dict[str, np.ndarray]) -> None:
        encoded = {f"video.{role}": encode_frame(frames[track]) for role, track in CAMERAS.items()}
        row = to_model(obs.state)
        self._client.set_observation(
            encoded,
            {
                "prompt": self._prompt,
                "state.left_arm": row[:6].tolist(), "state.left_grip": row[6:7].tolist(),
                "state.right_arm": row[7:13].tolist(), "state.right_grip": row[13:].tolist(),
            },
            timestamp=obs.timestamp_us / 1e6,
        )

    def _ask(self) -> tuple[float, np.ndarray]:
        asked_at = time.monotonic()
        reply = self._client.get_action()  # one call at a time, on the pool's single thread
        return asked_at, chunk_rows(reply)

    def _collect_reply(self, now: float) -> None:
        if self._request is None or not self._request.done():
            return
        try:
            asked_at, rows = self._request.result()
            self.calls += 1
            self._rtts.append(now - asked_at)
            self._first = min(len(rows) - 1, int((now - asked_at) * ROW_HZ))
            self._chunk = (asked_at, rows)
        except Exception as error:  # noqa: BLE001 - refused or failed: wait, then ask again
            self.refusals += 1
            self._retry_at = now + self._retry_delay(error)
        self._request = None

    def _retry_delay(self, error: Exception) -> float:
        d = self._dream
        if isinstance(error, d.DreamSessionExpired):
            return 0.0
        if isinstance(error, d.DreamError) and not (
            isinstance(error, (d.DreamTimeout, d.DreamProtocolError)) or getattr(error, "retryable", False)
        ):
            reason = getattr(error, "reason", None) or getattr(error, "code", None)
            if reason not in ("pool_warming", "rate_limited"):
                raise error
        self._log(f"[dream] {getattr(error, 'reason', None) or error!r}; asking again")
        return float(getattr(error, "retry_after_s", None) or 1.0)

    def _cancel_request(self) -> None:
        if self._request is not None:
            try:
                self._request.result(timeout=5)
            except Exception:  # noqa: BLE001 - a request that fails while cancelling is not ours to report
                pass
            self._request = None
