"""RelayTap: let the station's arm runner publish into the LiveKit relay room without touching its control loop.

The runner (`dream_right_arm_run.py`, the teammates' Dream environment, no LiveKit installed) hands the newest
camera frames, arm state and a status dict to `RelayTap.publish()`; a worker thread writes them to a tmpfs folder
(`/dev/shm/phyhill-relay` on Linux, `$RELAY_TAP_DIR` anywhere) as JPEGs plus `state.json`. A separate process,
`relay/station_bridge.py` in its own venv with `livekit-portal`, reads the folder and publishes it to the room as the
Portal robot (`overhead_camera`, `left_wrist_camera`, `right_wrist_camera`, the 14-value state) and answers the
dashboard's `status` RPC. If the bridge dies, the runner never notices: `publish()` never blocks, never raises.

    python tools/relay_tap.py --demo          # synthetic frames and a ticking status, for testing the bridge
"""

from __future__ import annotations

import json
import os
import pathlib
import tempfile
import threading
import time

ROLE_TO_TRACK = {"overhead": "overhead_camera", "wrist_l": "left_wrist_camera", "wrist_r": "right_wrist_camera"}
STATE_NAMES = (
    [f"left_joint{i}.pos" for i in range(1, 7)] + ["left_gripper.pos"]
    + [f"right_joint{i}.pos" for i in range(1, 7)] + ["right_gripper.pos"]
)
GRIPPER_MAX_MM = 80.0  # portal.yaml: the Metal stroke, 0 closed to 80 open
OUT_SIZE = (640, 360)  # what goes on the wire; the model's own 320x180 frames are made elsewhere


def default_dir() -> pathlib.Path:
    env = os.environ.get("RELAY_TAP_DIR")
    if env:
        return pathlib.Path(env)
    shm = pathlib.Path("/dev/shm")
    return (shm if shm.is_dir() else pathlib.Path(tempfile.gettempdir())) / "phyhill-relay"


def portal_state(right_joints, right_grip01, left_joints=None, left_grip01=None) -> dict[str, float]:
    """Runner units (joints rad, grip 0 closed..1 open) -> portal.yaml names (joints rad, gripper mm)."""
    left_joints = list(left_joints) if left_joints is not None else [0.0] * 6
    left_grip01 = 1.0 if left_grip01 is None else float(left_grip01)
    values = [float(q) for q in left_joints] + [left_grip01 * GRIPPER_MAX_MM]
    values += [float(q) for q in right_joints] + [float(right_grip01) * GRIPPER_MAX_MM]
    return dict(zip(STATE_NAMES, values, strict=True))


class RelayTap:
    def __init__(self, directory: pathlib.Path | None = None, fps: float = 15.0) -> None:
        self.dir = pathlib.Path(directory) if directory else default_dir()
        self.dir.mkdir(parents=True, exist_ok=True)
        self._fps = fps
        self._lock = threading.Lock()
        self._latest: tuple[dict, dict, dict] | None = None
        self._closing = threading.Event()
        self.written = 0
        self.errors = 0
        self._thread = threading.Thread(target=self._run, name="relay-tap", daemon=True)
        self._thread.start()

    def publish(self, frames: dict, state: dict, status: dict) -> None:
        """Keep only the newest sample; the worker writes at most `fps` per second. Never blocks the caller."""
        with self._lock:
            self._latest = (dict(frames), dict(state), dict(status))

    def close(self) -> None:
        self._closing.set()
        self._thread.join(timeout=1.0)
        with self._lock:
            sample, self._latest = self._latest, None
        if sample is not None:  # flush the last sample (the runner's final status)
            try:
                self._write(*sample)
            except Exception:  # noqa: BLE001
                self.errors += 1

    # worker --------------------------------------------------------------------------------------------------
    def _run(self) -> None:
        interval = 1.0 / self._fps
        while not self._closing.wait(interval):
            with self._lock:
                sample, self._latest = self._latest, None
            if sample is None:
                continue
            try:
                self._write(*sample)
                self.written += 1
            except Exception:  # noqa: BLE001 - the tap must never hurt the control loop
                self.errors += 1

    def _write(self, frames: dict, state: dict, status: dict) -> None:
        now = time.time()
        for role, rgb in frames.items():
            if rgb is None:
                continue
            track = ROLE_TO_TRACK.get(role, role)
            data = encode_jpeg(rgb)
            if data:
                atomic_write(self.dir / f"{track}.jpg", data)
        doc = {"t": now, "state": state, "status": status}
        atomic_write(self.dir / "state.json", json.dumps(doc).encode())


def atomic_write(path: pathlib.Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def encode_jpeg(rgb, size=OUT_SIZE, quality: int = 80) -> bytes:
    """RGB uint8 HxWx3 -> JPEG bytes at `size`, with OpenCV when present (the station), else Pillow."""
    try:
        import cv2

        small = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA) if rgb.shape[1] != size[0] else rgb
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(small, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buf.tobytes() if ok else b""
    except ImportError:
        import io

        from PIL import Image

        out = io.BytesIO()
        Image.fromarray(rgb).resize(size).save(out, "JPEG", quality=quality)
        return out.getvalue()


def _demo(seconds: float) -> None:
    """Synthetic cameras and a slowly moving arm, so the bridge and the dashboard can be tested without the station."""
    import numpy as np

    tap = RelayTap()
    print(f"[tap] demo writing to {tap.dir}")
    t0 = time.time()
    dropped = 0
    try:
        while not seconds or time.time() - t0 < seconds:
            t = time.time() - t0
            frames = {}
            for i, role in enumerate(("overhead", "wrist_r", "wrist_l")):
                img = np.full((360, 640, 3), 90 + 20 * i, dtype=np.uint8)
                x = int(320 + 200 * np.sin(t + i)); y = int(180 + 100 * np.cos(0.6 * t))
                img[max(0, y - 18): y + 18, max(0, x - 18): x + 18] = (230, 60, 60)
                frames[role] = img
            joints = [0.3 * np.sin(0.5 * t + k) for k in range(6)]
            grip = 0.5 + 0.5 * np.sin(0.8 * t)
            if int(t) % 20 == 0 and int(t) != dropped * 20:
                dropped = int(t) // 20
            status = {"mode": "demo", "phase": "model" if int(t) % 10 < 7 else "assist", "task": "unplug (demo)",
                      "stage": "unplug", "elapsed_s": round(t, 1), "budget_s": 120, "unplugged": dropped,
                      "event": f"demo tick {int(t)}", "rtt_s": 0.35}
            tap.publish(frames, portal_state(joints, grip), status)
            time.sleep(1 / 30)
    finally:
        tap.close()
        print(f"[tap] wrote {tap.written} samples, {tap.errors} errors")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--demo", action="store_true", help="write synthetic frames and status")
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()
    if args.demo:
        _demo(args.seconds)
    else:
        ap.error("nothing to do: --demo, or import RelayTap from the runner")
