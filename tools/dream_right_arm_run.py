"""Run the single-arm Dream-1 fine-tune (embodiment `metal-right-arm-7`) on the station's right Metal arm.

Runs on the station inside the teammates' environment, which owns the CAN drivers and cameras:

    cd ~/Superfast-harness
    ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_right_arm_run.py --check                       # imports + config only
    ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_right_arm_run.py --task "Unplug all the cables from the cable set." --dry-run
    ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_right_arm_run.py --task "..." --live --seconds 60   # a person on the e-stop

Dry run opens the cameras and reads the arms, asks the model and logs every target it would send; nothing is
commanded. Live mode energizes both arms through MetalArms (the teammates' class: soft limits, step caps, park on
exit), holds the left arm where it is and plays the model's right-arm rows. The model answers `per_key`
`action.right_arm` (6 joints, rad) and `action.right_gripper` (normalized stroke x 100, as recorded).
"""

from __future__ import annotations

import argparse
import base64
import dataclasses
import json
import os
import sys
import threading
import time
from pathlib import Path

EMBODIMENT = {
    "state": {"right_arm": [0, 6], "right_gripper": [6, 7]},
    "action": {"right_arm": [0, 6], "right_gripper": [6, 7]},
    "video": {"overhead": "observation.images.scene", "wrist_r": "observation.images.wrist"},
    "head_camera": "overhead",
}
DEFAULT_CHECKPOINT = os.environ.get("DREAM_CHECKPOINT", "")
SHORT_SIDE = 240
MAX_STEP_RAD = 0.06  # per row, on top of the SDK's own cap


def jpeg_b64(rgb, short_side: int) -> str:
    import cv2

    h, w = rgb.shape[:2]
    scale = short_side / min(h, w)
    small = cv2.resize(rgb, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(small, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    return base64.b64encode(buf.tobytes()).decode("ascii")


def camera_sources(station, overrides: list[str]) -> dict:
    """The overhead and right-wrist camera devices from the kit's StationConfig (a tuple of camera configs, each
    with a role and a device or serial), or from --camera role=source overrides."""
    sources = {}
    cams = getattr(station, "cameras", None) or ()
    entries = cams.items() if hasattr(cams, "items") else [(None, c) for c in cams]
    for key, cam in entries:
        get = (lambda n: cam.get(n)) if isinstance(cam, dict) else (lambda n: getattr(cam, n, None))
        role = key or get("role") or get("name") or get("id")
        device = get("device") or get("serial") or get("path")
        if role in ("overhead", "wrist_r") and device:
            sources[role] = str(device)
    for item in overrides:
        role, _, src = item.partition("=")
        sources[role] = src
    missing = [r for r in ("overhead", "wrist_r") if r not in sources]
    if missing:
        raise SystemExit(f"no camera source for {missing}; pass --camera role=/dev/video...")
    return sources


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path.home() / "physical-ai-hacks/configs/nuck4-subtasks.yaml"))
    ap.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT, help="dream-1-ft:<uid8>-right-arm-v1")
    ap.add_argument("--task", default="Unplug all the cables from the cable set.")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--camera", action="append", default=[], help="role=source override, e.g. wrist_r=/dev/video4")
    ap.add_argument("--log", default=str(Path.home() / "PhyHillClimb-runs"))
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="imports and config only; opens nothing")
    mode.add_argument("--dry-run", action="store_true", help="cameras and arm reads, model asked, nothing commanded")
    mode.add_argument("--live", action="store_true", help="energize the arms and play the model's rows")
    a = ap.parse_args()
    if not (a.check or a.dry_run or a.live):
        a.dry_run = True

    from dotenv import load_dotenv

    load_dotenv(Path.home() / "Superfast-harness/.env", override=True)
    from dream_hackathon import config as kit_config
    from embodiments.metal.teleop import MetalArms, validate_channels
    from embodiments.makermods.cameras import open_cameras

    station = kit_config.load(Path(a.config).expanduser())
    validate_channels(station)
    sources = camera_sources(station, a.camera)
    print(f"config ok: arm={station.arm} cameras={sources} checkpoint={a.checkpoint or '(none yet)'} mode={'check' if a.check else 'dry-run' if a.dry_run else 'LIVE'}")
    if a.check:
        import dream  # noqa: F401

        print("imports ok; nothing opened")
        return 0
    if not a.checkpoint:
        raise SystemExit("--checkpoint (or DREAM_CHECKPOINT) is required to run the model")

    import dream
    import numpy as np

    key = os.environ.get("DREAM_API_KEY") or os.environ.get("DREAMLABS_API_KEY")
    client = dream.DreamClient(api_key=key)
    client.policy({"checkpoint": a.checkpoint, "embodiment": EMBODIMENT, "action_hz": 20, "execution_horizon": 30,
                   "timeout_ms": 15000})
    hz = client.action_hz or 20.0
    log_dir = Path(a.log).expanduser(); log_dir.mkdir(parents=True, exist_ok=True)
    log = open(log_dir / f"right_arm_{time.strftime('%Y%m%d_%H%M%S')}_{'dry' if a.dry_run else 'live'}.jsonl", "a")

    cameras = open_cameras(sources, station)
    arms = MetalArms(station, energize=a.live)
    try:
        joints0, grips0 = arms.read()
        left_hold = (list(joints0["left"]), float(grips0["left"]))
        print(f"right arm at {np.round(joints0['right'], 3).tolist()} grip {grips0['right']:.2f}; left held")
        client.reset_episode()
        chunk, executed, pending, errors = None, 0, {}, []
        last_target = None

        def ask():
            try:
                pending["chunk"] = client.get_action()
            except Exception as failure:  # noqa: BLE001
                errors.append(failure)

        worker = None
        started = time.monotonic()
        while time.monotonic() - started < a.seconds:
            tick = time.monotonic()
            frames = {}
            for role, cam in cameras.items():
                rgb = cam.read()
                if rgb is not None:
                    frames[f"video.{role}"] = jpeg_b64(rgb, SHORT_SIDE)
            joints, grips = arms.read()
            obs = {"prompt": a.task, "state.right_arm": [float(q) for q in joints["right"]],
                   "state.right_gripper": [float(grips["right"]) * 100.0]}
            if len(frames) == 2:
                client.set_observation(frames, obs, timestamp=tick)
            if errors:
                err = errors[0]
                print(f"model error: {type(err).__name__}: {str(err)[:200]}")
                if "pool_warming" in str(err) or "DA-2000" in str(err):
                    errors.clear(); time.sleep(5); continue
                return 2
            if "chunk" in pending:
                reply = pending.pop("chunk")
                per_key = reply.get("per_key", reply)
                arm_rows = per_key.get("action.right_arm") or []
                grip_rows = per_key.get("action.right_gripper") or []
                chunk, executed = (arm_rows, grip_rows), 0
                print(f"chunk: {len(arm_rows)} rows")
            rows = len(chunk[0]) if chunk else 0
            idle = worker is None or not worker.is_alive()
            if idle and (chunk is None or client.refill_due(executed, rows - executed)):
                worker = threading.Thread(target=ask, name="dream-get-action", daemon=True); worker.start()
            if chunk is not None and executed < rows:
                target = [float(q) for q in chunk[0][executed][:6]]
                grip_units = float(chunk[1][executed][0]) if executed < len(chunk[1]) else grips["right"] * 100.0
                current = np.asarray(joints["right"], dtype=float)
                step = np.clip(np.asarray(target) - current, -MAX_STEP_RAD, MAX_STEP_RAD)
                target = (current + step).tolist()
                grip = min(1.0, max(0.0, grip_units / 100.0))
                log.write(json.dumps({"t": round(tick - started, 3), "row": executed, "target": [round(v, 4) for v in target],
                                      "grip": round(grip, 3), "measured": [round(float(v), 4) for v in current]}) + "\n")
                if a.live:
                    arms.command({"right": target, "left": left_hold[0]}, {"right": grip, "left": left_hold[1]})
                last_target = target
                executed += 1
            time.sleep(max(0.0, 1.0 / hz - (time.monotonic() - tick)))
        print(f"done: last target {np.round(last_target, 3).tolist() if last_target else None}")
        return 0
    finally:
        log.close()
        for cam in cameras.values():
            try: cam.close()
            except Exception: pass
        arms.close()  # live: parks the arms the teammates' way; dry: closes the readers


if __name__ == "__main__":
    sys.exit(main())
