"""Replay one recorded unplug demonstration on the right Metal arm, open loop, to debug the unplug.

The teammates recorded 10 unplug demos (leader-follower) on this station. Replaying one tells us whether the arm,
the jaw's grip and the recorded pull can free a seated plug when the strip sits where it sat in that demo, with no
model involved. Camera snapshots every 0.5 s show where the jaw lands.

    cd ~/Superfast-harness
    ~/dream-env/bin/python ~/PhyHillClimb-tools/replay_demo.py --episode 0 --dry-run          # prints, never moves
    ~/dream-env/bin/python ~/PhyHillClimb-tools/replay_demo.py --episode 0 --live --speed 0.5  # hand on the e-stop
    ... --live --pause-at-grasp     # stops just before the jaw closes and waits for Enter (look at the jaw first)

The arm first eases from where it is to the demo's first pose over a few seconds, then plays the recorded commands
(30 per second) at --speed, with a per-step cap. On exit the arms park the teammates' way.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

DATA = Path.home() / "hackathon_data/nuck4-right-arm-campaign-20261107/unplug-right/data/chunk-000"
MAX_STEP_RAD = 0.03  # per command, on top of the SDK's own cap


def grasp_index(grip_cmd) -> int | None:
    """First frame where the recorded jaw command drops below 5 (of 100) after having been above 30."""
    seen_open = False
    for k, g in enumerate(grip_cmd):
        seen_open = seen_open or g > 30
        if seen_open and g < 5:
            return k
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--speed", type=float, default=0.5, help="1.0 = recorded speed")
    ap.add_argument("--config", default=str(Path.home() / "physical-ai-hacks/configs/nuck4-subtasks.yaml"))
    ap.add_argument("--pause-at-grasp", action="store_true", help="stop 0.5 s before the jaw closes; Enter continues")
    ap.add_argument("--stop-after", type=float, default=None, help="end this many seconds (recorded time) after the grasp")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--live", action="store_true")
    a = ap.parse_args()
    if not a.live:
        a.dry_run = True

    import numpy as np
    import pyarrow.parquet as pq

    table = pq.read_table(DATA / f"episode_{a.episode:06d}.parquet")
    action = np.stack(table.column("action").to_pylist()).astype(float)  # 7 wide: joints rad, jaw 0-100
    state = np.stack(table.column("observation.state").to_pylist()).astype(float)
    g = grasp_index(action[:, 6])
    end = len(action) if (g is None or a.stop_after is None) else min(len(action), g + int(a.stop_after * 30))
    print(f"episode {a.episode}: {len(action)} frames ({len(action) / 30:.1f} s), grasp at "
          f"{'none' if g is None else f'{g / 30:.1f} s'}; playing to {end / 30:.1f} s at speed {a.speed}")
    if g is not None:
        print(f"recorded grasp pose {np.round(state[g, :6], 3).tolist()}, jaw held at {state[g + 30, 6] if g + 30 < len(state) else '?'} of 100")

    from dotenv import load_dotenv

    load_dotenv(Path.home() / "Superfast-harness/.env", override=True)
    from dream_hackathon import config as kit_config
    from embodiments.metal.teleop import MetalArms, validate_channels
    from embodiments.makermods.cameras import open_cameras

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from dream_right_arm_run import camera_sources, save_snapshots

    station = kit_config.load(Path(a.config).expanduser())
    validate_channels(station)
    log_dir = Path.home() / "PhyHillClimb-runs"
    log_dir.mkdir(exist_ok=True)
    stem = log_dir / f"replay_ep{a.episode}_{time.strftime('%Y%m%d_%H%M%S')}_{'live' if a.live else 'dry'}"
    snap_dir = stem
    snap_dir.mkdir()
    log = open(f"{stem}.jsonl", "a")
    print(f"log {log.name}; camera snapshots in {snap_dir}/")
    from dream_right_arm_run import open_cameras_16x9

    cameras = open_cameras_16x9(camera_sources(station, []))
    arms = MetalArms(station, energize=a.live)
    try:
        joints0, grips0 = arms.read()
        left = (list(joints0["left"]), float(grips0["left"]))
        cur = np.asarray(joints0["right"], dtype=float)
        jaw = float(grips0["right"])
        print(f"right arm at {np.round(cur, 3).tolist()} jaw {jaw:.2f}")

        def send(target, grip, note, t0):
            nonlocal cur
            step = np.clip(np.asarray(target) - cur, -MAX_STEP_RAD, MAX_STEP_RAD)
            cur = cur + step
            j, gm = arms.read()
            log.write(json.dumps({"t": round(time.monotonic() - t0, 3), "phase": note,
                                  "target": [round(float(v), 4) for v in cur], "grip": round(float(grip), 3),
                                  "measured": [round(float(v), 4) for v in j["right"]],
                                  "grip_measured": round(float(gm["right"]), 4)}) + "\n")
            if a.live:
                arms.command({"right": cur.tolist(), "left": left[0]}, {"right": float(grip), "left": left[1]})

        t0 = time.monotonic()
        if a.live:  # let every motor sync at rest (see dream_right_arm_run.py)
            for _ in range(30):
                arms.command({"right": cur.tolist(), "left": left[0]}, {"right": jaw, "left": left[1]})
                time.sleep(0.05)
        # ease to the demo's first pose
        first = action[0, :6]
        while np.abs(first - cur).max() > 0.005:
            send(first, action[0, 6] / 100.0, "ease-in", t0)
            time.sleep(1 / 30)
        save_snapshots(cameras, snap_dir, "000.0_start")
        period = 1.0 / (30.0 * a.speed)
        next_snap = 0.0
        for k in range(end):
            tick = time.monotonic()
            if a.pause_at_grasp and g is not None and k == max(0, g - 15):
                save_snapshots(cameras, snap_dir, f"{k / 30:05.1f}_before_grasp")
                input("At the grasp pose, jaw still open. Look at the jaw and the plug, then press Enter to close and pull "
                      "(Ctrl-C to stop): ")
            send(action[k, :6], action[k, 6] / 100.0, "grasp" if g is not None and g <= k < g + 30 else "replay", t0)
            if tick - t0 >= next_snap:
                next_snap = tick - t0 + 0.5
                save_snapshots(cameras, snap_dir, f"{k / 30:05.1f}")
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
        save_snapshots(cameras, snap_dir, f"{end / 30:05.1f}_end")
        jm = arms.read()[1]["right"]
        print(f"done: jaw measured {jm:.3f} (0 closed, 1 open; above ~0.03 while closed means something is in it)")
        return 0
    finally:
        log.close()
        for cam in cameras.values():
            try:
                cam.close()
            except Exception:  # noqa: BLE001
                pass
        arms.close()


if __name__ == "__main__":
    sys.exit(main())
