"""Run the single-arm Dream-1 fine-tune (embodiment `metal-right-arm-7`) on the station's right Metal arm.

Runs on the station inside the teammates' environment, which owns the CAN drivers and cameras:

    cd ~/Superfast-harness
    ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_right_arm_run.py --check                       # imports + config only
    ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_right_arm_run.py --task "Using the right arm, unplug all the cables from the cable set." --dry-run
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
SEAM_ROWS = 6        # rows over which a new chunk is blended from the last commanded target (0.3 s at 20 Hz)
MIN_ASK_INTERVAL_S = 0.75  # Dream's rate limit: 3 calls, then one per 0.7 s

# Assisted pull. The fine-tune reaches the plug (its settled pose matches the demos' grasp poses) but never closes
# fully or pulls. In the 10 recorded unplug demos the jaw closes onto the plug (it stops at 5-17 of 100 because the
# plug is in it), then the shoulder lift rises 0.09-0.17 rad within 2 s and the pan often swings ~0.3 rad away.
ASSIST_STILL_S = 2.0          # the arm has been still this long ...
ASSIST_STILL_RAD = 0.03       # ... moving less than this on every joint ...
ASSIST_MIN_LIFT_DEPTH = -1.9  # ... with the shoulder this far down (the model sometimes settles at -1.95 over a plug)
ASSIST_HOLD_MIN = 0.03        # measured jaw (0 closed, 1 open) above this after closing = something is in it
ASSIST_LIFT_RAD = 0.18        # shoulder lift rise for the pull (demos: +0.09 to +0.17 in 2 s)
ASSIST_PAN_RAD = -0.30        # then swing the pan away with the plug (demos: about -0.3)


def save_snapshots(cameras, folder, stem) -> None:
    """Write the newest frame of every camera as <folder>/<stem>_<role>.jpg (small, for debugging)."""
    import cv2

    for role, cam in cameras.items():
        rgb = cam.read()
        if rgb is None:
            continue
        small = cv2.resize(rgb, (480, int(round(rgb.shape[0] * 480 / rgb.shape[1]))), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(folder / f"{stem}_{role}.jpg"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR),
                    [int(cv2.IMWRITE_JPEG_QUALITY), 80])


PULL_TORQUE_STEP_NM = 0.25   # raise the hold torque by this when the plug resists
PULL_RESIST_RAD = 0.05       # shoulder lagging its pull target by more than this = the plug is resisting
PULL_SLIP_JAW = 0.02         # measured jaw below this during the pull = the grip slipped (closed on nothing)
PULL_EXTRA_MAX_RAD = 0.20    # most the shoulder may run ahead of the demo pull: enough to free a plug, not a leap
DROP_PAN_RAD = -0.90         # swing the pan this far after the pull before opening the jaw: well clear of the strip
pull_hold_torque_max = 1.0   # set from --pull-hold-torque; the vendor's range is 0.1 to 2.0 N.m


def gripper_controller(arms, side: str = "right"):
    """The driver's gripper holding controller for one arm, or None if this driver does not expose one.
    Path: MetalArms._arms[side] (FollowerArm) ._driver (MotorArm) .chain .gripper (MetalGripper) .controller."""
    try:
        return arms._arms[side]._driver.chain.gripper.controller
    except AttributeError:
        return None


class HoldTorque:
    """Raises the gripper's hold torque for a pull and always restores it (the station config's value, 0.5 N.m)."""

    def __init__(self, arms, log, started):
        self.ctrl, self.log, self.started = gripper_controller(arms), log, started
        self.original = getattr(self.ctrl, "torque_nm", None)
        self.now = self.original

    def set(self, nm: float, why: str) -> None:
        if self.ctrl is None or self.original is None:
            return
        nm = max(0.1, min(2.0, pull_hold_torque_max, float(nm)))
        if abs(nm - (self.now or 0)) < 1e-6:
            return
        self.ctrl.torque_nm = nm
        self.now = nm
        self.log.write(json.dumps({"t": round(time.monotonic() - self.started, 3), "hold_torque_nm": nm, "why": why}) + "\n")
        print(f"assist: hold torque {nm:.2f} N.m ({why})")

    def restore(self) -> None:
        if self.ctrl is not None and self.original is not None:
            self.ctrl.torque_nm = self.original
            self.now = self.original


DEMO_DIR = Path.home() / "hackathon_data/nuck4-right-arm-campaign-20261107/unplug-right/data/chunk-000"
DEMO_SNAP_MAX_RAD = 1.5  # use a demo's grasp only if its pre-grasp pose is this close (largest joint gap)


def load_demo_grasps():
    """Per recorded unplug demo: the commanded pose 0.5 s before the jaw closes, and the recorded commands from there
    through the close and 3 s of pull (30 per second, joints rad + jaw 0-100). Demos without a clear grasp are skipped."""
    import numpy as np
    import pyarrow.parquet as pq

    grasps = []
    for path in sorted(DEMO_DIR.glob("episode_*.parquet")):
        action = np.stack(pq.read_table(path).column("action").to_pylist()).astype(float)
        seen_open, g = False, None
        for k, jaw in enumerate(action[:, 6]):
            seen_open = seen_open or jaw > 30
            if seen_open and jaw < 5:
                g = k
                break
        if g is None or g < 15 or g + 90 > len(action):
            continue
        pre = g - 15
        grasps.append({"episode": path.stem, "pre": action[pre, :6].copy(), "tail": action[pre:g + 90].copy()})
    return grasps


def assist_demo_grasp(arms, joints, left_hold, log, started, grasps):
    """Ease from where the model stopped into the closest recorded pre-grasp pose (jaw as the demo had it), then
    play that demo's own close and pull. Returns None when no demo is close enough."""
    import numpy as np

    pose = np.asarray(joints["right"], dtype=float)
    near = [d for d in grasps if np.abs(d["pre"] - pose).max() <= DEMO_SNAP_MAX_RAD]
    if not near:
        return None

    def lift(d):  # the demo's pull: shoulder rise over the 3 s after the jaw closes, if it barely pans
        rise = d["tail"][-1, 1] - d["tail"][15, 1]
        pan = abs(d["tail"][-1, 0] - d["tail"][15, 0])
        return rise if pan < 0.06 else -1.0

    best = max(near, key=lift)
    tail = best["tail"]
    cur = pose.copy()

    def send(target, grip, note):
        nonlocal cur
        cur = cur + np.clip(np.asarray(target) - cur, -0.03, 0.03)
        arms.command({"right": cur.tolist(), "left": left_hold[0]}, {"right": float(grip), "left": left_hold[1]})
        j, g = arms.read()
        log.write(json.dumps({"t": round(time.monotonic() - started, 3), "assist": note, "demo": best["episode"],
                              "target": [round(float(v), 4) for v in cur], "grip": round(float(grip), 3),
                              "measured": [round(float(v), 4) for v in j["right"]],
                              "grip_measured": round(float(g["right"]), 4)}) + "\n")
        return float(g["right"])

    # Play the demo's close and pull as motion RELATIVE to where the model stopped. Moving into the demo's absolute
    # pose first (the earlier version) carried the jaw off a plug the model had already straddled, because the strip
    # never sits exactly where it sat in the demo.
    # The arm also stays exactly still until the jaw has closed (rows 0-44: 0.5 s before the close plus the 1 s close);
    # only the pull itself (row 45 on) moves it, by the demo's motion from that moment.
    tail = tail.copy()
    pull_from = tail[44, :6].copy()
    tail[:45, :6] = pose
    tail[45:, :6] = pose + (tail[45:, :6] - pull_from)
    held = None
    torque = HoldTorque(arms, log, started)
    try:
        extra = 0.0  # extra shoulder lift added when the plug resists
        for k, row in enumerate(tail):  # the demo's own close and pull, at recorded speed
            if k < 15:
                continue  # skip the demo's 0.5 s of open jaw before the close: the arm is already in place
            pulling = k >= 45
            target = row[:6].copy()
            if pulling:
                if k == 45:
                    torque.set(max(0.5, (torque.original or 0.5) + 3 * PULL_TORQUE_STEP_NM), "pull starts")
                target[1] += extra
            jm = send(target, row[6] / 100.0, "demo-close" if not pulling else "demo-pull")
            if k == 44:
                held = jm
                if held < ASSIST_HOLD_MIN:  # closed on nothing: do not pull; reopen, back off a little, retry
                    lifted = pose + np.array([0, 0.05, 0, 0, 0, 0])
                    for _ in range(20):
                        send(pose, 0.9, "miss-reopen")
                        time.sleep(1 / 30)
                    for _ in range(20):
                        send(lifted, 0.9, "miss-backoff")
                        time.sleep(1 / 30)
                    return f"missed: demo {best['episode']}: jaw closed to {held:.3f} (nothing in it); back to the model"
            if pulling and k % 6 == 0:  # every 0.2 s: is the plug resisting, or did the grip slip?
                j = arms.read()[0]["right"]
                if jm < PULL_SLIP_JAW:
                    log.write(json.dumps({"t": round(time.monotonic() - started, 3), "assist": "slip", "grip_measured": jm}) + "\n")
                    print(f"assist: grip slipped during the pull (jaw {jm:.3f})")
                    break
                if target[1] - j[1] > PULL_RESIST_RAD and torque.now is not None:
                    torque.set(torque.now + PULL_TORQUE_STEP_NM, f"plug resisting, shoulder {target[1] - j[1]:.3f} rad behind")
                    extra = min(extra + 0.06, PULL_EXTRA_MAX_RAD)  # lean further into the pull
            time.sleep(1 / 30)
        else:
            # The demo's pull is over. If the jaw still holds and the shoulder still lags (the plug has not let
            # go), keep leaning in: the motor pulls harder the further its target runs ahead of it.
            last = tail[-1].copy()
            for k in range(90):  # at most 3 s
                j, g = arms.read()
                lag = (last[1] + extra) - j["right"][1]
                if g["right"] < PULL_SLIP_JAW:
                    print(f"assist: grip slipped during the sustained pull (jaw {g['right']:.3f})")
                    break
                if lag < PULL_RESIST_RAD / 2:
                    print(f"assist: shoulder caught up (lag {lag:.3f} rad): the plug has let go")
                    break
                if k % 6 == 0:
                    extra = min(extra + 0.06, PULL_EXTRA_MAX_RAD)
                    if torque.now is not None:
                        torque.set(torque.now + PULL_TORQUE_STEP_NM, f"sustained pull, shoulder {lag:.3f} rad behind")
                target = last[:6].copy()
                target[1] += extra
                send(target, last[6] / 100.0, "sustained-pull")
                time.sleep(1 / 30)
            else:
                print(f"assist: plug still holding after the sustained pull (extra lead {extra:.2f} rad)")
        # Drop the cable away from the strip, as the demos did (they swing the pan 0.2-0.5 rad negative after the
        # pull), so the jaw is free and the cable cannot fall back onto the strip.
        if arms.read()[1]["right"] >= PULL_SLIP_JAW:
            here = np.asarray(arms.read()[0]["right"], dtype=float)
            away = here + np.array([DROP_PAN_RAD, 0, 0, 0, 0, 0])  # swing right; the pull already lifted the plug clear
            for _ in range(120):  # up to 4 s; send() caps each step at 0.03 rad
                send(away, 0.0, "carry-away")
                time.sleep(1 / 30)
            torque.restore()  # let go at the station's normal hold torque
            for _ in range(30):
                send(away, 1.0, "drop")
                time.sleep(1 / 30)
            print(f"assist: cable dropped beside the strip (jaw {arms.read()[1]['right']:.3f})")
            # Come back down to just above the grasp height, so the model starts the next plug from a low pose
            # instead of the high one the pull ended at.
            low = away.copy()
            low[1] = pose[1] + 0.05
            for _ in range(60):
                send(low, 1.0, "lower")
                time.sleep(1 / 30)
                if np.abs(low - cur).max() < 0.005:
                    break
    finally:
        torque.restore()
    final = arms.read()[1]["right"]
    return (f"demo {best['episode']}: jaw stopped at {held:.3f} after closing "
            f"({'something in it' if held is not None and held > ASSIST_HOLD_MIN else 'closed on nothing'}), "
            f"{final:.3f} after the pull")


def assist_pull(arms, joints, left_hold, log, started) -> str:
    """Close the jaw fully where the model stopped; if the jaw stopped on something, play the demos' pull."""
    import numpy as np

    pose = np.asarray(joints["right"], dtype=float)

    def hold(target, grip, seconds, note):
        end = time.monotonic() + seconds
        first = True
        while first or time.monotonic() < end:  # always send at least one command per step
            first = False
            arms.command({"right": list(target), "left": left_hold[0]}, {"right": grip, "left": left_hold[1]})
            j, g = arms.read()
            log.write(json.dumps({"t": round(time.monotonic() - started, 3), "assist": note,
                                  "target": [round(float(v), 4) for v in target], "grip": grip,
                                  "measured": [round(float(v), 4) for v in j["right"]],
                                  "grip_measured": round(float(g["right"]), 4)}) + "\n")
            time.sleep(0.05)
        return arms.read()

    _, g = hold(pose, 0.0, 1.5, "close")
    held = float(g["right"])
    if held < ASSIST_HOLD_MIN:
        hold(pose, 0.9, 1.0, "reopen")  # closed on nothing: open again for the model's next try
        return f"missed (jaw closed to {held:.3f}); back to the model"
    steps = 40  # 2 s ramp for the lift, then 2 s for the pan, jaw shut throughout
    for k in range(1, steps + 1):
        hold(pose + np.array([0, ASSIST_LIFT_RAD * k / steps, 0, 0, 0, 0]), 0.0, 0.05, "lift")
    lifted = pose + np.array([0, ASSIST_LIFT_RAD, 0, 0, 0, 0])
    for k in range(1, steps + 1):
        hold(lifted + np.array([ASSIST_PAN_RAD * k / steps, 0, 0, 0, 0, 0]), 0.0, 0.05, "pan")
    _, g = hold(lifted + np.array([ASSIST_PAN_RAD, 0, 0, 0, 0, 0]), 0.0, 1.0, "hold")
    return f"pulled (jaw held at {held:.3f}, still {float(g['right']):.3f} after the pull)"


CAPTURE_SIZE = (1280, 720)  # as Dream's SDK and the kit's station config open them; the demos were 16:9
POLICY_SIZE = (320, 180)    # the frame size Dream's SDK sends (policy_image_size in the teammates' loop)


class MjpgCamera:
    """A webcam opened the way Dream's SDK opens it (robots/cameras.py): MJPG first, then 1280x720 at 30 fps.
    V4L2's default is uncompressed YUYV, which two USB 2 webcams on one hub cannot carry at 720p: frames drop to
    10 fps or less and arrive torn. The teammates' camera class sets the size without MJPG."""

    def __init__(self, source, size=CAPTURE_SIZE, fps: float = 30.0):
        import cv2

        self.cv2 = cv2
        self.capture = cv2.VideoCapture(int(source) if str(source).isdigit() else str(source), cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"camera {source!r} did not open")
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.frame, self.frames, self.closing = None, 0, threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"camera-{source}", daemon=True)
        self.thread.start()

    def _run(self):
        while not self.closing.is_set():
            ok, bgr = self.capture.read()
            if ok:
                self.frame = self.cv2.cvtColor(bgr, self.cv2.COLOR_BGR2RGB)
                self.frames += 1
            else:
                self.closing.wait(0.03)

    def read(self):
        return self.frame

    def close(self):
        self.closing.set()
        self.thread.join(timeout=1)
        self.capture.release()


def open_cameras_16x9(sources: dict):
    """Open each camera at 1280x720 MJPG, as Dream's SDK does. The teammates' camera class defaults to 640x480,
    and in that 4:3 mode these webcams crop and zoom, so the model saw a narrower view than the 16:9 demos."""
    cameras = {}
    try:
        for role, source in sources.items():
            cameras[role] = MjpgCamera(source)
    except Exception:
        for cam in cameras.values():
            cam.close()
        raise
    return cameras


def jpeg_b64(rgb, short_side: int | None = None) -> str:
    import cv2

    h, w = rgb.shape[:2]
    if abs(w / h - 16 / 9) > 0.02:
        raise RuntimeError(f"camera frame is {w}x{h}, not 16:9 like the demos; refusing to send a cropped view")
    small = cv2.resize(rgb, POLICY_SIZE, interpolation=cv2.INTER_AREA)
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
    ap.add_argument("--task", default="Using the right arm, unplug all the cables from the cable set.")
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--camera", action="append", default=[], help="role=source override, e.g. wrist_r=/dev/video4")
    ap.add_argument("--log", default=str(Path.home() / "PhyHillClimb-runs"))
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="imports and config only; opens nothing")
    mode.add_argument("--dry-run", action="store_true", help="cameras and arm reads, model asked, nothing commanded")
    mode.add_argument("--live", action="store_true", help="energize the arms and play the model's rows")
    ap.add_argument("--pull-hold-torque", type=float, default=1.0,
                    help="highest gripper hold torque during an assisted pull, N.m (0.5 is the station default; "
                         "vendor range 0.1-2.0). Calibrate upward across runs: 1.0, then 1.5, then 2.0")
    ap.add_argument("--unplugs", type=int, default=4, help="cables to unplug in one run (default 4)")
    ap.add_argument("--no-assist-pull", dest="assist_pull", action="store_false",
                    help="do not take over at the plug (by default, live runs close the jaw and play the recorded pull)")
    a = ap.parse_args()
    global pull_hold_torque_max
    if not 0.1 <= a.pull_hold_torque <= 2.0:
        raise SystemExit("--pull-hold-torque must be within the vendor's 0.1 to 2.0 N.m")
    pull_hold_torque_max = a.pull_hold_torque
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

    cameras = open_cameras_16x9(sources)
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
                asked_at = time.monotonic()
                rows_at_ask = played[0]  # rows executed when this observation went out (the RTC anchor)
                reply = client.get_action()
                pending["chunk"] = (reply, asked_at, rows_at_ask)
            except Exception as failure:  # noqa: BLE001
                errors.append(failure)

        worker = None
        next_ask_at = 0.0
        observed = False  # the model may only be asked after the first full observation
        warmup_until = time.monotonic() + 15.0
        while time.monotonic() < warmup_until:  # cameras can take a few seconds to deliver frames
            if all(cam.read() is not None for cam in cameras.values()):
                break
            time.sleep(0.1)
        else:
            missing = [r for r, cam in cameras.items() if cam.read() is None]
            raise SystemExit(f"no frames from {missing} within 15 s")
        print("cameras delivering frames")
        if a.live:
            # The arm driver starts every motor in a "startup sync" mode that lets it aim only 1 degree past its
            # measured angle until it has stood within 3 degrees of its goal. The model's ever-changing gripper
            # targets kept the jaw in that mode for most of a run, so it barely squeezed. Hold everything still
            # until every motor has synced; then the model's rows play at full rate and full grip.
            hold_j, hold_g = arms.read()
            for _ in range(30):
                arms.command({"right": list(hold_j["right"]), "left": left_hold[0]},
                             {"right": float(hold_g["right"]), "left": left_hold[1]})
                time.sleep(0.05)
            print("arms synced at rest; starting the model")
        started = time.monotonic()
        history, assist_ready_at = [], 0.0
        played, last_grip, seam = [0], 0.0, {"from": None, "left": 0}  # RTC state
        done_cables, last_drop_pan = 0, None  # several cables per run; no grasp near the last drop spot
        demo_grasps = load_demo_grasps() if (a.live and a.assist_pull) else []
        if demo_grasps:
            print(f"assist: {len(demo_grasps)} recorded grasps loaded ({', '.join(d['episode'][-2:] for d in demo_grasps)})")
        snap_dir = Path(log.name).with_suffix("")  # <log>/ beside <log>.jsonl
        snap_dir.mkdir(exist_ok=True)
        next_snap_at = 0.0
        print(f"log {log.name}; camera snapshots in {snap_dir}/")
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
                observed = True
            if tick >= next_snap_at:  # debugging: what the cameras saw, every 0.5 s, beside the log
                next_snap_at = tick + 0.5
                save_snapshots(cameras, snap_dir, f"{tick - started:06.1f}")
            if errors:
                err = errors.pop(0)
                text = str(err)
                if "rate_limited" in text:
                    next_ask_at = time.monotonic() + 1.0  # keep playing the rows we have; ask again shortly
                elif "SessionExpired" in type(err).__name__ or "expired" in text:
                    # Dream drops the inference session when no observation arrives for ~10 s, which happens
                    # during a long grasp+pull+drop. Start a fresh episode and carry on with the next plug.
                    print("model session expired during the assist; starting a new episode")
                    try:
                        client.reset_episode()
                    except Exception as failure:  # noqa: BLE001
                        print(f"reset_episode failed: {failure}")
                    errors.clear(); chunk = None; observed = False
                    continue
                elif "pool_warming" in text or "DA-2000" in text:
                    print(f"model error: {type(err).__name__}: {text[:200]}")
                    errors.clear(); time.sleep(5); continue
                else:
                    print(f"model error: {type(err).__name__}: {text[:200]}")
                    return 2
            if "chunk" in pending:
                reply, asked_at, rows_at_ask = pending.pop("chunk")
                per_key = reply.get("per_key", reply)
                arm_rows = per_key.get("action.right_arm") or []
                grip_rows = per_key.get("action.right_gripper") or []
                # Real-time chunking (the teammates' RTC notes): skip only the rows actually executed since the
                # observation this chunk answers, not a wall-clock interval (while holding, nothing was executed,
                # so nothing is skipped), and blend the seam from the last commanded target over SEAM_ROWS rows.
                skip = min(max(0, played[0] - rows_at_ask), max(0, len(arm_rows) - 1))
                chunk, executed = (arm_rows, grip_rows), skip
                seam = {"from": None if last_target is None else list(last_target) + [last_grip], "left": SEAM_ROWS}
                print(f"chunk: {len(arm_rows)} rows, round trip {time.monotonic() - asked_at:.2f} s, starting at row {skip}")
            rows = len(chunk[0]) if chunk else 0
            idle = worker is None or not worker.is_alive()
            # Keep one request in flight at all times: with ~2 s round trips, waiting for refill_due leaves gaps.
            if observed and idle and time.monotonic() >= next_ask_at:
                next_ask_at = time.monotonic() + MIN_ASK_INTERVAL_S  # Dream allows 3 calls, then one per 0.7 s
                worker = threading.Thread(target=ask, name="dream-get-action", daemon=True); worker.start()
            history.append((tick, np.asarray(joints["right"], dtype=float)))
            while history and tick - history[0][0] > ASSIST_STILL_S:
                history.pop(0)
            if (a.live and a.assist_pull and tick - started > 15.0 and tick >= assist_ready_at
                    and history and tick - history[0][0] >= ASSIST_STILL_S * 0.9
                    and np.abs(history[-1][1] - history[0][1]).max() < ASSIST_STILL_RAD
                    and joints["right"][1] < ASSIST_MIN_LIFT_DEPTH
                    and (last_drop_pan is None or abs(joints["right"][0] - last_drop_pan) > 0.2)):
                save_snapshots(cameras, snap_dir, f"{tick - started:06.1f}_assist_start")
                outcome = assist_demo_grasp(arms, joints, left_hold, log, started, demo_grasps) if demo_grasps else None
                if outcome is not None:
                    print(f"assist: {outcome}")
                    save_snapshots(cameras, snap_dir, f"{time.monotonic() - started:06.1f}_assist_end")
                    if outcome.startswith("missed"):
                        assist_ready_at = time.monotonic() + 6.0  # let the model re-aim, then try again
                        history.clear()
                        chunk = None  # drop the stale chunk; the next reply plans from the new pose
                        continue
                    done_cables += 1
                    last_drop_pan = float(arms.read()[0]["right"][0])  # where the cable was dropped (pan)
                    if done_cables >= a.unplugs:
                        print(f"assist: {done_cables} cable(s) unplugged; done")
                        break
                    print(f"assist: {done_cables} of {a.unplugs} done; the model goes for the next plug")
                    assist_ready_at = time.monotonic() + 5.0
                    history.clear()
                    chunk = None
                    continue
                outcome = assist_pull(arms, joints, left_hold, log, started)
                save_snapshots(cameras, snap_dir, f"{time.monotonic() - started:06.1f}_assist_end")
                print(f"assist: {outcome}")
                if outcome.startswith("pulled"):
                    break
                assist_ready_at = time.monotonic() + 6.0  # missed: give the model a few seconds to regrasp
                history.clear()
                continue
            if chunk is not None and executed < rows:
                target = [float(q) for q in chunk[0][executed][:6]]
                if executed < len(chunk[1]):
                    g = chunk[1][executed]  # the service sends a number per row; the stand-in a one-item list
                    grip_units = float(g[0] if isinstance(g, (list, tuple)) else g)
                else:
                    grip_units = grips["right"] * 100.0
                current = np.asarray(joints["right"], dtype=float)
                step = np.clip(np.asarray(target) - current, -MAX_STEP_RAD, MAX_STEP_RAD)
                target = (current + step).tolist()
                grip = min(1.0, max(0.0, grip_units / 100.0))
                if seam["from"] is not None and seam["left"] > 0:  # blend into the new chunk, no jump at the seam
                    w = 1.0 - seam["left"] / (SEAM_ROWS + 1)
                    target = [(1 - w) * f0 + w * t1 for f0, t1 in zip(seam["from"][:6], target)]
                    grip = (1 - w) * seam["from"][6] + w * grip
                    seam["left"] -= 1
                last_grip = grip
                played[0] += 1
                log.write(json.dumps({"t": round(tick - started, 3), "row": executed, "target": [round(v, 4) for v in target],
                                      "grip": round(grip, 3), "measured": [round(float(v), 4) for v in current],
                                      "grip_measured": round(float(grips["right"]), 4)}) + "\n")
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
