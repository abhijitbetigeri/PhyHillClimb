"""Score a program in the real-arm configuration on MuJoCo: measured scene (no truth scene, no camera map).

Start the stack first (on an HF Jobs machine, never on the Mac):

    MEASURED_BENCH=stack dimos --simulation --n-workers 8 run phyhillclimb.metal-measured-sim --mujocosimmodule.headless true
    python real/verify_measured_sim.py --program sim/candidates/tower_order_v24.py --seeds 0 1 2 3 4 [--noise-mm 5]

Per seed: home the arm, reset the simulator with the three-cube lab's seeded placement of the red cube
(PlacementBounds x 0.18-0.28, y 0.08-0.18, yaw +-0.5; the other cubes at the bench's spots), read the TRUE poses
once and write them as the measured scene YAML (optionally with Gaussian "tape measure" noise), reload the scene
module, run the program through real/run_program_real.py's path (run_program + dimos.autoresearch.live:backend),
then judge from true poses: sim.judges.OrderedTowerJudge on the final poses, plus the strict lab's extras (every cube
within 3 mm over 3 s, gripper_base more than 16 cm from every cube). Results: real/runs/verify_<ts>/results.json.
For the cable bench (MEASURED_BENCH=cable) the judge is UnplugJudge and the layout sim.lab_cable.layout(seed).
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import random
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import yaml  # noqa: E402

from dimos.autoresearch.judge import BodyPose, Snapshot  # noqa: E402
from dimos.autoresearch.program import run_program  # noqa: E402
from dimos.autoresearch.world import PlacementBounds  # noqa: E402
from dimos.manipulation.sdk import Arm  # noqa: E402
from dimos.porcelain.dimos import Dimos  # noqa: E402
from dimos.simulation.engines.sim_scene_spec import SimSceneSpec  # noqa: E402

from real.blueprint_measured_sim import SIM_SCENE_FILE  # noqa: E402
from real.run_program_real import CABLE_TASK, LIVE, TOWER_TASK  # noqa: E402
from sim.judges import OrderedTowerJudge  # noqa: E402

CUBES = ("cube_yellow", "cube_red", "cube_blue")
CUBE = (0.05, 0.05, 0.05)
RED_BOUNDS = PlacementBounds(x=(0.18, 0.28), y=(0.08, 0.18), z=0.025, yaw=(-0.5, 0.5))


class Poses:
    def __init__(self, app: Dimos) -> None:
        [info] = [i for i in app.list_streams() if i.name == "body_poses" and i.direction == "out"]
        self._latest: dict[str, BodyPose] = {}
        self._count = 0
        # (time, {body: xyz}) about twice a second, so belief events can be matched to the true poses
        self.history: list[tuple[float, dict[str, list[float]]]] = []
        self._cv = threading.Condition()
        self._unsub = app.get_module(info.module_name).get_stream("body_poses").subscribe(self._on)

    def _on(self, msg) -> None:
        with self._cv:
            for t in msg.transforms:
                self._latest[t.child_frame_id] = BodyPose(
                    position=(t.translation.x, t.translation.y, t.translation.z),
                    orientation=(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                )
            self._count += 1
            now = time.time()
            if not self.history or now - self.history[-1][0] >= 0.5:
                self.history.append((now, {n: [round(v, 4) for v in p.position] for n, p in self._latest.items()}))
            self._cv.notify_all()

    def fresh(self, timeout: float = 10.0) -> Snapshot:
        with self._cv:
            start = self._count
            if not self._cv.wait_for(lambda: self._count >= start + 2, timeout):
                raise RuntimeError("no fresh body poses")
            return Snapshot(bodies=dict(self._latest))


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def write_scene(snap: Snapshot, bench: str, noise_m: float, rng: random.Random) -> dict:
    objects = {}
    if bench == "stack":
        for name in CUBES:
            p = snap.bodies[name].position
            objects[name] = {"xyz": [round(p[0] + rng.gauss(0, noise_m), 4), round(p[1] + rng.gauss(0, noise_m), 4)],
                             "size": list(CUBE), "yaw": round(yaw_of(snap.bodies[name].orientation), 4)}
    else:
        from sim.lab_cable import TRUTH

        for name in ("hub", "plug_a", "plug_b"):
            p, box = snap.bodies[name].position, TRUTH[name]
            objects[name] = {"xyz": [round(p[0] + box.centre[0] + rng.gauss(0, noise_m), 4),
                                     round(p[1] + box.centre[1] + rng.gauss(0, noise_m), 4),
                                     round(p[2] + box.centre[2], 4)],
                             "size": list(box.size), "yaw": round(yaw_of(snap.bodies[name].orientation), 4)}
    Path(SIM_SCENE_FILE).write_text(yaml.safe_dump({"frame": "world", "bench_z": 0.0, "objects": objects}))
    return objects


def release_errors(describe: str, history: list) -> list[dict]:
    """For each 'released X at (x, y)' belief event, where X truly was 1.5 s later: belief minus truth, cm."""
    import re

    out = []
    for m in re.finditer(r"(\d+\.\d+) released (\w+) at \(([-\d.]+), ([-\d.]+)\)", describe):
        t, name, bx, by = float(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4))
        later = [b for (ts, b) in history if ts >= t + 1.5 and name in b]
        if not later:
            continue
        tx, ty, tz = later[0][name]
        out.append({"t": t, "object": name, "belief": [bx, by], "truth": [tx, ty, tz],
                    "belief_minus_truth_cm": [round((bx - tx) * 100, 2), round((by - ty) * 100, 2)]})
    return out


def judge_tower(before: Snapshot, after: Snapshot, later: Snapshot, tool: tuple[float, float, float]) -> dict:
    v = OrderedTowerJudge(CUBES).verdict(before, later)
    drift = max(math.dist(after.bodies[n].position, later.bodies[n].position) for n in CUBES)
    clear = min(math.dist(tool, later.bodies[n].position) for n in CUBES)
    strict = v.success and drift <= 0.003 and clear > 0.16
    return {"success": strict, "geometry": v.success, "stage": v.stage, "note": v.note,
            "drift_mm": round(drift * 1000, 1), "jaw_clearance_cm": round(clear * 100, 1)}


def judge_cable(before: Snapshot, after: Snapshot, later: Snapshot, tool) -> dict:
    from sim.judges import UnplugJudge
    from sim.lab_cable import CABLES, PLUGS

    v = UnplugJudge(CABLES, PLUGS).verdict(before, later)
    return {"success": v.success, "stage": v.stage, "note": v.note}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--program", required=True, type=Path)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--bench", choices=("stack", "cable"), default="stack")
    parser.add_argument("--noise-mm", type=float, default=0.0, help="Gaussian error added to measured x and y")
    parser.add_argument("--timeout", type=float, default=1500.0)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--pick-check", action="store_true",
                        help="instead: pick and put back each cube (real/programs/pick_one.py) and report belief - truth")
    args = parser.parse_args()
    out = (args.out or HERE / "runs" / f"verify_{datetime.now():%Y%m%d_%H%M%S}").resolve()
    out.mkdir(parents=True, exist_ok=True)
    task = TOWER_TASK if args.bench == "stack" else CABLE_TASK
    app = Dimos.connect()
    poses = Poses(app)
    sim = app.find_module_by_spec(SimSceneSpec)
    arm = Arm.from_app(app)
    scene = app.get_module("MeasuredSceneModule")
    results = []
    if args.pick_check:
        arm.home(speed_scale=0.5)
        sim.reset(placements={})
        time.sleep(4.0)
        write_scene(poses.fresh(), "stack", 0.0, random.Random(0))
        scene.reload()
        for cube in CUBES:
            before = poses.fresh()
            outcome = run_program(HERE / "programs" / "pick_one.py", out / f"pick-{cube}", f"pick {cube}", 600, LIVE)
            time.sleep(2.0)
            truth = poses.fresh().bodies[cube].position
            d = scene.scan_scene([cube]).detections[0].bbox.center.position
            err = (d.x - truth[0], d.y - truth[1], d.z - truth[2])
            moved = math.dist(before.bodies[cube].position, truth)
            line = {"cube": cube, "status": outcome.status, "truth": [round(v, 4) for v in truth],
                    "belief": [round(d.x, 4), round(d.y, 4), round(d.z, 4)],
                    "belief_minus_truth_cm": [round(v * 100, 2) for v in err], "moved_cm": round(moved * 100, 2)}
            results.append(line)
            print(line, flush=True)
        (out / "pick_check.json").write_text(json.dumps(results, indent=2))
        print(scene.describe())
        app.stop()
        return 0
    for seed in args.seeds:
        rng = random.Random(1000 + seed)
        try:
            arm.home(speed_scale=0.5)
        except Exception as error:
            print(f"seed {seed}: home failed ({error}); resetting anyway")
        if args.bench == "stack":
            placements = {"cube_red": RED_BOUNDS.sample(seed)}
        else:
            from sim.lab_cable import layout

            placements = layout(seed, "unplug")
        if not sim.reset(placements=placements):
            raise RuntimeError("reset not applied")
        time.sleep(4.0)
        before = poses.fresh()
        measured = write_scene(before, args.bench, args.noise_mm / 1000.0, rng)
        scene.reload()
        trial = out / f"seed-{seed}"
        started = time.time()
        poses.history.clear()
        try:
            outcome = run_program(args.program.resolve(), trial, task, args.timeout, LIVE)
            status, error = outcome.status, outcome.error[-600:]
        except Exception as exc:  # noqa: BLE001
            status, error = "runner-error", str(exc)
        duration = time.time() - started
        time.sleep(1.0)
        after = poses.fresh()
        time.sleep(3.0)
        later = poses.fresh()
        p = arm.pose().position
        tool = (p.x, p.y, p.z)
        verdict = (judge_tower if args.bench == "stack" else judge_cable)(before, after, later, tool)
        record = {"seed": seed, "program": args.program.name, "status": status, "duration_s": round(duration, 1),
                  "noise_mm": args.noise_mm, "measured": measured, **verdict,
                  "final": {n: [round(v, 4) for v in later.bodies[n].position] for n in later.bodies},
                  "belief": scene.describe(), "error": error}
        record["releases"] = release_errors(scene.describe(), poses.history)
        trial.mkdir(parents=True, exist_ok=True)
        (trial / "truth.jsonl").write_text("".join(json.dumps({"t": t, "bodies": b}) + "\n" for t, b in poses.history))
        results.append(record)
        (out / "results.json").write_text(json.dumps(results, indent=2))
        print(f"seed {seed}: {'PASS' if verdict['success'] else 'fail'} {status} {duration:.0f}s "
              f"{ {k: v for k, v in verdict.items() if k != 'success'} }", flush=True)
    passes = sum(r["success"] for r in results)
    print(f"\n{args.program.name}: {passes}/{len(results)} under the measured scene; results {out / 'results.json'}")
    app.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
