"""Run one candidate program on the running real-arm stack (blueprint_real.py) through dimos's Autoresearch runner.

    python real/run_program_real.py --program sim/candidates/tower_order_v24.py
    python real/run_program_real.py --program sim/candidates/cable_v1.py --timeout 600
    python real/run_program_real.py --program real/programs/hover_one.py --task "hover over cube_red"
    python real/run_program_real.py --program sim/candidates/tower_order_v24.py --plan-only   # nothing moves

Use the dimos venv's python (`~/dimos-real/.venv/bin/python`): `run_program` starts the program in a
`python -I` child of this same interpreter, with `real.live_real:backend` (dimos.autoresearch.live:backend whose
gripper calls wait for the jaw to stop; `--backend dimos.autoresearch.live:backend` for the plain one) or
`real.plan_only:backend`, attaching to the stack over the bus. Each run goes to real/runs/<timestamp>/: trace.jsonl, keyframes/,
program.log, outcome.json (dimos's), plus run.json (what was run), scene_before.txt / scene_after.txt (the measured
scene's belief) and, with --verdict, verdict.json (the operator's y/n, OperatorJudge style).

Ctrl-C stops the program AND cancels the arm's current motion (the child dying alone would let a trajectory
already sent finish). The e-stop is still the stop.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from dimos.autoresearch.program import RunStoppedError, run_program  # noqa: E402
from dimos.porcelain.dimos import Dimos  # noqa: E402

TOWER_TASK = "stack the yellow cube first, then the red cube on top, then the blue cube on top"
CABLE_TASK = "Unplug all the cables from the cable set."
#: dimos's live backend with gripper calls that wait for the jaw to stop (real/live_real.py); --backend
#: dimos.autoresearch.live:backend runs the plain one
LIVE = "real.live_real:backend"
PLAN_ONLY = "real.plan_only:backend"


def default_task(program: Path) -> str:
    return CABLE_TASK if "cable" in program.name else TOWER_TASK


def scene_text(app: Dimos) -> str:
    try:
        return app.get_module("MeasuredSceneModule").describe()
    except Exception as error:
        return f"(no measured scene: {error})"


def cancel_motion(app: Dimos) -> None:
    try:
        from dimos.manipulation.sdk import Arm

        Arm.from_app(app).cancel()
        print("arm motion cancelled")
    except Exception as error:
        print(f"could not cancel the arm's motion ({error}): USE THE E-STOP if it is still moving")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--program", required=True, type=Path)
    parser.add_argument("--task", help="task text (default: the ordered yellow-red-blue tower, or unplug for cable_*)")
    parser.add_argument("--timeout", type=float, default=1200.0, help="seconds before the program is killed")
    parser.add_argument("--plan-only", action="store_true", help="plan moves, never execute (real.plan_only)")
    parser.add_argument("--backend", help="override the backend factory 'package.module:function'")
    parser.add_argument("--out", type=Path, help="run directory (default real/runs/<timestamp>)")
    parser.add_argument("--verdict", action="store_true", help="ask the operator y/n at the end (OperatorJudge)")
    parser.add_argument("--yes", action="store_true", help="skip the 'hand on the e-stop' confirmation")
    args = parser.parse_args(argv)

    program = args.program.resolve()
    if not program.is_file():
        parser.error(f"no program at {program}")
    from dimos.autoresearch.program import validate_program

    validate_program(program.read_text())
    task = args.task or default_task(program)
    factory = args.backend or (PLAN_ONLY if args.plan_only else LIVE)
    run_dir = (args.out or HERE / "runs" / datetime.now().strftime("%Y%m%d_%H%M%S")).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)

    app = Dimos.connect()
    try:
        before = scene_text(app)
        print(before)
        if not args.yes and not args.plan_only:
            answer = input(f"\nRun {program.name} for real ({task!r})? Hand on the e-stop, then type GO: ")
            if answer.strip() != "GO":
                print("not run")
                return 1
        (run_dir / "scene_before.txt").write_text(before + "\n")
        (run_dir / "run.json").write_text(json.dumps(
            {"program": str(program), "task": task, "backend": factory, "timeout_s": args.timeout,
             "started": time.time()}, indent=2))
        print(f"running {program.name} -> {run_dir}  (Ctrl-C stops it and cancels the motion)")
        stop = threading.Event()
        result: dict = {}

        def target() -> None:
            try:
                result["outcome"] = run_program(program, run_dir, task, args.timeout, factory, stop=stop)
            except BaseException as error:  # noqa: BLE001 - reported below
                result["error"] = error

        worker = threading.Thread(target=target, daemon=True)
        worker.start()
        try:
            while worker.is_alive():
                worker.join(0.2)
        except KeyboardInterrupt:
            print("\nstopping")
            stop.set()
            cancel_motion(app)
            worker.join(10.0)
        after = scene_text(app)
        (run_dir / "scene_after.txt").write_text(after + "\n")
        error = result.get("error")
        if isinstance(error, RunStoppedError):
            print("stopped by the operator")
            return 130
        if error is not None:
            print(f"runner failed: {error}")
            return 2
        outcome = result["outcome"]
        print(f"\noutcome: {outcome.status} after {outcome.duration_s:.1f} s")
        if outcome.error:
            print(outcome.error[-2000:])
        print(after)
        print(f"trace: {run_dir / 'trace.jsonl'}   log: {run_dir / 'program.log'}")
        if args.verdict:
            from dimos.autoresearch.judge import OperatorJudge, Snapshot

            verdict = OperatorJudge().verdict(Snapshot(bodies={}), Snapshot(bodies={}))
            (run_dir / "verdict.json").write_text(verdict.model_dump_json(indent=2))
            print(f"verdict: {verdict.success} {verdict.note}")
        return 0 if outcome.status == "completed" else 3
    finally:
        app.stop()


if __name__ == "__main__":
    raise SystemExit(main())
