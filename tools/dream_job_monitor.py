"""Overnight guardian for the Dream fine-tune: runs from cron on the station every few minutes.

    */5 * * * * ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_job_monitor.py >> ~/PhyHillClimb-tools/finetune_monitor.log 2>&1

State lives in ~/PhyHillClimb-tools/finetune_state.json (job id, dataset, suffix base, attempts, model). Each run:
- logs the job's status, phase, step, loss and queue position; writes a one-line finetune_status.txt;
- on `failed` with a service-side error, re-queues the same job with the next suffix (up to MAX_ATTEMPTS); an error
  that names the dataset, a payment refusal or a cancel is left for a person (NEEDS_HUMAN in the log);
- on `succeeded` with the checkpoint published, records the model id, runs `dream.check_model` once against one
  recorded unplug session (a few billed calls) and logs its PASS/PROBLEM lines;
- then keeps the model warm during WARM_HOURS (local time) with one small get_action every run, so the first real
  run of the morning is not a 25-50 minute cold start.
Credentials come from ~/.dreamlabs/credentials. Nothing here touches the arms or the cameras.
"""

from __future__ import annotations

import base64
import datetime as dt
import fcntl
import io
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HOME = Path.home()
TOOLS = HOME / "PhyHillClimb-tools"
STATE = TOOLS / "finetune_state.json"
STATUS = TOOLS / "finetune_status.txt"
LOCK = TOOLS / "finetune_monitor.lock"
MAX_ATTEMPTS = 4
WARM_HOURS = (5, 16)  # local hours during which the finished model is kept warm
CHECK_SESSION = HOME / "hackathon_data/nuck4-right-arm-campaign-20261107/unplug-right"
EMBODIMENT = {
    "state": {"right_arm": [0, 6], "right_gripper": [6, 7]},
    "action": {"right_arm": [0, 6], "right_gripper": [6, 7]},
    "video": {"overhead": "observation.images.scene", "wrist_r": "observation.images.wrist"},
    "head_camera": "overhead",
}


def now() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def log(line: str) -> None:
    print(f"{now()} {line}", flush=True)


def load_state() -> dict:
    return json.loads(STATE.read_text())


def save_state(state: dict) -> None:
    STATE.write_text(json.dumps(state, indent=1))


def write_status(text: str) -> None:
    STATUS.write_text(f"{now()} {text}\n")


def dataset_fault(error: str | None) -> bool:
    text = (error or "").lower()
    return any(w in text for w in ("dataset", "episode", "frame", "video", "payment", "credit", "embodiment"))


def requeue(state: dict, reason: str) -> None:
    from dream import fine_tuning

    attempts = int(state.get("attempts", 1))
    if attempts >= MAX_ATTEMPTS:
        log(f"NEEDS_HUMAN: {attempts} attempts used; last failure: {reason}")
        write_status(f"NEEDS_HUMAN after {attempts} attempts: {reason}")
        return
    suffix = f"{state['suffix_base']}-v{attempts + 1}"
    try:
        job = fine_tuning.create(state["dataset"], suffix, steps=int(state.get("steps", 1000)), use_eef=False)
    except Exception as error:  # noqa: BLE001
        log(f"requeue failed ({type(error).__name__}: {str(error)[:200]}); will retry next run")
        return
    state.update({"job_id": job["id"], "attempts": attempts + 1, "suffix": suffix, "model": None, "checked": False,
                  "history": state.get("history", []) + [{"job": state.get("job_id"), "failed": reason, "at": now()}]})
    save_state(state)
    log(f"REQUEUED as {job['id']} suffix={suffix} (attempt {attempts + 1}); previous failure: {reason}")
    write_status(f"requeued {job['id']} ({suffix}), attempt {attempts + 1}")


def run_check_model(state: dict) -> None:
    policy = TOOLS / "policy.json"
    policy.write_text(json.dumps({"checkpoint": state["model"], "embodiment": EMBODIMENT, "action_hz": 20,
                                  "execution_horizon": 30, "timeout_ms": 30000}))
    if not CHECK_SESSION.exists():
        log("check_model skipped: no recorded session found")
        return
    cmd = [sys.executable, "-m", "dream.check_model", str(CHECK_SESSION), "--config", str(policy), "--episode", "1", "--moments", "4"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=1500)
        lines = [l for l in (out.stdout + out.stderr).splitlines() if l.strip()]
        for l in lines[-12:]:
            log(f"check_model: {l[:200]}")
        verdict = "PROBLEM" if any("PROBLEM" in l for l in lines) else "PASS" if any("PASS" in l for l in lines) else "UNKNOWN"
        state["check_model"] = verdict
    except Exception as error:  # noqa: BLE001
        log(f"check_model failed to run: {type(error).__name__}: {str(error)[:200]}")
        state["check_model"] = "ERROR"


def warm(state: dict) -> None:
    """One cheap get_action so the serving pool keeps the model loaded."""
    import dream

    hour = dt.datetime.now().hour
    if not (WARM_HOURS[0] <= hour < WARM_HOURS[1]):
        return
    try:
        import numpy as np
        from PIL import Image

        buf = io.BytesIO()
        Image.fromarray(np.zeros((176, 320, 3), dtype=np.uint8)).save(buf, format="JPEG", quality=80)
        frame = base64.b64encode(buf.getvalue()).decode("ascii")
        client = dream.DreamClient()
        client.policy({"checkpoint": state["model"], "embodiment": EMBODIMENT, "action_hz": 20, "execution_horizon": 30,
                       "timeout_ms": 30000})
        client.reset_episode()
        client.set_observation({"video.overhead": frame, "video.wrist_r": frame},
                               {"prompt": "warm up", "state.right_arm": [0.0] * 6, "state.right_gripper": [50.0]}, timestamp=time.time())
        t0 = time.time()
        client.get_action()
        log(f"warm: model answered in {time.time() - t0:.1f} s")
        state["warm_ok_at"] = now()
    except Exception as error:  # noqa: BLE001
        text = f"{type(error).__name__}: {str(error)[:160]}"
        log(f"warm: {text}")
        state["warm_last"] = text


def main() -> int:
    TOOLS.mkdir(exist_ok=True)
    with open(LOCK, "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return 0  # the previous run is still going
        from dream import fine_tuning

        state = load_state()
        try:
            job = fine_tuning.get(state["job_id"])
        except Exception as error:  # noqa: BLE001
            log(f"get() failed: {type(error).__name__}: {str(error)[:200]}")
            write_status(f"monitor could not reach the service: {type(error).__name__}")
            return 1
        p = job.get("progress") or {}
        summary = (f"{job['id']} {job['status']} phase={p.get('phase')} step={p.get('step')}/{p.get('total_steps')} "
                   f"loss={p.get('loss')} queue={job.get('queue_position')} model={job.get('fine_tuned_model')} error={job.get('error')}")
        log(summary)
        write_status(summary)
        status = job["status"]
        if status in ("queued", "running"):
            return 0
        if status == "succeeded" and job.get("fine_tuned_model"):
            state["model"] = job["fine_tuned_model"]
            if not state.get("checked"):
                log(f"READY: {state['model']}")
                run_check_model(state)
                state["checked"] = True
                save_state(state)
            warm(state)
            save_state(state)
            write_status(f"READY {state['model']} check_model={state.get('check_model')} warm={state.get('warm_ok_at') or state.get('warm_last')}")
            return 0
        if status == "succeeded":
            log("succeeded, waiting for the checkpoint to be published")
            return 0
        if status == "cancelled":
            log("NEEDS_HUMAN: the job was cancelled")
            write_status("NEEDS_HUMAN: cancelled")
            return 0
        if status == "failed":
            reason = job.get("error") or "no error text"
            if dataset_fault(reason):
                log(f"NEEDS_HUMAN: failure names the dataset or billing: {reason}")
                write_status(f"NEEDS_HUMAN: {reason}")
                return 0
            requeue(state, reason)
        return 0


if __name__ == "__main__":
    sys.exit(main())
