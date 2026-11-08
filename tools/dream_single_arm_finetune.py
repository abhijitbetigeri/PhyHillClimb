"""Fine-tune Dream-1 on the team's single-arm Metal recordings through the Dream API, step by step.

The teammates' collector (Superfast-harness leader_collect) writes LeRobot v2.1 datasets with a 7-value state and
action (right arm joints 1-6 in radians, gripper 0-100) and two cameras (`observation.images.scene`,
`observation.images.wrist`). The hackathon kit's `finetune.py` expects its own 14-value bimanual layout, so this
script talks to the API directly with a matching embodiment instead:

    python tools/dream_single_arm_finetune.py register                      # once: the 7-value right-arm body
    python tools/dream_single_arm_finetune.py validate <parent folder>      # free local checks
    python tools/dream_single_arm_finetune.py upload <parent folder> --name phyhillclimb-right-arm-20261107
    python tools/dream_single_arm_finetune.py status [ds_... | job id]      # dataset processing / job progress
    python tools/dream_single_arm_finetune.py queue --dataset <user>/<name> --suffix unplug-v1 [--steps 1000]
    python tools/dream_single_arm_finetune.py jobs                          # every fine-tune job

Credentials: DREAM_API_KEY, DREAM_GATEWAY_URL, DREAM_API_URL, DREAM_DATA_URL from the environment or .env, or
`~/.dreamlabs/credentials` as the client reads it. Queueing is paid ($200 at 1000 steps) and never implicit.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from relay.common import load_env  # noqa: E402

load_env()

import dream.datasets as datasets  # noqa: E402
import dream.fine_tuning as fine_tuning  # noqa: E402

EMBODIMENT_NAME = "metal-right-arm-7"
EMBODIMENT_CONFIG = {
    "state": {"right_arm": [0, 6], "right_gripper": [6, 7]},
    "action": {"right_arm": [0, 6], "right_gripper": [6, 7]},
    "video": {"overhead": "observation.images.scene", "wrist_r": "observation.images.wrist"},
    "head_camera": "overhead",
}


def register() -> None:
    for body in datasets.list_embodiments():
        if body.name.endswith("/" + EMBODIMENT_NAME):
            print(f"embodiment already registered: {body.name} ({body.id})")
            print(json.dumps(body.embodiment_config, indent=1))
            return
    body = datasets.add_embodiment(EMBODIMENT_NAME, EMBODIMENT_CONFIG)
    print(f"registered {body.name} ({body.id})")


def validate(path: str) -> bool:
    report = datasets.validate_dataset(path, EMBODIMENT_CONFIG)
    print(json.dumps(report.__dict__, indent=1, default=str))
    return report.ok


def upload(path: str, name: str) -> None:
    user = next(b.name.split("/")[0] for b in datasets.list_embodiments() if b.name.endswith("/" + EMBODIMENT_NAME))
    ds = datasets.upload_dataset(path, name, f"{user}/{EMBODIMENT_NAME}")
    print(f"uploaded: id={ds.id} name={ds.name} status={ds.status} episodes={ds.episodes} size={ds.size_bytes / 1e6:.0f} MB")
    print("poll:  python tools/dream_single_arm_finetune.py status", ds.id)


def status(ident: str | None) -> None:
    if ident and ident.startswith("ds_"):
        ds = datasets.get_dataset(ident)
        print(f"{ds.name}: {ds.status} episodes={ds.episodes} error={ds.error}")
    elif ident:
        print(json.dumps(fine_tuning.get(ident), indent=1, default=str))
    else:
        for ds in datasets.list_datasets():
            print(f"{ds.id} {ds.name}: {ds.status} episodes={ds.episodes} error={ds.error}")


def wait_processed(ds_id: str, minutes: int = 60) -> bool:
    for _ in range(minutes * 2):
        ds = datasets.get_dataset(ds_id)
        print(time.strftime("%H:%M:%S"), ds.status, ds.error or "")
        if ds.status == "processed":
            return True
        if ds.status == "failed":
            return False
        time.sleep(30)
    return False


def queue(dataset: str, suffix: str, steps: int | None, base_share: float | None, lr_schedule: str | None) -> None:
    job = fine_tuning.create(dataset, suffix, steps=steps, use_eef=False, base_share=base_share, lr_schedule=lr_schedule)
    print(json.dumps(job, indent=1, default=str))


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("register")
    p = sub.add_parser("validate"); p.add_argument("path")
    p = sub.add_parser("upload"); p.add_argument("path"); p.add_argument("--name", required=True)
    p = sub.add_parser("status"); p.add_argument("ident", nargs="?")
    p = sub.add_parser("wait"); p.add_argument("ds_id"); p.add_argument("--minutes", type=int, default=60)
    p = sub.add_parser("queue"); p.add_argument("--dataset", required=True); p.add_argument("--suffix", required=True)
    p.add_argument("--steps", type=int, default=None); p.add_argument("--base-share", type=float, default=None)
    p.add_argument("--lr-schedule", default=None)
    sub.add_parser("jobs")
    a = ap.parse_args()
    if a.cmd == "register":
        register()
    elif a.cmd == "validate":
        return 0 if validate(a.path) else 1
    elif a.cmd == "upload":
        upload(a.path, a.name)
    elif a.cmd == "status":
        status(a.ident)
    elif a.cmd == "wait":
        return 0 if wait_processed(a.ds_id, a.minutes) else 1
    elif a.cmd == "queue":
        queue(a.dataset, a.suffix, a.steps, a.base_share, a.lr_schedule)
    elif a.cmd == "jobs":
        for job in fine_tuning.list():
            print(json.dumps(job, default=str)[:300])
    return 0


if __name__ == "__main__":
    sys.exit(main())
