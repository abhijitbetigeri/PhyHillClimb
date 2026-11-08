"""Score candidate programs on a running Autoresearch lab, without a person in the loop.

Used by hfjobs/bootstrap.sh in batch mode, and usable on any machine where the lab's MCP server is up:

    python hfjobs/driver.py --candidates stack_v7.py,stack_tower_v5.py [--heldout] --out results.json

For each candidate: submit, run the dev trials, poll until done, print the verdicts and the program's log lines.
With --heldout the best candidate (most dev passes) is frozen and its held-out trials are scored too.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


def call(dimos: str, tool: str, args: dict | None = None) -> str:
    cmd = [dimos, "mcp", "call", tool]
    if args:
        cmd += ["--json-args", json.dumps(args)]
    done = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if done.returncode != 0:
        raise RuntimeError(f"{tool}: {done.stderr.strip()[-400:]}")
    return done.stdout


def results(dimos: str, candidate: int) -> dict:
    out = call(dimos, "results", {"candidate": candidate})
    start = out.find("{")
    return json.loads(out[start:])


def wait(dimos: str, candidate: int, split: str, expected: int, timeout_s: float = 3600) -> dict:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            r = results(dimos, candidate)
        except Exception as error:  # the lab answers slowly while a trial resets
            print(f"  results() failed: {error}", flush=True)
            time.sleep(15)
            continue
        trials = [t for t in r.get("trials", []) if t.get("split") == split]
        if not r.get("running") and (len(trials) >= expected or r.get("error")):
            return r
        time.sleep(15)
    raise TimeoutError(f"candidate {candidate} {split} trials did not finish in {timeout_s} s")


def program_logs(trace: str) -> list[str]:
    lines = []
    try:
        for raw in open(trace):
            j = json.loads(raw)
            if j.get("call") == "log":
                lines.append(j["args"].get("text", ""))
            elif j.get("status") not in (None, "OK"):
                lines.append(f"{j.get('call')} {j.get('status')}: {str(j.get('result'))[:160]}")
    except OSError:
        pass
    return lines


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dimos", default="dimos")
    ap.add_argument("--candidates-dir", default="sim/candidates")
    ap.add_argument("--candidates", required=True, help="comma-separated file names under --candidates-dir")
    ap.add_argument("--heldout", action="store_true")
    ap.add_argument("--out", default="results.json")
    a = ap.parse_args()

    task = call(a.dimos, "task")
    print(task[:600], flush=True)
    report: dict = {"task": task[:600], "candidates": []}
    for name in [n.strip() for n in a.candidates.split(",") if n.strip()]:
        path = Path(a.candidates_dir) / name
        source = path.read_text()
        doc = source.split('"""')[1].strip() if '"""' in source else name
        number = int("".join(ch for ch in call(a.dimos, "submit", {"program": source, "hypothesis": doc[:900]}) if ch.isdigit()) or "0")
        print(f"\n=== {name} -> candidate {number}", flush=True)
        started = call(a.dimos, "run", {"candidate": number}).strip()
        print(started, flush=True)
        expected = int("".join(ch for ch in started.split(" on ")[-1].split(" ")[0] if ch.isdigit()) or "5")
        r = wait(a.dimos, number, "dev", expected)
        entry = {"name": name, "candidate": number, "dev": r.get("dev"), "error": r.get("error"), "trials": []}
        print(f"dev: {r.get('dev')} error={r.get('error')}", flush=True)
        for t in r.get("trials", []):
            verdict = "PASS" if t.get("success") else "FAIL"
            print(f"  {t['split']} {t['index']} {verdict} {t.get('stage')} | {str(t.get('note'))[:160]} | {round(t.get('duration_s', 0))} s", flush=True)
            logs = program_logs(t.get("trace", ""))
            for line in logs:
                print(f"      {line}", flush=True)
            entry["trials"].append({**{k: t.get(k) for k in ("split", "index", "success", "stage", "note", "status", "duration_s")}, "log": logs})
        report["candidates"].append(entry)
        Path(a.out).write_text(json.dumps(report, indent=1))

    if a.heldout and report["candidates"]:
        best = max(report["candidates"], key=lambda c: (c.get("dev") or {}).get("dev_passed", 0))
        print(f"\n=== freezing candidate {best['candidate']} ({best['name']}) for held-out trials", flush=True)
        print(call(a.dimos, "freeze", {"candidate": best["candidate"]}).strip(), flush=True)
        r = wait(a.dimos, best["candidate"], "heldout", 10, timeout_s=5400)
        held = [t for t in r.get("trials", []) if t.get("split") == "heldout"]
        passed = sum(1 for t in held if t.get("success"))
        print(f"held-out: {passed}/{len(held)}", flush=True)
        for t in held:
            print(f"  heldout {t['index']} {'PASS' if t.get('success') else 'FAIL'} {t.get('stage')} | {str(t.get('note'))[:160]}", flush=True)
        best["heldout"] = {"passed": passed, "trials": len(held), "records": held}
        Path(a.out).write_text(json.dumps(report, indent=1))
    try:
        print(call(a.dimos, "leaderboard")[:1500], flush=True)
    except Exception as error:
        print(f"leaderboard: {error}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
