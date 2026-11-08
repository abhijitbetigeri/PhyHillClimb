#!/usr/bin/env bash
# Wait for a Dream fine-tune job to publish its model, then queue a continuation of that model for more steps
# (`resume_from`: charged for the new steps only, same dataset, same learning rate and batch size).
#   scripts/resume_when_done.sh <job id> <dataset user/name> <new suffix> [steps=500] [DREAM_ARM_SIDE=right]
# Writes runs/resume_<suffix>.log and runs/resume_<suffix>.json (the queued job). Exits 1 if the job failed.
set -uo pipefail
JOB=${1:?job id}; DATASET=${2:?dataset}; SUFFIX=${3:?suffix}; STEPS=${4:-500}; export DREAM_ARM_SIDE=${5:-right}
ROOT=$(cd "$(dirname "$0")/.." && pwd); cd "$ROOT"; mkdir -p runs
LOG="runs/resume_${SUFFIX}.log"
status() { .venv/bin/python tools/dream_single_arm_finetune.py status "$JOB" 2>/dev/null | .venv/bin/python -c '
import json,sys; s=sys.stdin.read(); d=json.loads(s[s.find("{"):]); p=d.get("progress") or {}
print(d.get("status"), p.get("step"), d.get("fine_tuned_model") or "", d.get("error") or "")'; }
for i in $(seq 1 90); do
  read -r ST STEP MODEL ERR <<<"$(status)"
  echo "$(date +%H:%M) $JOB $ST step=$STEP model=$MODEL $ERR" | tee -a "$LOG"
  case "$ST" in
    succeeded)
      if [ -n "$MODEL" ]; then
        .venv/bin/python tools/dream_single_arm_finetune.py queue --dataset "$DATASET" --suffix "$SUFFIX" --steps "$STEPS" \
          --resume-from "$MODEL" > "runs/resume_${SUFFIX}.json" 2>>"$LOG"
        if grep -q '"id"' "runs/resume_${SUFFIX}.json"; then
          echo "$(date +%H:%M) QUEUED continuation of $MODEL: $(grep -o '"id": "[^"]*"' "runs/resume_${SUFFIX}.json")" | tee -a "$LOG"; exit 0
        fi
        echo "$(date +%H:%M) queue refused; see $LOG; retrying next poll" | tee -a "$LOG"
      fi;;
    failed|cancelled) echo "$(date +%H:%M) job ended as $ST; no continuation" | tee -a "$LOG"; exit 1;;
  esac
  sleep 600
done
echo "$(date +%H:%M) gave up waiting" | tee -a "$LOG"; exit 1
