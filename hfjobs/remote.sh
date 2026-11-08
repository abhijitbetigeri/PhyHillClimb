#!/usr/bin/env bash
# Run a command inside the current HF Jobs lab container over SSH (job started with `hfjobs/launch.sh ssh`).
#   hfjobs/remote.sh 'cd /work/dimos && .venv/bin/dimos mcp call task'
#   JOB=abhijitbetigeri/<id> hfjobs/remote.sh 'tail -20 /work/lab.log'
#   hfjobs/remote.sh push sim/candidates/stack_tower_v6.py      # copy a file into /work/PhyHillClimb/<same path>
# The job id comes from $JOB or from the newest running ssh job (`hf jobs ps`).
set -euo pipefail
HF=${HF:-$(command -v hf || echo "$HOME/OneRobotLoop/.venv/bin/hf")}
KEY=${KEY:-$HOME/.ssh/hfjobs_ed25519}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
JOB=${JOB:-$("$HF" jobs ps 2>/dev/null | awk 'NR>1 && /RUNNING/ {print $1; exit}')}
[ -n "$JOB" ] || { echo "no running job; start one with hfjobs/launch.sh ssh" >&2; exit 1; }
ID=${JOB##*/}
SSH=(ssh -i "$KEY" -o StrictHostKeyChecking=accept-new -o ConnectTimeout=20 -o ServerAliveInterval=30 -o BatchMode=yes "$ID@ssh.hf.jobs")
if [ "${1:-}" = "push" ]; then
  REL=${2:?path relative to the repo}
  "${SSH[@]}" "mkdir -p /work/PhyHillClimb/$(dirname "$REL")"
  "${SSH[@]}" "cat > /work/PhyHillClimb/$REL" < "$ROOT/$REL"
  echo "pushed $REL to job $ID"
  exit 0
fi
exec "${SSH[@]}" "$@"
