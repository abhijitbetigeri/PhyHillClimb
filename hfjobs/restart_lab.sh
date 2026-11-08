#!/usr/bin/env bash
# Inside the HF job: restart the lab (a frozen session takes no new candidates) and optionally start the batch
# driver detached, so an ssh session timeout cannot kill it.
#   bash /work/restart_lab.sh [lab blueprint] [candidates comma-list] [--heldout]
# Run it as a file (not an inline ssh command): pkill below matches "dimos --simulation", and an inline command
# line that contains that text would kill its own shell.
set -uo pipefail
LAB=${1:-phyhillclimb.autoresearch-three-cube}
CANDS=${2:-}
HELDOUT=${3:-}
export PATH="$HOME/.local/bin:$PATH"
export MUJOCO_GL=${MUJOCO_GL:-osmesa}
export TOWER_ORDER=${TOWER_ORDER:-yellow,red,blue,red stick}
cd /work/dimos
command -v ffmpeg >/dev/null || (apt-get install -y -qq ffmpeg >/dev/null 2>&1 || true)
pkill -f "dimos --simulation" 2>/dev/null
pkill -f "hfjobs/driver.py" 2>/dev/null
pkill -f "dimos-memory-recorder" 2>/dev/null
pkill -f "voxel_ray_tracing" 2>/dev/null
sleep 6
nohup .venv/bin/dimos --simulation --n-workers "${N_WORKERS:-12}" run "$LAB" --mujocosimmodule.headless true > /work/lab.log 2>&1 &
sleep 30
for i in $(seq 1 40); do
  grep -q "Blueprint started" /work/lab.log && break
  sleep 5
done
if ! grep -q "Blueprint started" /work/lab.log; then
  echo "LAB RESTART FAILED"; grep -a -E "\[err\]|Error" /work/lab.log | tail -8 | cut -c1-200; exit 1
fi
sleep 10
echo "LAB RESTARTED ($LAB)"
if [ -n "$CANDS" ]; then
  STAMP=$(date +%H%M%S)
  nohup .venv/bin/python /work/PhyHillClimb/hfjobs/driver.py --dimos .venv/bin/dimos \
    --candidates-dir /work/PhyHillClimb/sim/candidates --candidates "$CANDS" ${HELDOUT:+--heldout} \
    --out "/work/out/results_$STAMP.json" > "/work/out/driver_$STAMP.log" 2>&1 &
  echo "DRIVER STARTED log=/work/out/driver_$STAMP.log"
fi
