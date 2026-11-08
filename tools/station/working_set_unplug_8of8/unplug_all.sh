#!/usr/bin/env bash
# Unplug N cables one at a time with the 8/8 working-set runner: one run per cable, the arm parks between runs.
#   bash ~/PhyHillClimb-tools/working_set_unplug_8of8/unplug_all.sh [N=4] [seconds per cable=120]
# Ctrl-C once stops the current run (it parks) and ends the loop.
N=${1:-4}; SECS=${2:-120}
W=$(cd "$(dirname "$0")" && pwd)
cd ~/Superfast-harness || exit 1
trap "echo; echo stopping after this run; STOP=1" INT
for i in $(seq 1 "$N"); do
  echo "=============== cable $i of $N ($(date +%H:%M:%S)) ==============="
  ~/dream-env/bin/python "$W/dream_right_arm_run.py" --checkpoint dream-1-ft:2d43713b-right-arm-v1-plus500 \
      --live --seconds "$SECS" --pull-hold-torque 1.5 || { echo "run $i ended with an error; stopping"; break; }
  [ -n "$STOP" ] && break
  sleep 3
done
echo "done at $(date +%H:%M:%S)"
