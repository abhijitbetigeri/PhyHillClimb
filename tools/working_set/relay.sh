#!/usr/bin/env bash
# Relay: unplug cables, then stack a cube, in one go.
#   bash ~/PhyHillClimb-tools/relay.sh [unplugs=1] [unplug seconds=90] [stack seconds=90]
# Table: strip centred with all cables seated; cubes in their diagonal line at the far LEFT edge, out of the way
# for the unplug. After the unplug stage the arm parks, then the stack stage starts.
# Ctrl-C once stops the current stage (it parks) and ends the relay.
N=${1:-1}; US=${2:-90}; SS=${3:-90}
T=$(cd "$(dirname "$0")" && pwd)
CK=dream-1-ft:2d43713b-right-arm-v1-plus500
cd ~/Superfast-harness || exit 1
trap 'echo; echo relay stopped; exit 130' INT
echo "=============== STAGE 1: unplug $N cable(s) ($(date +%H:%M:%S)) ==============="
~/dream-env/bin/python "$T/working_set_unplug_8of8/dream_right_arm_run_multi.py" --checkpoint "$CK" --live \
    --seconds "$US" --pull-hold-torque 2.0 --unplugs "$N" || { echo "stage 1 ended with an error; stopping"; exit 1; }
sleep 3
STACKS=${4:-2}   # cubes to place: 2 placements = a three-layer tower
for i in $(seq 1 "$STACKS"); do
  echo "=============== STAGE 2: stack, cube $i of $STACKS ($(date +%H:%M:%S)) ==============="
  ~/dream-env/bin/python "$T/dream_right_arm_run_stack.py" --checkpoint "$CK" --live --seconds "$SS" \
      --task "Using the right arm, stack one block on top of another block." || { echo "stack run $i ended with an error; stopping"; exit 1; }
  sleep 3
done
echo "relay done at $(date +%H:%M:%S)"
