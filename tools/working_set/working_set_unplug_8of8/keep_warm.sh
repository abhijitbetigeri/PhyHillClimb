#!/usr/bin/env bash
# Keep a Dream checkpoint loaded by asking it about a recorded session (real frames) every few minutes.
#   keep_warm.sh <suffix> [minutes=150]
CK=$1; END=$(( $(date +%s) + ${2:-150}*60 )); T=~/PhyHillClimb-tools
while [ $(date +%s) -lt $END ]; do
  ~/dream-env/bin/python -m dream.check_model ~/hackathon_data/nuck4-right-arm-campaign-20261107/unplug-right \
    --config $T/policy_$CK.json --episode 1 --moments 1 > $T/warm_$CK.last 2>&1
  echo "$(date +%H:%M:%S) $(grep -E "answers" $T/warm_$CK.last | head -1 | cut -c1-150)" >> $T/warm_$CK.log
  if grep -q pool_warming $T/warm_$CK.last; then sleep 60; else sleep 240; fi
done
