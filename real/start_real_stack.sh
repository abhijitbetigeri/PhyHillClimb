#!/usr/bin/env bash
# Start the real-arm stack holding the kit's arms lock, so dhk-collect / dreamcli refuse to start while it runs
# (and this refuses if they hold it). Physical start-up ENABLES THE MOTORS and CALIBRATES THE JAW (it drives the
# jaw to both ends): hand on the e-stop, arm resting, jaw clear of everything.
#   bash ~/dimos-real/PhyHillClimb/real/start_real_stack.sh                 # real arm + wrist_r camera
#   TESTCAM=1 bash start_real_stack.sh                                       # real arm, generated frames
#   MOCK=1 bash start_real_stack.sh                                          # echo mock + generated frames (no CAN)
#   REAL_SPEED_SCALE=0.05 bash start_real_stack.sh                           # slower (default 0.1)
# Extra arguments go to `dimos run`, e.g. --wrist.hardware.camera-index /dev/video4
# Stop with Ctrl-C in this terminal (or `~/dimos-real/.venv/bin/dimos stop`): the connection disables the motors,
# and the Metal has NO BRAKES: support the arm or have it resting before you stop.
set -euo pipefail
DEST=${DEST:-$HOME/dimos-real}
CAN=${CAN:-can_follower_r}
LOCK=${LOCK:-$HOME/.dreamlabs/arms.lock}
mkdir -p "$(dirname "$LOCK")"
cd "$DEST"
export REAL_SCENE=${REAL_SCENE:-$DEST/PhyHillClimb/real/scene.yaml}
if [ "${MOCK:-0}" = 1 ]; then
  exec .venv/bin/dimos --simulation run phyhillclimb.metal-real-program-testcam "$@"
fi
BP=phyhillclimb.metal-real-program
[ "${TESTCAM:-0}" = 1 ] && BP=phyhillclimb.metal-real-program-testcam
echo "scene: $REAL_SCENE   speed: ${REAL_SPEED_SCALE:-0.1}   can: $CAN   lock: $LOCK"
flock -n -E 75 "$LOCK" .venv/bin/dimos run "$BP" --can-port "$CAN" "$@" \
  || { rc=$?; [ $rc = 75 ] && echo "the arms lock is held: stop dhk-collect / dreamcli / the harness first"; exit $rc; }
