#!/usr/bin/env bash
# Read-only checks before starting the real-arm stack on the NUC. Moves nothing, opens no CAN socket.
#   bash ~/dimos-real/PhyHillClimb/real/preflight.sh            # the right arm, wrist_r camera
#   CAN=can_follower_r CAMERA=/dev/v4l/by-path/... SKIP_HW=1 bash preflight.sh   # SKIP_HW: no CAN/camera checks
set -uo pipefail
DEST=${DEST:-$HOME/dimos-real}
CAN=${CAN:-can_follower_r}
CAMERA=${CAMERA:-/dev/v4l/by-path/pci-0000:67:00.3-usb-0:1.3:1.0-video-index0}
LOCK=${LOCK:-$HOME/.dreamlabs/arms.lock}
FAIL=0
ok() { printf '  ok    %s\n' "$*"; }
bad() { printf '  FAIL  %s\n' "$*"; FAIL=1; }
warn() { printf '  warn  %s\n' "$*"; }

echo "== who else could drive the arm"
# The kit's collector and runner, dreamcli, the teammates' harness, our Dream runner, a leader-follower teleop, dimos
PROCS=$(pgrep -af "dhk-collect|dreamcli|dream_right_arm_run|Superfast-harness|superfast|leader|follower|lerobot|teleop|dimos run|check_cameras" \
  | grep -v -E "pgrep|preflight" || true)
if [ -n "$PROCS" ]; then bad "processes that may hold the arm or the cameras:"; echo "$PROCS" | sed 's/^/          /'; else ok "no kit / harness / teleop / dimos process"; fi
if command -v tmux >/dev/null && tmux ls 2>/dev/null | grep -q .; then
  warn "tmux sessions (look for collect/teleop/harness ones; stop the program inside, never kill the server):"
  tmux ls 2>/dev/null | sed 's/^/          /'
fi
if [ -e "$LOCK" ]; then
  if command -v flock >/dev/null && flock -n "$LOCK" true 2>/dev/null; then ok "arms lock $LOCK is free"; else bad "arms lock $LOCK is HELD (the kit or dreamcli has the arms)"; fi
else
  warn "no $LOCK (the kit's lock file); start_real_stack.sh creates and holds it"
fi
crontab -l 2>/dev/null | grep -v '^#' | grep -q . && warn "crontab entries (the Dream fine-tune monitor is API-only; check nothing else drives arms):" && crontab -l 2>/dev/null | grep -v '^#' | sed 's/^/          /'

if [ "${SKIP_HW:-0}" != 1 ]; then
  echo "== CAN $CAN"
  if ip link show "$CAN" >/dev/null 2>&1; then
    STATE=$(ip -details link show "$CAN")
    echo "$STATE" | grep -q "state UP\|,UP" && ok "$CAN is up" || bad "$CAN is not up (the kit's udev/setup brings it up: sudo ip link set $CAN up type can bitrate 1000000 is the usual fix; ask before changing it)"
    echo "$STATE" | grep -o "bitrate [0-9]*" | head -1 | sed 's/^/          /'
    echo "$STATE" | grep -q "BUS-OFF" && bad "$CAN is BUS-OFF"
    if [ -r /proc/net/can/rcvlist_all ]; then
      N=$(grep -c " $CAN " /proc/net/can/rcvlist_all 2>/dev/null || true)
      [ "${N:-0}" -gt 0 ] && bad "$N CAN receive filters registered on $CAN: another process has a socket open on it" || ok "no open CAN sockets on $CAN"
    fi
  else
    bad "no interface $CAN (ip link show)"
  fi
  echo "== camera $CAMERA"
  if [ -e "$CAMERA" ]; then
    ok "camera device exists -> $(readlink -f "$CAMERA")"
    DEV=$(readlink -f "$CAMERA")
    if command -v fuser >/dev/null; then
      fuser "$DEV" >/dev/null 2>&1 && bad "camera $DEV is open by: $(fuser "$DEV" 2>&1 | cut -d: -f2)" || ok "camera not in use"
    fi
  else
    bad "no camera at $CAMERA (ls /dev/v4l/by-path/); or run the -testcam blueprint"
  fi
fi

echo "== install $DEST"
PY="$DEST/.venv/bin/python"
if [ -x "$PY" ]; then
  "$PY" - <<'PY' && ok "dimos, CAN driver and real/ import; SceneObject has yaw" || bad "imports failed"
import cv2, can_motor_control, dimos.autoresearch.live, real.blueprint_real, real.measured_scene  # noqa
from dimos.autoresearch.program import SceneObject
assert "yaw" in SceneObject.__dataclass_fields__
PY
  "$DEST/.venv/bin/dimos" list 2>/dev/null | grep -q "phyhillclimb.metal-real-program" && ok "entry point phyhillclimb.metal-real-program registered" || bad "entry point not registered (rerun install_station.sh)"
  SCENE=${REAL_SCENE:-$DEST/PhyHillClimb/real/scene.yaml}
  "$PY" -c "from real.measured_scene import load_scene_file; s=load_scene_file('$SCENE'); print('          ' + ', '.join(f'{n} {o.xyz}' for n, o in s.objects.items()))" \
    && ok "scene file $SCENE parses (positions above: are they MEASURED?)" || bad "scene file $SCENE does not parse"
else
  bad "no venv at $DEST/.venv (run install_station.sh)"
fi
echo
[ "$FAIL" = 0 ] && echo "PREFLIGHT OK" || echo "PREFLIGHT FAILED: fix the FAIL lines first"
exit $FAIL
