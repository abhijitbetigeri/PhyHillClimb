#!/usr/bin/env bash
# Mac side: pull the station's fine-tune status line (written by tools/dream_job_monitor.py from cron there), keep
# a local copy in runs/finetune_status.txt and raise a macOS notification when it changes. Runs from the Mac's
# crontab every 15 minutes while the Mac is awake; the station's cron is the one that never sleeps.
#   */15 * * * * ~/projects/PhyHillClimb/scripts/finetune_status.sh
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT/runs"
OUT="$ROOT/runs/finetune_status.txt"
NEW=$(ssh -i "$HOME/.ssh/hfjobs_ed25519" -o BatchMode=yes -o ConnectTimeout=15 pah@10.103.0.129 'cat ~/PhyHillClimb-tools/finetune_status.txt 2>/dev/null' 2>/dev/null)
[ -n "$NEW" ] || exit 0
OLD=$(cat "$OUT" 2>/dev/null || true)
echo "$NEW" > "$OUT"
# notify on a change of state (not on every step count), e.g. queued -> running -> READY / NEEDS_HUMAN
key() { echo "$1" | grep -o -E "READY|NEEDS_HUMAN|requeued|queued|running|failed|cancelled" | head -1; }
if [ "$(key "$NEW")" != "$(key "$OLD")" ]; then
  /usr/bin/osascript -e "display notification \"${NEW:20:120}\" with title \"Dream fine-tune\"" 2>/dev/null || true
fi
