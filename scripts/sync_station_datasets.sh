#!/usr/bin/env bash
# Copy the station's recorded datasets (read-only) into notes/station-backup/datasets and push them to the Hub as
# private LeRobot dataset repos. Re-running only transfers new files, so run it again as recording continues.
#   scripts/sync_station_datasets.sh            # all known sets
#   scripts/sync_station_datasets.sh unplug-right
set -euo pipefail
STATION=${STATION:-pah@10.103.0.129}
KEY=${KEY:-$HOME/.ssh/hfjobs_ed25519}
HF=${HF:-$(command -v hf || echo "$HOME/OneRobotLoop/.venv/bin/hf")}
USER_NS=${USER_NS:-abhijitbetigeri}
DATE=${DATE:-20261107}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
B="$ROOT/notes/station-backup/datasets"
mkdir -p "$B"
declare -A SRC=(
  [stack]="stack"
  [stack-right]="nuck4-right-arm-campaign-20261107/stack-right"
  [insert-right]="nuck4-right-arm-campaign-20261107/insert-right"
  [unplug-right]="nuck4-right-arm-campaign-20261107/unplug-right"
  [stack-left]="nuck4-subtask-campaign-20261107/stack-left"
  [insert-left]="nuck4-subtask-campaign-20261107/insert-left"
)
SETS=("$@"); [ ${#SETS[@]} -gt 0 ] || SETS=("${!SRC[@]}")
for name in "${SETS[@]}"; do
  src=${SRC[$name]:?unknown set $name}
  rsync -a --quiet -e "ssh -i $KEY -o BatchMode=yes" "$STATION:~/hackathon_data/$src/" "$B/$name/"
  n=$(python3 -c "import json; print(json.load(open('$B/$name/meta/info.json'))['total_episodes'])")
  echo "$name: $n episodes, $(du -sh "$B/$name" | cut -f1)"
  "$HF" upload "$USER_NS/nuck4-$name-$DATE" "$B/$name" . --repo-type dataset --private --quiet >/dev/null
  echo "  -> https://huggingface.co/datasets/$USER_NS/nuck4-$name-$DATE"
done
