#!/usr/bin/env bash
# Run the simulation lab on Hugging Face Jobs instead of this Mac.
#
#   hfjobs/launch.sh batch stack_tower_v5.py [more.py ...]   # score candidates on the lab's dev seeds, results uploaded
#   HELDOUT=1 hfjobs/launch.sh batch stack_v7.py             # ...and freeze the best one for its 10 held-out trials
#   hfjobs/launch.sh ssh                                      # interactive lab; connect with `hf jobs ssh <job id>`
#   hfjobs/launch.sh fetch <run name>                         # download a batch run's results into runs/hf/<run name>
#
# Knobs: FLAVOR (cpu-xl 16 vCPU $1/h default; cpu-upgrade 8 vCPU $0.03/h), TIMEOUT (4h), LAB (blueprint name),
# N_WORKERS (12). The job pulls the dimos checkout and this repo from the private dataset repo $REPO, which this
# script uploads first (nothing from .env*, notes/, runs/ or videos/ is packed). The first job builds the dimos
# environment (~10 min) and caches it in the repo; later jobs restore it in about a minute.
# ssh mode needs ~/.ssh/hfjobs_ed25519.pub registered at https://huggingface.co/settings/keys.
set -euo pipefail
HF=${HF:-$(command -v hf || echo "$HOME/OneRobotLoop/.venv/bin/hf")}
REPO=${REPO:-abhijitbetigeri/phyhillclimb-sim}
FLAVOR=${FLAVOR:-cpu-xl}
TIMEOUT=${TIMEOUT:-4h}
LAB=${LAB:-phyhillclimb.autoresearch-three-cube}
N_WORKERS=${N_WORKERS:-12}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DIMOS=${DIMOS_ROOT:-$HOME/projects/dimos}
MODE=${1:-batch}
shift || true

if [ "$MODE" = "fetch" ]; then
  RUN=${1:?run name}
  mkdir -p "$ROOT/runs/hf"
  "$HF" download "$REPO" --repo-type dataset --include "runs/$RUN/*" --local-dir "$ROOT/runs/hf"
  echo "results in $ROOT/runs/hf/runs/$RUN"
  exit 0
fi

TMP=$(mktemp -d)
echo "packing dimos ($DIMOS) and $ROOT"
tar --exclude=.venv --exclude=target --exclude=recordings --exclude=.git --exclude=__pycache__ --exclude='*.pyc' \
    --exclude=MUJOCO_LOG.TXT -czf "$TMP/dimos-src.tgz" -C "$(dirname "$DIMOS")" "$(basename "$DIMOS")"
tar --exclude=.venv --exclude=runs --exclude=notes --exclude=videos --exclude='.env*' --exclude=.git \
    --exclude=__pycache__ --exclude='*.pyc' -czf "$TMP/phyhillclimb.tgz" -C "$(dirname "$ROOT")" "$(basename "$ROOT")"
ls -lh "$TMP"
echo "uploading sources to $REPO"
"$HF" upload "$REPO" "$TMP/dimos-src.tgz" src/dimos-src.tgz --repo-type dataset >/dev/null
"$HF" upload "$REPO" "$TMP/phyhillclimb.tgz" src/phyhillclimb.tgz --repo-type dataset >/dev/null
"$HF" upload "$REPO" "$ROOT/hfjobs/bootstrap.sh" src/bootstrap.sh --repo-type dataset >/dev/null
rm -rf "$TMP"

RUN=$(date +%Y%m%d_%H%M%S)
# python:3.12-bookworm ships curl and git; plain ubuntu:24.04 has neither, so the bootstrap could not even download
IMAGE=${IMAGE:-python:3.12-bookworm}
BOOT='curl -fsSL -H "Authorization: Bearer $HF_TOKEN" "https://huggingface.co/datasets/'"$REPO"'/resolve/main/src/bootstrap.sh" | bash'
COMMON=(--detach --flavor "$FLAVOR" --timeout "$TIMEOUT" --secrets HF_TOKEN -e "HF_REPO=$REPO" -e "RUN_NAME=$RUN" -e "LAB=$LAB" -e "N_WORKERS=$N_WORKERS")
case "$MODE" in
  batch)
    [ $# -ge 1 ] || { echo "batch mode needs candidate file names (under sim/candidates)"; exit 2; }
    CANDS=$(IFS=,; echo "$*")
    echo "launching batch run $RUN on $FLAVOR: $CANDS (lab $LAB)"
    "$HF" jobs run "${COMMON[@]}" -e "CANDIDATES=$CANDS" ${HELDOUT:+-e HELDOUT=1} "$IMAGE" bash -c "$BOOT"
    echo "watch:  $HF jobs logs <job id>      fetch when done:  hfjobs/launch.sh fetch $RUN"
    ;;
  ssh)
    echo "launching interactive lab $RUN on $FLAVOR (lab $LAB)"
    "$HF" jobs run "${COMMON[@]}" --ssh -e KEEP_ALIVE=1 "$IMAGE" bash -c "$BOOT"
    echo "connect once the log says 'ssh mode':  $HF jobs ssh <job id>   then  cd /work/dimos && .venv/bin/dimos mcp call task"
    ;;
  *) echo "usage: $0 batch <candidates...> | ssh | fetch <run>"; exit 2 ;;
esac
