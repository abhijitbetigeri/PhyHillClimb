#!/usr/bin/env bash
# PhyHillClimb simulation lab on Hugging Face Jobs. This runs INSIDE the job container (python:3.12-bookworm, root).
# Launched by hfjobs/launch.sh, which uploads the sources to the private dataset repo first.
#
# Environment (set by launch.sh): HF_TOKEN (secret), HF_REPO, RUN_NAME, LAB, CANDIDATES (batch mode, comma-separated
# file names under sim/candidates), HELDOUT=1 (freeze the best candidate for its held-out trials), KEEP_ALIVE=1 (ssh mode:
# keep the lab up until the job times out), N_WORKERS, DEV_TRIALS (informational; the lab's Budget decides).
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
REPO=${HF_REPO:-abhijitbetigeri/phyhillclimb-sim}
WORK=/work
ARCH=$(uname -m)
mkdir -p "$WORK/pkg" "$WORK/out"
cd "$WORK"
log() { printf '\n[bootstrap %s] %s\n' "$(date +%H:%M:%S)" "$*"; }

log "system packages"
apt-get update -qq
# Debian names the JPEG library libturbojpeg0 (Ubuntu: libturbojpeg); install one package at a time so a name
# that does not exist on this image is skipped instead of aborting the whole job
# cmake + nasm: the memory recorder's turbojpeg crate builds libjpeg-turbo from source; libsqlite3-dev: rusqlite
for pkg in ca-certificates curl git g++ build-essential pkg-config xz-utils cmake nasm libsqlite3-dev libturbojpeg0 \
    libturbojpeg libgl1 libegl1 libglib2.0-0 libsndfile1 libusb-1.0-0 libgfortran5 \
    libosmesa6 libegl-mesa0 libgl1-mesa-dri libglu1-mesa libglvnd0 libgles2; do
  apt-get install -y -qq --no-install-recommends "$pkg" >/dev/null 2>&1 || echo "  (no package $pkg on this image, skipped)"
done
log "uv and the hf CLI"
curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
uv tool install -q "huggingface_hub[cli]" >/dev/null
hf auth whoami | head -1

log "sources from $REPO"
hf download "$REPO" --repo-type dataset --include "src/*" --local-dir "$WORK/pkg" >/dev/null
tar xzf pkg/src/dimos-src.tgz -C "$WORK"
tar xzf pkg/src/phyhillclimb.tgz -C "$WORK"
cd "$WORK/dimos"
# dimos checks for a .git directory to treat itself as a checkout (get_project_root); without it, it clones the
# upstream repo into a data dir and rebuilds the native helpers with nix, which this image does not have
[ -d .git ] || git init -q .

CACHE="cache/venv-$ARCH.tgz"
if hf download "$REPO" --repo-type dataset --include "$CACHE" --local-dir "$WORK/pkg" >/dev/null 2>&1 && [ -f "$WORK/pkg/$CACHE" ]; then
  log "restoring the cached venv and native binaries ($CACHE)"
  tar xzf "$WORK/pkg/$CACHE" -C "$WORK/dimos" 2>/dev/null
  HAD_CACHE=1
else
  HAD_CACHE=0
fi
# the venv's python is a symlink into uv's managed interpreters, which live outside the venv: install it every time,
# then sync (fast when the cached venv already matches the lock, ~1 min otherwise)
log "python 3.12 and the dimos environment"
uv python install 3.12 >/dev/null
uv sync --python 3.12 --extra planning --extra web --extra agents --extra sim --extra learning --extra perception --extra cpu --no-group tests 2>&1 | tail -3
BIN_RAY=dimos/mapping/ray_tracing/rust/result/bin/voxel_ray_tracing
BIN_MEM=dimos/experimental/memory/rust/result/bin/dimos-memory-recorder
if [ ! -x "$BIN_RAY" ] || [ ! -x "$BIN_MEM" ]; then
  log "rust toolchain and native helpers (~4 min)"
  curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal >/dev/null
  export PATH="$HOME/.cargo/bin:$PATH"
  cargo build --release --manifest-path dimos/mapping/ray_tracing/rust/Cargo.toml --target-dir "$WORK/dimos/target" 2>&1 | grep -v "^\s*Compiling" | tail -8
  cargo build --release --manifest-path dimos/experimental/memory/rust/Cargo.toml --target-dir "$WORK/dimos/target" 2>&1 | grep -v "^\s*Compiling" | tail -25
  mkdir -p "$(dirname "$BIN_RAY")" "$(dirname "$BIN_MEM")"
  cp target/release/voxel_ray_tracing "$BIN_RAY"
  cp target/release/dimos-memory-recorder "$BIN_MEM"
  HAD_CACHE=0
fi
[ -x "$BIN_RAY" ] && [ -x "$BIN_MEM" ] && log "native helpers present" || { log "native helpers missing"; exit 1; }
if [ "$HAD_CACHE" = 0 ]; then
  log "uploading the environment cache for the next job"
  tar czf "$WORK/venv-cache.tgz" .venv dimos/mapping/ray_tracing/rust/result dimos/experimental/memory/rust/result
  hf upload "$REPO" "$WORK/venv-cache.tgz" "$CACHE" --repo-type dataset >/dev/null 2>&1 && log "cache uploaded" || log "cache upload failed (continuing)"
fi
uv pip install -q --python .venv/bin/python -e "$WORK/PhyHillClimb"
.venv/bin/dimos list 2>/dev/null | grep -i phyhillclimb || true

# MuJoCo renders the cameras offscreen; a CPU container has no GPU, so pick a software GL backend that works
GL_OK=""
for gl in osmesa egl; do
  if MUJOCO_GL=$gl .venv/bin/python - <<'PY' >/dev/null 2>&1
import mujoco
m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><light pos="0 0 1"/><geom size="0.1"/></worldbody></mujoco>')
r = mujoco.Renderer(m, 32, 32); r.update_scene(mujoco.MjData(m)); r.render()
PY
  then GL_OK=$gl; break; fi
done
if [ -z "$GL_OK" ]; then log "no working MuJoCo GL backend (osmesa, egl)"; exit 1; fi
export MUJOCO_GL=$GL_OK
log "MuJoCo offscreen rendering works with MUJOCO_GL=$GL_OK"

LAB=${LAB:-phyhillclimb.autoresearch-three-cube}
log "starting the lab $LAB with ${N_WORKERS:-12} workers on $(nproc) cpus"
.venv/bin/dimos --simulation --n-workers "${N_WORKERS:-12}" run "$LAB" --mujocosimmodule.headless true > "$WORK/lab.log" 2>&1 &
for i in $(seq 1 90); do
  grep -q "Blueprint started" "$WORK/lab.log" && break
  pgrep -f "dimos" >/dev/null || { log "lab died at startup"; grep -a -E "\[err\]|\[wrn\]|Error|rror:" "$WORK/lab.log" | grep -a -v -E "KeyError|resource_tracker|self filter" | head -40 | cut -c1-240; exit 1; }
  sleep 5
done
grep -q "Blueprint started" "$WORK/lab.log" || { log "lab did not start in time"; grep -a -E "\[err\]|\[wrn\]|Error|rror:" "$WORK/lab.log" | grep -a -v -E "KeyError|resource_tracker|self filter" | head -40 | cut -c1-240; exit 1; }
sleep 15
if grep -q "joint state stopped updating" "$WORK/lab.log"; then log "WARNING: the sim's joint state stalled at startup"; fi
.venv/bin/dimos mcp call task 2>/dev/null | head -c 300 || true
echo

if [ -n "${CANDIDATES:-}" ]; then
  log "batch mode: $CANDIDATES"
  .venv/bin/python "$WORK/PhyHillClimb/hfjobs/driver.py" --dimos .venv/bin/dimos \
    --candidates-dir "$WORK/PhyHillClimb/sim/candidates" --candidates "$CANDIDATES" \
    --out "$WORK/out/results.json" ${HELDOUT:+--heldout} || log "driver exited with an error"
  log "uploading results to runs/${RUN_NAME:-run}/"
  cp "$WORK/lab.log" "$WORK/out/lab.log" || true
  tar czf "$WORK/out/recordings.tgz" -C "$WORK/dimos" recordings 2>/dev/null || true
  hf upload "$REPO" "$WORK/out" "runs/${RUN_NAME:-run}" --repo-type dataset >/dev/null && log "results uploaded"
fi

if [ "${KEEP_ALIVE:-0}" = "1" ]; then
  log "ssh mode: the lab stays up until the job times out. Drive it with: cd /work/dimos && .venv/bin/dimos mcp call task"
  tail -f "$WORK/lab.log"
fi
log "done"
