#!/usr/bin/env bash
# Install the real-arm program stack on the station NUC (Ubuntu x86_64), in ~/dimos-real only.
#
#   bash ~/dimos-real-src/PhyHillClimb/real/install_station.sh --check   # report what is there / what it would do
#   bash ~/dimos-real-src/PhyHillClimb/real/install_station.sh           # install or update (idempotent)
#
# What it touches: ~/dimos-real (dimos clone + .venv + a copy of this repo), uv's own user caches and its managed
# Python under ~/.local/share/uv. NOT the Dream kit (~/physical-ai-hacks, ~/dream-env, ~/.dreamlabs), NOT the
# teammates' ~/Superfast-harness, no system packages (it prints the apt line if one is missing; APT=1 runs it with
# sudo), no CAN or camera access. Anything it would replace is moved to <path>.bak.<timestamp> first.
#
# Steps
#   1. dimos: clone https://github.com/dimensionalOS/dimos (branch hackathon/feat/metal-autoresearch) into
#      ~/dimos-real, check out the commit we tested (DIMOS_REF, default 1cb67a6220) and apply real/dimos-local.patch
#      (SceneObject.yaw, which tower_order_v24 reads). DIMOS_SRC=/path uses a copy instead of cloning (no internet).
#   2. venv: uv sync --python 3.12 with EXTRAS (default "planning web": the coordinator, CAN driver, planner,
#      viser). FULL=1 uses hfjobs/bootstrap.sh's extras (planning web agents sim learning perception cpu), needed
#      only for the MuJoCo stacks. The HF cached venv (cache/venv-x86_64.tgz) is NOT relocatable: it was built as
#      root at /work/dimos, with absolute interpreter links and shebangs, so it is not restored as a venv.
#   3. Rust helpers (voxel ray tracer, memory recorder): not used by the real-program stack, skipped unless
#      HELPERS=1 (then fetched from the HF cache with HF_TOKEN set, else built with cargo if present).
#   4. this repo: copied (rsync, without notes/ runs/ videos/ .env*) from the checkout the script runs from into
#      ~/dimos-real/PhyHillClimb and pip-installed editable into the venv, which registers the entry points
#      phyhillclimb.metal-real-program and phyhillclimb.metal-real-program-testcam.
#   5. checks: imports, entry points, the measured-scene unit tests (pytest is added to the venv for that).
set -euo pipefail
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1
TS=$(date +%Y%m%d_%H%M%S)
DEST=${DEST:-$HOME/dimos-real}
DIMOS_URL=${DIMOS_URL:-https://github.com/dimensionalOS/dimos.git}
DIMOS_BRANCH=${DIMOS_BRANCH:-hackathon/feat/metal-autoresearch}
DIMOS_REF=${DIMOS_REF:-1cb67a6220cca1eec222b6c181f7d8e4e40160a4}
EXTRAS=${EXTRAS:-planning web}
[ "${FULL:-0}" = 1 ] && EXTRAS="planning web agents sim learning perception cpu"
HERE=$(cd "$(dirname "$0")" && pwd)
SRC_REPO=$(cd "$HERE/.." && pwd)
PATCH="$HERE/dimos-local.patch"
MARK="$DEST/.phyhillclimb-real"
say() { printf '[install %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
would() { if [ "$CHECK" = 1 ]; then say "would: $*"; return 0; fi; return 1; }
backup() { # move an existing path aside, never delete
  if [ -e "$1" ] || [ -L "$1" ]; then
    if would "move $1 to $1.bak.$TS"; then return 0; fi
    mv "$1" "$1.bak.$TS"; say "backed up $1 -> $1.bak.$TS"
  fi
}

say "target $DEST, repo source $SRC_REPO, dimos $DIMOS_BRANCH @ ${DIMOS_REF:0:10}, extras: $EXTRAS"
[ "$(uname -s)" = Linux ] || say "WARNING: not Linux; SocketCAN needs the NUC"
[ "$(uname -m)" = x86_64 ] || say "WARNING: not x86_64"
case "$SRC_REPO" in "$DEST"/*) say "WARNING: running from inside $DEST; copy the repo elsewhere (e.g. ~/dimos-real-src) first";; esac

# ---- 0. tools: uv (user-level), git, rsync; system libraries only reported ----
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
for tool in git rsync; do command -v $tool >/dev/null || { say "missing $tool: sudo apt-get install -y $tool"; exit 1; }; done
if ! command -v uv >/dev/null; then
  if [ "${INSTALL_UV:-0}" = 1 ]; then
    would "install uv into ~/.local/bin (curl -LsSf https://astral.sh/uv/install.sh | sh)" || curl -LsSf https://astral.sh/uv/install.sh | sh
  else
    say "uv not on PATH (the kit uses it, try: ls ~/.local/bin ~/.cargo/bin). INSTALL_UV=1 installs it user-level"; exit 1
  fi
fi
command -v uv >/dev/null && say "uv $(uv --version 2>/dev/null | cut -d' ' -f2)"
# OpenCV (the camera, the keyframes) needs libGL and GLib; a desktop Ubuntu has them, a minimal image does not
MISSING=""
for lib in libgl1 libglib2.0-0; do
  dpkg -s "$lib" >/dev/null 2>&1 || dpkg -s "${lib}t64" >/dev/null 2>&1 || MISSING="$MISSING $lib"
done
if [ -n "$MISSING" ]; then
  say "REQUIRED system libraries missing:$MISSING. Would run: sudo apt-get install -y$MISSING (APT=1 runs it)"
  if [ "${APT:-0}" = 1 ] && [ "$CHECK" = 0 ]; then sudo -n true 2>/dev/null && SUDO=sudo || SUDO=""; [ "$(id -u)" = 0 ] && SUDO=""
    $SUDO apt-get install -y -qq $MISSING >/dev/null && say "installed$MISSING" || say "apt failed"; fi
fi

# ---- 1. dimos ----
if [ -e "$DEST" ] && [ ! -f "$MARK" ]; then
  say "$DEST exists and is not ours"; backup "$DEST"
fi
if [ "$CHECK" = 1 ] && [ ! -d "$DEST/.git" ]; then
  would "clone $DIMOS_URL ($DIMOS_BRANCH) into $DEST, check out ${DIMOS_REF:0:10}, apply $(basename "$PATCH")"
else
  if [ ! -d "$DEST/.git" ]; then
    if [ -n "${DIMOS_SRC:-}" ]; then
      say "copying dimos from $DIMOS_SRC"; mkdir -p "$DEST"
      rsync -a --exclude .venv --exclude recordings --exclude target "$DIMOS_SRC"/ "$DEST"/
      [ -d "$DEST/.git" ] || git -C "$DEST" init -q
    else
      say "cloning dimos"; git clone -q --branch "$DIMOS_BRANCH" "$DIMOS_URL" "$DEST"
    fi
    touch "$MARK"
  fi
  touch "$MARK"
  if [ -z "${DIMOS_SRC:-}" ]; then
    if [ "$(git -C "$DEST" rev-parse HEAD)" != "$DIMOS_REF" ]; then
      if [ -n "$(git -C "$DEST" status --porcelain --untracked-files=no)" ]; then
        say "dimos has local changes; saving them as $DEST.local-changes.$TS.patch"; git -C "$DEST" diff > "$DEST.local-changes.$TS.patch"
        git -C "$DEST" checkout -q -- .
      fi
      git -C "$DEST" fetch -q origin "$DIMOS_BRANCH" || say "fetch failed (offline?), using what is there"
      git -C "$DEST" checkout -q "$DIMOS_REF"
    fi
    if git -C "$DEST" apply --reverse --check "$PATCH" 2>/dev/null; then
      say "local dimos patch already applied"
    else
      git -C "$DEST" apply "$PATCH" && say "applied $(basename "$PATCH")"
    fi
  fi
  grep -q "yaw: float = 0.0" "$DEST/dimos/autoresearch/program.py" || { say "SceneObject.yaw missing: patch failed"; exit 1; }
fi

# ---- 2. venv ----
if [ "$CHECK" = 1 ]; then
  if [ -x "$DEST/.venv/bin/python" ]; then say "venv present: $("$DEST/.venv/bin/python" -V 2>&1)"; else would "uv python install 3.12 && uv sync --python 3.12 $(printf -- '--extra %s ' $EXTRAS)--no-group tests (5-15 min, several GB download)"; fi
else
  say "python 3.12 (uv-managed, user level)"; uv python install 3.12 >/dev/null
  say "uv sync with extras: $EXTRAS (first time 5-15 min)"
  # shellcheck disable=SC2046
  (cd "$DEST" && uv sync --python 3.12 $(printf -- '--extra %s ' $EXTRAS) --no-group tests 2>&1 | tail -3)
fi

# ---- 3. rust helpers (not needed for the real program) ----
if [ "${HELPERS:-0}" = 1 ] && [ "$CHECK" = 0 ]; then
  BIN_RAY=dimos/mapping/ray_tracing/rust/result/bin/voxel_ray_tracing
  BIN_MEM=dimos/experimental/memory/rust/result/bin/dimos-memory-recorder
  if [ ! -x "$DEST/$BIN_RAY" ] || [ ! -x "$DEST/$BIN_MEM" ]; then
    if [ -n "${HF_TOKEN:-}" ]; then
      say "fetching the helper binaries from the HF cache"; TMP=$(mktemp -d)
      curl -fsSL -H "Authorization: Bearer $HF_TOKEN" -o "$TMP/c.tgz" \
        "https://huggingface.co/datasets/${HF_REPO:-abhijitbetigeri/phyhillclimb-sim}/resolve/main/cache/venv-x86_64.tgz"
      tar xzf "$TMP/c.tgz" -C "$DEST" dimos/mapping/ray_tracing/rust/result dimos/experimental/memory/rust/result; rm -rf "$TMP"
    elif command -v cargo >/dev/null; then
      (cd "$DEST" && cargo build --release --manifest-path dimos/mapping/ray_tracing/rust/Cargo.toml --target-dir target \
        && cargo build --release --manifest-path dimos/experimental/memory/rust/Cargo.toml --target-dir target \
        && mkdir -p "$(dirname "$BIN_RAY")" "$(dirname "$BIN_MEM")" && cp target/release/voxel_ray_tracing "$BIN_RAY" \
        && cp target/release/dimos-memory-recorder "$BIN_MEM")
    else say "no HF_TOKEN and no cargo: helpers skipped"; fi
  fi
else
  say "rust helpers skipped (not used by phyhillclimb.metal-real-program)"
fi

# ---- 4. this repo ----
PHC="$DEST/PhyHillClimb"
if would "rsync $SRC_REPO -> $PHC (without notes/ runs/ videos/ .env*), backing up a changed copy; uv pip install -e"; then :; else
  if [ -d "$PHC" ] && [ "$(cd "$PHC" && pwd)" != "$SRC_REPO" ]; then
    if ! diff -rq --exclude=__pycache__ --exclude=runs --exclude=notes --exclude=videos --exclude='.env*' --exclude=.git --exclude=.venv \
         "$SRC_REPO" "$PHC" >/dev/null 2>&1; then
      say "PhyHillClimb copy differs from the source; backing up"; cp -a "$PHC" "$PHC.bak.$TS"
    fi
  fi
  mkdir -p "$PHC"
  rsync -a --delete --exclude .git --exclude .venv --exclude notes --exclude runs --exclude videos --exclude '.env*' \
    --exclude __pycache__ --exclude 'real/runs' "$SRC_REPO"/ "$PHC"/
  mkdir -p "$PHC/real/runs"
  uv pip install -q --python "$DEST/.venv/bin/python" -e "$PHC" pytest
  say "PhyHillClimb installed editable"
fi

# ---- 5. checks ----
if [ "$CHECK" = 0 ] || [ -x "$DEST/.venv/bin/python" ]; then
  PY="$DEST/.venv/bin/python"
  "$PY" - <<'PY' && say "imports ok" || { say "IMPORT CHECK FAILED (cv2 needs: sudo apt-get install -y libgl1 libglib2.0-0)"; exit 1; }
import cv2, can_motor_control, dimos.autoresearch.live, dimos.robot.manipulators.makermods_metal.connection  # noqa
import real.blueprint_real, real.measured_scene, real.plan_only  # noqa
from dimos.autoresearch.program import SceneObject
assert "yaw" in SceneObject.__dataclass_fields__
PY
  "$DEST/.venv/bin/dimos" list 2>/dev/null | grep -E "metal-real-program" && say "entry points registered" || say "ENTRY POINTS MISSING"
  (cd "$DEST/PhyHillClimb" && "$PY" -m pytest -c pyproject.toml real/tests -q -p no:cacheprovider 2>&1 | tail -1) || say "UNIT TESTS FAILED"
fi
say "done. Next: bash $DEST/PhyHillClimb/real/preflight.sh"
