#!/usr/bin/env bash
# Runs INSIDE a clean python:3.12-bookworm HF Job (x86_64, like the NUC): install_station.sh from scratch, again
# (idempotence), preflight without CAN/camera, then the echo-mock stack with the pattern camera and two programs.
# Launched by hand:
#   hf upload <repo> /tmp/phyhillclimb-real.tgz src/install-test/phyhillclimb-real.tgz --repo-type dataset
#   hf jobs run --detach --flavor cpu-xl --timeout 1h --secrets HF_TOKEN -e HF_REPO=<repo> python:3.12-bookworm \
#     bash -c 'curl -fsSL -H "Authorization: Bearer $HF_TOKEN" https://huggingface.co/datasets/<repo>/resolve/main/src/install-test/install_test_job.sh | bash'
set -uo pipefail
REPO=${HF_REPO:?}
log() { printf '\n[install-test %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
# The NUC is Ubuntu with rsync; this image lacks it. Nothing else is pre-installed for the test.
apt-get update -qq >/dev/null && apt-get install -y -qq rsync >/dev/null && log "rsync installed (the NUC has it)"
export APT=1   # libGL for OpenCV, through the installer's own APT=1 path
mkdir -p ~/dimos-real-src && cd ~/dimos-real-src
curl -fsSL -H "Authorization: Bearer $HF_TOKEN" -o repo.tgz \
  "https://huggingface.co/datasets/$REPO/resolve/main/src/install-test/phyhillclimb-real.tgz"
tar xzf repo.tgz 2>/dev/null && ls
INSTALL="bash $HOME/dimos-real-src/PhyHillClimb/real/install_station.sh"
log "1. --check on a clean machine"; INSTALL_UV=1 $INSTALL --check; echo "exit $?"
log "2. install"; START=$(date +%s); INSTALL_UV=1 $INSTALL; RC=$?; log "install exit $RC after $(( $(date +%s) - START )) s"
log "3. install again (idempotent)"; START=$(date +%s); $INSTALL; log "second install exit $? after $(( $(date +%s) - START )) s"
ls -d ~/dimos-real* ; cat ~/dimos-real/.phyhillclimb-real 2>/dev/null; git -C ~/dimos-real log --oneline -1; git -C ~/dimos-real status --short | head
log "4. preflight without hardware"; SKIP_HW=1 bash ~/dimos-real/PhyHillClimb/real/preflight.sh; echo "preflight exit $?"
log "5. echo-mock stack + pattern camera"
cd ~/dimos-real && (MOCK=1 nohup bash PhyHillClimb/real/start_real_stack.sh > /tmp/stack.log 2>&1 &)
for i in $(seq 1 60); do grep -aq "Blueprint started" /tmp/stack.log && break; sleep 5; done
grep -a -E "Blueprint started|Traceback|rror" /tmp/stack.log | cut -c1-200 | head -8
PY=~/dimos-real/.venv/bin/python
cd ~/dimos-real/PhyHillClimb
timeout 120 $PY -m real.scene_ctl pose
timeout 600 $PY real/run_program_real.py --program real/programs/hover_one.py --task "hover over cube_red" --yes --out /tmp/run_hover | tail -4
timeout 900 $PY real/run_program_real.py --program sim/candidates/tower_order_v24.py --yes --out /tmp/run_tower | grep -E "outcome|released|grasped" | head -12
timeout 300 $PY real/run_program_real.py --program sim/candidates/tower_order_v24.py --plan-only --timeout 120 --out /tmp/run_plan | grep -E "outcome" ; grep -c "planned, not executed" /tmp/run_plan/program.log
~/dimos-real/.venv/bin/dimos stop >/dev/null 2>&1
log "done"
