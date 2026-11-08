# Running the simulation lab on Hugging Face Jobs

The dimos Autoresearch lab is CPU work (MuJoCo headless plus the planner). On the Mac it competes with everything
else and the simulator's physics loop stalls under load, so the lab runs on a Hugging Face Jobs machine instead.
Nothing trains here; Dream-1 fine-tuning stays on Dream Labs' cloud from the station's teleop demos.

## One-time setup

1. Prepaid credits on the HF account (Jobs refuse with `402 Payment Required` otherwise):
   https://huggingface.co/settings/billing. A `cpu-xl` job (16 vCPU) is $1 per hour; `cpu-upgrade` (8 vCPU) is $0.03.
2. For interactive use, register `~/.ssh/hfjobs_ed25519.pub` at https://huggingface.co/settings/keys.
3. Private dataset repo `abhijitbetigeri/phyhillclimb-sim` exists; it holds the packed sources, the cached
   environment and every run's results.

## Modes

```sh
hfjobs/launch.sh batch stack_tower_v5.py            # score candidates on the lab's seeded dev trials
HELDOUT=1 hfjobs/launch.sh batch stack_v7.py        # ...then freeze the best and score its 10 held-out trials
LAB=phyhillclimb.autoresearch-stack hfjobs/launch.sh batch stack_v7.py   # the level-1 lab instead of the tower
hfjobs/launch.sh ssh                                 # interactive lab, then: hf jobs ssh <job id>
hfjobs/launch.sh fetch <run name>                    # results.json, lab.log and recordings.tgz into runs/hf/
hf jobs logs <job id>                                # live log of any job
hfjobs/remote.sh 'cd /work/dimos && .venv/bin/dimos mcp call leaderboard'   # run a command in the ssh job
hfjobs/remote.sh push sim/candidates/stack_tower_v6.py                       # copy a new candidate into the job
hf jobs cancel <job id>                              # stop paying when done (jobs bill per minute)
```

Lessons from the first launches (all fixed in `bootstrap.sh`): the image must have curl (`python:3.12-bookworm`);
dimos needs a `.git` directory or it clones itself elsewhere and rebuilds its helpers with nix; the cached venv's
Python is a uv-managed interpreter, so `uv python install 3.12` runs every time; the memory recorder crate needs
cmake, nasm and SQLite headers; MuJoCo's offscreen renderer needs software GL (`libosmesa6`, `MUJOCO_GL=osmesa`).

The first job builds the dimos environment and the two Rust helpers (about 10 minutes) and uploads them as
`cache/venv-x86_64.tgz`; later jobs restore that in about a minute. Each run uploads `runs/<run name>/results.json`
(verdicts, judge notes, the program's own log lines per trial), `lab.log` and `recordings.tgz` (traces and keyframes).

Inside an ssh job the lab is at `/work/dimos` and this repo at `/work/PhyHillClimb`; drive it exactly as on the Mac:
`cd /work/dimos && .venv/bin/dimos mcp call task`, `submit`, `run`, `results`, `leaderboard`, `freeze`.

## Files

- `bootstrap.sh`: runs inside the container; installs packages, restores or builds the environment, starts the
  lab, runs `driver.py` in batch mode, uploads results, or keeps the lab alive in ssh mode.
- `driver.py`: submits each candidate, runs its dev trials, prints verdicts and per-trial program logs, optionally
  freezes the best for held-out trials. Works on any machine where the lab's MCP server is up.
- `launch.sh`: packs `~/projects/dimos` (no venv, no recordings) and this repo (no `.env*`, `notes/`, `runs/`,
  `videos/`), uploads them, and starts the job.
