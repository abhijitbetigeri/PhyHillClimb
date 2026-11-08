# PhyHillClimb

Hill Climb Relay entry for the MakerMods Physical AI Hackathon (Founders Inc, San Francisco, 7–8 November 2026).
Two MakerMods Metal arms on a station, a 5-minute relay: unplug a USB cable, stack three layers, call stop.
Dream-1 (Dream Labs' robot model) moves the arm for the unplug; a planner program found by simulation search
builds the stack; the **Relay Director** sequences the stages, verifies each pass bar and tells a teammate when
to call stop. `AGENTS.md` is the full brief (rubric, strategy, station facts); this file is the map and the
instructions.

## What is where

| Folder | What it is | Status |
|---|---|---|
| `AGENTS.md` | The team brief: rubric, scoring strategy, station, Dream kit, dimos, LiveKit, timeline, safety rules | read first |
| `harness/` | Dream-1 control loop: observation encoding, 48-row action chunks played by time, refill, recovery, stopping rule | passes `dream.check_loop` offline |
| `relay/` | LiveKit Portal room as the hardware abstraction: a robot process (fake backend or the station), the Relay Director as the operator, a Dream skill | local and cloud room verified end to end |
| `web/viewer.html` | Team dashboard: every camera track plus the Director's live score and advice | works in Chrome and Safari |
| `sim/` | dimos Autoresearch labs on a MuJoCo bench with the three foam cubes; candidate programs; judges | tower program 5/5 dev, 9/10 held-out |
| `hfjobs/` | Runs the simulation lab on Hugging Face Jobs instead of a laptop | working, see its README |
| `notes/`, `runs/`, `videos/`, `.env*` | internal notes, run logs, station videos, credentials | gitignored |

## Results so far (simulation, Saturday evening)

The stacking skill is a program against the dimos arm API, found by writing a candidate, running it on seeded
trials, reading the traces and keyframes, and fixing the measured cause. Every fix is a property of the real
arm, not a simulation trick, so the program should transfer.

| Program | Task | Seeded dev trials | Held-out trials | Time per pass |
|---|---|---|---|---|
| `sim/candidates/stack_v7.py` | red cube on blue cube | 5/5 | 6/10 | ~20 s |
| `sim/candidates/stack_tower_v5.py` | three-cube tower | 5/5 | 9/10 | 45–51 s |

What the search found about the Metal arm (details in `sim/README.md`): the planner addresses the jaw's mount,
and the finger pads sit 7.8 cm further out; the arm settles 2–4 cm below any commanded pose (gravity sag, on
purpose in the sim because the hardware does it), and only an absolute re-aim cancels it; gripper commands return
before the jaw moves; a cube ends 0.9 cm from the tool axis along the jaw's opening direction when released; a
cube cleared from the obstacle map stays cleared only where it was.

## Quick start

### 1. Environment on a laptop

```sh
git clone https://github.com/abhijitbetigeri/PhyHillClimb && cd PhyHillClimb
uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python -r requirements.txt
source .venv/bin/activate
cp .env.example .env              # Dream credentials (never commit .env)
cp .env.local.example .env.local  # LiveKit credentials: `lk app env -w` fills it
```

### 2. Check the Dream loop with no arm and no model

```sh
python -m dream.check_loop --rate 20          # terminal 1; type "the workspace is clear" when asked
DREAM_GATEWAY_URL=http://127.0.0.1:8642 DREAM_API_URL=http://127.0.0.1:8642 DREAM_DATA_URL=http://127.0.0.1:8642 \
DREAM_API_KEY=x python harness/dream_harness.py --fake --no-recover --seconds 40   # terminal 2
```

The stand-in prints PASS or PROBLEM for hold, part isolation, direction and size, lag, play rate and circle.

### 3. The relay room and dashboard (no hardware)

```sh
livekit-server --dev --bind 127.0.0.1                      # or set LIVEKIT_URL/KEY/SECRET in .env.local for the cloud room
python -m relay.robot_sim --backend fake                    # a robot publishing 3 cameras and 14 state values
python -m relay.director --seconds 25 --pass-after 5        # the Director: claims control, runs stages, projects the score
python -m http.server 8090 --bind 0.0.0.0 --directory web   # dashboard at http://<host>:8090/viewer.html#url=...&token=...
```

Tokens for viewers: `lk token create --join --room relay-nuck4 --identity viewer-1 --valid-for 48h`.
`relay/README.md` has the codec and viewer notes.

### 4. The stacking simulation

The lab needs the dimos checkout (hackathon branch `hackathon/feat/metal-autoresearch`) beside this repo and
is CPU-only. It does not train anything; it scores programs.

```sh
uv pip install --python ~/projects/dimos/.venv/bin/python -e .          # registers the two labs with dimos
cd ~/projects/dimos
.venv/bin/dimos --simulation --n-workers 8 run phyhillclimb.autoresearch-three-cube --mujocosimmodule.headless true
# in another terminal: score a candidate on the lab's seeded trials
python hfjobs/driver.py --dimos ~/projects/dimos/.venv/bin/dimos --candidates stack_tower_v5.py --heldout
```

Or run the whole thing on a 16-vCPU Hugging Face Jobs machine ($1 per hour) and drive it over ssh:

```sh
hfjobs/launch.sh ssh                                   # builds once (~10 min), then restores a cache in ~2 min
hfjobs/remote.sh 'cd /work/dimos && .venv/bin/python /work/PhyHillClimb/hfjobs/driver.py --dimos .venv/bin/dimos \
  --candidates-dir /work/PhyHillClimb/sim/candidates --candidates stack_tower_v5.py --heldout'
hf jobs cancel <job id>                                # stop paying when done
```

`sim/README.md` lists every candidate, what its traces showed and the fix that followed. `hfjobs/README.md`
lists the container lessons (curl, `.git`, uv's Python, cmake/nasm/sqlite, software OpenGL).

### 5. The station (Dream kit, Quest teleop, fine-tune)

Everything on the station runs over ssh in the official kit at `~/physical-ai-hacks`; `AGENTS.md` sections 3–5
have the commands. The short version:

```sh
ssh -L 8080:localhost:8080 pah@10.103.0.129 && cd ~/physical-ai-hacks
uv run dhk-collect --config configs/station.yaml --dry-run                      # arms never move
uv run dhk-collect --config configs/station.yaml --dataset-dir ~/hackathon_data/unplug --task "unplug the usb cable"
~/dream-env/bin/python tools/add_eef.py ~/hackathon_data/unplug --arm metal     # tool poses for Metal
~/dream-env/bin/python tools/finetune.py ~/hackathon_data/unplug-eef --arm metal --check   # free
~/dream-env/bin/python tools/finetune.py ~/hackathon_data/unplug-eef --arm metal --name phyhillclimb --suffix unplug-v1  # $200, 7–8 h
dreamcli robot run --task "unplug the usb cable" --checkpoint dream-1-ft:<acct>-unplug-v1 --dry-run
```

Fine-tuning data comes only from the kit's collector on the station: public datasets exist for other arms, but
Dream-1's Metal embodiment needs this station's cameras and joint layout. Aim for 50 varied unplug episodes.

## Rules for everyone

- A dry mode for anything that moves an arm; a person with a hand on the e-stop for every real run.
- Never print or commit a key; `.env`, `.env.local`, `notes/` and `runs/` stay out of git.
- Touching a later stage's object before the current pass bar is a disqualification: the Director's DQ guard
  exists for that, and so should your caution.
- Disclosure: the harness skeleton in `harness/` was written before the event against the public Dream client.
  Everything that touches the station, the dataset and the fine-tune is built during the hackathon.

## Resources

- Getting started: https://www.livekit.info/pah
- Dream kit: https://github.com/dreamlabs-co/physical-ai-hack-2026
- dimos (Dimensional): https://github.com/dimensionalOS/dimos
- Maker Arm SDK: https://github.com/makermods-robotics/maker-arm-sdk
- LiveKit Portal example: https://github.com/livekit-examples/portal-makermods
