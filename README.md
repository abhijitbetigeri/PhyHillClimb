# PhyHillClimb

Hill Climb Relay entry for the MakerMods Physical AI Hackathon (Founders Inc, San Francisco, 7–8 November 2026).
Two MakerMods Metal arms on a station, a 5-minute relay: unplug a USB cable, stack three layers, call stop.
Dream-1 (Dream Labs' robot model) moves the arm for the unplug; a planner program found by simulation search
builds the stack; the **Relay Director** sequences the stages, verifies each pass bar and tells a teammate when
to call stop. `AGENTS.md` is the full brief (rubric, strategy, station facts); this file is the map and the
instructions.

## Submission summary (Sunday 8 November, 15:00)

What works, and where it was verified:

| Result | Where | Measured |
|---|---|---|
| Unplug a USB cable with the real right Metal arm: Dream-1 fine-tune reaches the plug, the runner's assist closes, raises the hold torque and pulls | station NUCK-4, `tools/dream_right_arm_run.py` | 5 of 5 releases in a row, about 35 s each |
| Three-cube tower in the ordered task, planner program found by Autoresearch search | MuJoCo Metal bench, `sim/candidates/tower_order_v24.py` | 5/5 seeded dev trials, 9/10 held-out, 90–97 s, strict 3 s stability judge |
| Unplug both plugs from the hub, planner program | MuJoCo cable bench, `sim/candidates/cable_v1.py` | 5/5 dev, 10/10 held-out, 36–38 s for both plugs |
| Real station live in a LiveKit room: cameras, joint state, runner status, team dashboard | LiveKit Cloud room `relay-nuck4`, `relay/station_bridge.py`, `web/viewer.html` | 29 synchronized observations/s, three tracks, status RPC |
| Relay Director as a LiveKit Portal operator with a Dream-1 skill | `relay/director.py`, `relay/skills/dream.py` | passes Dream's loop check through the cloud room |

Not solved: stacking on the real arm (the fine-tune reaches a cube but never closes; 10 demos), the stick in the
top cube (the carry fails in the simulator), transfer of the tower program to the real arm (runbook in `real/`,
measured-scene mode scored 1/5 in simulation). Everything in this table has a log in the repo or a command below
that reproduces it; nothing here is a projection.

Dream-1 fine-tunes (Dream Labs API, single right arm, 7-wide state): `dream-1-ft:2d43713b-right-arm-v1` (30 episodes:
unplug, stack, insert), `-right-arm-v1-plus500` (continued 500 steps, the match model), `-right-unplug-v1` (unplug only),
`-left-arm-v1` and `-left-arm-v1-plus500` (12 left-arm episodes). Built with `tools/dream_single_arm_finetune.py`.

Stack: Dream Labs Dream-1 (policy and fine-tuning), LiveKit Cloud + LiveKit Portal (transport, dashboard),
Dimensional dimos (Autoresearch lab, planner, MuJoCo Metal), MakerMods Metal arms, Hugging Face Jobs (simulation
compute). Disclosure: the Dream harness skeleton in `harness/` predates the event; everything else was built during
the hackathon. How LiveKit was used: see the section below.

## What is where

| Folder | What it is | Status |
|---|---|---|
| `AGENTS.md` | The team brief: rubric, scoring strategy, station, Dream kit, dimos, LiveKit, timeline, safety rules | read first |
| `harness/` | Dream-1 control loop: observation encoding, 48-row action chunks played by time, refill, recovery, stopping rule | passes `dream.check_loop` offline |
| `relay/` | LiveKit Portal room as the hardware abstraction: a robot process (fake backend or the station bridge), the Relay Director as the operator, a Dream skill, a room peek tool | local and cloud room verified end to end; the real station publishes through `station_bridge.py` |
| `web/viewer.html` | Team dashboard: every camera track plus the Director's live score and advice | works in Chrome and Safari |
| `sim/` | dimos Autoresearch labs on a MuJoCo bench: the three foam cubes (`stack_bench.xml`, `lab_stack.py`) and the stage 1 cable set, a USB hub with plugs standing in its sockets (`cable_bench.xml`, `lab_cable.py`); candidate programs; judges | tower program 5/5 dev, 9/10 held-out; unplug program `cable_v1` 5/5 dev, 10/10 held-out, 36 s for both plugs |
| `hfjobs/` | Runs the simulation lab on Hugging Face Jobs instead of a laptop | working, see its README |
| `notes/`, `runs/`, `videos/`, `.env*` | internal notes, run logs, station videos, credentials | gitignored |

## Results so far (simulation, Saturday evening)

The stacking skill is a program against the dimos arm API, found by writing a candidate, running it on seeded
trials, reading the traces and keyframes, and fixing the measured cause. These are simulation measurements; hardware transfer has not been established.
The rates below are historical geometry-only scores, not results from the current release/stability verifier.

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

## How we used LiveKit

The LiveKit room is the hardware abstraction of the whole entry: one wire format for the fake robot, the simulator
and the real Metal station, and one place where the policy, the dashboard and the team meet. Built on
**LiveKit Portal** (the robotics transport behind [portal-makermods](https://github.com/livekit-examples/portal-makermods))
on a LiveKit Cloud project, with the hackathon credits.

| Piece | LiveKit feature | File | Verified |
|---|---|---|---|
| Station publisher | Portal **robot**: three VP8 camera tracks + the 14-value joint state at 30 Hz, `status` RPC | `relay/station_bridge.py` + `tools/relay_tap.py` (hooked into the arm runner with `--relay`) | local room: 28.8 synchronized observations/s, 3 tracks decoded, RPC answered; cloud room `relay-nuck4` from the station |
| Relay Director | Portal **operator**: claims control, synchronized observations, actions, publishes `status` over RPC | `relay/director.py` | 108 observations and 177 actions in 25 s against the fake robot |
| Dream-1 skill | the official Portal control loop ported behind the Director's `Skill` interface | `relay/skills/dream.py` | passes Dream's `check_loop` through the cloud room (hold, parts, play rate, circle) |
| Fake robot | Portal robot with synthetic cameras and a tracking arm, for development without hardware | `relay/robot_sim.py` | end to end with the Director |
| Dashboard | browser client (`livekit-client`), subscribes to every track, calls the `status` RPC each second | `web/viewer.html` | Chrome and Safari; teammates' phones on the venue Wi-Fi |
| Room peek | Portal operator that reports rates, track sizes, state and status; sends nothing | `relay/peek.py` | used for every check above |

What it bought us:

- **Policy and hardware decoupled.** The Director was written and tested against the fake robot on a laptop, then
  pointed at the station with no code change: same tracks, same state names, same RPC.
- **Safety by construction.** The station bridge is publish-only and runs in its own venv and process. If it dies
  the arm runner never notices; actions from the room are counted and ignored, the runner keeps the motors.
- **Everyone sees the same thing.** Judges and teammates open one page: live cameras, the arm state, the runner's
  phase (model / assist), the Dream round trip, points, clock and the "CALL STOP NOW" advice, all from the room.
- **Measured, not assumed.** Cloud hop adds 0.46 s to the control loop (`relay/README.md`), so the match plan is a
  local `livekit-server --dev` on the venue LAN for control and the cloud room for viewers.

Codec lesson: browsers could not decode Portal's H.264 tracks, so `relay/portal.yaml` publishes VP8, which
Chrome, Safari and the Python subscribers all decode.

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

## Sunday: real-arm results, station tooling, HF Jobs

- **Real arm:** `tools/station/` is the code that ran on the station (NUCK-4) during the event, copied back verbatim:
  the Dream runner with the grasp-and-pull assist (`dream_right_arm_run_stack.py`, the multi-cable `working_set_unplug_8of8/`
  copy that scored 8 of 8, `relay.sh` for unplug-then-stack in one command, the fine-tune cron monitor, keep-warm and
  demo-replay tools). `tools/station/RUN_DREAM.md` is the operator guide.
- **Results:** `results/README.md` has the tables (8/8 single plugs, 3/4 in one run, the judged run), the five Dream
  fine-tunes, the dimos sim scores, 1080p videos in `results/videos/`, scored sim sessions in `results/sim_sessions/`
  and every station run log in `results/station_runs/`.
- **HF Jobs:** `hfjobs/` runs the dimos simulation lab and the renders on Hugging Face Jobs machines (`launch.sh`,
  `remote.sh`, `restart_lab.sh`, `driver.py`, `make_video.sh`, `setup_actor.sh` + `actor_rpc.py` for Dimensional's actor).
- **Real-arm path for the sim programs:** `real/` (measured-scene module, real-arm stack, runbook), verified on a mock arm
  and in MuJoCo; not run on the hardware.
