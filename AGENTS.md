# PhyHillClimb: brief for every agent and teammate

Everything an agent needs to work on this project in parallel. Read it fully before touching code or the robot.
Event: MakerMods Physical AI Hackathon, Founders Inc, San Francisco, Sat 7 to Sun 8 November 2026. Demos and
judging open Sunday 15:00. Team repo: https://github.com/abhijitbetigeri/PhyHillClimb.

## 1. The competition: 1v1 Hill Climb Relay

Two matched stations run the same relay at the same time, 5-minute clock. After "go", **only the policy moves the
robot**: no teleop, no keyboard, nobody touches the robot or the objects (voice input is undecided). A designated
teammate may call "stop" at any moment. Stages run in order; touching a later stage's object before the current
pass bar is a disqualification (0 points, loss).

| Stage | Objects | Pass bar | Points |
|---|---|---|---|
| 1 Unplug | USB cables plugged into a hub, plus loose cables on the table (Ethernet "hard mode" optional) | at least 1 cable unplugged. Unplug only cables that start plugged; plug only loose cables into free ports; an unplugged cable may not go back in | pass +10, each further unplug +5, each loose cable plugged in +20 |
| 2 Stack | about 10 stackable shapes | 3 layers or 6 inches, free-standing 3 s after release. If it falls later the stage still counts but stack points are lost | pass +20, each further block +10 |
| 3 Hungry Hungry Robots | 12 ping-pong balls of each of 2 colours, one black 8-ball; colour by coin flip | none | own colour in bin +5, other colour -5, **8-ball in bin = you lose** |
| Time bonus | only if stages 1 and 2 are passed and a teammate calls stop | | +1 per 2 s left |

### Strategy (decided)

Time bonus is 0.5 points per second. Every extra action pays the same or less per second and carries risk:

| Action | Points | Realistic time | Rate |
|---|---|---|---|
| Time bonus | +1 per 2 s | | 0.5 pt/s |
| Extra block | +10 | ~20 s | 0.5 pt/s |
| Extra unplug | +5 | ~10 s | 0.5 pt/s |
| Plug in a loose cable | +20 | ~40 s, often fails | 0.5 pt/s at best |
| Own-colour ball | +5 | 15 to 20 s | 0.3 pt/s plus the 8-ball risk |

**Plan per match:** one clean unplug, exactly three layers, call stop. Example score 10 + 20 + 95 (190 s left) = 125.
Take extras only when near-certain and under 20 s. Never enter stage 3 unless stage 2 is impossible.

### What the stages really are

- **Stage 1 is a pull.** Grasp the plug housing, lift straight out of the hub. Plug-in is a stretch goal only.
- **Stage 2 objects, confirmed from the station videos (`videos/`, Sat 14:00):** three foam cubes about 5 cm with a
  square through-hole each (red, blue, yellow) and two loose foam pegs (orange about 10 cm, red about 6 cm). The videos
  show the Metal arm picking the red cube and setting it on the blue one. Three cubes stacked = 3 layers = pass; a peg
  standing in a cube's hole adds height but is a harder grasp. `sim/stack_bench.xml` mirrors this set.
- **Earlier reading of the toy (superseded):** a base cube with an orange peg, a blue cube with a through-hole, a red
  peg, a yellow cube with a through-hole. Assembled it is 3 layers and over 6 inches, and the pegs self-centre. The
  approach is always vertical: hover over the peg, descend, release, retreat.
- **Stage 3 only if forced.** A black-ball veto on every grasp is mandatory if it is ever attempted.

## 2. Architecture: the Relay Director

The deliverable is an autonomous relay runner, not a single policy.

```
Relay Director (our harness)
  stage sequencer: unplug -> stack -> (stop)
  per stage: a Skill = {planner program | Dream-1 prompt ladder}, a pass-bar verifier, recovery rules
  DQ guard: halts the policy if the gripper nears a later stage's objects
  clock + live score projection + "CALL STOP NOW" advice on a dashboard
  JSONL trace of every call, verdict and keyframe
Skills
  stack: planned, perception-driven program (dimos Arm API: scene.objects, move_pose, move_linear, gripper)
  unplug: Dream-1 (base model first; fine-tune on 50 demos tonight)
Verifiers
  overhead camera + VLM question per pass bar, or planner scene state; stall detector on joints
```

### Workstreams (independent, pick one, say which in the channel)

| # | Workstream | Where | Done when |
|---|---|---|---|
| A | Demos and fine-tune | station, Dream kit | 50 unplug + 50 stack episodes recorded, `--check` passes, job queued by Sat 21:00 |
| B | Stacking program in sim | `sim/lab_stack.py`: `uv pip install --python ~/projects/dimos/.venv/bin/python -e ~/projects/PhyHillClimb` once, then `cd ~/projects/dimos && .venv/bin/dimos --simulation run phyhillclimb.autoresearch-stack --mujocosimmodule.headless true`; drive it with `dimos mcp call task/scene/submit/run/results` | a program that assembles 3 layers on the MuJoCo bench ≥ 80% over 10 seeded trials |
| C | Dream-1 control loop + prompt ladder | `harness/dream_harness.py` + station | zero-shot unplug trial scores logged; loop passes `dream.check_loop`; runs on the station with a dry mode |
| D | Relay Director | `harness/` | sequencer, verifier, DQ guard, clock and dashboard run against recorded frames and the sim |
| E | Eval and rehearsal | station | a rubric scorer, 5-trial runs per stage with a results table, match rehearsal with a stop caller |

Rules for all workstreams: a dry mode for anything that moves an arm, every run writes JSONL, numbers go in a
table, never in prose. No commits of keys, passwords or personal notes. `notes/`, `runs/`, `.venv/` are gitignored.

## 3. The station (NUCK-4)

- **Metal Arm station** (two MakerMods Metal arms, Damiao motors, no brakes), three cameras (overhead, left wrist,
  right wrist), a Meta Quest for teleop. The dashboard title confirms "metal station". Use `--arm metal` everywhere.
- Host: NUC at `10.103.0.129` on the "Founders Inc" Wi-Fi (not the Guest network, which is a different subnet).
  Account `pah`, password from the organizers. No screen: everything runs over SSH.
- Connect from a laptop (the port forward is for the dashboard):
  `ssh -L 8080:localhost:8080 pah@10.103.0.129`, then on the station `tmux new -A -s collect`, then
  `cd ~/physical-ai-hacks`.
- One program drives the arms at a time (a flock on `~/.dreamlabs/arms.lock`, shared by the kit and `dreamcli`).
  Close `dhk-collect` before `dreamcli` or our harness. The refusal line names who holds it.
- Cameras are shared devices: a camera preview left running (we found one from a Codex session in tmux
  `codex-pah-cameras`) blocks the collector with OpenCV "can't be used to capture by name" warnings. Check with
  `pgrep -af "check_cameras|dhk-collect"` and `tmux ls`; kill only the preview, never the tmux server.
- Metal rest pose is upright with the gripper closed. The arms return to the pose they had when A opened them, so
  start every session with them resting. Hand on the e-stop for every real run.

## 4. The Dream hackathon kit (official, on the station at `~/physical-ai-hacks`)

Repo: https://github.com/dreamlabs-co/physical-ai-hack-2026. Docs: `docs/data_collection.md`,
`docs/fine_tuning.md`, `docs/running_your_model.md`. Its `CLAUDE.md` safety rules apply to every agent here.

- **Collect:** `uv run dhk-collect --config configs/station.yaml --dry-run` first (arms never move), then
  `uv run dhk-collect --config configs/station.yaml --dataset-dir ~/hackathon_data/<team>-<task> --task "..."`.
  Dashboard at http://localhost:8080 while it runs. Quest: grip held drives that arm (clutched, like a mouse; poses are
  relative to the headset, keep your head still), trigger closes the gripper, A opens the arms the first time, then A
  starts and saves an episode, B discards, right-stick click homes.
- **Dataset:** LeRobot v2.1, 30 fps, videos 640x360 (16:9 crop), state and action 14 wide:
  `[left joints x6 rad, left gripper mm, right joints x6, right gripper mm]`, action = commanded IK target. Episodes
  must be at least 1.6 s (49 frames). A camera with no frame for 0.5 s during recording discards the episode. An arm
  more than 0.35 rad behind its command is re-synced and the clutch drops: push cables gently.
- **Before fine-tuning (Metal):** `~/dream-env/bin/python tools/add_eef.py ~/hackathon_data/<ds> --arm metal` writes
  `<ds>-eef` with tool poses from Dream's own kinematics; `--replace` after more episodes.
- **Check (free, local):** `~/dream-env/bin/python tools/finetune.py ~/hackathon_data/<ds>-eef --arm metal --check`.
- **Fine-tune (paid):** `~/dream-env/bin/python tools/finetune.py ~/hackathon_data/<ds>-eef --arm metal --name <n> --suffix <s>`
  inside `tmux new -A -s finetune`. **$200 for the default 1000 steps, held from team credits when queued, cannot be
  cancelled; get the organizers' OK first.** 7 to 8 h plus queue; a new model's first start takes up to 1 h.
  `tools/finetune.py` has no steps flag; a 500-step job ($100) is a direct `dream.DreamClient().finetune(name, suffix,
  steps=500, use_eef=True)` after the upload (resume from the printed `ds_` id). One job can learn several tasks: put
  each `-eef` folder under one parent and upload the parent.
- **Run:** `source ~/dream-env/bin/activate; dreamcli robot run --task "..." --dry-run`, then without `--dry-run`.
  `--checkpoint dream-1-ft:<acct>-<suffix>` for a fine-tune. **`dreamcli robot run` homes both arms at the start of
  every run**, so a stage-by-stage harness must own the control loop instead of restarting dreamcli per stage.
- **Embodiment for Metal fine-tunes:** `src/dream_hackathon/embodiments/makermods.json` (state: left_arm,
  left_gripper, right_arm, right_gripper, then right and left eef_xyz, eef_rot6, grip; action eef-first; video roles
  `scene`, `wrist_l`, `wrist_r` mapped to `observation.images.scene/left_wrist/right_wrist`), registered as
  `hackathon-metal`.

## 5. The Dream API (client `dreamlabs-client` 1.1.3, verified against the installed package)

- Credentials in `~/.dreamlabs/credentials` (`DREAM_GATEWAY_URL`, `DREAM_API_URL`, `DREAM_DATA_URL`,
  `DREAM_API_KEY`). Never print or commit a key.
- Loop: `client.policy({"checkpoint": ..., "embodiment": makermods.json, "ik": {"arm": "metal", "head": "eef"},
  "action_hz": 20})`; `client.set_observation(frames, observation, timestamp=t)` on every camera tick with
  `frames = {"video.<role>": base64 JPEG 320x176 q85}` and `observation = {"state.<part>": array, "prompt": str}`;
  `client.get_action()` on its own thread returns a 48-row chunk (rows 1/30 s apart, played at `action_hz`);
  `client.refill_due(executed, remaining)` says when to ask again; `client.reset_episode()` between attempts.
- Rows are indexed by time since the request went out (`due = (now - asked_at) * action_hz`), skipping the rows the
  round trip used. Rate limit: 3 calls then one per 0.7 s. Cold model: `DA-2000 pool_warming`, retry for up to an
  hour. Typical round trip 250 to 450 ms.
- Offline loop check with no arm and no model: `python -m dream.check_loop --rate 20` in one terminal, the loop with
  `DREAM_GATEWAY_URL=http://127.0.0.1:8642` in another. Our skeleton passes all its checks (`harness/README.md`).
- Dream-1 outputs gripper poses; the client's IK converts them. Infeasible poses near joint limits happen. Research
  beta, no safety validation.

## 6. Dimensional dimos (framework for the planner-based skill and the Autoresearch loop)

Repo: https://github.com/dimensionalOS/dimos, branch `hackathon/feat/metal-autoresearch` (local clone
`~/projects/dimos`, branch `metal-autoresearch`). Docs: `docs/capabilities/manipulation/makermods_metal_integration.md`,
`dimos/autoresearch/README.md`, `AGENTS.md`.

- Sim: `uv run --no-group tests --extra planning dimos --simulation run makermods-metal-planner-coordinator`, UI at
  http://127.0.0.1:8095 (the `tests` group pulls pyaudio, which fails to build on macOS without portaudio).
  Metal on a bench in MuJoCo: `dimos --simulation run metal-sim` (cube, cylinder, bowl; wrist `color_image`; fixed
  `scene_image`; true poses on `body_poses`), `metal-quest-collection-sim`, `autoresearch-metal-sim`.
- **Autoresearch:** a coding agent writes `run(robot, task)` against a fixed API (`robot.arm.move_pose(xyz, rpy)`,
  `move_linear(dx, dy, dz)`, `move_joints`, `home`, `robot.gripper.open/close/set`, `robot.scene.objects(words)`,
  `scene.clear(id)`, `observe()`, `log()`; only `import math`). The lab runs candidates on seeded trials (same scenes
  for every candidate), traces every call with before and after keyframes, judges from true poses (`LiftJudge` 5 cm,
  `PlaceJudge` inside a box, `PlaceAtJudge` within 5 cm of another body's old spot, `OperatorJudge` asks a person),
  keeps a leaderboard, records successful episodes, scores the frozen candidate on held-out seeds. Lab skills over
  MCP on :9990: `task, scene, submit, run, stop_run, results, leaderboard, freeze, note`. Sessions under
  `~/.local/state/dimos/autoresearch/<name>/`.
- **Program-writing gotcha (cost us v1–v3):** `arm.pose()`/`move_pose()` address `gripper_base`, the jaw's mount.
  The finger pads are ~7.8 cm further along the approach (`METAL_HEURISTIC_AIM` in `makermods_metal/config.py`), so
  a grasp at a cube's centre height needs the mount at `z_cube + 0.078`, and a release "1 cm above the seat" needs
  `z_seat + 0.078 + 0.01`. Object positions read while the cube is in the jaw are biased ~1 cm (occlusion); do not
  correct placement from them. Reference program: `sim/candidates/stack_v4.py`.
- Real hardware: `dimos run makermods-metal-planner-coordinator --can-port <port>`, Quest teleop
  `teleop-webxr-metal`, rollout `metal-lerobot-rollout`. Jaw joint `arm/joint7` in metres: 0 closed, -0.05 open.
- **There is no Dream code in dimos.** The integration (a `robot.policy.dream(prompt, seconds)` primitive, or a
  Dream policy module beside `LeRobotPolicyModule`) is ours to build if we want the Autoresearch actor to mix
  planned moves with Dream-1.
- Data mismatch: dimos records one wrist camera 480x640 and 7 joints with the jaw in metres; the Dream kit records
  3 cameras 640x360 with the gripper in mm. Dream fine-tuning data comes only from the kit's collector.

## 6b. LiveKit integration (workstream F): the room is the hardware abstraction

Goal: the Relay Director talks to one interface whether the arms are simulated or real. That interface is a
LiveKit room using **LiveKit Portal** (`pip install livekit-portal`, the package behind
https://github.com/livekit-examples/portal-makermods), which carries three camera tracks, a 14-value state, actions,
and an "active operator" gate. Everything else (dashboard, recorder, voice) subscribes to the same room.

Roles in the room (`portal.yaml` from the Portal example is the schema: `overhead_camera`, `left_wrist_camera`,
`right_wrist_camera` H.264; state and action = `left_joint1..6.pos`, `left_gripper.pos`, `right_...`, radians and mm):

| Role | Real station | Simulation | Code |
|---|---|---|---|
| Robot (publishes state + 3 videos, applies the active operator's actions) | Portal's `src/robot.py` on the NUC over the Metal SDK | a bridge module in dimos: subscribes to the sim's `color_image` + `scene_image` + `coordinator_joint_state`, publishes them; writes received actions to `joint_command` | `livekit.portal.Robot`: `send_state`, `send_video_frame`, `on_action`, `set_active_operator` |
| Operator (the policy) | the Relay Director: Dream-1 loop + planner stages | same Director, unchanged | `livekit.portal.Operator`: `on_observation`, `get_video_frame`, `send_action`, `perform_rpc` |
| Observer | recorder, team dashboard, verifier | same | Portal observer / `lk` CLI |
| Voice agent (LiveKit Agents) | announces stage passes, score, "CALL STOP NOW"; answers "status?" | same | `livekit-agents`, LiveKit Inference models, no extra keys |

Voice is **output to humans** unless the organizers allow voice input; a spoken announcement is not human input to
the policy. If voice input is allowed, the agent gains `start_relay` and `stop` tools and nothing else.

Setup (one person, once):
1. Sign up at https://cloud.livekit.io, redeem the hackathon credits at https://cloud.livekit.io/projects/p_/redeem
   with the code from https://www.livekit.info/pah.
2. On the Mac: `brew install livekit-cli` (done), `lk cloud auth` (opens a browser), then in the project
   `lk app env -w` writes `LIVEKIT_URL`, `LIVEKIT_API_KEY`, `LIVEKIT_API_SECRET` into `.env.local` (gitignored).
3. Tokens for robot, operator and observer come from `lk token create --join --room relay-nuck4 --identity <role>`
   or from the API secret in code (Portal's `mint_token` helper in the example).
4. Voice agent: `lk app create --template agent-starter-python voice` then edit `src/agent.py`; `uv run src/agent.py
   console` to talk to it locally, `dev` to join rooms.

Status 2026-11-07 afternoon: `relay/` has the robot (fake backend), the Director operator and the local-server loop passing end to end (see `relay/README.md`).

Order of work: (1) Director as a Portal Operator against the sim bridge; (2) the same Director against Portal's
robot process on the NUC in dry mode (`--dry-run` prints targets, no motion); (3) voice announcer; (4) voice tools
only if allowed. Dream fine-tuning data still comes from the kit's collector, not from the room.

## 7. Timeline (Saturday into Sunday)

| When | What |
|---|---|
| Sat afternoon | zero-shot baseline on unplug and stack, 5 trials each, scored on the rubric; unplug demos; stack demos |
| Sat 19:00 | insurance 500-step job on whatever demos exist (needs organizer OK) |
| Sat 21:00 at the latest | both demo sets validated, `-eef` built, main job queued |
| Sat night | stacking program in sim; Relay Director; Dream loop on the station in dry mode |
| Sun 06:00 to 08:00 | warm the fine-tuned checkpoints (first `get_action` until it answers); keep them warm |
| Sun 08:00 to 13:00 | 5-trial runs per stage, ablation table (base vs fine-tune, with vs without verifier), pick the match configuration |
| Sun 13:00 to 15:00 | freeze, two full match rehearsals with a stop caller, rewarm at 14:00 |

## 8. Open questions for the organizers

- Ethernet hard mode on or off? Which hub, which port type, are plug-ins judged by the plug seating or by a link light?
- Does a software watchdog that halts our own policy count as human input? (No human touches anything.)
- Is voice input allowed? If yes, LiveKit credits (code on the getting-started page) are available.
- Fine-tune budget per team, and whether two jobs may run in parallel.
- Whether the match stations are identical to NUCK-4 (same cameras, same layout, same hub).

## 9. Safety rules (from the kit's CLAUDE.md, binding for agents)

- Never run anything that powers, moves or releases the arms. A person runs those with a hand on the e-stop:
  `dhk-collect` without `--dry-run`, `dreamcli robot run` or `up` without `--dry-run`, `dreamcli setup`, calibration.
- Ask before running anything that opens the arms even read-only.
- Never guess the tool centre point or gripper degrees; copy them from `~/.dreamlabs/station.yaml`.
- Never print, copy or commit an API key. Confirm before a paid fine-tune or deleting episodes.
- Disclose pre-event code: the harness skeleton in `harness/` was written before the event.
