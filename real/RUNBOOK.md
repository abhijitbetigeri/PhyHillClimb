# Real right Metal: tower_order_v24 and cable_v1 through dimos Autoresearch

Goal: run the sim-found programs on the station's RIGHT follower arm through `dimos.autoresearch.program.run_program`
and `dimos.autoresearch.live:backend`. You install, measure where the cubes are, and run. Hand on the e-stop for
every step that says MOVES. The Metal has **no brakes**: when the motors switch off, the arm drops.

Total at the arm: **about 75 to 95 min** the first time (install 20 to 30, checks 25, measuring 10, the first tower
5 to 8), then about 10 min for each further tower (re-measure 3, run 5 to 7).

| # | Step | Time | Moves the arm? |
|---|---|---|---|
| 0 | Copy the repo to the NUC (from the Mac) | 3 min | no |
| 1 | Stop the kit and the harness, preflight | 5 min | no |
| 2 | Install into ~/dimos-real | 15 to 25 min (uv download) | no |
| 3 | Optional: rehearse on the echo mock on the NUC | 5 min | no |
| 4 | Start the real stack (motors on, jaw calibration) | 2 min | jaw only |
| 5 | Home check: rest reading, jogs at speed 0.05, home | 10 min | yes, small |
| 6 | Pad offset check | 5 min | no (arm at home) |
| 7 | Measure the cubes, reload | 10 min | no |
| 8 | Hover test over each cube | 5 min | yes |
| 9 | One cube pick and put-back | 5 min | yes |
| 10 | Full tower_order_v24 | 5 to 8 min | yes |
| 11 | Optional: cable_v1 | 10 min | yes |
| 12 | Restore the station for the Dream runs | 5 min | rest pose |

All commands on the NUC run in `tmux new -A -s real` so an ssh drop does not kill the stack. Abbreviations:
`PY=~/dimos-real/.venv/bin/python`, `R=~/dimos-real/PhyHillClimb`, `SC="$PY -m real.scene_ctl"`.
Run `scene_ctl` and `run_program_real.py` from `$R` (`cd $R`).

## 0. Copy the repo (Mac, 3 min)

The `real/` files are not committed, so copy the working tree (no notes, runs, videos or keys):

```sh
rsync -a --exclude .git --exclude .venv --exclude notes --exclude runs --exclude videos --exclude '.env*' \
  --exclude __pycache__ ~/projects/PhyHillClimb/ pah@10.103.0.129:~/dimos-real-src/PhyHillClimb/
ssh -L 8095:localhost:8095 pah@10.103.0.129      # 8095 = dimos's viser planner view (optional)
tmux new -A -s real
```

## 1. Stop the kit and the harness (5 min)

Only one program may drive the arm. The kit, `dreamcli` and our Dream runner share `~/.dreamlabs/arms.lock`; the
dimos stack does not use the kit's driver, so `start_real_stack.sh` takes that lock itself.

```sh
tmux ls                                          # find collect / teleop / harness / codex sessions
pgrep -af "dhk-collect|dreamcli|dream_right_arm_run|Superfast|leader|lerobot|check_cameras"
```

In each session that runs one of these, stop the program inside it (Ctrl-C), never `tmux kill-server`. Write down
what you stopped, for step 12. Leave the Dream fine-tune monitor in cron alone (API only). Then:

```sh
SKIP_HW=1 bash ~/dimos-real-src/PhyHillClimb/real/preflight.sh   # before install: only the process/lock part
```

## 2. Install (15 to 25 min, first time)

```sh
bash ~/dimos-real-src/PhyHillClimb/real/install_station.sh --check   # says what it would do, changes nothing
bash ~/dimos-real-src/PhyHillClimb/real/install_station.sh           # idempotent; rerun after copying new files
bash ~/dimos-real/PhyHillClimb/real/preflight.sh                     # now with CAN and camera: must say PREFLIGHT OK
```

It clones dimos (`hackathon/feat/metal-autoresearch` at the commit we tested, 1cb67a6220) into `~/dimos-real`,
applies `real/dimos-local.patch` (SceneObject.yaw, which v24 reads), runs `uv sync --extra planning --extra web`,
copies this repo into `~/dimos-real/PhyHillClimb`, installs it editable and runs the unit tests. It touches nothing
else; anything it would replace goes to `<path>.bak.<timestamp>`. No internet? `DIMOS_SRC=<copy of dimos>` instead
of the clone. The first stack start downloads the Metal URDF from GitHub (makermods metal-python-ros, pinned) into
dimos's cache; if the NUC has no internet, start the stack once while it does.

Preflight FAIL lines: a process still holding the arm or the camera (stop it), `can_follower_r` not UP (the kit's
setup brings it up; do not reconfigure it without asking), camera missing (`ls /dev/v4l/by-path/`; then start with
`TESTCAM=1`, the keyframes are only for the record).

## 3. Optional: rehearse on the echo mock (5 min)

```sh
MOCK=1 bash $R/real/start_real_stack.sh          # terminal 1: no CAN, generated camera frames
cd $R && $PY real/run_program_real.py --program sim/candidates/tower_order_v24.py --yes   # terminal 2, ~3 min
$SC describe                                     # the belief: three cubes stacked at the build spot
~/dimos-real/.venv/bin/dimos stop
```

## 4. Start the real stack (2 min, the jaw moves)

Arm resting (the folded rest pose), jaw clear of everything, e-stop in hand.

```sh
bash $R/real/start_real_stack.sh                 # REAL_SPEED_SCALE=0.05 for slower; TESTCAM=1 without the webcam
```

Start-up enables the six arm motors (they hold where they are) and **calibrates the jaw by driving it to both
ends**. Wait for `Blueprint started`. Planned and straight moves run at 0.1 of the joint limits (dimos's own real
default is 0.2). Stopping the stack (Ctrl-C here) switches the motors off: put the arm at rest first (step 12).

## 5. Home check (10 min, small moves at speed 0.05)

dimos reads the Damiao motors directly (not the kit's driver and not its leader-follower offsets), so check that
its zero and signs match the URDF before anything else.

1. At rest: `$SC pose`. Expect joints about `(0, 0, -0.05, 0, 0, 0)` rad (joint3 rests on its stop at -0.0505), each
   within about 3 degrees, and gripper_base about `(0.118, 0.000, 0.173)` m. A joint off by tens of degrees or by
   pi means a zero mismatch: **stop here**, nothing below is safe.
2. `$SC jog 1 5 --go`: joint1 +5 degrees turns the arm counter-clockwise seen from above, the jaw moves about
   1 cm toward the arm's LEFT (+y; standing behind the arm, to your left). `$SC jog 1 -5 --go` back.
3. `$SC jog 6 10 --go` and `-10`: the jaw rolls about its own axis, nothing else moves.
4. `$SC home --go`: a planned move to the sim's home (`METAL_SIM_HOME`, jaw pointing down, mount 0.33 m up,
   0.28 m out). Watch the path; it lifts the folded arm up and forward. Then `$SC pose`: expect about
   `(0.281, 0.000, 0.330)` minus a few cm of sag in z (no gravity compensation; the sim sagged 3 to 4 cm).
5. At home: `$SC jog 2 5 --go` raises the mount about 2.4 cm and pulls it back 1.9 cm; `$SC jog 4 5 --go` moves it
   1.6 cm forward. Each the other way to undo. A move in the opposite direction = a sign flip: **stop**.

Expected moves come from forward kinematics of the station's URDF (notes/station-backup/.../urdf).

## 6. Pad offset (5 min)

The programs aim the finger pads 7.8 cm ahead of gripper_base (`pad = 0.078`, dimos's METAL_HEURISTIC_AIM). The
kit's station.yaml puts Dream's tool point (between the closed fingertips) at 0.135 m along link6, which is only
5.9 cm ahead of gripper_base (0.0758 m along link6). The two disagree by about 2 to 3 cm; measure:

At home (jaw down), `$SC pose` gives the mount z. Measure from the bench to the CENTRE of the finger pads (the last
2.7 cm of the fingers) with a ruler. `pad = mount z - pad centre height`. If it is within 5 mm of 0.078, change
nothing. Otherwise copy the program (`cp sim/candidates/tower_order_v24.py real/programs/tower_real.py`), set
`pad = <measured>` there, and restart the stack with `--measuredscenemodule.pad-m <measured>` (the scene's grasp
belief uses it too).

## 7. Measure the cubes (10 min)

Frame: the RIGHT arm's base frame, which is `world` on this stack. The URDF's `base_link` origin is on joint1's
vertical axis (the centre of the round base), at the bottom of the base, i.e. on the bench surface (joint1 sits
6.7 cm above it). x points the way the arm faces at joint1 = 0 (the folded jaw at rest points along +x), y to the
arm's left, z up. Metres. Positions are cube CENTRES; leave z out for a cube on the bench.

1. Put the cubes where the sim had them, all within reach: x 0.18 to 0.32, y -0.05 to 0.18, at least 8 cm apart.
   Keep **the build spot (0.20, 0.00) empty with 10 cm clear around it**, and the retreat path to (0.10, -0.12)
   clear (the tower is built at the build spot; the program ends there).
2. Tape from the base axis: x along the arm's forward line, y square to it. Square the cubes to the x axis
   (yaw 0) or note the yaw.
3. Edit `$R/real/scene.yaml` (`cube_yellow`, `cube_red`, `cube_blue`: `xyz: [x, y]`), then `$SC reload`. No restart.
4. Check: `$SC describe`.

## 8. Hover test (5 min, MOVES)

```sh
$PY real/run_program_real.py --program real/programs/hover_one.py --task "hover over cube_red"
```

One planned move puts the pads 10 cm above the cube centre, up to two re-aims against sag, a 5 s hold. Look from
the side and from the front: is the jaw centred over the cube? If it is off by more than 1 cm, the measurement or
the base frame is off: fix `scene.yaml`, `$SC reload`, repeat. The log (`program.log`, and `trace.jsonl` lines
`attempt N: ... off (...) cm`) says how far the arm itself landed from the command. Repeat for the other two cubes
(`--task "hover over cube_yellow"`; `"... at 0.05"` hovers 5 cm).

## 9. One pick (5 min, MOVES)

```sh
$PY real/run_program_real.py --program real/programs/pick_one.py --task "pick cube_red"
```

v24's grasp: pads at the cube centre, close, lift 10 cm, hold 3 s, set it back down 3 mm above its spot, open,
rise. Watch for:

- **Sag**: the logged `grasp: ... off (...) cm` after the re-aims should be under 0.5 cm. If the arm sinks more
  when holding the cube, the program's re-aims still cancel it at the set-down.
- **Pad height**: the fingers should close around the middle of the cube, not its top edge (else step 6).
- **Gripper timing**: `run_program_real.py` uses `real.live_real:backend`, dimos's live backend whose gripper calls
  return only once the jaw has stopped moving (0.3 to 3 s). The programs' own waits are counts of `arm.pose()`
  reads, which last only as long as the RPCs (about 5 ms each on this stack, 30 ms on the sim lab v24 was tuned on);
  without the jaw wait the arm rose while the jaw was still opening and tipped the placed cube in the sim. If the
  arm still starts moving before the jaw has finished, say so: the settle tolerance is `SETTLE_TOL` in
  `real/live_real.py`.
- **Release drag**: does the cube shift when the jaw opens? In the sim it lands 0.8 to 1.5 cm further along the
  tool's x (away from the base when yaw is 0) than the jaw mount; v24 compensates 0.9 cm (`drag = (0.009, -0.003)`)
  and the scene's belief assumes 1.2 cm (`release_offset`). Measure the real shift here: `$SC describe` shows where
  the belief put the cube (`released ... pads at (x, y)`), the tape shows where it is. If it differs by more than
  5 mm, set the scene's `--measuredscenemodule.release-offset '[dx,dy]'` (restart the stack) and, if the shift is
  large, v24's `drag` in a copy of the program.

After this the cube is roughly where it was; `$SC reload` puts the belief back to the file.

## 10. The tower (5 to 8 min, MOVES)

Cubes back at their measured spots (re-measure any that moved; `$SC reload`), build spot clear, then:

```sh
$PY real/run_program_real.py --program sim/candidates/tower_order_v24.py --verdict
```

Default task: `stack the yellow cube first, then the red cube on top, then the blue cube on top` (`--task` for
another order; the program reads the colours from it). Type `GO`. Ctrl-C stops the program **and cancels the
motion**; the e-stop is still the stop. On the echo mock it takes about 3 min at speed 0.1; on MuJoCo 5 to 8 min,
longer when it re-places a cube. At the end answer the y/n verdict. Everything is in `$R/real/runs/<timestamp>/`.

How the program sees the world here: the measured scene answers `scene.objects` from `scene.yaml`, then keeps a
belief from the arm's own encoders: a cube the jaw closes on rides with the tool; an opened jaw sets it on whatever
is below, 0.8 cm further along the tool's x (the release drag); a jaw that ends fully closed (within 4 mm of 0; on a
5 cm cube it stalls near -18 mm) held nothing, so the cube stays put and v24's "did it lift" check retries 5 mm
deeper, as it did against the simulator's truth. What it cannot see: a cube knocked over or pushed by the
fingers, a cube that slid off the one below after release. If that happens, stop (Ctrl-C), put the cubes back,
fix the file, `$SC reload`, start again. `$SC describe` lists every grasp and release with where the jaw was.
The stand-in obstacle map is the bench just below z = 0 (outside 12 cm around the base) plus every cube not in the
jaw; `scene.clear` cuts boxes out of it as in the sim.

What to watch and what to change (all in the program file or as stack flags):

| You see | Change |
|---|---|
| jaw closes on the cube's top edge or below the bench | `pad` (step 6) and `--measuredscenemodule.pad-m` |
| cube lands 1 to 2 cm off the one below, always the same way | `drag` in v24 and `--measuredscenemodule.release-offset` |
| arm rises before the jaw is open (cube lifted again) | `MIN_S`/`SETTLE_S` in `real/live_real.py` up (0.6 / 0.4) |
| `straight move failed` / `planned move refused` everywhere | positions too far out: move the cubes nearer (x 0.20 to 0.28) |
| planned moves swing wide near the tower | expected; v24 transits in straight lines, keep the build spot clear |
| too slow | `REAL_SPEED_SCALE=0.15` when you trust it (dimos default 0.2) |
| re-placing a seated cube over and over ("not seated") | the belief's place differs from the real one by over 1.5 cm: tune `release-offset`, or Ctrl-C once the tower stands |

## 11. Optional: cable_v1 (10 min, MOVES)

Edit `$R/real/scene_cable.yaml` (hub box and each plug HOUSING centre: housing top height minus 2 cm; yaw = the
strip's direction), `$SC reload $R/real/scene_cable.yaml`, hover test over `plug_a`
(`hover_one.py --task "hover over plug_a at 0.05"`), then
`$PY real/run_program_real.py --program sim/candidates/cable_v1.py --verdict` (task: unplug all the cables). The jaw
squeezes the housing across the strip, 5 mm above its centre, and lifts 8 cm. `$SC reload $R/real/scene.yaml` to
switch back.

## 12. Restore the station (5 min)

```sh
$SC rest --go                                    # MOVES: planned move to all joints 0 (folded rest)
```

Support the arm, then Ctrl-C the stack in its tmux window (the motors switch off; the arms lock is released).
`pgrep -af dimos` must be empty. Restart what you stopped in step 1, in the same tmux sessions. The Dream runs
(`dreamcli robot run`, `tools/dream_right_arm_run.py`) home the arms themselves at start. Nothing outside
`~/dimos-real` and `~/dimos-real-src` was changed; delete both to remove everything.

## Plan-only

`run_program_real.py --plan-only` (backend `real.plan_only:backend`) plans every `move_pose`, `move_joints` and
`home` from the arm's real current state and never executes; `move_linear` has no plan-only form in the Arm API (the
RPC plans and executes in one call), so it and the gripper are skipped. Nothing moves, so the program's feedback
sees the resting arm: this checks the wiring (state, scene, map clears, keyframes) and whether the first targets
plan from rest, not the sequence. Rehearse the sequence on the echo mock (step 3) instead.

## What was verified before the event (HF Jobs, x86_64, 2026-11-08)

| Check | Result |
|---|---|
| unit tests `real/tests` (scene, belief, map + box filter timing, blueprint wiring, CAN flag) | 13 passed |
| `install_station.sh` in a clean `python:3.12-bookworm` container: `--check`, install, re-install, preflight (no CAN/camera), echo-mock stack | install 77 s on HF's network (expect 10 to 20 min on the venue Wi-Fi), re-install 13 s, PREFLIGHT OK; needed `libgl1` (the installer says so; APT=1 installs it) |
| echo mock + pattern camera: hover_one, tower_order_v24, cable_v1, v24 `--plan-only` | all complete; v24 about 3 min at speed 0.1, cable_v1 67 s; plan-only planned 70 moves and the joints did not change |
| MuJoCo Metal in the real configuration (measured scene, no truth scene, no camera map, speed 0.1), v24 on the lab's dev seeds 0 to 4 | **1/5** (strict: order, 3 mm drift, jaw 16 cm clear). Before the jaw wait and the empty-jaw check: 0/2 |

Why the measured scene scores far below the sim lab (5/5 dev, 9/10 held-out against true poses): v24 recovers from
the sim's flaky grasps and placements by re-reading the scene (did the cube come up, is it seated, did it come up
with the jaw). A measured scene only knows what the encoders imply. The failures, from the true poses recorded
during each trial: a cube carried 1.8 cm higher in the jaw than at the grasp, released 2 cm above the cube below and
bouncing off; a placed top cube swept off on the retreat; a cube that slipped mid-carry. The belief catches a jaw
that closes on nothing; it cannot see a cube knocked or tipped. On the real arm, watch every release and stop and
re-measure when a cube is not where `$SC describe` says.
