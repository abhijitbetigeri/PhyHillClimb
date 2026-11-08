# Stacking simulation

The bench contains three hollow 5 cm cubes and two optional pegs. This is simulation work only;
these candidates are not calibrated or approved for the real station.

Two lab entry points are available:

| Blueprint | Task | Judge |
|---|---|---|
| `phyhillclimb.autoresearch-stack` | red on blue | original two-cube placement check |
| `phyhillclimb.autoresearch-three-cube` | ordered tower, optional stick (`TOWER_ORDER`) | validated geometry, gripper clearance and fresh-pose stability dwell |

The standalone geometry judges check truth poses only. The three-cube lab now wraps its ordered
judge with `StableTowerJudge`: it requires conservative gripper clearance and fresh snapshots
spanning at least 3 wall-clock seconds, with at most 3 mm translation and 5 degrees rotation from
the first sample. Observation gaps over 1 second fail verification. Samples and verdicts are saved
in the session's `verification.jsonl`. A stalled pose stream raises through the trial infrastructure.

This is an observed simulation dwell, not 3 seconds measured on the MuJoCo physics clock. The
clearance sphere rules out gripper support but does not inspect contacts with other arm links.
Artificial floor plates, exact object poses, limited layout randomization and uncalibrated foam
physics still limit hardware transfer. Historical geometry-only pass rates below must not be
reported as passes under this stronger verifier. Restart the lab to load the new judge; existing
sessions and their results are preserved.

Install/update the entry points into the existing DimOS environment:

```sh
uv pip install --python ~/projects/dimos/.venv/bin/python -e ~/projects/PhyHillClimb
cd ~/projects/dimos
.venv/bin/dimos --simulation --n-workers 8 run phyhillclimb.autoresearch-three-cube --mujocosimmodule.headless true
# --n-workers 8 gives MuJoCo its own process; without it the planner starves physics and trials fail with stale joint state
```

Run only one lab at a time (MCP uses the same port). Finish any existing trial loop before switching labs.
Use `dimos mcp call task`, `scene`, `submit`, `run`, and `results` as described in DimOS's Autoresearch README.
Trials write JSONL traces and before/after keyframes beneath the lab's recording directory.

`candidates/stack_verified.py` builds blue/red/yellow, checks each lift and placement from the scene,
and measures the tool-to-held-cube offset before carrying. It uses the **simulator's** documented
`METAL_HEURISTIC_AIM` pad offset (0.078 m ahead of the planning tip, `gripper_base`). The original
candidate instead descends the planning tip to the cube centre. The new candidate remains experimental.
Scene results are sampled asynchronously; a single lift/placement check is not a stability verifier.

Run the geometry regression checks without starting a robot:

```sh
cd ~/projects/PhyHillClimb
~/projects/dimos/.venv/bin/python -m unittest discover -s sim/tests -v
```

## Local evaluation, 2026-11-07

Existing two-cube lab, identical development seeds for both candidates. These scores do not certify a three-cube stack.

| Candidate | Red-on-blue passes | Trials | Mean program time (s) |
|---|---:|---:|---:|
| Original `stack_v1.py` | 1 | 5 | 17.41 |
| Experimental `stack_verified.py` | 0 | 5 | 9.08 |

The experimental candidate aborted on every failed lift; it did not reach the third-cube step.
The offset hypothesis did not improve performance. Keep it experimental; the original candidate remains the baseline.

Trace directory: `/Users/abhijitbetigeri/projects/dimos/recordings/autoresearch_stack_20261107_141555`. Each candidate contains per-trial `trace.jsonl` and `keyframes/`.
The running lab was left idle after both trial batches completed. Restart into the new blueprint only when ready for a new session.

## Bench change at v35 (Sun 8 Nov, 05:10)

The red stick moved from (0.27, −0.12) to (0.20, −0.12) in `sim/stack_bench.xml`. At the old spot, the arm's reach edge, the wrist pitched to 45° during straight moves and dropped the lifted stick (v34 frames). The tower results below were measured with the stick at the old spot; the tower program never touches the sticks, so they stand. Stick candidates from v35 on run on the new bench.

## Results (Sun 8 Nov, 03:00)

| Program | Task | Judge | Dev | Held-out | Time per pass |
|---|---|---|---|---|---|
| `tower_order_v24.py` | ordered three-cube tower (TOWER_ORDER, e.g. yellow, red, blue) | strict: order geometry, jaw 16 cm clear, 3 s observed stability | **5/5** | **9/10** | 90–97 s |
| `stack_tower_v5.py` | three-cube tower, fixed order | geometry only | 5/5 | 9/10 | 45–51 s |
| `stack_v7.py` | red on blue | geometry only | 5/5 | 6/10 | ~20 s |
| `tower_order_v37.py`–`v42.py` with the stick | tower + red stick in the top cube | strict | 0/5 judged; grasp solved (stick lifted 5/5 from v37), carried over the hole 2–3/5, **stood in the blue cube on the bench 1/5 (v41, v42)**, lost when that cube is lifted for the tower | | |

How to reproduce the tower result on a Mac (one lab at a time, 4 workers):

```sh
uv pip install --python ~/projects/dimos/.venv/bin/python -e ~/projects/PhyHillClimb
cd ~/projects/dimos && TOWER_ORDER="yellow,red,blue" .venv/bin/dimos --simulation --n-workers 4 run \
  phyhillclimb.autoresearch-three-cube --mujocosimmodule.headless true --mujocosimmodule.width 480 --mujocosimmodule.height 360 --mujocosimmodule.scene-camera-fps 5
cd ~/projects/PhyHillClimb && .venv/bin/python hfjobs/driver.py --dimos ~/projects/dimos/.venv/bin/dimos --candidates tower_order_v24.py --heldout
```

The strict judge (`sim/judges.py`, `StableTowerJudge` around `OrderedTowerJudge`) watches the tower for 3 s after the
program ends and requires the jaw more than 16 cm from every cube; the program ends with a 20 cm retreat for that.
The video of a passing session is `runs/videos/tower_v24_oblique.mp4` (scene camera, 8 fps; the recording session itself scored 4/5), with a 100 s first-pass clip `tower_v24_oblique_first_pass.mp4`. The stick remains open, but the failure is now understood and the grasp is solved (Sun 07:00, Mac lab stopped):

- **Grasp (solved, v37):** the pads, not the mount, are aimed at the stick, using the pose quaternion (finger axis = mount local +z, verified by offline forward kinematics of logged joints); a vertical descent is measured and repeated once. Result: stick lifted in 5 of 5 trials in every run from v37 on.
- **Carry (open):** a cylinder between flat pads is one contact point per pad in MuJoCo, so the stick swings freely about the pinch axis. Straight moves drift the jaw's pitch by about 6° per 10 cm; planned moves swing the wrist; either lets a centre-held stick pivot out (v37–v39). Pinching above the centre so it hangs (v40) topples the standing stick at the close instead. Levelling the jaw by folding the planner's measured orientation error into the commanded quaternion works (5° → 1°, v38) but does not survive the next straight move.
- **Best stick runs:** v41 and v42 each stood the stick in the blue cube on the bench in 1 of 5 trials (1.1–1.2 cm off the hole's axis; clip `runs/videos/stick_v41_first_insertion_dev00.mp4`, still `stick_v41_inserted.jpg`). v41 then could not retreat from the reach edge at (0.30, 0); v42 retreats with a planned move but the stick falls out when the cube is lifted for the tower. v38/v41/v42 carried the stick over the hole in 1–3 of 5 trials; the carry is the bottleneck (stick lost in 3–4 of 5).
- **Next step to try:** MuJoCo's `multiccd` flag (several contact points per convex pair) or a square-section stick, either of which gives the pinch a real contact patch; then re-run v38.
- The stick candidate table and the chain of evidence are in the candidate table below; the scene frames that settled each step are listed in `notes/PREP.md` §15.

## Results so far (Sat 7 Nov, 5 seeded dev trials per candidate, lab `autoresearch-stack`, judge: red on blue within 2 cm)

| Candidate | Change | Dev | What the traces showed |
|---|---|---|---|
| stack_v1 | top-down grasp at the cube centre, press to seat | 0/5 | cube slid off after release |
| stack_v2 | release 0.5 cm above the seat | 0/5 | still 2.7 cm off |
| stack_v3 | yaw-aligned grasp, in-hand offset correction | 0/5 | grasp slips on corners, placement 5–6 cm off |
| stack_v4 | aim the finger pads (7.8 cm ahead of `gripper_base`), no in-hand correction | 0/5 | grasp and lift work when the arm reaches the cube; arm settles 3 cm short near the base |
| stack_v5 | closed loop with `move_linear` on `arm.pose()`, straight carry | 0/5 | sag is not cancellable by relative moves; straight carry hits a wrist singularity |
| stack_v6 | re-aim with absolute `move_pose` + error, planned carry at grasp yaw, 20 g cubes | 4/5 | 1.4 cm residual: re-aim overshoot + in-hand offset |
| stack_v7 | 0.7 re-aim gain, release at base minus the in-hand offset measured at closing | **5/5** (held-out 6/10) | 0.5–1.8 cm off centre |

Three-cube tower (lab `autoresearch-three-cube`, judge: consecutive cubes within 1.5 cm, 5 ± 0.8 cm apart, tilt ≤ 15°):

| Candidate | Change | Dev | What the traces showed |
|---|---|---|---|
| tower v1 | stack_v7 twice | 0/5 | the arm swept the finished level off on the way to the next cube: cleared objects stay out of the obstacle map |
| tower v2 | set-down 4 mm above the seat, verify and retry | 0/5 | the first descent lands 3 cm low (sag) and presses the stack; retries hit IK/collision failures |
| tower v3 | travel at 0.30 m, straight descents, sag fed forward | 0/2 | level 1 still 1.3–2.4 cm off; the Mac sim stalled |
| tower v4 | release-drag compensation, 35 % jaw opening, judge-matched seat check | 0/5 | level 1 seated 4/4 at 0.8–1.4 cm; no IK solution at 0.30 m over the yellow cube |
| tower v5 | travel at 0.25 m with fallbacks (run on HF Jobs) | **5/5, held-out 9/10** (45–51 s per pass) | full tower geometry passes; the one held-out miss was the carry refused because the lifted cube left its cleared map zone |
| tower v6 | re-clear the held cube after the lift | not scored (lab switched to the ordered task) | |

Ordered task (`TOWER_ORDER="yellow,red,blue,red stick"`, judge `OrderedTowerJudge`: order, 1.5 cm, stick in the top cube within 1.2 cm):

| Candidate | Change | Dev | What the traces showed |
|---|---|---|---|
| tower_order_v1 | task text parsed for the order; stick as a fourth level | 0/5 | two levels in 4 of 5, then planner failures at the far corner where the yellow cube starts |
| tower_order_v2 | base cube carried to a build spot 26 cm in front of the arm | 0/5 | no IK solution rising to 0.25 m at the far corner |
| tower_order_v3 | rise steps down to a reachable height | 0/5 | set-down over bare bench refused: the carried cube is an obstacle again outside its cleared zone |
| tower_order_v4 | carried object re-cleared over its destination; bench objects moved within reach | 0/5 | towers go up (0.6–1.1 cm) but the 120 s program timeout kills every trial; stick knocked over |
| tower_order_v5 | stick grasped higher, tighter set-down, lab timeout 300 s | 0/5 | three cubes stack (blue 0.3–1.5 cm off red); the jaw flicks the stick over when closing; 113–246 s |
| tower_order_v6 | jaw closes on the stick in four steps; fallen stick skipped | 0/5 | stick still knocked on the descent (first planned move 2–3 cm short in x, re-aims sweep the open jaw at stick height) or dropped during the planned carry |
| tower_order_v7 | re-aim 4 cm above the grasp, one straight final descent; stick hang in the set-down height | 0/5 | the final 4 cm straight descent drifts 1.5–2 cm in x; the jaw missed every cube |
| tower_order_v8 | stick gripped at its centre, straight-line carry at fixed orientation, given up when fallen | not run | superseded by v9 |
| tower_order_v9 | v6's grasp with v8's stick handling | 0/5 | **tower complete in 4 of 5 trials, then the planned move to the stick swept the jaw through it**: cleared objects are invisible to the planner (dev-04 keyframes 170 to 172) |
| tower_order_v10 | straight-line transits only; clear height = tower top + 16 cm (cap 33 cm) | 0/5 | the tower now survives every transit (built in 4/5); the stick is released 0.9 cm off the hole axis and 1.4 cm high, catches the rim and falls; one trial started inside an object's map volume |
| tower_order_v11 | stick insertion measured from the scene (slide by the offset, lower onto the floor plate, stepped release); straight rise at trial start | 0/5 | the stick spun out of the flat pads during the straight slide and the unguarded offset-slide dragged the empty jaw through the tower; trials 1 to 4 had their first planned move refused for collision with the map's ghost of the previous tower |
| tower_order_v12 | slide guards (stick must still be in the jaw, offset under 2 cm), 3 s start dwell so the map sheds the old tower, straight fallback when a planned re-aim is refused, stick gripped 1 cm above centre; sticks given torsional and rolling friction | 0/5 | **three-cube tower built and left intact in 4 of 5 trials** (stage 'stick' in the judge); the higher stick grasp regressed: flicked over in 3 trials, dropped in 1; 113–142 s |
| tower_order_v13 | stick gripped at its centre again (v10's grasp lifted it 5/5) | 0/5 | tower built in 4/5 again; the stick now drops right after the lift or during the carry: the friction added in v12 made the pad-cylinder contact unstable |
| tower_order_v14 | v13 on the bench's original stick friction | 0/5 | tower 4/5; the stick is carried to the top cube every time and rolls out of the pads during the planned re-aims there (seen 8–19 cm away by the guard) |
| tower_order_v15 | straight corrections only while the stick is in the jaw; safe give-up; patient map reads | 0/5 | stick kept in the jaw to the top cube, but straight corrections cannot close a 3 cm x / 4 cm z error near the top of the reach |
| tower_order_v16 | release pose over the top cube learned empty-handed (planned re-aims), then the stick carried in straight moves to the learned command; map clears best-effort | 0/5 | **first stick in the hole** (dev-01: 0.6 cm off axis) but leaning; the learned z offset applied to a straight descent drove the stick 4 cm too deep in the other trials |
| tower_order_v17 | straight descents from the measured pose (no z feed-forward); final descent to the floor plate before release | 0/5 | tower in order 4/5 (0.3–1.5 cm); the stick reaches the hole command within 1–2 cm and still leaves the jaw on the way down |
| tower_order_v17 (no stick) | same program, TOWER_ORDER=yellow,red,blue: the three-cube ordered tower as the stage-2 pass bar | 0/5, 0/10 | the lab's judge now also watches the tower for 3 s and wants the jaw more than 16 cm from every cube; the towers themselves stood in 4/5 dev and 6/10 held-out, every one refused as 'gripper not clear' because the program parked the jaw right above the stack |
| tower_order_v18 | stick inserted into the last cube while it is on the bench (13–17 cm height, where the jaw lands within millimetres), then that cube stacked with the stick in it | superseded | v19 adds the retreat |
| tower_order_v19 | v18 plus a 20 cm retreat toward the base at the end, outside the judge's clearance radius | 0/5 | at bench level the straight descent lands ~1 cm low; the stick's bottom touches the cube top and the alignment slides push it out of the jaw |
| tower_order_v20 | stick stopped 2.5 cm above the hole, aligned by its measured offset, then dropped in (no contact while held) | 0/5 | the stick slides down in the pads during every horizontal move and leaves the jaw before the hole (bench level too); one trial reached the hole 1.8 cm off |
| tower_order_v20 (no stick) | same program, TOWER_ORDER=yellow,red,blue, with the 20 cm retreat for the stricter judge | 1/4 (stopped) | first pass of the strict judge (geometry, jaw clear, 3 s stable; 102 s); the other trials lost the red cube to a carry that sank 3 cm while sliding |
| tower_order_v21 | every straight move carries a running sag estimate (a Cartesian move ends one sag lower than commanded, 3–4 cm) | 0/2 (stopped) | a name collision made every straight move fall back to a planned one after the first set-down |
| tower_order_v22 | v21 with the sag estimate renamed | 1/3 (stopped) | one strict-judge pass (97 s); the blue cube's start 32 cm out cannot rise above 0.21 m, so its carry grazed the red cube off the tower; the build spot at 0.26 m was 4 cm from the blue's start |
| tower_order_v23 | build spot at 0.20 m (10 cm from every object); when a rise is refused, slide to 9 cm before the target, rise there, finish the slide | **4/4 dev under the strict judge** (98–126 s) when HF Jobs cancelled every job (credit exhausted, 01:20); on the Mac (4 workers, 480x360 cameras) the same program scored 2/5 dev, the red cube displaced mid-trial in the failures: the retreat lifted the just-placed cube again because the jaw had not finished opening (the Mac sim runs below real time) |
| tower_order_v24 | full jaw opening, 30-read wait, 2 cm verified rise before the retreat | **5/5 dev, 9/10 held-out under the strict judge** (Mac, 90–97 s per pass) | geometry, jaw clear and 3 s stability verified; the one miss seated the red cube 1.2 cm high (tilted on the rim) | geometry, jaw clear and 3 s stability all verified |


Facts every candidate must respect (all measured, see `candidates/stack_v7.py`):

- `arm.pose()` and `move_pose()` address `gripper_base`; the pads are 0.078 m further along the approach.
- The arm settles 2–4 cm below and up to 3 cm beside a commanded pose (gravity sag, on purpose: `gravity=False`
  in the Metal config to match the hardware). Re-issue `move_pose` with the error added; `move_linear` plans from
  the measured pose and cannot cancel it.
- `gripper.open()`/`close()` return in about 10 ms; wait (about 20 `arm.pose()` reads) before lifting.
- Object positions read while the cube is in the jaw are biased by about 1 cm; use the tool error at closing instead.
- Keyframe sheets: `runs/sim_v6_dev00_sheet.png`, `runs/sim_v7_sheet.png`.

## Cable lab (stage 1: unplug, plug in, cycle)

`sim/cable_bench.xml` puts the station's cable set on the same Metal bench: a 26 x 5 x 3.5 cm strip (the hub,
a 1.5 kg free body so a trial's reset can place it) lying along the arm's x axis 0.23–0.28 m out, with two USB
plugs standing in vertical sockets 5 cm apart along it. Each plug is a free body: a 1.2 x 0.45 x 2.5 cm blade in a
slot with 1 mm clearance per side over a 1 cm floor, a bright 1.8 x 1.2 x 4 cm housing above the hub for the jaw,
a short cable stub on top. Gravity and wall friction hold a plug; a 0.5 N pull lifts it straight out (checked in
plain MuJoCo). The sockets are 5 cm apart because the Metal's fingers are 4 cm wide along the tool's x axis
(measured from `metal.xml`), so the jaw squeezes a plug across the strip and its open fingers (inner faces 5 cm
apart) straddle the hub without touching the other plug. A third plug lies on its side as the loose cable of the
plug-in task; the unplug labs park it at the far corner. In the station videos (`notes/station-backup/datasets/
unplug-right`) the cables are short USB leads standing in the top-face ports of a power strip, so the slot axis
is vertical and an unplug is a straight lift, as the brief says.

`sim/lab_cable.py` subclasses the dimos lab with `CableWorld`, whose reset places the hub, its yaw and both plugs
together for each seed (the base lab places one body, which would tear a plug out of its socket), and records the
highest z every body reached since the reset for the cycle judge. `CABLE_TASK` overrides the task text.

| Blueprint | Task text | Judge (`sim/judges.py`) |
|---|---|---|
| `phyhillclimb.autoresearch-cable-unplug` | Unplug all the cables from the cable set. | `UnplugJudge`: a plug 4 cm up its socket axis (held in the air counts) or clear of the socket (> 2 cm off axis) and still on the bench; a plug below the bench top or a hub displaced > 1 cm fails |
| `phyhillclimb.autoresearch-cable-plug` | Plug the loose cable into the free socket of the cable set. | `PlugJudge`: within 4 mm of the socket axis, within 3 mm of full depth, within 10 degrees of vertical, hub still |
| `phyhillclimb.autoresearch-cable-cycle` | Unplug a cable from the cable set, then plug it back into its socket. | `UnplugPlugJudge`: some plug rose 4 cm during the trial (peak z from the world), then `PlugJudge` on it |

Socket positions follow the hub's pose (`CableSet.seated`), so the judges hold wherever the seed put the strip.
Unit tests: `sim/tests/test_cable_judges.py`. Same seeded trials as the stacking labs (5 dev, 10 held-out,
300 s program timeout). Run on HF Jobs: `LAB=phyhillclimb.autoresearch-cable-unplug hfjobs/launch.sh ssh`, then
`JOB=<id> hfjobs/remote.sh 'bash /work/PhyHillClimb/hfjobs/restart_lab.sh phyhillclimb.autoresearch-cable-cycle cable_cycle_v1.py'`.

Results (Sun 8 Nov, HF Jobs cpu-xl, 12 workers):

| Candidate | Lab | Change | Dev | Held-out | What the traces showed |
|---|---|---|---|---|---|
| cable_v1 | unplug | tower-v5 idioms on the plug housing: pads aimed 7.8 cm ahead of the mount and 5 mm above the housing centre, tool yaw = plug yaw, hover + sag-fed grasp with 0.7 re-aims, dwell after jaw commands, 8 cm straight lift, cable set down 10 cm across the strip, nearest plug first then the other | **5/5** (36–38 s for both plugs) | **10/10** (35–46 s) | grasps land 0.4–8.9 mm off the housing centre and every lift takes the plug (5.2–6.8 cm of the 8 cm commanded: the jaw holds, the arm sags); both plugs came out in 17 of the 20 trials, the judge counted one in held-out 2, 3 and 8 (in 3 the first grasp of plug_b closed 1.2 cm high and lifted nothing, the regrasp 5 mm deeper took it; in 2 and 8 the straight sideways carry was refused and the planned fallback swung the plug out of the jaw onto the strip, where it lay within 2 cm of its socket: the scene frame after held-out 2 shows plug_b lying on the hub; a plug in the jaw must only move in straight lines) |
| cable_cycle_v1 | cycle | cable_v1's grasp and a 6 cm lift, then the re-plug driven by straight moves on the measured plug offset, two-step straight descent | 0/5 | | the lift raised the plug 4.2–4.9 cm (3.2 once: lift 6 cm minus the sag); the straight 1–1.5 cm xy corrections moved the plug 0–5 mm and the straight descents stopped the housing 2.0–2.6 cm above its seat, blade still above the mouth: **a straight move plans from the measured pose, so it re-commands the sag and shifts the settled pose by the request minus the sag** (the stacking lesson, now measured with a plug); the release dropped the plug onto the socket tilted (stage insert, 18–26 mm high) |
| cable_v2 | unplug | cable_v1 with the unplugged cable carried in straight lines only (whole, halves, 4 cm higher, 3 cm lower), released where it hangs if none is possible | not scored | | the machine running it (new scene camera) was cancelled by the platform after the first dev trials: the HF pre-paid credits ran out at 01:20 PST and every job on the account stopped |
| cable_cycle_v2 | cycle | 8 cm lift; re-plug on absolute re-aims with the command carried along (0.7 gain on the measured plug offset), insertion in 1.2 cm planned steps, 2 cm back-off on a jam | 0/5 | | alignment at the lifted height now reaches 0.3–1.4 mm, but each 1.2 cm planned step lands the plug 2–7 mm off the axis (the xy sag changes with height; the planner settles each step differently), so the blade meets the 1 mm-clearance mouth off-centre and stops with the housing 25 mm above the seat in all 5 trials; the 2 cm back-off pulled the caught plug out of the pads twice (it then lay on the hub). **Fix on the bench, not only the program: the sockets get a funnel mouth (5 mm lead-in over 1 cm), the stand-in for a receptacle's flared shell and the cable's compliance; a plug dropped 1 cm above it seats from 6 mm off axis in plain MuJoCo** |
| cable_cycle_v3 | cycle | v2 on the funnel bench: alignment with the blade 1 cm above the mouth (1.5 mm / 4 mm), insertion in 4 mm absolute steps with the xy correction continued, 6 mm back-off on a stall, a dropped plug ends the attempt | 1/5 (Mac, 4 workers, 27–30 s) | | **first re-plug (dev-01: 1.1 mm off axis, full depth)**; a planned move asked to go 4 mm down lands the plug anywhere from 10 mm up to 18 mm down and 5–10 mm sideways, so the stepping is noise and the funnel does the work: three trials ended with the blade in the slot but held 12–21 mm high by friction (released there, the plug did not slide home), one grasp missed (lift 0.8 cm) |
| cable_cycle_v4 | cycle | second hypothesis from the v2 traces: align 1 cm above the mouth, one 1.4 cm step to put the blade tip in the funnel, open the jaw a third so gravity and the funnel centre the plug, close and push 1.5 cm, release when seated; retried twice from above the mouth | 0/5 (Mac) | | the one-third opening freed nothing: the plug's pose was identical before and after it in every trial (the hanging plug leans in the pads, its housing edge rests on the hub top while the blade tip leans in the funnel); the push with the jaw closed again left it 13–21 mm high and tilted |
| cable_cycle_v5 | cycle | blade tip into the funnel, jaw opened fully, then the cable stub pressed down with the closed fingertips (9.2 cm ahead of the mount), 1 cm deeper per press | 0/5 (Mac) | | the plug again did not move when let go (it stood 2–5 mm above the mouth, leaning); the first press knocked the leaning plug 9–15 mm sideways, but presses on a plug whose blade was in the slot drove it down 4–18 mm each (36 → 14 mm, 35 → 11.5 mm): a press works once the blade is constrained |
| cable_cycle_v6 | cycle | v3's stepping until the housing is within 15 mm of the seat (blade in the slot), release, then up to four closed-jaw presses on the stub | **3/5** (Mac, 20–23 s per pass) | | three plugs seated 0.5–0.9 mm off axis at full depth (two by the stepping alone, one by the press); dev-01: the jaw closed for the press while still around the housing and squeezed the leaning plug 4 cm up out of the slot; dev-02: missed grasp (lift 2.4 cm, the 5 mm-deeper regrasp the same) |
| cable_cycle_v7 | cycle | v6 with the command raised 6 cm after the release before the jaw closes into a pressing tip | **3/5** (Mac, 29–35 s per pass) | | seated 0.8–1.2 mm off axis at full depth in three; dev-02 (hub at 0.278 m, yaw −0.27) fails the unplug in every version: grasp within 2 mm, but the 8 cm straight lift raises the plug only 2.0–3.8 cm; dev-01: blade in the slot 9 mm high, the closed-jaw presses lifted it (10 → 21 mm) instead of seating it |
| cable_cycle_v8 | cycle | v7 plus a second 5 cm lift whenever the first raised the plug 1.5–4 cm | 0/5 (Mac) | | the second lift fixed dev-02's unplug (5.8 cm) but every insertion stopped with the blade at the mouth (+21 mm) on seeds that v6 and v7 had seated with the same code path: **the re-plug outcome is dominated by trial-to-trial variance of the planned steps, not by the candidate** (v6 3/5, v7 3/5, v8 0/5 with identical insertion code) |

Official scored unplug session (Sun 8 Nov 09:13–10:13 PST, HF Jobs cpu-xl, funnel bench): cable_v1 frozen as candidate 1, status `scored`, `heldout_rate` 1.0 (dev 5/5, held-out 10/10), at `runs/sessions/cable_unplug_20261108_171242/`; video of dev-00 `runs/videos/cable_v1_frozen_dev00.mp4`.

Cycle summary (Mac, `--n-workers 4`, 03:05–03:30 PST): best candidates v6 and v7 at 3/5 dev (seated 0.5–1.2 mm off
axis at full depth, 20–35 s per pass); held-out not run (the 4/5 bar was not reached in the 45-minute Mac slot).
The unplug half of the cycle succeeds in 14 of 15 trials across v6–v8; the re-plug is where the variance is: a
planned move asked to descend 4 mm lands anywhere within ±1 cm, so whether the blade finds the funnel is a coin
toss per step. The next lever is not another program but a steadier descent primitive (a short straight move
with the sag undershoot fed forward, or a joint-space nudge of the wrist/elbow only) or a wider funnel (8 mm).
Mac driver logs: `runs/cable_mac_driver*.log`; results `runs/hf_cable/cycle_mac_*.json`.

Plug-in of the loose cable (`autoresearch-cable-plug`) has a bench, a layout and a judge but no candidate yet: the
loose plug lies on its side with a 1.2 cm tall housing, and the Metal's fingertips reach 1.4 cm below the pad
centre (`fingertip_reach` 0.092 against `pad_behind_tcp` 0.078), so a top-down grasp of a lying plug puts the
fingertips into the bench; it needs a horizontal-jaw grasp and a 90 degree re-orientation before the socket,
which the planned moves in these labs have not been tried on. The cycle lab is the plug-in path that is in reach.

Video of dev-00 (both plugs out, 218 scene frames at 5 fps played at 10 fps, h264): `runs/videos/cable_v1_dev00.mp4`
(gitignored). The scene camera sat straight behind the arm's base for this session, so the arm hides the hub for
part of the clip; `cable_bench.xml` now puts the camera high to the right of the base, where a local render shows
the strip and both plugs beside the arm. The unplug scores above were measured on the bench before the funnel
mouths were added; the funnel changes only the top centimetre of each slot and the seated plug geometry is the
same, so a plug lifts out at least as freely. Results JSON: `runs/hf_cable/unplug_v1_results.json`. Wrist keyframes
show the orange housing centred between the pads after the lift.

Video of a passing re-plug (cycle v6 dev-00, Mac lab, new camera: the strip and both plugs beside the arm, real
time at 5 fps): `runs/videos/cable_cycle_v6_dev00.mp4` (gitignored).

### Why the sockets have funnel mouths

The first bench had plain slots with 1 mm of clearance per side, the order of a real USB-A fit. The unplug does not
care, but every re-plug attempt (cycle v1, v2) ended with the blade stopped on the mouth: the arm places a hanging
plug to about 1 mm at a fixed height, yet the planned move that lowers it by 1.2 cm lands it 2–7 mm off, because the
gravity sag of the position-controlled joints changes with height and each planned step settles differently. The
real socket has a flared metal shell and the real cable bends; both forgive a few millimetres. The rigid sim has
neither, so each slot mouth is now a funnel with 5 mm of lead-in over 1 cm (63 degrees from horizontal, steeper
than the 42 degree friction angle of the plastic contacts). In plain MuJoCo a plug released 1 cm above the mouth
with up to 6 mm of offset slides in to within 1.5 mm of full depth and 1.3 mm of the axis. The judge tolerances
(4 mm lateral, 3 mm depth, 10 degrees) are unchanged: a plug that passes them is in the slot, not in the funnel.

### Facts the cable candidates rely on (measured in these sessions)

- The housing of a seated plug is 5.5 cm above the bench; the grasp command is housing centre + 0.5 cm + 7.8 cm
  (pads centred 5 mm above the housing centre, fingertips 1.2 cm above the hub). Grasps land 0.4–8.9 mm off the
  housing centre after three 0.7-gain re-aims; a 1.8 x 1.2 cm housing in a 10 cm jaw forgives all of that.
- Tool yaw = plug yaw (the jaw opens along the tool's y axis, the fingers are 4 cm wide along x). The open fingers
  straddle the 5 cm strip; the other plug 5 cm along the strip is 2 cm from the finger's edge.
- An 8 cm straight lift raises the plug 5.2–7.1 cm (sag); a 6 cm lift only 3.2–4.9 cm, so lift 8 cm for the 4 cm bar.
- A plug in the jaw must move in straight lines or short absolute re-aims: planned fallbacks swung it out twice.
- Straight moves shift the settled pose by the request minus the sag (1–1.5 cm xy corrections moved the plug
  0–5 mm); absolute re-aims with the command carried along converge to about 1 mm in 3–6 steps.
- Each planned descent step changes the hanging plug's xy by 2–7 mm; hence the funnel and the 4 mm steps of v3.
- The hub never moved more than 0.05 mm in any trial (1.5 kg, friction 1.2); the hub-displacement check is a
  safety net, not a discriminator.

On the Mac (one lab at a time, port 9990): `cd ~/projects/dimos && .venv/bin/dimos --simulation --n-workers 4 run
phyhillclimb.autoresearch-cable-cycle --mujocosimmodule.headless true`, then
`python hfjobs/driver.py --dimos ~/projects/dimos/.venv/bin/dimos --candidates cable_cycle_v3.py,cable_cycle_v4.py`.

| tower_order_v24 (stick order) | v24 on yellow, red, blue, red stick (Mac) | 0/5 | the stick is never lifted: planned re-aims at grasp height knock it over or miss; tower stood in 4/5 |
| tower_order_v25 | two-leg stick carry with a check between legs | 0/5 | same grasp failure as v24: the stick is missed or knocked at grasp height; the three cubes stood in every trial |
| tower_order_v26 | closed-loop stepped descent for the stick's hole approach | prepared | |
| tower_order_v27 | stick approached by re-aiming only at the hover, then a closed-loop straight descent in x, y, z | 0/5 | the descent settles 0.8 cm off in x every time (a repeatable horizontal bias of straight moves) and the jaw closes on nothing: it had not finished closing when the lift began |
| tower_order_v28 | 2 s wait after closing on the stick, verified 2 cm lift | 0/5 | keyframes: the yellow stick was already lying across the red stick's spot and the jaw closed on it; the retreat path to (0.10, −0.12) had knocked it over |
| tower_order_v29 | v28 plus learned x/y bias on straight moves | prepared | |
| tower_order_v30 | v29 with the retreat moved to (0.08, 0.10) and the yellow stick placed at (0.13, −0.17) in the bench | 0/5 | the jaw now reaches the red stick within 1–3 mm, but the stepped x/y corrections at stick height knock it over (lying in 5/5) |
| tower_order_v31 | planned re-aims only at the hover (offset by the learned horizontal bias), one purely vertical descent, stepped close | 0/5 | the vertical descent lands 0.9–1.0 cm off in x every time; the first pad to arrive pushes the stick over |
| tower_order_v32 | descend, measure the landing error, rise, re-aim the hover by it, descend again (up to twice), close only within 3 mm | 0/5 | second descent lands within 2 mm and the stick is no longer knocked over, but the jaw closes on nothing: the scene frames show the tool pitched 30–45° at the stick, so the pads (7.8 cm along the tilted tool) are 4 cm from the stick while the mount is on it |
| tower_order_v33 | pads (mount + 7.8 cm along the tool axis from the quaternion) aimed at the stick; jaw tilt > 8° at the hover or after a descent → rise, re-plan the hover, descend again | 0/5 | never reached the stick: all five towers failed on planner refusals (map ghosts after 40 min of lab uptime; lab restarted), and the tilt read 44° at every vertical cube grasp, so the finger axis is not the pose frame's +z |
| tower_order_v34 | tool axis calibrated from the first cube grasp (known vertical): the local vector that maps to straight down; quaternions logged | 0/5 | **first stick lifts** (11 cm in 2 of 5), lost on the carry's first leg: frames show the wrist pitching to 45° during the straight rise at (0.27, −0.12), the reach edge; the other 3 trials landed 1.5 cm off at 9° tilt and the shift was withheld above 8° |
| tower_order_v35 | shift always applied, tilt allowance 15°, and the bench moves the red stick to (0.20, −0.12), 23 cm from the base like the red cube (bench change, noted above) | 0/5 | pads land within 1 mm every time; 2 trials lift the stick 11 cm and lose it on the carry, 3 close on nothing. Frames: the jaw reaches the stick pitched ~35° at the new spot too, while the pose quaternion says 7–9°; the two outcomes follow the two IK branches of the first grasp |
| tower_order_v36 | stick hover seeded by a joint-space move to the mirrored red-cube hover (a jaw known vertical), then the short planned move; joints logged at the calibration grasp and at each descent for an offline FK check | 0/5 | seed changed nothing. Offline FK of the logged joints: the pose quaternion is right, the jaw reaches the stick pitched 11.5° (the oblique camera exaggerates it), the calibration grasp was itself 5.6° off, so the pads were aimed 0.7–0.9 cm off along their width: 0.7 cm still lifted (3 of 5), 0.9 cm closed on the pad's edge. Lifted sticks are lost on the carry's first leg |
| tower_order_v37 | finger axis = mount local +z from the quaternion (FK-verified), no seed move, stick carried only 4 cm above insertion height | 0/5 | **stick lifted in 5 of 5** (11 cm, pads within 2 mm after one re-aim); lost on the carry's first leg every time: the jaw holds it pitched 12°, and the off-plumb cylinder pivots out of the flat pads during the rise |
| tower_order_v38 | jaw levelled at the hover: the measured tool axis gives the planner's orientation error, its inverse is folded into the commanded quaternion and the planned move repeated until tilt ≤ 3°; re-aims keep the levelled quaternion | 0/5 | levelling works (5° → 1° at the hover) but the straight descent drifts back to 7°; stick lifted 5/5 and **carried over the blue cube's hole in 3 of 5** (1–2.4 cm off), where the off-plumb stick had pivoted in the pads (3–4 cm from the tool axis) and the guard gave it up |
| tower_order_v39 | jaw re-levelled after the lift; rise, both carry legs and the descent to the hole are planned hops at the levelled quaternion (straight only if refused) | 0/5 | planned moves with the stick in the jaw fling it: the after-lift levelling swung 17° then 15° and lost the stick in 2 trials, the first planned hop lost it in the other 3. Straight moves drift pitch, planned moves swing: neither carries a centre-held stick |
| tower_order_v40 | stick pinched 1.8 cm above its centre (a pendulum that hangs instead of balancing: a cylinder in flat pads is a single contact per pad, free to swing about the pinch axis); insertion height raised to match; v38 carry (straight, low), levelling only at the empty hover | 0/5 | the higher pinch topples the standing stick at the close in all 3 trials that reached it (longer lever arm for the first pad to touch); 2 trials lost on tower-stage planner refusals (lab degraded within 10 min this time) |
| tower_order_v41 | v38 grasp and carry (centre pinch, straight moves), carry 2 cm above insertion height, over-the-hole guard tolerates a 5 cm swing before the measured alignment slide | 0/5 | **first insertion**: trial 0 stood the stick in the blue cube on the bench (1.1 cm off the hole's axis), trial 4 missed by 1.9 cm; 3 trials lost the stick on the carry. After the insertion the arm stalled at (0.30, 0): straight rises refused, planned move at joint limits, so the blue cube was never stacked |
| tower_order_v42 | v41 + after the insertion a planned retreat to (0.22, 0, 0.25) before picking the blue cube | 0/5 | trial 4 stood the stick in the blue cube again (1.2 cm off) and the retreat now works, but the stick fell when the cube was lifted for the tower; the other 4 trials lost the stick on the carry. Last run before the 07:15 stop |

## Dimensional's actor on the lab (Sun 8 Nov, 09:16–09:43 PST, HF Jobs cpu-xl)

Dimensional's own Autoresearch actor (Pi 1.3.0 started by the lab's `start_actor` RPC, prompt from
`dimos/autoresearch/prompt.py`) ran against `phyhillclimb.autoresearch-three-cube` with `TOWER_ORDER=yellow,red,blue`
and the strict judge. Model `claude-sonnet-5-5` via provider `anthropic`. Budget 2 iterations x 2 candidates, 3 dev and
5 held-out trials, threshold 0.8 (set through the `AR_*` variables now read by `sim/lab_stack.py`). The session's
`library/` was seeded with our measured arm facts (`arm-facts.md`, served by `task()`). The actor finished on its own
after 13 steps in about 27 minutes (`ended_by: answer`, status `done`), froze candidate 4 and wrote one library note.

| Candidate | Hypothesis (the actor's) | Dev | Held-out | What failed |
|---|---|---|---|---|
| 01 | in-place stack on yellow with sag-aim | 0/3 | | `scene.objects(["cube"])` returns nothing (the truth scene matches whole body names such as `cube_red`): IndexError before any move |
| 02 | same, query by `cube_<colour>` | 0/3 | | `move_pose` refused (no IK) carrying high near x ≈ 0.30 |
| 03 | lower hover to stay reachable; clear all cubes | 0/3 | | first place reached, release 2.5 cm too high, final `move_linear` retreat refused |
| 04 (frozen) | fix drop height, tolerant retreat | 0/3 | 0/5 | programs complete in 39–41 s, red cube ends 5.7–14.4 cm off the yellow one |

It never built the build-spot carry, the sag re-aims or the verified rise that `tower_order_v24` (5/5, 9/10) needs, and
it did not read candidate 4's traces before freezing (its final message says so). Our seeded facts mention all of
these; with 4 candidates and 3 trials each it spent its budget on the query and reach errors first.
Session on the Mac (no `recording.mcap`, no scene frames): `runs/sessions/actor_20261108_0916/` with `session.json`,
`candidates/*/` (program, hypothesis, results, trial traces and keyframes), `actor/trajectory.json` (ATIF) and
`actor/system-prompt.txt`. No candidate passed, so no video was rendered.

## High-quality renders (Sun 8 Nov, 12:30 PST)

Demo videos are rendered offline from captured motion, not from the lab's live 640x480 camera.

- `sim/motion_capture.py` (`MotionCaptureModule`): with `CAPTURE_MOTION=1` in the lab's environment, `lab_stack.py` and
  `lab_cable.py` add it to the blueprint. It appends every `coordinator_joint_state` message (~100 Hz, gripper included)
  and every `body_poses` message (5 Hz) to `<session>/motion.jsonl`, stamped with wall time, the same clock as the trial
  traces. Without the flag the blueprints are unchanged.
- `sim/render_replay.py`: builds the lab's own model (bench XML plus the dimos Metal attached at the origin, as
  `MujocoSimModule` does), restyles it in memory (the physics XML is untouched), sets qpos from the capture at 30 fps,
  renders 1920x1080 with shadows and 8x multisampling, and adds a lower third, a clock, a close-up picture in picture
  and a verdict badge. A body carried in the jaw is interpolated in the jaw's frame between its 5 Hz samples. Window =
  the trial's `trace.jsonl` (first call - 1 s to last call + `--post`). Encodes H.264 yuv420p, CRF 18, faststart.

```sh
# in an HF ssh job (hfjobs/launch.sh ssh), never on the Mac:
CAPTURE_MOTION=1 AR_DEV_TRIALS=2 TOWER_ORDER=yellow,red,blue bash /work/PhyHillClimb/hfjobs/restart_lab.sh \
  phyhillclimb.autoresearch-three-cube tower_order_v24.py
MUJOCO_GL=osmesa DIMOS_ROOT=/work/dimos /work/dimos/.venv/bin/python sim/render_replay.py --lab tower \
  --motion <session>/motion.jsonl --trace <session>/candidates/01/trials/dev-00/trace.jsonl --out /work/render/tower \
  --name hq_tower_v24 --pip --title "..." --program tower_order_v24.py --score "5/5 dev, 9/10 held-out" \
  --verdict "Judge: PASS ..." --workers 28        # --stills 10,50 renders only those seconds, for checking
```

| Video (`runs/videos/`) | Trial | Length | Highlight cut |
|---|---|---|---|
| `hq_tower_v24.mp4` | tower_order_v24, yellow/red/blue, dev-00 of a 2-trial capture run (both passed, 99 s and 102 s) | 104.7 s | `hq_tower_v24_short.mp4`, 30 s at 3.5x |
| `hq_cable_v1.mp4` | cable_v1 unplug, dev-00 (passed, both plugs out, 37 s) | 42.0 s | `hq_cable_v1_short.mp4`, 28 s at 1.5x |
| `hq_cable_cycle_v6.mp4` | cable_cycle_v6, dev-00 (passed, plug_a re-seated 1.0 mm off axis, 26 s) | 31.4 s | `hq_cable_cycle_v6_short.mp4`, 25 s at 1.25x |

All 1920x1080, 30 fps; four stills each as `hq_*_still1..4.jpg`. The capture runs (HF cpu-xl, 20:00–20:27 UTC) scored
cable_v1 2/5 and cable_cycle_v6 2/5. Five of the six failures were `waited 2.0 s for a map without '...'` 2–4 s into the
trial (cable_v1 dev-02..04, cycle dev-02..03), while a render used 28 cores of the same machine: infrastructure, not
the programs. The sixth (cycle dev-04) is a real miss: the plug ended tilted at the socket, 18 mm off axis.
