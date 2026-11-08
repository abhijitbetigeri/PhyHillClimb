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
| tower_order_v7 | re-aim 4 cm above the grasp, one straight final descent; stick hang in the set-down height | running | |
| tower_order_v8 | stick gripped at its centre, straight-line carry at fixed orientation, given up when fallen | queued | |


Facts every candidate must respect (all measured, see `candidates/stack_v7.py`):

- `arm.pose()` and `move_pose()` address `gripper_base`; the pads are 0.078 m further along the approach.
- The arm settles 2–4 cm below and up to 3 cm beside a commanded pose (gravity sag, on purpose: `gravity=False`
  in the Metal config to match the hardware). Re-issue `move_pose` with the error added; `move_linear` plans from
  the measured pose and cannot cancel it.
- `gripper.open()`/`close()` return in about 10 ms; wait (about 20 `arm.pose()` reads) before lifting.
- Object positions read while the cube is in the jaw are biased by about 1 cm; use the tool error at closing instead.
- Keyframe sheets: `runs/sim_v6_dev00_sheet.png`, `runs/sim_v7_sheet.png`.
