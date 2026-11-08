# Results (MakerMods Physical AI Hackathon, 7–8 November 2026)

Everything here was produced during the event. Nothing is committed from `notes/` (internal) or `.env*` (keys).

## Real arm (station NUCK-4, right MakerMods Metal arm)

Model: `dream-1-ft:2d43713b-right-arm-v1-plus500`, Dream-1 fine-tuned on 30 teleoperated right-arm demos
(10 unplug, 10 stack, 10 insert) for 1500 steps. The runner (`tools/station/working_set_unplug_8of8/dream_right_arm_run_multi.py`
for cables, `tools/station/dream_right_arm_run_stack.py` for cubes, both started by `tools/station/relay.sh`) lets the
model drive the reach, then finishes the grasp: it closes the jaw where the model settled, replays a recorded human pull
with the gripper hold torque stepping 1.25 → 2.0 N·m while the plug resists, releases, and hands back to the model.

| Run (Sunday PST) | What | Result |
|---|---|---|
| 12:55–13:40, 8 runs | one plug per run, clean table | 8 of 8 plugs out, ~35 s each |
| 14:33 | four plugs in one run | 3 of 4 (time ran out on the fourth) |
| 15:00 and 15:15, relay | unplug then stack | 1–2 plugs, then a cube placed on a cube, twice |
| 15:28, judged run | relay | Dream round trips 1–3.7 s (normally 0.45 s): no grasp settled in 150 s; one cube placed |

Every station run's joint log is in `station_runs/` (`right_arm_<time>_live.jsonl`: commanded target, measured
joints, jaw, assist phases and torque steps).

## Simulation (Dimensional dimos Autoresearch lab, MuJoCo Metal bench)

| Lab | Program | Dev | Held-out | Video |
|---|---|---|---|---|
| ordered three-cube tower, strict judge | `sim/candidates/tower_order_v24.py` | 5/5 | 9/10 | `videos/hq_tower_v24.mp4` |
| cable unplug | `sim/candidates/cable_v1.py` | 5/5 | 10/10 | `videos/hq_cable_v1.mp4` |
| unplug + re-plug | `sim/candidates/cable_cycle_v6.py` | 3/5 | – | `videos/hq_cable_cycle_v6.mp4` |

Scored session records (frozen candidate, trial results, programs) are in `sim_sessions/`; the cable session carries
`heldout_rate 1.0`. `sim_sessions/actor_20261108_0916/` is Dimensional's own actor run on the tower lab (4 candidates,
none passed; `actor/trajectory.json`). Videos are offline re-renders of the recorded physics at 1080p (`sim/render_replay.py`).

## Fine-tunes (Dream Labs, all `dream-1-ft:2d43713b-<suffix>`)

| Suffix | Arm | Data | Steps | Final loss |
|---|---|---|---|---|
| right-arm-v1 | right | unplug + stack + insert, 30 ep | 1000 | 0.0022 |
| right-arm-v1-plus500 | right | same, continued | 1500 | 0.0022 |
| right-unplug-v1 | right | unplug only, 10 ep | 700 | 0.0028 |
| left-arm-v1 | left | stack + insert, 12 ep | 1000 | 0.0020 |
| left-arm-v1-plus500 | left | same, continued | 1500 | 0.0013 |

Tooling: `tools/dream_single_arm_finetune.py` (register, validate, upload, queue, resume), `tools/dream_job_monitor.py`
(overnight cron guardian on the station), `scripts/resume_when_done.sh`. Datasets mirrored to Hugging Face as
`abhijitbetigeri/nuck4-*-20261107` (private).

## What we learned on the real arm (in order of impact)

1. Cameras must be opened at 1280x720 MJPEG; the default 640x480 mode crops and the model saw a different scene.
2. The arm driver's startup sync must finish before the model drives, or the jaw stays in a 1°/tick mode and never squeezes.
3. The fine-tune reaches a plug but never closes or pulls (10 unplug demos); the harness finishes the grasp.
4. Grip hold torque 1.25–2.0 N·m frees a plug; the station default 0.5 slips. The pull must keep leaning in until the shoulder catches up.
5. The model only commits from the trained scene: all cables seated, no loose cables, no cubes near the strip, nobody in view, the park pose not drifted.
6. Dream's session drops after ~10 s without observations (during a long pull): reset the episode and carry on.
7. Server latency decides the day: at 0.45 s round trips the loop works; at 1–3 s the model crawls.
