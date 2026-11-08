# relay: the LiveKit room as the hardware abstraction

`portal.yaml` is the wire schema (from livekit-examples/portal-makermods, minus the `screencast` flag
that Portal 0.2.3 on PyPI does not know): three H.264 camera tracks and a 14-value state and action
(`left_joint1..6.pos`, `left_gripper.pos`, `right_...`; radians and mm).

| File | Role | Run |
|---|---|---|
| `robot_sim.py` | Portal **robot** with a fake backend (synthetic cameras, tracking arm). On the station, Portal's own `uv run robot` replaces it. | `python -m relay.robot_sim --backend fake` |
| `director.py` | Portal **operator**: the Relay Director. Stage sequencer, pluggable Skill and Verifier, clock, score projection, `status` RPC. | `python -m relay.director --seconds 25 --pass-after 5` |
| `common.py` | env, tokens, pacer. Without `LIVEKIT_URL` it targets the local dev server. | `livekit-server --dev --bind 127.0.0.1` |

Local loop, verified 2026-11-07: robot joins, director claims control, 108 synchronized observations and 177
actions in 25 s, placeholder verifier passes unplug then stack, final advice "CALL STOP NOW: 174 points".

Next: a Dream-1 `Skill` (the loop in `harness/dream_harness.py` fed from `obs.frames`), an overhead-camera
`Verifier`, the dimos sim backend for `robot_sim.py`, and a voice agent that calls the `status` RPC.

## Viewer and codec notes (2026-11-07 afternoon)

- LiveKit Meet does not render Portal's tracks: it shows only camera and screen-share sources, and Portal 0.2.3
  publishes plain named tracks. `web/viewer.html` is our viewer and dashboard: renders every video track, polls
  the Director's `status` RPC every second, takes `#url=...&token=...` in the fragment. Serve it with
  `python -m http.server 8090 --bind 0.0.0.0 --directory web` on the venue network, or any static host.
- Browsers could not decode Portal's H.264 (tracks subscribed, 0x0 decode). `portal.yaml` now uses `codec: vp8`
  for the three cameras, which Chrome and Safari decode; Python subscribers decode either.
- Portal 0.3.0 (the version the LiveKit example pins) only ships wheels for macOS 26 and manylinux 2.35, so the
  Mac stays on 0.2.3; the station (Ubuntu) can use 0.3.0.
- Viewer links for teammates are minted with `lk token create --join --room relay-nuck4 --identity viewer-N
  --valid-for 48h` and kept in `notes/relay-links.md` (gitignored).

## The real station in the room (relay/station_bridge.py + tools/relay_tap.py, 2026-11-08)

The arm runner on the station (`tools/dream_right_arm_run.py`, the teammates' Dream environment, no LiveKit in it)
gets a `--relay` flag: a `RelayTap` thread writes the newest camera frames (JPEG 640x360) plus `state.json` (the
14 portal.yaml values and the runner's status: mode, phase model/assist, task, elapsed, Dream round trip, cables
unplugged, cubes placed, last event) to `/dev/shm/phyhill-relay` at 15 Hz. `publish()` never blocks or raises.

`python -m relay.station_bridge --room relay-nuck4` (own venv `~/relay-env`, own process, tmux session `relay`)
reads that folder and publishes it as the Portal **robot**: VP8 tracks `overhead_camera`, `left_wrist_camera`
(a placeholder in single-arm runs), `right_wrist_camera` and the state at 30 Hz, and answers the `status` RPC in
the Director's vocabulary, so `web/viewer.html` shows the real arm with no change. The bridge is publish-only:
operator actions are counted and ignored. On the station only a pre-minted robot token lives in
`~/PhyHillClimb-relay/.env.local` (`LIVEKIT_ROBOT_TOKEN`), never the API secret.

Checked on the Mac with `livekit-server --dev`: `python tools/relay_tap.py --demo` + the bridge + `python -m relay.peek`
= 28.8 observations/s, three tracks decoded, state and status RPC correct. `tools/apply_relay_hook.py` applies the
runner hook to a teammate's copy of the runner (anchored edits; the station's two variants both took it).

## Dream-1 skill (relay/skills/dream.py)

`python -m relay.director --skill dream [--checkpoint dream-1-ft:...] [--dry-run]` runs Dream-1 through the room:
the Portal loop ported behind the Skill interface (Metal embodiment, `ik: {arm: metal, head: eef}`, 320x176 JPEG
frames, time-indexed chunk rows, `refill_due` pacing, per-tick ramp `DREAM_MAX_JOINT_RAD_S`, default 0.5).

Checked 2026-11-07 against Dream's stand-in (`python -m dream.check_loop`, `DREAM_GATEWAY_URL=http://127.0.0.1:8642`)
with the fake robot on the **cloud** room: hold PASS, right part moved PASS, play rate PASS, circle PASS; lag 0.46 s
(limit 0.25) and one axis at x0.8 remain, which is the cloud round trip plus the ramp. For the match, run a local
`livekit-server --dev` on the venue LAN (or on the NUC) so the robot, the Director and the viewers share one hop;
keep the cloud project for remote viewers.
