# LiveKit presentation demo

From the project root:

```sh
.venv/bin/python -m relay.demo --open
```

Open **http://127.0.0.1:18090/demo**. The loop plays the cable simulation, then the tower simulation,
then holds a scripted score/“call stop” preview for eight seconds. Ctrl-C stops only the demo's
own processes. It uses an isolated local room and ports 17880–17882 plus dashboard port 18090;
it does not connect to the station, start a Dream model, or load cloud credentials.

Prerequisites: `livekit-server` on PATH, project Python dependencies, and
`uv pip install --python .venv/bin/python -r relay/requirements-demo.txt` for video decoding.
The local clips `runs/videos/hq_cable_v1_short.mp4` and `runs/videos/hq_tower_v24_short.mp4` are
required and intentionally gitignored. Supply alternatives with `--cable-video PATH --tower-video PATH`.
The browser SDK is vendored under `web/vendor/`, with its license, so internet is not required.

## What to say

1. “LiveKit transports the robot's camera tracks and state into one room.”
2. “This is recorded MuJoCo simulation, broadcast through the real Portal bridge. The main view is replay;
   both wrist tracks explicitly say that those views are unavailable.”
3. “The dashboard reads status over RPC. This presentation scripts stage transitions and score;
   it is not evidence of a live policy, task verifier, or real robot run.”
4. “The station uses the same publish-only bridge. Its arm runner controls motion independently.”

Logs, scripted stage events, and connection metadata go under `runs/livekit_demo/<room>/`.
The `/demo` redirect mints a subscribe-only browser token for the local room. Do not expose this dev server
publicly. For actual station viewing, use the existing station bridge and authorized viewer token;
this launcher intentionally does not operate hardware.

Portal transport and synchronization reference:
https://docs.livekit.io/robotics/integrations/portal/

## Verification

Checked locally on 2026-11-08: Portal observer received 27.1 synchronized observations/s and
decoded all three 640×360 tracks. Headless Chrome connected, played all three videos, displayed
the replay banner, and read the bridge status through RPC. Both scripted stage transitions and
the final score preview were observed. No station connection or hardware access was used.
