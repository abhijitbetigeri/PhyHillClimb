# Dream-1 harness skeleton

Loop, verifier, recovery and stopping rule around the Dream API (`dreamlabs-client` 1.1.3). Station specifics are TODOs.

## Check the loop with no arm and no model

```sh
cd ~/projects/PhyHillClimb
source .venv/bin/activate

# terminal 1: the stand-in model. It asks you to type "the workspace is clear" before answering.
python -m dream.check_loop --rate 20

# terminal 2: the harness against it, fake robot and cameras
DREAM_GATEWAY_URL=http://127.0.0.1:8642 DREAM_API_URL=http://127.0.0.1:8642 DREAM_DATA_URL=http://127.0.0.1:8642 \
DREAM_API_KEY=x python harness/dream_harness.py --fake --no-recover --seconds 40
```

Ctrl-C the stand-in; it prints PASS or PROBLEM per check (hold, right part moved, direction and size, lag, play rate, circle, camera rate).
`--no-recover` because the stand-in's hold phase looks like a stall to the v0 verifier. Last run (2026-11-07): all checks PASS, camera rate skipped (fake frames repeat).

For the Metal station use `metal_embodiment()` and add `"ik": {"arm": "metal", "head": "eef"}` to the policy config, as the official
[portal-makermods](https://github.com/livekit-examples/portal-makermods) Dream-1 loop does.

## Plug in the station

1. Implement `Robot` (`read_state`, `command`, `safe_pose`) over the station's driver. Joints in radians, grippers 0..1.
2. Implement `Cameras` (`frames`) returning `video.<role>` base64 JPEGs at 320x176 q85. Use `encode_frame`.
3. Replace `PARTS`, `ROLES`, `EMBODIMENT` with the spans and video keys from `add_embodiment()`.
4. Turn `strict` on for a fine-tuned checkpoint.
5. Give `Verifier` a VLM verdict (`done`) so the stopping rule works; v0 only detects stalls.

Every round trip and verdict is one JSONL line in `runs/`, which feeds the eval table.
