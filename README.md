# PhyHillClimb

Hill Climb entry for the MakerMods Physical AI Hackathon (Founders Inc, San Francisco, 7–8 Nov 2026).
Dream-1 does the motion. This repo is the harness around it: the control loop, a success check between
action chunks, recovery moves, a stopping rule, and the eval that scores every attempt the same way.

## Layout

```
harness/dream_harness.py   control loop + verifier + recovery + stopping rule (passes dream.check_loop)
harness/README.md          how to check the loop with no arm and no model
runs/                      JSONL logs, one line per round trip and verdict (gitignored)
```

## Setup

```sh
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
source .venv/bin/activate
```

Dream credentials go in `~/.dreamlabs/credentials` (`DREAM_GATEWAY_URL`, `DREAM_API_URL`, `DREAM_DATA_URL`,
`DREAM_API_KEY`), `chmod 600`. Never commit keys.

## Check the loop before touching an arm

```sh
python -m dream.check_loop --rate 20          # terminal 1, type "the workspace is clear"
DREAM_GATEWAY_URL=http://127.0.0.1:8642 DREAM_API_URL=http://127.0.0.1:8642 DREAM_DATA_URL=http://127.0.0.1:8642 \
DREAM_API_KEY=x python harness/dream_harness.py --fake --no-recover --seconds 40   # terminal 2
```

The stand-in prints PASS or PROBLEM for hold, part isolation, direction and size, lag, play rate and circle.

## Station facts

From the official Dream-1 loop for the Metal station
([livekit-examples/portal-makermods](https://github.com/livekit-examples/portal-makermods)):
embodiment per side `arm`(6) `grip`(1) `eef_xyz`(3) `eef_rot6`(6), policy config
`"ik": {"arm": "metal", "head": "eef"}`, cameras `overhead`, `wrist_l`, `wrist_r` cropped 16:9 and resized to
320x176 JPEG q85, joints in radians, grippers 0..1, targets ramped at 0.5 rad/s. `metal_embodiment()` in the
harness carries it.

Dream-1 timing: a 1000-step fine-tune takes about 7–8 hours, a cold model takes 25–50 minutes to warm, the
rate limit is 3 calls then one per 0.7 s. Plan the day around those three numbers.

## Resources

- Getting started: https://www.livekit.info/pah
- Maker Arm SDK: https://github.com/makermods-robotics/maker-arm-sdk
- LeRobot Maker Arm branch: https://github.com/makermods-robotics/lerobot/tree/robot/makermods-maker-arm
- Dream client: `pip install dreamlabs` (never `pip install dream`)

## Disclosure

The harness skeleton was written before the event against the public Dream client. Everything that touches
the station, the dataset and the fine-tune is built during the hackathon.
