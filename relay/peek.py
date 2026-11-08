"""Peek into the relay room as a Portal operator: who publishes, at what rate, what the status RPC says.

    python -m relay.peek --seconds 8

Prints the observation rate, each camera track's decoded size, the latest 14-value state and the `status` RPC
answer of the director (if present) or the robot. It claims nothing and sends no action.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time

from livekit.portal import Observation, Operator, OperatorConfig, frame_bytes_to_numpy_rgb

from relay.common import CONFIG_PATH, livekit_url, load_env, mint_token, room_name


async def main() -> None:
    load_env()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--room", default=room_name())
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    cfg = OperatorConfig.from_yaml_file(CONFIG_PATH, args.room)
    op = Operator(cfg)
    count = {"obs": 0}
    latest: dict = {}

    def on_observation(obs: Observation) -> None:
        count["obs"] += 1
        latest["obs"] = obs

    op.on_observation(on_observation)
    await op.connect(livekit_url(), mint_token("peek", args.room, ttl_hours=1))
    print(f"[peek] room {args.room!r} at {livekit_url()}; robot={op.robot_identity()} operators={op.operators()}")
    t0 = time.monotonic()
    await asyncio.sleep(args.seconds)
    elapsed = time.monotonic() - t0
    print(f"[peek] {count['obs']} observations in {elapsed:.1f} s = {count['obs'] / elapsed:.1f}/s")
    obs = latest.get("obs")
    if obs is not None:
        for name, frame in obs.frames.items():
            try:
                rgb = frame_bytes_to_numpy_rgb(frame.data, frame.width, frame.height)
                print(f"[peek] track {name}: {rgb.shape[1]}x{rgb.shape[0]} mean {rgb.mean():.0f}")
            except Exception as e:  # noqa: BLE001
                print(f"[peek] track {name}: undecodable ({e})")
        print("[peek] state", {k: round(float(v), 3) for k, v in obs.state.items()})
    for identity in ("director", "robot"):
        try:
            text = await op.perform_rpc("status", "", destination=identity, response_timeout_ms=3000)
            print(f"[peek] status from {identity}:", json.dumps(json.loads(text), indent=1)[:1200])
            break
        except Exception as e:  # noqa: BLE001
            print(f"[peek] no status from {identity}: {type(e).__name__}: {str(e)[:80]}")
    await op.disconnect()
    op.close()


if __name__ == "__main__":
    asyncio.run(main())
