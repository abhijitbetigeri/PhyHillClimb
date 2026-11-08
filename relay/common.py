"""Shared plumbing for the relay room: env, LiveKit tokens, the tick pacer.

The room is the hardware abstraction (AGENTS.md §6b): a Portal robot publishes three camera
tracks and the 14-value state, the Relay Director is a Portal operator, everything else observes.
`relay/portal.yaml` is the schema, copied from livekit-examples/portal-makermods so a real Metal
station and our sim bridge speak the same wire format.

Without LIVEKIT_URL in the environment or .env/.env.local, everything targets the local dev server
(`livekit-server --dev`, ws://127.0.0.1:7880, key devkey / secret secret).
"""

from __future__ import annotations

import asyncio
import datetime
import os
import pathlib
import time
from collections.abc import AsyncIterator


from livekit import api
from livekit.protocol.room import RoomConfiguration

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "relay" / "portal.yaml"

ROBOT_IDENTITY = "robot"
DIRECTOR_IDENTITY = "director"
DEFAULT_ROOM = "relay-dev"

LOCAL_URL = "ws://127.0.0.1:7880"
LOCAL_KEY, LOCAL_SECRET = "devkey", "secret"

SIDES = ("left", "right")
JOINTS = 6
# portal.yaml order: left joints, left gripper, right joints, right gripper.
STATE_NAMES = (
    [f"left_joint{i}.pos" for i in range(1, 7)]
    + ["left_gripper.pos"]
    + [f"right_joint{i}.pos" for i in range(1, 7)]
    + ["right_gripper.pos"]
)
GRIPPER_MAX_MM = 80.0  # the Metal SDK's stroke, 0 closed to 80 open, as portal.yaml says


def config_fps(path: pathlib.Path = CONFIG_PATH) -> float:
    """The observation rate from portal.yaml (`fps:`); Portal 0.2.x does not expose it on the config."""
    for line in path.read_text().splitlines():
        key, sep, value = line.partition(":")
        if sep and key.strip() == "fps":
            return float(value.split("#")[0].strip())
    return 30.0


def load_env() -> None:
    """Fill the environment from `.env` and `.env.local` (repo root, then cwd; later files win among
    themselves), but never override a variable the process already has, and never set an empty value: a
    test can point DREAM_GATEWAY_URL at the stand-in model from the command line."""
    from dotenv import dotenv_values

    merged: dict[str, str] = {}
    for directory in (ROOT, pathlib.Path.cwd()):
        for name in (".env", ".env.local"):
            path = directory / name
            if path.exists():
                merged.update({k: v for k, v in dotenv_values(path).items() if v})
    for key, value in merged.items():
        os.environ.setdefault(key, value)
    os.environ.setdefault("RUST_LOG", "warn,livekit_portal::sync_buffer=error,libwebrtc=error")


def livekit_url() -> str:
    return os.environ.get("LIVEKIT_URL") or LOCAL_URL


def room_name() -> str:
    return os.environ.get("LIVEKIT_ROOM") or DEFAULT_ROOM


def mint_token(identity: str, room: str, ttl_hours: int = 6) -> str:
    """A LiveKit JWT for `room`; `can_update_own_metadata` lets Portal publish its role attribute."""
    key = os.environ.get("LIVEKIT_API_KEY") or LOCAL_KEY
    secret = os.environ.get("LIVEKIT_API_SECRET") or LOCAL_SECRET
    grants = api.VideoGrants(
        room_join=True, room=room, can_publish=True, can_subscribe=True, can_update_own_metadata=True
    )
    return (
        api.AccessToken(key, secret)
        .with_identity(identity)
        .with_grants(grants)
        .with_room_config(RoomConfiguration(name=room, min_playout_delay=0, max_playout_delay=1))
        .with_ttl(datetime.timedelta(hours=ttl_hours))
        .to_jwt()
    )


async def pace(fps: float) -> AsyncIterator[int]:
    """Yield a tick index at `fps`; a late tick moves the schedule instead of piling up drift."""
    interval = 1.0 / fps
    next_tick = time.perf_counter()
    tick = 0
    while True:
        yield tick
        tick += 1
        next_tick += interval
        now = time.perf_counter()
        if next_tick > now:
            await asyncio.sleep(next_tick - now)
        else:
            next_tick = now
            await asyncio.sleep(0)
