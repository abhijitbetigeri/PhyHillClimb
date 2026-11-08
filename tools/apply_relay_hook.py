"""Apply the LiveKit relay hook (--relay flag + RelayTap publishing) to a copy of the station's arm runner.

Anchored string edits, each required to match exactly once (except the assist print, replaced everywhere).
Usage: python apply_relay_hook.py <runner.py>   (edits in place, prints what it changed)
"""
import re
import sys

path = sys.argv[1]
src = open(path).read()
orig = src
changes = []


def once(old, new, label):
    global src
    n = src.count(old)
    if n != 1:
        raise SystemExit(f"{path}: anchor for {label!r} found {n} times, expected 1")
    src = src.replace(old, new)
    changes.append(label)


once("    a = ap.parse_args()\n",
     '    ap.add_argument("--relay", action="store_true",\n'
     '                    help="publish the cameras, the arm state and this runner\'s status to the LiveKit relay room: "\n'
     '                         "writes to a tmpfs folder (relay_tap.py beside this file); `python -m relay.station_bridge` publishes it")\n'
     "    a = ap.parse_args()\n", "--relay flag")

log_line = "    log = open(log_dir / f\"right_arm_{time.strftime('%Y%m%d_%H%M%S')}_{'dry' if a.dry_run else 'live'}.jsonl\", \"a\")\n"
once(log_line, log_line + '''
    # LiveKit relay (optional): the newest frames, state and status go to a tmpfs folder; a separate bridge process
    # publishes them to the room. publish() never blocks and never raises, so the control loop is unaffected.
    tap, portal_state = None, None
    relay_status = {"mode": "live" if a.live else "dry-run", "phase": "warmup", "task": a.task, "budget_s": a.seconds,
                    "checkpoint": a.checkpoint, "unplugged": 0, "placed": 0, "event": "", "rtt_s": None}
    if a.relay:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from relay_tap import RelayTap, portal_state

        tap = RelayTap()
        print(f"relay: writing frames and status to {tap.dir} (start `python -m relay.station_bridge` to publish)")

    def note(text: str) -> None:
        """Print an event and show it on the relay dashboard; count the cables it reports as out."""
        print(text)
        relay_status["event"] = text
        if "released in place" in text or "dropped at the drop spot" in text or text.startswith("assist: pulled"):
            relay_status["unplugged"] = relay_status.get("unplugged", 0) + 1
''', "tap setup")

has_unplug_state = "unplug_state = {" in src
loop_old = ("        while time.monotonic() - started < a.seconds:\n"
            "            tick = time.monotonic()\n"
            "            frames = {}\n")
loop_new = ('        relay_status["phase"] = "model"\n'
            "        while time.monotonic() - started < a.seconds:\n"
            "            tick = time.monotonic()\n"
            "            frames, raw = {}, {}\n")
once(loop_old, loop_new, "loop head")
once('                    frames[f"video.{role}"] = jpeg_b64(rgb, SHORT_SIDE)\n',
     '                    frames[f"video.{role}"] = jpeg_b64(rgb, SHORT_SIDE)\n'
     "                    raw[role] = rgb\n", "raw frames")
publish = ("            joints, grips = arms.read()\n"
           "            if tap is not None:\n"
           '                relay_status["elapsed_s"] = round(tick - started, 1)\n')
if has_unplug_state:
    publish += '                relay_status["unplugged"] = unplug_state.get("dropped", 0)\n'
publish += ('                tap.publish(raw, portal_state(joints["right"], grips["right"], left_hold[0], left_hold[1]), relay_status)\n')
once("            joints, grips = arms.read()\n", publish, "publish per tick")
once('                print(f"chunk: {len(arm_rows)} rows, round trip {time.monotonic() - asked_at:.2f} s, starting at row {skip}")\n',
     '                relay_status["rtt_s"] = round(time.monotonic() - asked_at, 2)\n'
     '                print(f"chunk: {len(arm_rows)} rows, round trip {time.monotonic() - asked_at:.2f} s, starting at row {skip}")\n',
     "chunk rtt")
n = src.count('print(f"assist: {outcome}")')
src = src.replace('print(f"assist: {outcome}")', 'note(f"assist: {outcome}")')
changes.append(f"assist notes x{n}")
if "outcome = stack_release(arms, joints, left_hold, log, started)\n" in src:
    once("                    outcome = stack_release(arms, joints, left_hold, log, started)\n",
         "                    outcome = stack_release(arms, joints, left_hold, log, started)\n"
         '                    relay_status["placed"] += 1\n', "stack placed")
# assist phase markers around the two takeover calls, when present
for call in ("outcome = assist_demo_grasp(arms, joints, left_hold, log, started, demo_grasps, unplug_state) if demo_grasps else None\n",
             "outcome = assist_demo_grasp(arms, joints, left_hold, log, started, demo_grasps) if demo_grasps else None\n"):
    if src.count("                " + call) == 1:
        once("                " + call,
             '                relay_status["phase"] = "assist"\n                ' + call + '                relay_status["phase"] = "model"\n',
             "assist phase")
once("    finally:\n        log.close()\n",
     "    finally:\n"
     "        if tap is not None:\n"
     '            relay_status["phase"] = "done"\n'
     "            tap.publish({}, {}, relay_status)  # the final status reaches the dashboard\n"
     "            tap.close()\n"
     "        log.close()\n", "tap close")

open(path, "w").write(src)
print(f"{path}: {len(changes)} edits: {', '.join(changes)}; +{src.count(chr(10)) - orig.count(chr(10))} lines")
