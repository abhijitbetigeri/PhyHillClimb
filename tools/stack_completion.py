"""Three-block completion: freeze on the first candidate; confirm while held.

Uses the station's existing face-blurred Decisions input and credential loader.
Nothing in this module opens an arm, resumes a policy, parks, or disables motors.
"""
import asyncio
import json
import math
import time

from embodiments.metal.completion import CompletionMonitor, MAX_AGE, scores

STACK_TASK = (
    "Using the right arm, stack all three square blocks into one vertical tower on the table. "
    "Place one block on another, then place the third block on top. Center each block so the "
    "tower stands without support. Release the top block and move the open gripper clear. "
    "Once all three blocks are stacked and released, stop and do not touch the tower again. "
    "Leave the sticks and cables alone."
)
PLACEMENT = (
    "All THREE distinct square blocks form ONE freestanding vertical tower with exactly "
    "three block levels: the bottom block rests on the tabletop, the middle block rests "
    "on the bottom block, and the top block rests on the middle block. All three blocks "
    "are centered sufficiently to stand without gripper support. No block remains "
    "separate on the table or in a gripper. Two stacked blocks are NOT complete. "
    "Do not count rods, holes, gripper fingers, or a raised/held block as another level."
)
SETTLE_S = 3.0


class StackMonitor(CompletionMonitor):
    def __init__(self, output):
        super().__init__(PLACEMENT, "any of the three square blocks", output)
        # The bottom block rests on the table, not on another block.
        self.questions[1]["choices"][0]["description"] = (
            "Both grippers are visibly clear of ALL three blocks. Neither gripper "
            "holds, touches or supports any block; the tower supports itself on the table."
        )
        self.last_stamp = -1.0
        self.candidate_stamp = None
        self.check_started_at = None

    def overdue(self, now):
        return self.check_started_at is not None and now - self.check_started_at > MAX_AGE

    def evaluate(self, reply, stamp, now):
        if (not math.isfinite(stamp) or not math.isfinite(now)
                or not 0 <= now - stamp <= MAX_AGE):
            raise ValueError("Stack decision is stale")
        if stamp <= self.last_stamp:
            raise ValueError("Stack camera frame did not advance")
        self.last_stamp = stamp
        probabilities = scores(reply)
        if probabilities["unsafe"] >= 0.8:
            return "unsafe", probabilities
        complete = probabilities["placement"] >= 0.90 and probabilities["release"] >= 0.95
        return ("candidate" if complete else "continue"), probabilities

    async def check(self, readers):
        self.check_started_at = time.monotonic()
        body, stamp = await asyncio.to_thread(self.snapshot, readers)
        reply, latency = await asyncio.to_thread(self.backend.decisions, body)
        now = time.monotonic()
        action, probabilities = self.evaluate(reply, stamp, now)
        with self.output.open("a") as stream:
            stream.write(json.dumps({"frame_time": stamp, "result_time": now,
                                     "age_s": now - stamp, "latency_s": latency,
                                     "action": action, **probabilities}) + "\n")
        if action == "candidate":
            self.candidate_stamp = stamp
        self.check_started_at = None
        return action

    async def watch_candidate(self, readers, done):
        try:
            while not done.is_set():
                action = await self.check(readers)
                if action != "continue":
                    return action
                await asyncio.sleep(0.1)
            return "ended"
        except Exception as error:
            return f"monitor unavailable: {type(error).__name__}: {error}"

    async def confirm_held(self, readers, held_at):
        # The first positive vote already stopped motion. Never run extra policy
        # rows just to collect the second vote, and never resume after this latch.
        await asyncio.sleep(max(0, held_at + SETTLE_S - time.monotonic()))
        try:
            action = await self.check(readers)
            if action == "candidate" and self.last_stamp >= held_at + SETTLE_S - 0.1:
                return "complete"
            return "held for review: tower not confirmed after settling"
        except Exception as error:
            return f"held for review: {type(error).__name__}: {error}"


async def stop_policy(loop, runner, followers, hold_pair):
    """Finish the command writer, hold measured pose, then drain late inference.

    Inference cannot install its reply after the runner has been cancelled. Hold
    must happen BEFORE waiting for a slow API call or closing its client.
    """
    if runner is not None:
        runner.cancel()
        try:
            await runner
        except (asyncio.CancelledError, Exception):
            pass
    command_thread = loop._commands._thread
    await asyncio.to_thread(loop._commands.stop)
    if command_thread is not None:
        await asyncio.to_thread(command_thread.join)
    loop._buffer.clear()
    await asyncio.to_thread(hold_pair, followers)
    held_at = time.monotonic()
    if loop._refill is not None:
        try:
            await asyncio.shield(loop._refill)
        except Exception:
            pass
    return held_at


async def wait_for_park(page, reason):
    page.status = f"HOLDING: {reason}. Policy stopped; motors on. STOP explicitly parks both arms."
    while not page.stop.is_set():
        await asyncio.sleep(0.1)
