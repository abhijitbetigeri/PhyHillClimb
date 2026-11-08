"""Completion decisions and actual SDK command-worker shutdown; no hardware."""
import asyncio
import collections
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, AsyncMock

import numpy as np
from robots import control, types
from stack_completion import StackMonitor, stop_policy, wait_for_park


def reply(placed=0.99, released=0.99, unsafe=0):
    return {"answers": [
        {"name": "placement", "type": "choice", "probabilities": [
            {"value": "placed", "probability": placed}]},
        {"name": "release", "type": "choice", "probabilities": [
            {"value": "clear", "probability": released}]},
        {"name": "unsafe", "type": "predicate", "probability": unsafe}]}


def monitor():
    result = object.__new__(StackMonitor)
    result.last_stamp = -1
    result.candidate_stamp = None
    result.check_started_at = None
    return result


class DecisionTests(unittest.TestCase):
    def test_two_blocks_or_held_third_cannot_complete(self):
        for body in (reply(placed=0.1), reply(released=0.2), reply(placed=0.89)):
            self.assertEqual(monitor().evaluate(body, 10, 10.4)[0], "continue")

    def test_first_positive_requests_hold_without_waiting_for_second_vote(self):
        self.assertEqual(monitor().evaluate(reply(), 10, 10.4)[0], "candidate")

    def test_stale_duplicate_missing_or_refused_answers_are_rejected(self):
        for body, stamp, now in [(reply(), 10, 12.1), (reply(), 10, 9),
                                 ({"answers": []}, 10, 10.2), (reply(float("nan")), 10, 10.2)]:
            with self.assertRaises(ValueError):
                monitor().evaluate(body, stamp, now)
        gate = monitor()
        gate.evaluate(reply(), 10, 10.2)
        with self.assertRaises(ValueError):
            gate.evaluate(reply(), 10, 10.5)

    def test_unsafe_has_priority_over_success(self):
        self.assertEqual(monitor().evaluate(reply(unsafe=0.99), 10, 10.1)[0], "unsafe")

    def test_watchdog_stops_motion_even_before_a_stalled_api_thread_returns(self):
        gate = monitor()
        self.assertFalse(gate.overdue(10))
        gate.check_started_at = 10
        self.assertFalse(gate.overdue(11.9))
        self.assertTrue(gate.overdue(12.01))


class HoldingTests(unittest.IsolatedAsyncioTestCase):
    async def test_requires_fresh_post_hold_confirmation(self):
        gate = monitor()
        held_at = time.monotonic() - 5
        gate.check = AsyncMock(return_value="candidate")
        gate.last_stamp = held_at - 0.1
        self.assertIn("review", await gate.confirm_held([], held_at))
        gate.last_stamp = time.monotonic()
        self.assertEqual(await gate.confirm_held([], held_at), "complete")
        gate.check = AsyncMock(return_value="continue")
        self.assertIn("review", await gate.confirm_held([], held_at))

    async def test_failed_confirmation_stays_held(self):
        gate = monitor()
        gate.check = AsyncMock(side_effect=TimeoutError("fixture"))
        self.assertIn("review", await gate.confirm_held([], time.monotonic() - 5))

    async def test_holds_before_slow_inference_finishes_and_never_writes_late_reply(self):
        arms = {s: Mock() for s in ("left", "right")}
        cmd = types.ArmCommand(np.zeros(6), 0.5)
        for arm in arms.values():
            arm.read.return_value = types.ArmState(np.zeros(6), np.zeros(6), 0.5, None, None)
        writer = control._CommandWriter(arms, command_hz=250, command_interp_max_s=0.1,
                                        clock=time.monotonic)
        writer.start()
        pending = asyncio.get_running_loop().create_future()
        loop = SimpleNamespace(_commands=writer, _buffer=collections.deque([1, 2, 3]), _refill=pending)

        async def policy():
            while True:
                writer.submit(types.Action(cmd, cmd), time.monotonic())
                await asyncio.sleep(0.01)

        runner = asyncio.create_task(policy())
        await asyncio.sleep(0.04)
        held = threading.Event()
        stop = asyncio.create_task(stop_policy(loop, runner, arms, lambda pair: held.set()))
        try:
            self.assertTrue(await asyncio.to_thread(held.wait, 1))
            self.assertFalse(stop.done(), "Must drain inference after holding")
            self.assertFalse(loop._buffer)
            counts = [a.write.call_count for a in arms.values()]
            pending.set_result({"late": "policy response"})
            await stop
            await asyncio.sleep(0.04)
            self.assertEqual(counts, [a.write.call_count for a in arms.values()])
            self.assertTrue(runner.cancelled())
            for arm in arms.values():
                arm.disengage.assert_not_called()
        finally:
            writer.stop()
            runner.cancel()

    async def test_success_hold_waits_for_explicit_park_without_timeout_or_resume(self):
        page = SimpleNamespace(stop=threading.Event(), status="")
        task = asyncio.create_task(wait_for_park(page, "complete"))
        await asyncio.sleep(0.03)
        self.assertFalse(task.done())
        self.assertIn("motors on", page.status)
        page.stop.set()
        await task


if __name__ == "__main__":
    unittest.main()
