"""Two independent Dream sessions, one selected arm from each, one paired SDK executor."""
from concurrent.futures import ThreadPoolExecutor, wait
import copy
import json
import logging
import threading
import time

import numpy as np
from dream.pacing import Pacer


PROMPTS = {
    "right": (
        "Use the right arm to unplug a cable connector from the power strip. "
        "Choose a plugged connector on the right arm's side, separate from the left arm's target. "
        "Approach the rigid plug housing sideways from its cable-facing side. Keep the gripper's "
        "long fingers and wrist-to-fingertip axis horizontal and parallel to the tabletop, "
        "0 degrees above the table and 90 degrees from vertical, throughout the approach and grasp. "
        "Grasp the rigid plastic plug housing, then pull the plug out along its insertion axis. "
        "Do not pull on the wire. The left arm has its own controller and may move: keep clear of "
        "its observed position, do not cross arms, and do not grasp a connector held by the left arm."
    ),
    "left": (
        "Use the left arm to unplug a cable connector from the power strip. "
        "Choose a plugged connector on the left arm's side, separate from the right arm's target. "
        "Approach the upper side of the rigid plastic plug housing from above. Grasp the upper "
        "plastic housing opposite the end inserted into the socket, then pull the plug out along "
        "its insertion axis. Do not grasp or pull the flexible wire. The right arm has its own "
        "controller and may move: keep clear of its observed position, do not cross arms, and do "
        "not grasp a connector held by the right arm."
    ),
}
KEYS = {"left": "q_l_chunk", "right": "q_r_chunk"}


class DualClient:
    """Parallel API calls with separate prompts/windows/sessions and SDK-owned motor playback.

    A refill waits for both replies before publishing a pair. A failed side publishes neither.
    The slower reply sets the playback pace, using the SDK's own Pacer and actual paired RTT.
    """

    @classmethod
    def open(cls, policy, output, wire, horizon, right_checkpoint=None, training_archive=None):
        from robots import run
        clients = {}
        try:
            if right_checkpoint:
                from fine_tuned_client import RightFineTuneClient
                left_policy = copy.deepcopy(policy)
                left_policy.update(action_hz=20, sync_above_ms=900)
                clients["left"] = run.open_client(left_policy)
                clients["right"] = RightFineTuneClient.open(right_checkpoint, training_archive, output)
                instance = cls(clients, policy, output, wire, horizon)
                instance.source_hz = {"left": 20, "right": clients["right"].info["fps"]}
                instance.models["right"] = right_checkpoint
                instance.prompts["right"] = "Using the right arm, unplug all the cables from the cable set."
                return instance
            for side in KEYS:
                clients[side] = run.open_client(copy.deepcopy(policy))
            return cls(clients, policy, output, wire, horizon)
        except BaseException:
            for client in clients.values():
                client.close()
            raise

    def __init__(self, clients, policy, output, wire, horizon):
        self.clients = clients
        self.output = output
        self.wire = wire
        self.horizon = horizon
        self.action_hz = policy["action_hz"]
        self.source_hz = {s: self.action_hz for s in KEYS}
        self.models = {s: policy.get("checkpoint", "test") for s in KEYS}
        self.prompts = dict(PROMPTS)
        self._pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dream-arm")
        self._lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._closed = False
        self._pair = 0
        self.calls = {side: 0 for side in KEYS}
        self.succeeded = {side: 0 for side in KEYS}
        self.events = []
        self._session_ids = {}
        self.last_timing = None
        self._pacer = Pacer(action_hz=policy["action_hz"],
                            sync_above_ms=policy["sync_above_ms"])

    @property
    def session_id(self):
        # The ordinary SDK log has one string slot; keep the two real IDs in our separate log.
        return json.dumps({s: c.session_id for s, c in self.clients.items()}, sort_keys=True)

    @property
    def mode(self):
        return self._pacer.mode

    def refill_due(self, executed, remaining):
        return self._pacer.refill_due(executed, remaining)

    def wait_for_ik(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        return all(client.wait_for_ik(max(0, deadline - time.monotonic()))
                   for client in self.clients.values())

    def set_observation(self, frames, observation, *, timestamp=None):
        for side, client in self.clients.items():
            client.set_observation(frames, {**observation, "prompt": self.prompts[side]},
                                   timestamp=timestamp)

    def _ask_side(self, side, pair, commanded, barrier):
        barrier.wait(timeout=10)
        started = time.monotonic()
        client = self.clients[side]
        event = {"pair": pair, "arm": side, "prompt": self.prompts[side], "model": self.models[side],
                 "started_monotonic_s": started}
        with self._lock:
            self.calls[side] += 1
        try:
            reply = client.get_action(commanded=commanded)
            with self._lock:
                self.succeeded[side] += 1
            event["status"] = "success"
            return reply
        except BaseException as error:
            event.update(status="error", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            event.update(finished_monotonic_s=time.monotonic(), session_id=client.session_id,
                         timing=client.last_timing)
            event["elapsed_ms"] = (event["finished_monotonic_s"] - started) * 1000
            with self._lock:
                self._session_ids[side] = event["session_id"]
                self.events.append(event)
                with (self.output / "arm-api-calls.jsonl").open("a") as stream:
                    stream.write(json.dumps(event, default=str) + "\n")

    def get_action(self, *, commanded=None):
        if not self._request_lock.acquire(blocking=False):
            raise RuntimeError("A paired inference is already running")
        try:
            if self._closed:
                raise RuntimeError("Dual client is closed")
            self._pair += 1
            pair = self._pair
            started = time.monotonic()
            barrier = threading.Barrier(2)
            jobs = {s: self._pool.submit(self._ask_side, s, pair, commanded, barrier) for s in KEYS}
            # Always drain both jobs, including when one fails, before releasing/resetting clients.
            wait(jobs.values())
            replies = {s: job.result() for s, job in jobs.items()}
            selected = {key: np.asarray(replies[s][key], dtype=float).copy() for s, key in KEYS.items()}
            if any(hz != self.action_hz for hz in self.source_hz.values()):
                from fine_tuned_client import resample_rows
                selected = {key: resample_rows(selected[key], self.source_hz[s], self.action_hz,
                                               self.horizon) for s, key in KEYS.items()}
            if any(not np.isfinite(rows).all() for rows in selected.values()):
                raise ValueError("A selected arm reply contains non-finite commands")
            self.wire.from_dream(selected, started, self.horizon)
            elapsed = time.monotonic() - started
            self._pacer.record(elapsed)
            # Per-stream timing stays in arm-api-calls.jsonl; this is the paired wall time.
            self.last_timing = None
            np.savez_compressed(self.output / f"selected-{pair:04d}.npz", **selected)
            logging.info("Paired inference %d: separate left/right sessions, %.0f ms, %s",
                         pair, elapsed * 1000, self.mode)
            return selected
        finally:
            self._request_lock.release()

    def reset_episode(self):
        for client in self.clients.values():
            client.reset_episode()

    def stats(self):
        return {"paired_api_requests": self._pair, "api_calls": sum(self.calls.values()),
                "successful_api_calls": sum(self.succeeded.values()),
                "api_calls_by_arm": dict(self.calls), "successful_api_calls_by_arm": dict(self.succeeded),
                "session_ids": dict(self._session_ids), "models_by_arm": self.models,
                "native_action_hz": self.source_hz, "playback_hz": self.action_hz}

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._pool.shutdown(wait=True)
        # Release each separate session even if the other close fails.
        errors = []
        for client in self.clients.values():
            try:
                client.close()
            except Exception as error:
                errors.append(error)
        if errors:
            raise errors[0]
