"""Record the simulator's true motion (tracked body poses and the arm's joint state) for offline rendering.

Joins a lab blueprint when CAPTURE_MOTION=1 (sim/lab_stack.py, sim/lab_cable.py); its ``body_poses`` and
``coordinator_joint_state`` inputs autoconnect to the sim's and the coordinator's outputs. Every message is appended
as one JSON line to ``out_path``, stamped with wall time (the same clock as the trial traces' ``ts``):

    {"t": 1794110457.75, "k": "j", "n": ["arm/joint1", ...], "q": [0.1, ...]}
    {"t": 1794110457.76, "k": "b", "b": {"cube_red": [x, y, z, qw, qx, qy, qz], ...}}

sim/render_replay.py turns a trial's window of this file into a high-quality video."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class MotionCaptureConfig(ModuleConfig):
    out_path: str
    flush_s: float = 1.0


class MotionCaptureModule(Module):
    config: MotionCaptureConfig
    body_poses: In[TFMessage]
    coordinator_joint_state: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._stream = None
        self._last_flush = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        path = Path(self.config.out_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = path.open("a", buffering=1 << 16)
        self.register_disposable(Disposable(self.body_poses.subscribe(self._on_poses)))
        self.register_disposable(Disposable(self.coordinator_joint_state.subscribe(self._on_joints)))

    def _write(self, row: dict) -> None:
        line = json.dumps(row, separators=(",", ":")) + "\n"
        with self._lock:
            if self._stream is None:
                return
            self._stream.write(line)
            now = time.time()
            if now - self._last_flush > self.config.flush_s:
                self._stream.flush()
                self._last_flush = now

    def _on_poses(self, msg: TFMessage) -> None:
        bodies = {}
        for tr in msg.transforms:
            p, r = tr.translation, tr.rotation
            bodies[tr.child_frame_id] = [round(v, 6) for v in (p.x, p.y, p.z, r.w, r.x, r.y, r.z)]
        if bodies:
            self._write({"t": time.time(), "k": "b", "b": bodies})

    def _on_joints(self, msg: JointState) -> None:
        self._write({"t": time.time(), "k": "j", "n": list(msg.name), "q": [round(float(v), 6) for v in msg.position]})

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self._stream is not None:
                self._stream.close()
                self._stream = None
        super().stop()
