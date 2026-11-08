"""A stand-in camera: publishes a generated frame on ``color_image`` so the program's keyframes work with no webcam
(the echo-mock rehearsal, or a station whose wrist camera another process holds)."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class PatternCameraConfig(ModuleConfig):
    width: int = 640
    height: int = 480
    fps: float = Field(default=5.0, gt=0.0)


class PatternCamera(Module):
    """A moving gradient with the frame counter burnt in, ``fps`` times a second."""

    config: PatternCameraConfig
    color_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_frames, daemon=True, name="pattern-camera")
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    def _run_frames(self) -> None:
        h, w = self.config.height, self.config.width
        ramp = np.linspace(0, 255, w, dtype=np.float32)[None, :].repeat(h, axis=0)
        n = 0
        while not self._stop.wait(1.0 / self.config.fps):
            n += 1
            frame = np.zeros((h, w, 3), dtype=np.uint8)
            frame[..., 0] = ramp.astype(np.uint8)
            frame[..., 1] = np.roll(ramp, n * 8, axis=1).astype(np.uint8)
            frame[..., 2] = 64
            try:
                import cv2

                cv2.putText(frame, f"pattern camera {n}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            except Exception:
                pass
            self.color_image.publish(
                Image.from_numpy(frame, format=ImageFormat.RGB, frame_id="pattern_camera", ts=time.time())
            )
