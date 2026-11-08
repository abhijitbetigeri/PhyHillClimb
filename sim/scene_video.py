"""Save the simulator's fixed scene camera as numbered JPEG frames, so a trial can be watched as a video.

Joins the lab blueprint beside the MuJoCo sim module; its ``scene_image`` input autoconnects to the sim's
``scene_image`` output. Frames go to ``out_dir/<index>_<timestamp>.jpg`` at up to ``fps`` per second; turn them
into a video with ffmpeg afterwards (hfjobs/make_video.sh)."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class SceneVideoConfig(ModuleConfig):
    out_dir: str
    fps: float = 10.0
    quality: int = 88


class SceneVideoModule(Module):
    config: SceneVideoConfig
    scene_image: In[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._last = 0.0
        self._index = 0

    @rpc
    def start(self) -> None:
        super().start()
        Path(self.config.out_dir).mkdir(parents=True, exist_ok=True)
        self.register_disposable(Disposable(self.scene_image.subscribe(self._on_frame)))

    def _on_frame(self, img: Image) -> None:
        now = time.time()
        with self._lock:
            if now - self._last < 1.0 / max(self.config.fps, 0.1):
                return
            self._last = now
            self._index += 1
            index = self._index
        data = img.data
        if img.format == ImageFormat.RGB:
            data = data[..., ::-1]  # cv2 writes BGR
        try:
            import cv2

            cv2.imwrite(str(Path(self.config.out_dir) / f"{index:06d}_{now:.3f}.jpg"), data,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.quality)])
        except Exception:  # noqa: BLE001  (pillow fallback)
            from PIL import Image as PilImage

            PilImage.fromarray(data[..., ::-1]).save(Path(self.config.out_dir) / f"{index:06d}_{now:.3f}.jpg",
                                                     quality=int(self.config.quality))

    @rpc
    def stop(self) -> None:
        super().stop()
