from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from vidar.config import AppConfig
from vidar.camera.source import FrameSource, MockFrameSource, StreamFrameSource, UsbFrameSource


@dataclass
class CameraSlot:
    index: int
    source: FrameSource


class CameraManager:
    """Manages 1-4 camera inputs with a unified read API."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._slots: list[CameraSlot] = []
        self._build_sources()

    def _build_sources(self) -> None:
        count = min(self._config.camera_count, self._config.max_workers)

        for index in range(count):
            source = self._create_source(index)
            self._slots.append(CameraSlot(index=index, source=source))

    def _create_source(self, index: int) -> FrameSource:
        mode = self._config.camera_mode
        width = self._config.capture_width
        height = self._config.capture_height
        fps = self._config.fps_target

        if mode == "usb":
            device_id = self._config.device_ids[index] if index < len(self._config.device_ids) else index
            return UsbFrameSource(device_id, width, height, fps)

        if mode == "stream":
            port = self._config.stream_ports[index] if index < len(self._config.stream_ports) else 5555 + index
            return StreamFrameSource(port)

        return MockFrameSource(width, height)

    @property
    def count(self) -> int:
        return len(self._slots)

    def read_all(self) -> list[tuple[int, bool, Optional[np.ndarray]]]:
        frames: list[tuple[int, bool, Optional[np.ndarray]]] = []
        for slot in self._slots:
            ok, frame = slot.source.read()
            frames.append((slot.index, ok, frame))
        return frames

    def release(self) -> None:
        for slot in self._slots:
            slot.source.release()
