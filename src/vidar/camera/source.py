from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import cv2
import numpy as np
import zmq


class FrameSource(ABC):
    @abstractmethod
    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        raise NotImplementedError

    @abstractmethod
    def release(self) -> None:
        raise NotImplementedError


class UsbFrameSource(FrameSource):
    def __init__(
        self,
        device_id: int,
        width: int,
        height: int,
        fps: int,
    ) -> None:
        self._capture = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        return self._capture.read()

    def release(self) -> None:
        self._capture.release()


class StreamFrameSource(FrameSource):
    """Receives JPEG frames from the host camera bridge over ZMQ."""

    def __init__(self, port: int) -> None:
        context = zmq.Context.instance()
        self._socket = context.socket(zmq.SUB)
        self._socket.connect(f"tcp://host.docker.internal:{port}")
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.setsockopt(zmq.RCVTIMEO, 2000)

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        try:
            payload = self._socket.recv()
        except zmq.Again:
            return False, None

        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return False, None
        return True, frame

    def release(self) -> None:
        self._socket.close()


class MockFrameSource(FrameSource):
    """Synthetic scene for dev when no camera is attached."""

    def __init__(self, width: int, height: int) -> None:
        self._width = width
        self._height = height
        self._tick = 0

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        frame[:] = (30, 30, 30)

        # Moving yellow blob = game element (sized for low-res pipeline)
        x = int((self._tick * 4) % (self._width - 80))
        cv2.rectangle(frame, (x, self._height // 2 - 20), (x + 60, self._height // 2 + 20), (0, 220, 220), -1)

        # Static red blob = robot/obstacle
        cv2.circle(frame, (self._width // 4, self._height // 3), 36, (0, 0, 220), -1)

        self._tick += 1
        return True, frame

    def release(self) -> None:
        return
