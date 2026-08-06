from __future__ import annotations

import json
import socket
import time
from typing import Iterable

from vidar.detection.pipeline import Detection


class TelemetryPublisher:
    """Publishes detection snapshots for FTC OpMode / dashboard consumers."""

    def __init__(self, port: int) -> None:
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def publish(self, detections: Iterable[Detection], fps: float) -> None:
        payload = {
            "ts": time.time(),
            "fps": round(fps, 1),
            "detections": [
                {
                    "label": det.label,
                    "category": det.category,
                    "camera": det.camera_index,
                    "cx": round(det.cx, 1),
                    "cy": round(det.cy, 1),
                    "area": round(det.area, 1),
                    "range": None if det.range is None else round(det.range, 1),
                    "confidence": None if det.confidence is None else round(det.confidence, 2),
                }
                for det in detections
            ],
        }
        message = json.dumps(payload).encode("utf-8")
        self._socket.sendto(message, ("255.255.255.255", self._port))

    def close(self) -> None:
        self._socket.close()
