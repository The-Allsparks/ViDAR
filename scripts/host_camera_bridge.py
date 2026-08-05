#!/usr/bin/env python3
"""Host-side USB camera bridge for Windows Docker Desktop.

Docker on Windows cannot attach /dev/video* directly. This script captures
from a local USB camera and publishes JPEG frames over ZMQ for the container.

Usage:
  pip install -r requirements-host.txt
  python scripts/host_camera_bridge.py --device 0 --port 5555
"""

from __future__ import annotations

import argparse
import time

import cv2
import zmq


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish USB camera frames to ViDAR container")
    parser.add_argument("--device", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ bind port")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device, cv2.CAP_DSHOW)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)

    if not capture.isOpened():
        raise SystemExit(f"Could not open camera device {args.device}")

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://0.0.0.0:{args.port}")

    print(f"[bridge] publishing camera {args.device} on tcp://0.0.0.0:{args.port}")
    frame_interval = 1.0 / max(args.fps, 1)

    try:
        while True:
            started = time.perf_counter()
            ok, frame = capture.read()
            if not ok:
                time.sleep(0.05)
                continue

            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue

            socket.send(encoded.tobytes())

            elapsed = time.perf_counter() - started
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
    except KeyboardInterrupt:
        print("[bridge] stopped")
    finally:
        capture.release()
        socket.close()


if __name__ == "__main__":
    main()
