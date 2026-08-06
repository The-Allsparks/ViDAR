#!/usr/bin/env python3
"""Bench Phase 0 metrics — element FPS and detection sanity (no robot required).

Run: python scripts/bench_metrics.py
Writes a summary to stdout; paste results into docs/validation-log.md Phase 0.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from vidar.config_loader import default_robot, default_season, parse_season
from vidar.contour_processor import ContourProcessor


FRAME_W, FRAME_H = 640, 480
WARMUP = 20
FRAMES = 200
TARGET_FPS = 15.0


def _bench_season():
    """Minimal season aligned with the synthetic frame (detection sanity only)."""
    return parse_season(
        {
            "seasonId": "bench-synthetic",
            "fusion": {"minElementConfidence": 0.20},
            "elements": [
                {
                    "id": "marker",
                    "label": "Marker",
                    "diameter": 2.8,
                    "detector": "color_blob",
                    "hsv": {
                        "hMin": 20,
                        "hMax": 40,
                        "sMin": 100,
                        "sMax": 255,
                        "vMin": 100,
                        "vMax": 255,
                    },
                    "filters": {
                        "minAreaPx": 14,
                        "maxAreaPx": 4000,
                        "minCircularity": 0.45,
                        "minInteriorScore": 0.0,
                    },
                    "interior": {"brightMin": 58, "spreadMax": 65},
                    "morphology": {"closePasses": 1},
                }
            ],
            "plates": [],
        }
    )


def _synthetic_frame() -> np.ndarray:
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    # Saturated yellow — matches bench season HSV; uniform enough for interior gate
    cv2.circle(frame, (320, 350), 18, (0, 220, 220), -1)
    return frame


def main() -> int:
    season = default_season(ROOT)
    bench_season = _bench_season()
    robot = default_robot(ROOT)
    profile = robot.cameras[0].profile
    processor = ContourProcessor(profile, profile.name, season, roi_scale=0.5)
    sanity_processor = ContourProcessor(profile, profile.name, bench_season, roi_scale=0.5)

    frame = _synthetic_frame()
    for _ in range(WARMUP):
        processor.detect(frame)

    started = time.perf_counter()
    for _ in range(FRAMES):
        processor.detect(frame)
    elapsed = time.perf_counter() - started
    fps = FRAMES / elapsed if elapsed > 0 else 0.0

    sanity = sanity_processor.detect(frame)
    elements = sum(1 for d in sanity if d.category == "element")

    print("ViDAR bench metrics (Phase 0 — no robot)")
    print(f"  season (FPS): {season.season_id}")
    print(f"  season (detection sanity): {bench_season.season_id}")
    print(f"  camera: {profile.name}")
    print(f"  frames: {FRAMES} (+{WARMUP} warmup)")
    print(f"  element FPS: {fps:.1f}  target: >={TARGET_FPS}")
    print(f"  synthetic element hits: {elements}  target: >=1")
    print(f"  PASS element FPS: {'yes' if fps >= TARGET_FPS else 'NO'}")
    print(f"  PASS detection sanity: {'yes' if elements >= 1 else 'NO'}")
    if elements == 0:
        print("  NOTE: sanity season/frame mismatch — adjust _bench_season() or _synthetic_frame()")

    categories = {d.category for d in sanity}
    print(f"  categories seen: {sorted(categories)}")
    print()
    print("Paste into docs/validation-log.md Phase 0 when run on your machine.")

    return 0 if fps >= TARGET_FPS and elements >= 1 else 1


if __name__ == "__main__":
    raise SystemExit(main())
