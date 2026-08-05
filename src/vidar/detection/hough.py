from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from vidar.config import AppConfig, ColorRange


@dataclass(frozen=True)
class RawCircle:
    cx: float
    cy: float
    radius: float


def _interior_score(frame_bgr: np.ndarray, cx: float, cy: float, radius: float, bright: int, spread: int) -> float:
    ri = int(radius)
    icx, icy = int(round(cx)), int(round(cy))
    r2 = radius * radius
    inside = 0
    matched = 0

    h, w = frame_bgr.shape[:2]
    for dy in range(-ri, ri + 1):
        for dx in range(-ri, ri + 1):
            if dx * dx + dy * dy > r2:
                continue
            x, y = icx + dx, icy + dy
            if x < 0 or y < 0 or x >= w or y >= h:
                continue
            inside += 1
            b, g, r = frame_bgr[y, x]
            mx, mn = max(r, g, b), min(r, g, b)
            if mx >= bright and mx - mn <= spread:
                matched += 1

    return 0.0 if inside == 0 else matched / inside


def detect_hough_balls(
    frame: np.ndarray,
    config: AppConfig,
    min_radius: int | None = None,
    max_radius: int | None = None,
) -> list[RawCircle]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    min_r = min_radius if min_radius is not None else config.hough_min_radius
    max_r = max_radius if max_radius is not None else config.hough_max_radius

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=config.hough_dp,
        minDist=config.hough_min_dist,
        param1=config.hough_param1,
        param2=config.hough_param2,
        minRadius=min_r,
        maxRadius=max_r,
    )

    if circles is None:
        return []

    out: list[RawCircle] = []
    for cx, cy, r in circles[0]:
        area = np.pi * r * r
        if area < config.hough_min_area:
            continue
        interior = _interior_score(
            frame,
            float(cx),
            float(cy),
            float(r),
            config.hough_interior_bright,
            config.hough_interior_spread,
        )
        if interior < config.hough_min_interior:
            continue
        out.append(RawCircle(float(cx), float(cy), float(r)))

    return out
