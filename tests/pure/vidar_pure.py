"""Pure-Python mirror of ViDAR geometry/ROI/motion logic for offline tests."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class RoiRect:
    x: int
    y: int
    width: int
    height: int
    enabled: bool = True

    @staticmethod
    def lower_fraction(frame_w: int, frame_h: int, fraction: float) -> "RoiRect":
        h = max(1, int(round(frame_h * fraction)))
        return RoiRect(0, frame_h - h, frame_w, h)

    @staticmethod
    def upper_fraction(frame_w: int, frame_h: int, fraction: float) -> "RoiRect":
        h = max(1, int(round(frame_h * fraction)))
        return RoiRect(0, 0, frame_w, h)

    def to_full_x(self, local_x: float) -> float:
        return local_x + self.x

    def to_full_y(self, local_y: float) -> float:
        return local_y + self.y

    def clamped(self, frame_w: int, frame_h: int) -> "RoiRect":
        cx = max(0, min(self.x, frame_w - 1))
        cy = max(0, min(self.y, frame_h - 1))
        cw = min(self.width, frame_w - cx)
        ch = min(self.height, frame_h - cy)
        return RoiRect(cx, cy, max(1, cw), max(1, ch), self.enabled)


@dataclass
class CameraRoiConfig:
    ball_lower_fraction: float = 0.65
    plate_start_fraction: float = 0.30
    plate_band_fraction: float = 0.40
    tag_upper_fraction: float = 0.65

    def ball_roi(self, frame_w: int, frame_h: int) -> RoiRect:
        return RoiRect.lower_fraction(frame_w, frame_h, self.ball_lower_fraction)

    def tag_roi(self, frame_w: int, frame_h: int) -> RoiRect:
        return RoiRect.upper_fraction(frame_w, frame_h, self.tag_upper_fraction)


@dataclass
class RangeEstimate:
    source: str
    distance_in: float
    weight: float
    uncertainty_in: float
    rejection_reason: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        return self.weight > 0 and not math.isnan(self.distance_in) and self.rejection_reason is None


@dataclass
class RangeResult:
    distance_in: float
    uncertainty_in: float
    confidence: float
    sources: List[RangeEstimate]

    @property
    def is_valid(self) -> bool:
        return not math.isnan(self.distance_in) and self.confidence > 0


def distance_from_size_inches(diameter_in: float, focal_px: float, radius_px: float) -> float:
    if radius_px <= 0 or focal_px <= 0:
        return float("nan")
    return (diameter_in * focal_px) / (2.0 * radius_px)


def distance_from_width_inches(physical_width_in: float, focal_px: float, pixel_width: float) -> float:
    if pixel_width <= 0 or focal_px <= 0:
        return float("nan")
    return (physical_width_in * focal_px) / pixel_width


def build_ball_size_estimate(d_size, radius_px, circle_fit_quality, partial, boundary) -> RangeEstimate:
    if math.isnan(d_size) or d_size <= 0:
        return RangeEstimate("SIZE", float("nan"), 0, float("nan"), "invalid_distance")
    unc = d_size * 0.08 / max(0.3, circle_fit_quality)
    if partial:
        unc *= 1.8
    if boundary:
        unc *= 2.0
    w = 1.0 / (unc * unc)
    return RangeEstimate("SIZE", d_size, w, unc)


def build_floor_estimate(d_floor, cy_px, horizon_conf, near_horizon) -> RangeEstimate:
    if math.isnan(d_floor) or d_floor <= 0:
        return RangeEstimate("FLOOR", float("nan"), 0, float("nan"), "invalid_lut")
    if near_horizon:
        return RangeEstimate("FLOOR", float("nan"), 0, float("nan"), "near_horizon")
    unc = d_floor * 0.12 / max(0.2, horizon_conf)
    w = 1.0 / (unc * unc)
    return RangeEstimate("FLOOR", d_floor, w, unc)


def build_plate_width_estimate(d_width, pixel_width, rectangularity, white_ratio,
                               partial, touches, rotation_penalty) -> RangeEstimate:
    if math.isnan(d_width) or pixel_width < 20:
        return RangeEstimate("PLATE_WIDTH", float("nan"), 0, float("nan"), "invalid_width")
    unc = d_width * 0.10 * (2.0 - min(1.0, rectangularity)) * (1.0 + rotation_penalty)
    w = 1.0 / (unc * unc)
    return RangeEstimate("PLATE_WIDTH", d_width, w, unc)


def fuse_range_weighted(*estimates: RangeEstimate) -> RangeResult:
    valid = [e for e in estimates if e and e.is_valid]
    if not valid:
        return RangeResult(float("nan"), float("nan"), 0, list(estimates))
    ws = sum(e.weight for e in valid)
    dist = sum(e.weight * e.distance_in for e in valid) / ws
    unc = math.sqrt(sum(e.weight * e.uncertainty_in ** 2 for e in valid) / ws)
    conf = min(1.0, ws / len(valid))
    if len(valid) >= 2:
        max_diff = 0
        for a in valid:
            for b in valid:
                if a is b:
                    continue
                denom = max(a.distance_in, b.distance_in)
                if denom > 0:
                    max_diff = max(max_diff, abs(a.distance_in - b.distance_in) / denom)
        if max_diff > 0.28:
            conf *= max(0.2, 1.0 - max_diff)
    return RangeResult(dist, unc, conf, valid)


class MotionTransform:
    def __init__(self, delta_x_in: float, delta_y_in: float, delta_heading_deg: float):
        self.delta_x_in = delta_x_in
        self.delta_y_in = delta_y_in
        self.delta_heading_deg = delta_heading_deg

    def transform_point(self, prev_x: float, prev_y: float) -> Tuple[float, float]:
        rad = math.radians(-self.delta_heading_deg)
        cos_v = math.cos(rad)
        sin_v = math.sin(rad)
        rx = prev_x * cos_v - prev_y * sin_v
        ry = prev_x * sin_v + prev_y * cos_v
        return rx - self.delta_x_in, ry - self.delta_y_in

    @staticmethod
    def from_odom_delta(px, py, ph, cx, cy, ch) -> "MotionTransform":
        dx = cx - px
        dy = cy - py
        dh = normalize_deg(ch - ph)
        rad = math.radians(-ph)
        cos_v = math.cos(rad)
        sin_v = math.sin(rad)
        robot_dx = dx * cos_v - dy * sin_v
        robot_dy = dx * sin_v + dy * cos_v
        return MotionTransform(robot_dx, robot_dy, dh)


def normalize_deg(deg: float) -> float:
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


class LocalizationFusionPure:
    MAX_TRANSLATION = 18.0
    MAX_HEADING = 25.0
    MAX_AGE_MS = 500

    def __init__(self):
        self.last_fused: Optional[Tuple[float, float, float]] = None

    def fused_field_pose_now(self, decoded_tag, scout_obs) -> Optional[Tuple[float, float, float]]:
        if scout_obs is not None and decoded_tag is None:
            return self.last_fused
        if decoded_tag is None:
            return self.last_fused
        if decoded_tag.get("id", -1) < 0:
            return self.last_fused
        if decoded_tag.get("age_ms", 0) > self.MAX_AGE_MS:
            return self.last_fused
        pose = decoded_tag["pose"]
        if self.last_fused is not None:
            dx = pose[0] - self.last_fused[0]
            dy = pose[1] - self.last_fused[1]
            if math.hypot(dx, dy) > self.MAX_TRANSLATION:
                return self.last_fused
        self.last_fused = pose
        return self.last_fused

    def would_scout_alter_pose(self, scout) -> bool:
        return False
