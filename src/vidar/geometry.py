from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CameraProfile:
    name: str
    bearing_deg: float
    horizon_row_px: int
    focal_length_px: float
    floor_lut: tuple[tuple[float, float], ...]  # (cy, dist_in)


@dataclass(frozen=True)
class GeometryConfig:
    ball_diameter_in: float
    max_range_mismatch_ratio: float
    min_ball_confidence: float
    pickup_stop_in: float
    seek_max_range_in: float
    active_camera_index: int
    cameras: tuple[CameraProfile, ...]


@dataclass(frozen=True)
class BallObservation:
    cx: float
    cy: float
    radius_px: float
    range_in: float
    d_size_in: float
    d_floor_in: float
    confidence: float
    robot_x_in: float
    robot_y_in: float
    label: str
    camera_index: int


def distance_from_size_inches(diameter_in: float, focal_px: float, radius_px: float) -> float:
    if radius_px <= 0 or focal_px <= 0 or diameter_in <= 0:
        return math.nan
    return (diameter_in * focal_px) / (2.0 * radius_px)


def distance_from_floor_inches(cy_px: float, profile: CameraProfile) -> float:
    if not profile.floor_lut:
        return math.nan

    xs = [p[0] for p in profile.floor_lut]
    ys = [p[1] for p in profile.floor_lut]

    if cy_px <= xs[0]:
        return ys[0]
    if cy_px >= xs[-1]:
        return ys[-1]

    for i in range(len(xs) - 1):
        x0, x1 = xs[i], xs[i + 1]
        if x0 <= cy_px <= x1:
            t = (cy_px - x0) / (x1 - x0)
            return ys[i] + t * (ys[i + 1] - ys[i])
    return math.nan


def fuse_range_inches(d_size: float, d_floor: float) -> float:
    have_size = not math.isnan(d_size) and d_size > 0
    have_floor = not math.isnan(d_floor) and d_floor > 0
    if have_size and have_floor:
        return 0.5 * (d_size + d_floor)
    if have_size:
        return d_size
    if have_floor:
        return d_floor
    return math.nan


def range_confidence(d_size: float, d_floor: float, max_mismatch: float) -> float:
    have_size = not math.isnan(d_size) and d_size > 0
    have_floor = not math.isnan(d_floor) and d_floor > 0
    if not have_size and not have_floor:
        return 0.0
    if have_size ^ have_floor:
        return 0.55

    denom = max(d_size, d_floor)
    mismatch = abs(d_size - d_floor) / denom
    if mismatch > max_mismatch:
        return max(0.0, 0.35 * (1.0 - mismatch))
    return 1.0 - (mismatch / max_mismatch) * 0.35


def fuse_observation(
    cx: float,
    cy: float,
    radius_px: float,
    label: str,
    camera_index: int,
    geometry: GeometryConfig,
) -> BallObservation:
    profile = geometry.cameras[camera_index] if camera_index < len(geometry.cameras) else geometry.cameras[0]
    d_size = distance_from_size_inches(geometry.ball_diameter_in, profile.focal_length_px, radius_px)
    d_floor = distance_from_floor_inches(cy, profile)
    range_in = fuse_range_inches(d_size, d_floor)
    confidence = range_confidence(d_size, d_floor, geometry.max_range_mismatch_ratio)

    robot_x = math.nan
    robot_y = math.nan
    if not math.isnan(range_in):
        rad = math.radians(profile.bearing_deg)
        robot_x = range_in * math.cos(rad)
        robot_y = range_in * math.sin(rad)

    return BallObservation(
        cx=cx,
        cy=cy,
        radius_px=radius_px,
        range_in=range_in,
        d_size_in=d_size,
        d_floor_in=d_floor,
        confidence=confidence,
        robot_x_in=robot_x,
        robot_y_in=robot_y,
        label=label,
        camera_index=camera_index,
    )
