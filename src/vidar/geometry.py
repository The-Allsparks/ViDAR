from __future__ import annotations

import math

from vidar.models import ElementDetectorType, CameraProfile, ElementSpec, PlateSpec, SeasonConfig
from vidar.types import (
    ElementObservation,
    PlateObservation,
    RangeEstimate,
    RangeResult,
    RangeSource,
)

HOUGH_MIN_RADIUS = 8
MAX_RANGE_MISMATCH_RATIO = 0.28

# Re-export canonical types for backward-compatible imports.
__all__ = [
    "RangeSource",
    "RangeEstimate",
    "RangeResult",
    "ElementObservation",
    "PlateObservation",
    "HOUGH_MIN_RADIUS",
    "MAX_RANGE_MISMATCH_RATIO",
    "distance_from_size",
    "distance_from_width",
    "distance_from_floor",
    "build_size_estimate",
    "build_floor_estimate",
    "build_plate_width_estimate",
    "fuse_range_weighted",
    "ray_direction_robot_frame",
    "floor_point_in_robot",
    "robot_x",
    "robot_y",
    "compose_element_confidence",
    "compose_plate_confidence",
    "fuse_element_observation",
    # Java-parity aliases
    "distanceFromSize",
    "distanceFromWidth",
    "distanceFromFloor",
    "buildSizeEstimate",
    "buildFloorEstimate",
    "buildPlateWidthEstimate",
    "fuseRangeWeighted",
    "floorPointInRobot",
    "robotX",
    "robotY",
    "composeElementConfidence",
    "composePlateConfidence",
    "fuseElementObservation",
]


def distance_from_size(diameter: float, focal_px: float, radius_px: float) -> float:
    if radius_px <= 0 or focal_px <= 0 or diameter <= 0:
        return math.nan
    return (diameter * focal_px) / (2.0 * radius_px)


def distance_from_width(physical_width: float, focal_px: float, pixel_width: float) -> float:
    if pixel_width <= 0 or focal_px <= 0 or physical_width <= 0:
        return math.nan
    return (physical_width * focal_px) / pixel_width


def distance_from_floor(cy_px: float, profile: CameraProfile) -> float:
    xs = profile.floor_cy_px
    ys = profile.floor_dist
    if not xs or len(xs) != len(ys):
        return math.nan
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


def _profile_horizon_confidence(horizon_row_px: int) -> float:
    return max(0.3, 1.0 - horizon_row_px / 120.0)


def build_size_estimate(
    d_size: float,
    radius_px: float,
    circle_fit_quality: float,
    partial_occlusion: bool,
    touches_boundary: bool,
) -> RangeEstimate:
    if math.isnan(d_size) or d_size <= 0:
        return RangeEstimate.rejected(RangeSource.SIZE, "invalid_distance")
    if radius_px < HOUGH_MIN_RADIUS:
        return RangeEstimate.rejected(RangeSource.SIZE, "radius_too_small")
    base_uncertainty = d_size * 0.08 / max(0.3, circle_fit_quality)
    if partial_occlusion:
        base_uncertainty *= 1.8
    if touches_boundary:
        base_uncertainty *= 2.0
    weight = 1.0 / (base_uncertainty * base_uncertainty)
    return RangeEstimate(RangeSource.SIZE, d_size, weight, base_uncertainty)


def build_floor_estimate(
    d_floor: float,
    cy_px: float,
    horizon_confidence: float,
    near_horizon: bool,
) -> RangeEstimate:
    del cy_px
    if math.isnan(d_floor) or d_floor <= 0:
        return RangeEstimate.rejected(RangeSource.FLOOR, "invalid_lut")
    if near_horizon:
        return RangeEstimate.rejected(RangeSource.FLOOR, "near_horizon")
    base_uncertainty = d_floor * 0.12 / max(0.2, horizon_confidence)
    weight = 1.0 / (base_uncertainty * base_uncertainty)
    return RangeEstimate(RangeSource.FLOOR, d_floor, weight, base_uncertainty)


def build_plate_width_estimate(
    d_width: float,
    pixel_width: float,
    rectangularity: float,
    white_ratio: float,
    partial_visibility: bool,
    touches_roi_boundary: bool,
    rotation_penalty: float,
) -> RangeEstimate:
    if math.isnan(d_width) or d_width <= 0:
        return RangeEstimate.rejected(RangeSource.PLATE_WIDTH, "invalid_width")
    if pixel_width < 20:
        return RangeEstimate.rejected(RangeSource.PLATE_WIDTH, "width_too_small")
    base_uncertainty = d_width * 0.10
    base_uncertainty *= 2.0 - min(1.0, rectangularity)
    base_uncertainty *= 1.5 - min(0.5, white_ratio)
    if partial_visibility:
        base_uncertainty *= 1.6
    if touches_roi_boundary:
        base_uncertainty *= 1.4
    base_uncertainty *= 1.0 + rotation_penalty
    weight = 1.0 / (base_uncertainty * base_uncertainty)
    return RangeEstimate(RangeSource.PLATE_WIDTH, d_width, weight, base_uncertainty)


def fuse_range_weighted(
    *estimates: RangeEstimate | None,
    max_range_mismatch_ratio: float = MAX_RANGE_MISMATCH_RATIO,
) -> RangeResult:
    first_valid: RangeEstimate | None = None
    second_valid: RangeEstimate | None = None
    first_any: RangeEstimate | None = None
    second_any: RangeEstimate | None = None
    any_count = 0

    for est in estimates:
        if est is None:
            continue
        if any_count == 0:
            first_any = est
            any_count = 1
        elif any_count == 1:
            second_any = est
            any_count = 2
        if not est.is_valid:
            continue
        if first_valid is None:
            first_valid = est
        elif second_valid is None:
            second_valid = est

    if first_valid is None:
        if any_count == 0:
            return RangeResult.invalid()
        if any_count == 1:
            return RangeResult(float("nan"), float("nan"), 0.0, first_any, None, 1)
        return RangeResult(float("nan"), float("nan"), 0.0, first_any, second_any, 2)

    weight_sum = first_valid.weight
    weighted_dist = first_valid.weight * first_valid.distance
    variance_sum = first_valid.weight * first_valid.uncertainty * first_valid.uncertainty
    valid_count = 1

    if second_valid is not None:
        weight_sum += second_valid.weight
        weighted_dist += second_valid.weight * second_valid.distance
        variance_sum += second_valid.weight * second_valid.uncertainty * second_valid.uncertainty
        valid_count = 2

    if weight_sum <= 0:
        return RangeResult.invalid()

    fused = weighted_dist / weight_sum
    uncertainty = math.sqrt(variance_sum / weight_sum)

    disagreement_penalty = 1.0
    if second_valid is not None:
        denom = max(first_valid.distance, second_valid.distance)
        if denom > 0:
            max_diff = abs(first_valid.distance - second_valid.distance) / denom
            if max_diff > max_range_mismatch_ratio:
                disagreement_penalty = max(0.2, 1.0 - max_diff)

    confidence = min(1.0, (weight_sum / valid_count) * disagreement_penalty)
    return RangeResult(
        fused, uncertainty, confidence, first_valid, second_valid, valid_count
    )


def _normalize3(x: float, y: float, z: float) -> tuple[float, float, float]:
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (x / length, y / length, z / length)


def _rotate_x(v: tuple[float, float, float], rad: float) -> tuple[float, float, float]:
    c, s = math.cos(rad), math.sin(rad)
    return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)


def _rotate_z(v: tuple[float, float, float], rad: float) -> tuple[float, float, float]:
    c, s = math.cos(rad), math.sin(rad)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])


def ray_direction_robot_frame(cx: float, cy: float, profile: CameraProfile) -> tuple[float, float, float]:
    u = (cx - profile.principal_point_x) / profile.focal_length_px
    v = (cy - profile.principal_point_y) / profile.focal_length_y_px
    cam = _normalize3(u, v, 1.0)
    base = (cam[2], -cam[0], -cam[1])
    bearing_rad = math.radians(profile.bearing_deg + profile.mount_yaw_deg)
    panned = _rotate_z(base, bearing_rad)
    pitched = _rotate_x(panned, math.radians(profile.mount_pitch_deg))
    return _normalize3(*_rotate_z(pitched, math.radians(profile.mount_roll_deg)))


def floor_point_in_robot(
    cx: float, cy: float, slant_range: float, profile: CameraProfile
) -> tuple[float, float, float]:
    if math.isnan(slant_range) or slant_range <= 0:
        return (math.nan, math.nan, math.nan)
    direction = ray_direction_robot_frame(cx, cy, profile)
    return (
        profile.mount_x + slant_range * direction[0],
        profile.mount_y + slant_range * direction[1],
        profile.mount_z + slant_range * direction[2],
    )


def robot_x(
    range: float, bearing_deg: float, profile: CameraProfile | None = None
) -> float:
    del profile
    rad = math.radians(bearing_deg)
    return range * math.cos(rad)


def robot_y(
    range: float, bearing_deg: float, profile: CameraProfile | None = None
) -> float:
    del profile
    rad = math.radians(bearing_deg)
    return range * math.sin(rad)


def compose_element_confidence(
    interior_score: float,
    circularity: float,
    fill_ratio: float,
    circle_fit_quality: float,
    touches_boundary: bool,
    range_result: RangeResult,
    area_px: float,
    element: ElementSpec,
) -> float:
    mask_score = min(1.0, interior_score)
    shape_score = 0.35 * circularity + 0.25 * fill_ratio + 0.40 * circle_fit_quality
    size_score = min(1.0, area_px / (element.min_area_px * 3.0))
    range_score = range_result.confidence if range_result.is_valid else 0.3
    boundary_penalty = 0.7 if touches_boundary else 1.0
    return max(
        0.0,
        min(
            1.0,
            boundary_penalty
            * (0.20 * mask_score + 0.30 * shape_score + 0.15 * size_score + 0.35 * range_score),
        ),
    )


def compose_plate_confidence(
    white_ratio: float,
    area: float,
    rectangularity: float,
    aspect_ratio: float,
    range_result: RangeResult,
    viewing_angle_penalty: float,
    partial_penalty: float,
    plate: PlateSpec,
) -> float:
    white_score = min(1.0, white_ratio / max(0.01, plate.min_white_ratio))
    area_score = min(1.0, area / (plate.min_area_px * 4.0))
    rect_score = min(1.0, rectangularity)
    aspect_score = 1.0 if plate.min_aspect <= aspect_ratio <= plate.max_aspect else 0.5
    range_score = range_result.confidence if range_result.is_valid else 0.35
    penalty = viewing_angle_penalty * partial_penalty
    return max(
        0.0,
        min(
            1.0,
            penalty
            * (
                0.25 * white_score
                + 0.20 * area_score
                + 0.15 * rect_score
                + 0.10 * aspect_score
                + 0.30 * range_score
            ),
        ),
    )


def fuse_element_observation(
    cx: float,
    cy: float,
    radius_px: float,
    area_px: float,
    aspect_ratio: float,
    circularity: float,
    fill_ratio: float,
    interior_score: float,
    detector_type: ElementDetectorType,
    profile: CameraProfile,
    camera_name: str,
    touches_boundary: bool,
    partial_occlusion: bool,
    circle_fit_quality: float,
    element: ElementSpec,
    season: SeasonConfig,
    *,
    capture_time_nanos: int = 0,
    bounding_width_px: float | None = None,
    bounding_height_px: float | None = None,
    fitted_cx: float | None = None,
    fitted_cy: float | None = None,
    hough_votes: int = 0,
) -> ElementObservation:
    d_size = distance_from_size(element.diameter, profile.focal_length_px, radius_px)
    d_floor = distance_from_floor(cy, profile)
    near_horizon = cy <= profile.horizon_row_px + 8
    size_est = build_size_estimate(
        d_size, radius_px, circle_fit_quality, partial_occlusion, touches_boundary
    )
    floor_est = build_floor_estimate(
        d_floor, cy, _profile_horizon_confidence(profile.horizon_row_px), near_horizon
    )
    range_result = fuse_range_weighted(
        size_est, floor_est, max_range_mismatch_ratio=season.max_range_mismatch_ratio
    )
    confidence = compose_element_confidence(
        interior_score,
        circularity,
        fill_ratio,
        circle_fit_quality,
        touches_boundary,
        range_result,
        area_px,
        element,
    )
    robot_x, robot_y, _ = floor_point_in_robot(cx, cy, range_result.distance, profile)
    bw = bounding_width_px if bounding_width_px is not None else radius_px * 2.0
    bh = bounding_height_px if bounding_height_px is not None else radius_px * 2.0
    return ElementObservation(
        camera_name=camera_name,
        capture_time_nanos=capture_time_nanos,
        cx=cx,
        cy=cy,
        bounding_width_px=bw,
        bounding_height_px=bh,
        fitted_cx=fitted_cx if fitted_cx is not None else cx,
        fitted_cy=fitted_cy if fitted_cy is not None else cy,
        radius_px=radius_px,
        area_px=area_px,
        aspect_ratio=aspect_ratio,
        circularity=circularity,
        fill_ratio=fill_ratio,
        interior_validation_score=interior_score,
        detector_type=detector_type,
        confidence=confidence,
        range=range_result.distance,
        range_uncertainty=range_result.uncertainty,
        d_size=d_size,
        d_floor=d_floor,
        range_result=range_result,
        robot_x=robot_x,
        robot_y=robot_y,
        hough_votes=hough_votes,
        element_id=element.id,
    )


# Java-parity function aliases (same parameter order as VidarGeometry).
distanceFromSize = distance_from_size
distanceFromWidth = distance_from_width
distanceFromFloor = distance_from_floor
buildSizeEstimate = build_size_estimate
buildFloorEstimate = build_floor_estimate
buildPlateWidthEstimate = build_plate_width_estimate
fuseRangeWeighted = fuse_range_weighted
floorPointInRobot = floor_point_in_robot
robotX = robot_x
robotY = robot_y
composeElementConfidence = compose_element_confidence
composePlateConfidence = compose_plate_confidence
fuseElementObservation = fuse_element_observation
