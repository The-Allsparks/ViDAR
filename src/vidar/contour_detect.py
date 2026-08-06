from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np

from vidar.frame_pipeline import ScaledRoi
from vidar.models import (
    Alliance,
    ElementDetectorType,
    CameraProfile,
    ElementShape,
    ElementSpec,
    HsvRange,
    PlateSpec,
    SeasonConfig,
)


class ContourKind(str, Enum):
    GAME = "game"
    PLATE = "plate"


@dataclass(frozen=True)
class ContourTarget:
    id: str
    kind: ContourKind
    shape: ElementShape
    alliance: Alliance | None
    hsv: HsvRange
    hsv_wrap: HsvRange | None
    detector: ElementDetectorType
    ranging_size: float
    min_area_px: float
    max_area_px: float
    min_width_px: float
    max_width_px: float
    min_height_px: float
    max_height_px: float
    min_circularity: float
    min_rectangularity: float
    min_aspect: float
    max_aspect: float
    max_bounding_aspect: float
    min_fill_ratio: float
    min_interior_score: float
    interior_bright: int
    interior_spread: int
    hole_dark_max: int
    morph_erode_passes: int
    morph_dilate_passes: int
    morph_open_passes: int
    morph_close_passes: int
    morph_ellipse_kernel: bool
    hough_dp: float
    hough_min_dist: float
    hough_param1: float
    hough_param2: float
    hough_min_radius: int
    hough_max_radius: int
    hough_min_interior: float
    min_area_hough_px: float
    min_white_ratio: float
    white_sample_grid: int
    white_bright_min: int
    white_spread_max: int

    @classmethod
    def from_element(cls, spec: ElementSpec) -> ContourTarget:
        min_rect = spec.min_rectangularity
        if spec.shape == ElementShape.RECT and min_rect <= 0:
            min_rect = 0.40
        return cls(
            id=spec.id,
            kind=ContourKind.GAME,
            shape=spec.shape,
            alliance=None,
            hsv=spec.hsv,
            hsv_wrap=None,
            detector=spec.detector,
            ranging_size=spec.diameter,
            min_area_px=spec.min_area_px,
            max_area_px=spec.max_area_px,
            min_width_px=spec.min_width_px,
            max_width_px=spec.max_width_px,
            min_height_px=spec.min_height_px,
            max_height_px=spec.max_height_px,
            min_circularity=spec.min_circularity if spec.shape == ElementShape.CIRCLE else 0.0,
            min_rectangularity=min_rect,
            min_aspect=spec.min_aspect,
            max_aspect=spec.max_aspect,
            max_bounding_aspect=spec.max_aspect_ratio,
            min_fill_ratio=spec.min_fill_ratio,
            min_interior_score=spec.min_interior_score,
            interior_bright=spec.interior_bright,
            interior_spread=spec.interior_spread,
            hole_dark_max=spec.hole_dark_max,
            morph_erode_passes=spec.morph_erode_passes,
            morph_dilate_passes=spec.morph_dilate_passes,
            morph_open_passes=spec.morph_open_passes,
            morph_close_passes=spec.morph_close_passes,
            morph_ellipse_kernel=True,
            hough_dp=spec.hough_dp,
            hough_min_dist=spec.hough_min_dist,
            hough_param1=spec.hough_param1,
            hough_param2=spec.hough_param2,
            hough_min_radius=spec.hough_min_radius,
            hough_max_radius=spec.hough_max_radius,
            hough_min_interior=spec.hough_min_interior,
            min_area_hough_px=spec.min_area_hough_px,
            min_white_ratio=0.0,
            white_sample_grid=0,
            white_bright_min=0,
            white_spread_max=0,
        )

    @classmethod
    def from_plate(cls, spec: PlateSpec) -> ContourTarget:
        return cls(
            id=f"{spec.alliance.value}_plate",
            kind=ContourKind.PLATE,
            shape=ElementShape.RECT,
            alliance=spec.alliance,
            hsv=spec.hsv,
            hsv_wrap=spec.hsv_wrap,
            detector=ElementDetectorType.COLOR_BLOB,
            ranging_size=spec.width,
            min_area_px=spec.min_area_px,
            max_area_px=spec.max_area_px,
            min_width_px=4,
            max_width_px=500,
            min_height_px=4,
            max_height_px=500,
            min_circularity=0.0,
            min_rectangularity=spec.min_rectangularity,
            min_aspect=spec.min_aspect,
            max_aspect=spec.max_aspect,
            max_bounding_aspect=spec.max_aspect,
            min_fill_ratio=0.0,
            min_interior_score=0.0,
            interior_bright=0,
            interior_spread=0,
            hole_dark_max=0,
            morph_erode_passes=0,
            morph_dilate_passes=0,
            morph_open_passes=1,
            morph_close_passes=1,
            morph_ellipse_kernel=False,
            hough_dp=0,
            hough_min_dist=0,
            hough_param1=0,
            hough_param2=0,
            hough_min_radius=0,
            hough_max_radius=0,
            hough_min_interior=0,
            min_area_hough_px=0,
            min_white_ratio=spec.min_white_ratio,
            white_sample_grid=spec.white_sample_grid,
            white_bright_min=spec.white_bright_min,
            white_spread_max=spec.white_spread_max,
        )

    @classmethod
    def from_season(cls, season: SeasonConfig) -> list[ContourTarget]:
        targets = [cls.from_element(element) for element in season.elements]
        targets.extend(cls.from_plate(plate) for plate in season.plates)
        return targets


@dataclass(frozen=True)
class CircleHit:
    cx: float
    cy: float
    radius: float
    area: float
    aspect_ratio: float
    circularity: float
    fill_ratio: float
    interior_score: float
    circle_fit_quality: float
    touches_boundary: bool


@dataclass(frozen=True)
class RectHit:
    cx: float
    cy: float
    width: float
    height: float
    angle: float
    contour_area: float
    rectangularity: float
    aspect: float
    white_ratio: float
    touches_boundary: bool


def _hsv_bounds(hsv: HsvRange) -> tuple[np.ndarray, np.ndarray]:
    lower = np.array([hsv.h_min, hsv.s_min, hsv.v_min], dtype=np.uint8)
    upper = np.array([hsv.h_max, hsv.s_max, hsv.v_max], dtype=np.uint8)
    return lower, upper


def build_mask(
    hsv: np.ndarray,
    target: ContourTarget,
    mask: np.ndarray | None = None,
    wrap_scratch: np.ndarray | None = None,
) -> np.ndarray:
    lower, upper = _hsv_bounds(target.hsv)
    if mask is None:
        mask = cv2.inRange(hsv, lower, upper)
    else:
        cv2.inRange(hsv, lower, upper, dst=mask)
    if target.hsv_wrap is not None:
        w_lower, w_upper = _hsv_bounds(target.hsv_wrap)
        if wrap_scratch is not None:
            cv2.inRange(hsv, w_lower, w_upper, dst=wrap_scratch)
            cv2.bitwise_or(mask, wrap_scratch, dst=mask)
        else:
            wrap = cv2.inRange(hsv, w_lower, w_upper)
            mask = cv2.bitwise_or(mask, wrap)
    return mask


_KERNEL_ELLIPSE = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
_KERNEL_RECT = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def morph_kernel(target: ContourTarget) -> np.ndarray:
    return _KERNEL_ELLIPSE if target.morph_ellipse_kernel else _KERNEL_RECT


def create_kernel(target: ContourTarget) -> np.ndarray:
    """Alias for morph_kernel (cached 3×3 ellipse/rect kernels)."""
    return morph_kernel(target)


def apply_morphology(mask: np.ndarray, kernel: np.ndarray, target: ContourTarget) -> None:
    for _ in range(target.morph_erode_passes):
        cv2.erode(mask, kernel, dst=mask)
    for _ in range(target.morph_dilate_passes):
        cv2.dilate(mask, kernel, dst=mask)
    for _ in range(target.morph_open_passes):
        cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, dst=mask)
    for _ in range(target.morph_close_passes):
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, dst=mask)


def _interior_score(
    rgba: np.ndarray, cx: float, cy: float, radius: float, target: ContourTarget
) -> float:
    ri = int(math.floor(radius))
    icx = int(round(cx))
    icy = int(round(cy))
    x0 = max(0, icx - ri)
    y0 = max(0, icy - ri)
    x1 = min(rgba.shape[1], icx + ri + 1)
    y1 = min(rgba.shape[0], icy + ri + 1)
    patch = rgba[y0:y1, x0:x1]
    if patch.size == 0:
        return 0.0

    ph, pw = patch.shape[:2]
    yy, xx = np.mgrid[0:ph, 0:pw]
    lx = xx + x0 - icx
    ly = yy + y0 - icy
    disk = (lx * lx + ly * ly) <= radius * radius
    if not np.any(disk):
        return 0.0

    pixels = patch[disk]
    b = pixels[:, 0].astype(np.int32)
    g = pixels[:, 1].astype(np.int32)
    r = pixels[:, 2].astype(np.int32)
    max_c = np.maximum(np.maximum(r, g), b)
    min_c = np.minimum(np.minimum(r, g), b)
    inside = int(pixels.shape[0])
    bright = int(
        np.sum(
            (max_c >= target.interior_bright)
            & (max_c - min_c <= target.interior_spread)
        )
    )
    dark_hole = int(np.sum(max_c < target.hole_dark_max))
    bright_ratio = bright / inside
    hole_bonus = 0.15 if dark_hole > inside * 0.05 else 0.0
    return min(1.0, bright_ratio + hole_bonus)


def _bilinear(corners: list[tuple[float, float]], u: float, v: float) -> tuple[float, float]:
    x = (
        (1 - u) * (1 - v) * corners[0][0]
        + u * (1 - v) * corners[1][0]
        + u * v * corners[2][0]
        + (1 - u) * v * corners[3][0]
    )
    y = (
        (1 - u) * (1 - v) * corners[0][1]
        + u * (1 - v) * corners[1][1]
        + u * v * corners[2][1]
        + (1 - u) * v * corners[3][1]
    )
    return x, y


def _white_digit_ratio(rgba: np.ndarray, box: tuple, target: ContourTarget) -> float:
    center, size, angle = box
    rect = ((float(center[0]), float(center[1])), (float(size[0]), float(size[1])), float(angle))
    corners = cv2.boxPoints(rect)
    grid = target.white_sample_grid
    samples = 0
    white = 0
    rows, cols = rgba.shape[:2]

    for gy in range(1, grid):
        for gx in range(1, grid):
            u = gx / grid
            v = gy / grid
            px, py = _bilinear([(c[0], c[1]) for c in corners], u, v)
            ix = int(round(px))
            iy = int(round(py))
            if ix < 0 or iy < 0 or ix >= cols or iy >= rows:
                continue
            pixel = rgba[iy, ix]
            samples += 1
            r, g, b = int(pixel[2]), int(pixel[1]), int(pixel[0])
            max_c = max(r, g, b)
            min_c = min(r, g, b)
            if max_c >= target.white_bright_min and max_c - min_c <= target.white_spread_max:
                white += 1
    return 0.0 if samples == 0 else white / samples


def find_circle_hits(
    rgba: np.ndarray,
    mask: np.ndarray,
    target: ContourTarget,
    scaled: ScaledRoi,
    frame_w: int,
    frame_h: int,
    profile: CameraProfile,
) -> list[CircleHit]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    element_roi = profile.roi_config
    from vidar.roi import element_roi as build_element_roi

    roi = build_element_roi(element_roi, frame_w, frame_h)
    hits: list[CircleHit] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < target.min_area_px or area > target.max_area_px:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        full_bw = bw * scaled.scale
        full_bh = bh * scaled.scale
        if (
            full_bw < target.min_width_px
            or full_bh < target.min_height_px
            or full_bw > target.max_width_px
            or full_bh > target.max_height_px
        ):
            continue
        aspect = max(full_bw, full_bh) / max(1.0, min(full_bw, full_bh))
        if aspect > target.max_bounding_aspect:
            continue
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
        if target.min_circularity > 0 and circularity < target.min_circularity:
            continue
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        center = (float(center_x), float(center_y))
        fill_ratio = area / max(1.0, math.pi * radius * radius) if target.min_fill_ratio > 0 else 1.0
        if target.min_fill_ratio > 0 and fill_ratio < target.min_fill_ratio:
            continue
        interior = (
            _interior_score(rgba, center[0], center[1], radius, target)
            if target.min_interior_score > 0
            else 1.0
        )
        if target.min_interior_score > 0 and interior < target.min_interior_score:
            continue
        full_cx = scaled.to_full_x(center[0])
        full_cy = scaled.to_full_y(center[1])
        full_radius = radius * scaled.scale
        touches = roi.touches_boundary(frame_w, frame_h, full_cx, full_cy, 3)
        fit_quality = min(1.0, max(0.35, circularity * fill_ratio))
        hits.append(
            CircleHit(
                cx=full_cx,
                cy=full_cy,
                radius=full_radius,
                area=area * scaled.scale * scaled.scale,
                aspect_ratio=aspect,
                circularity=circularity,
                fill_ratio=fill_ratio,
                interior_score=interior,
                circle_fit_quality=fit_quality,
                touches_boundary=touches,
            )
        )
    return hits


def find_rect_hits(
    rgba: np.ndarray,
    mask: np.ndarray,
    target: ContourTarget,
    scaled: ScaledRoi,
    frame_w: int,
    frame_h: int,
    profile: CameraProfile,
) -> list[RectHit]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    from vidar.roi import element_roi as build_element_roi

    roi = build_element_roi(profile.roi_config, frame_w, frame_h)
    hits: list[RectHit] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < target.min_area_px or area > target.max_area_px:
            continue
        box = cv2.minAreaRect(contour)
        w, h = box[1]
        if w < 4 or h < 4:
            continue
        short_side = min(w, h)
        long_side = max(w, h)
        aspect = long_side / short_side
        if aspect < target.min_aspect or aspect > target.max_aspect:
            continue
        rectangularity = area / (short_side * long_side)
        if rectangularity < target.min_rectangularity:
            continue
        white_ratio = _white_digit_ratio(rgba, box, target) if target.min_white_ratio > 0 else 1.0
        if target.min_white_ratio > 0 and white_ratio < target.min_white_ratio:
            continue
        full_cx = scaled.to_full_x(box[0][0])
        full_cy = scaled.to_full_y(box[0][1])
        touches = roi.touches_boundary(frame_w, frame_h, full_cx, full_cy, 4)
        hits.append(
            RectHit(
                cx=full_cx,
                cy=full_cy,
                width=long_side * scaled.scale,
                height=short_side * scaled.scale,
                angle=box[2],
                contour_area=area * scaled.scale * scaled.scale,
                rectangularity=rectangularity,
                aspect=aspect,
                white_ratio=white_ratio,
                touches_boundary=touches,
            )
        )
    return hits


def apply_local_hough(
    scaled: ScaledRoi,
    hits: list[CircleHit],
    target: ContourTarget,
    gray: np.ndarray,
) -> list[CircleHit]:
    if not hits:
        return hits
    validated: list[CircleHit] = []
    x0, y0, _, _ = scaled.source_crop
    for hit in hits:
        local_cx = (hit.cx - x0) / scaled.scale
        local_cy = (hit.cy - y0) / scaled.scale
        local_r = hit.radius / scaled.scale
        pad = int(math.ceil(local_r * 1.5))
        x = max(0, int(local_cx) - pad)
        y = max(0, int(local_cy) - pad)
        w = min(scaled.image.shape[1] - x, pad * 2)
        h = min(scaled.image.shape[0] - y, pad * 2)
        if w < 8 or h < 8:
            validated.append(hit)
            continue
        patch = gray[y : y + h, x : x + w]
        circles = cv2.HoughCircles(
            patch,
            cv2.HOUGH_GRADIENT,
            target.hough_dp,
            max(8.0, local_r),
            param1=target.hough_param1,
            param2=target.hough_param2,
            minRadius=max(4, int(local_r * 0.6)),
            maxRadius=max(6, int(local_r * 1.4)),
        )
        if circles is not None:
            validated.append(
                CircleHit(
                    cx=hit.cx,
                    cy=hit.cy,
                    radius=hit.radius,
                    area=hit.area,
                    aspect_ratio=hit.aspect_ratio,
                    circularity=hit.circularity,
                    fill_ratio=hit.fill_ratio,
                    interior_score=hit.interior_score,
                    circle_fit_quality=min(1.0, hit.circle_fit_quality + 0.15),
                    touches_boundary=hit.touches_boundary,
                )
            )
    return validated if validated else hits
