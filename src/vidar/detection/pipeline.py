from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from vidar.config import AppConfig, ColorRange
from vidar.detection.hough import detect_hough_balls
from vidar.geometry import fuse_observation


@dataclass(frozen=True)
class Detection:
    label: str
    category: str
    cx: float
    cy: float
    area: float
    camera_index: int
    radius_px: float | None = None
    range_in: float | None = None
    d_size_in: float | None = None
    d_floor_in: float | None = None
    confidence: float | None = None
    robot_x_in: float | None = None
    robot_y_in: float | None = None


def _find_blobs(
    mask: np.ndarray,
    label: str,
    category: str,
    camera_index: int,
    min_area: int,
) -> list[Detection]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[Detection] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        detections.append(
            Detection(
                label=label,
                category=category,
                cx=cx,
                cy=cy,
                area=area,
                camera_index=camera_index,
            )
        )

    return detections


def _mask_for_range(frame_hsv: np.ndarray, color_range: ColorRange) -> np.ndarray:
    lower = np.array(color_range.hsv_low, dtype=np.uint8)
    upper = np.array(color_range.hsv_high, dtype=np.uint8)
    mask = cv2.inRange(frame_hsv, lower, upper)

    if color_range.hsv_low[0] <= 10 and color_range.hsv_high[0] <= 15:
        lower_wrap = np.array([170, color_range.hsv_low[1], color_range.hsv_low[2]], dtype=np.uint8)
        upper_wrap = np.array([180, color_range.hsv_high[1], color_range.hsv_high[2]], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(frame_hsv, lower_wrap, upper_wrap))

    return mask


class DetectionPipeline:
    """Hough ball detection + HSV plate blobs with fused range when geometry is configured."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config

    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        resized = cv2.resize(
            frame,
            (self._config.process_width, self._config.process_height),
            interpolation=cv2.INTER_AREA,
        )
        if self._config.blur_kernel > 1:
            k = self._config.blur_kernel
            resized = cv2.GaussianBlur(resized, (k, k), 0)
        return resized

    def detect(self, frame: np.ndarray, camera_index: int) -> list[Detection]:
        detections: list[Detection] = []

        circles = detect_hough_balls(frame, self._config)
        element_label = self._config.elements[0].name if self._config.elements else "ball"

        for circle in circles:
            area = math.pi * circle.radius * circle.radius
            det = Detection(
                label=element_label,
                category="element",
                cx=circle.cx,
                cy=circle.cy,
                area=area,
                camera_index=camera_index,
                radius_px=circle.radius,
            )

            if self._config.geometry is not None:
                obs = fuse_observation(
                    circle.cx,
                    circle.cy,
                    circle.radius,
                    element_label,
                    camera_index,
                    self._config.geometry,
                )
                if obs.confidence < self._config.geometry.min_ball_confidence:
                    continue
                det = Detection(
                    label=element_label,
                    category="element",
                    cx=obs.cx,
                    cy=obs.cy,
                    area=area,
                    camera_index=camera_index,
                    radius_px=obs.radius_px,
                    range_in=None if math.isnan(obs.range_in) else obs.range_in,
                    d_size_in=None if math.isnan(obs.d_size_in) else obs.d_size_in,
                    d_floor_in=None if math.isnan(obs.d_floor_in) else obs.d_floor_in,
                    confidence=obs.confidence,
                    robot_x_in=None if math.isnan(obs.robot_x_in) else obs.robot_x_in,
                    robot_y_in=None if math.isnan(obs.robot_y_in) else obs.robot_y_in,
                )

            detections.append(det)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for color_range in self._config.robots:
            mask = _mask_for_range(hsv, color_range)
            detections.extend(
                _find_blobs(mask, color_range.name, "robot", camera_index, color_range.min_area)
            )

        return detections

    def annotate(self, frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
        output = frame.copy()
        for det in detections:
            color = (0, 255, 0) if det.category == "element" else (0, 0, 255)
            center = (int(det.cx), int(det.cy))
            if det.category == "element" and det.radius_px:
                cv2.circle(output, center, int(det.radius_px), color, 2)
            cv2.circle(output, center, 4, color, -1)
            label = det.label
            if det.range_in is not None:
                label = f"{label} {det.range_in:.0f}in"
            cv2.putText(
                output,
                label,
                (center[0] + 6, center[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
                cv2.LINE_AA,
            )
        return output
