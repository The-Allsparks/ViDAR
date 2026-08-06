from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from vidar.contour_detect import (
    ContourKind,
    ContourTarget,
    apply_local_hough,
    apply_morphology,
    build_mask,
    find_circle_hits,
    find_rect_hits,
    morph_kernel,
)
from vidar.frame_pipeline import ScaledRoi, detection_scaled
from vidar.geometry import (
    build_plate_width_estimate,
    compose_plate_confidence,
    distance_from_width,
    floor_point_in_robot,
    fuse_element_observation,
    fuse_range_weighted,
)
from vidar.models import ElementDetectorType, CameraProfile, SeasonConfig
from vidar.types import ElementObservation, PlateObservation, RankedElementFrame

PROCESS_ROI_SCALE = 0.5
DEFAULT_MAX_RANKED = 5


@dataclass(frozen=True)
class Detection:
    """Lightweight sim/telemetry DTO; prefer ``ElementObservation`` / ``PlateObservation``."""

    label: str
    category: str
    cx: float
    cy: float
    area: float
    camera_index: int
    radius_px: float | None = None
    width_px: float | None = None
    height_px: float | None = None
    range: float | None = None
    d_size: float | None = None
    d_floor: float | None = None
    confidence: float | None = None
    robot_x: float | None = None
    robot_y: float | None = None
    alliance: str | None = None


def _select_top_k_scored[T](candidates: list[tuple[float, T]], k: int) -> list[T]:
    """Partial top-K by score (matches Java ``VidarContourProcessor.selectTopK``)."""
    out: list[tuple[float, T]] = []
    scores: list[float] = []
    for score, item in candidates:
        if len(out) < k:
            out.append((score, item))
            scores.append(score)
            idx = len(out) - 1
            while idx > 0 and scores[idx] > scores[idx - 1]:
                out[idx], out[idx - 1] = out[idx - 1], out[idx]
                scores[idx], scores[idx - 1] = scores[idx - 1], scores[idx]
                idx -= 1
        elif score <= scores[k - 1]:
            continue
        else:
            out[k - 1] = (score, item)
            scores[k - 1] = score
            idx = k - 1
            while idx > 0 and scores[idx] > scores[idx - 1]:
                out[idx], out[idx - 1] = out[idx - 1], out[idx]
                scores[idx], scores[idx - 1] = scores[idx - 1], scores[idx]
                idx -= 1
    return [item for _, item in out]


class ContourProcessor:
    """
    Unified color-contour processor for season elements and alliance plates.

    Public query API mirrors Java ``VidarContourProcessor``:
    ``get_best_element()``, ``get_ranked_elements()``, ``get_game_element(id)``, ``get_best_plate()``.
    """

    def __init__(
        self,
        profile: CameraProfile,
        camera_name: str,
        season: SeasonConfig,
        camera_index: int = 0,
        roi_scale: float = PROCESS_ROI_SCALE,
        max_ranked: int = DEFAULT_MAX_RANKED,
    ) -> None:
        self.profile = profile
        self.camera_name = camera_name
        self.season = season
        self.camera_index = camera_index
        self.roi_scale = roi_scale
        self.max_ranked = max_ranked
        self.targets = ContourTarget.from_season(season)
        self._element_by_id = {element.id: element for element in season.elements}
        self._plate_by_alliance = {plate.alliance: plate for plate in season.plates}
        self._needs_local_hough = any(
            t.detector == ElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH
            for t in self.targets
            if t.kind == ContourKind.GAME
        )
        self._mask: np.ndarray | None = None
        self._wrap_mask: np.ndarray | None = None
        self._ranked_elements = RankedElementFrame.empty(camera_name, max_ranked)
        self._game_elements: dict[str, ElementObservation] = {}
        self._best_plate: PlateObservation | None = None

    def set_max_ranked_elements(self, max_ranked: int) -> None:
        self.max_ranked = max(1, max_ranked)

    def max_ranked_elements(self) -> int:
        return self.max_ranked

    def get_best_element(self) -> ElementObservation | None:
        return self._ranked_elements.best()

    def get_ranked_elements(self) -> RankedElementFrame:
        return self._ranked_elements

    def get_game_element(self, element_id: str) -> ElementObservation | None:
        return self._game_elements.get(element_id)

    def get_game_elements(self) -> dict[str, ElementObservation]:
        return dict(self._game_elements)

    def get_best_plate(self) -> PlateObservation | None:
        return self._best_plate

    def detect(self, frame: np.ndarray) -> list[Detection]:
        capture_time_nanos = time.time_ns()
        frame_h, frame_w = frame.shape[:2]
        scaled = detection_scaled(frame, self.profile, self.roi_scale)
        if scaled is None:
            self._finalize_frame([], [], capture_time_nanos)
            return []

        rgba = scaled.image
        if rgba.ndim == 2:
            rgba = cv2.cvtColor(rgba, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(rgba, cv2.COLOR_BGR2HSV)

        gray: np.ndarray | None = None
        if self._needs_local_hough:
            gray = cv2.cvtColor(rgba, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (3, 3), 0)

        if self._mask is None or self._mask.shape != hsv.shape[:2]:
            self._mask = np.empty(hsv.shape[:2], dtype=np.uint8)
            self._wrap_mask = np.empty(hsv.shape[:2], dtype=np.uint8)

        element_candidates: list[tuple[float, ElementObservation]] = []
        plate_candidates: list[tuple[float, PlateObservation]] = []
        detections: list[tuple[float, Detection]] = []

        for target in self.targets:
            build_mask(hsv, target, mask=self._mask, wrap_scratch=self._wrap_mask)
            apply_morphology(self._mask, morph_kernel(target), target)

            if target.kind == ContourKind.GAME:
                game_out = self._process_game_target(
                    rgba,
                    self._mask,
                    target,
                    scaled,
                    frame_w,
                    frame_h,
                    gray,
                    capture_time_nanos,
                )
                element_candidates.extend(game_out)
                detections.extend((score, self._element_detection(obs)) for score, obs in game_out)
            else:
                plate_out = self._process_plate_target(
                    rgba, self._mask, target, scaled, frame_w, frame_h, capture_time_nanos
                )
                plate_candidates.extend(plate_out)
                detections.extend((score, self._plate_detection(obs)) for score, obs in plate_out)

        self._finalize_frame(element_candidates, plate_candidates, capture_time_nanos)
        return _select_top_k_scored(detections, self.max_ranked)

    def _finalize_frame(
        self,
        element_candidates: list[tuple[float, ElementObservation]],
        plate_candidates: list[tuple[float, PlateObservation]],
        capture_time_nanos: int,
    ) -> None:
        ranked = _select_top_k_scored(element_candidates, self.max_ranked)
        overflow = max(0, len(element_candidates) - self.max_ranked)
        self._ranked_elements = RankedElementFrame(
            tuple(ranked),
            len(ranked),
            overflow,
            capture_time_nanos,
            self.camera_name,
            self.max_ranked,
        )
        self._game_elements = {}
        for score, obs in element_candidates:
            prev = self._game_elements.get(obs.element_id)
            if prev is None or obs.confidence > prev.confidence:
                self._game_elements[obs.element_id] = obs
        self._best_plate = None
        if plate_candidates:
            self._best_plate = max(plate_candidates, key=lambda item: item[0])[1]

    def _process_game_target(
        self,
        rgba: np.ndarray,
        mask: np.ndarray,
        target: ContourTarget,
        scaled: ScaledRoi,
        frame_w: int,
        frame_h: int,
        gray: np.ndarray | None,
        capture_time_nanos: int,
    ) -> list[tuple[float, ElementObservation]]:
        hits = find_circle_hits(rgba, mask, target, scaled, frame_w, frame_h, self.profile)
        if (
            target.detector == ElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH
            and gray is not None
        ):
            hits = apply_local_hough(scaled, hits, target, gray)

        element = self._element_by_id[target.id]
        output: list[tuple[float, ElementObservation]] = []
        for hit in hits:
            obs = fuse_element_observation(
                hit.cx,
                hit.cy,
                hit.radius,
                hit.area,
                hit.aspect_ratio,
                hit.circularity,
                hit.fill_ratio,
                hit.interior_score,
                target.detector,
                self.profile,
                self.camera_name,
                hit.touches_boundary,
                False,
                hit.circle_fit_quality,
                element,
                self.season,
                capture_time_nanos=capture_time_nanos,
            )
            if obs.confidence < self.season.min_element_confidence:
                continue
            output.append((obs.confidence, obs))
        return output

    def _process_plate_target(
        self,
        rgba: np.ndarray,
        mask: np.ndarray,
        target: ContourTarget,
        scaled: ScaledRoi,
        frame_w: int,
        frame_h: int,
        capture_time_nanos: int,
    ) -> list[tuple[float, PlateObservation]]:
        if target.alliance is None:
            return []
        plate = self._plate_by_alliance[target.alliance]
        hits = find_rect_hits(rgba, mask, target, scaled, frame_w, frame_h, self.profile)
        output: list[tuple[float, PlateObservation]] = []
        for hit in hits:
            d_width = distance_from_width(
                plate.width, self.profile.focal_length_px, hit.width
            )
            width_est = build_plate_width_estimate(
                d_width,
                hit.width,
                hit.rectangularity,
                hit.white_ratio,
                False,
                hit.touches_boundary,
                0.1,
            )
            range_result = fuse_range_weighted(
                width_est, max_range_mismatch_ratio=self.season.max_range_mismatch_ratio
            )
            confidence = compose_plate_confidence(
                hit.white_ratio,
                hit.contour_area,
                hit.rectangularity,
                hit.aspect,
                range_result,
                1.0,
                1.0,
                plate,
            )
            if confidence < self.season.min_plate_confidence:
                continue
            robot_x, robot_y, _ = floor_point_in_robot(
                hit.cx, hit.cy, range_result.distance, self.profile
            )
            obs = PlateObservation(
                alliance=target.alliance,
                cx=hit.cx,
                cy=hit.cy,
                width_px=hit.width,
                height_px=hit.height,
                angle_deg=hit.angle,
                aspect_ratio=hit.aspect,
                white_ratio=hit.white_ratio,
                range=range_result.distance,
                range_uncertainty=range_result.uncertainty,
                size_based_range=d_width,
                floor_based_range=math.nan,
                range_result=range_result,
                viewing_angle_penalty=1.0,
                partial_visibility_penalty=1.0,
                confidence=confidence,
                robot_x=robot_x,
                robot_y=robot_y,
                camera_name=self.camera_name,
                capture_time_nanos=capture_time_nanos,
            )
            output.append((confidence, obs))
        return output

    def _element_detection(self, obs: ElementObservation) -> Detection:
        return Detection(
            label=obs.element_id,
            category="element",
            cx=obs.cx,
            cy=obs.cy,
            area=obs.area_px,
            camera_index=self.camera_index,
            radius_px=obs.radius_px,
            range=_nan_none(obs.range),
            d_size=_nan_none(obs.d_size),
            d_floor=_nan_none(obs.d_floor),
            confidence=obs.confidence,
            robot_x=_nan_none(obs.robot_x),
            robot_y=_nan_none(obs.robot_y),
        )

    def _plate_detection(self, obs: PlateObservation) -> Detection:
        return Detection(
            label=f"plate_{obs.alliance.value}",
            category="plate",
            cx=obs.cx,
            cy=obs.cy,
            area=obs.width_px * obs.height_px,
            camera_index=self.camera_index,
            width_px=obs.width_px,
            height_px=obs.height_px,
            range=_nan_none(obs.range),
            confidence=obs.confidence,
            robot_x=_nan_none(obs.robot_x),
            robot_y=_nan_none(obs.robot_y),
            alliance=obs.alliance.value,
        )

    @staticmethod
    def annotate(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
        output = frame.copy()
        for det in detections:
            if det.category == "element":
                color = (0, 255, 0)
            elif det.category == "plate":
                color = (255, 128, 0) if det.alliance == "red" else (255, 0, 128)
            else:
                color = (0, 0, 255)
            center = (int(det.cx), int(det.cy))
            if det.category == "element" and det.radius_px:
                cv2.circle(output, center, int(det.radius_px), color, 2)
            elif det.category == "plate" and det.width_px and det.height_px:
                x = int(det.cx - det.width_px / 2)
                y = int(det.cy - det.height_px / 2)
                cv2.rectangle(
                    output,
                    (x, y),
                    (x + int(det.width_px), y + int(det.height_px)),
                    color,
                    2,
                )
            cv2.circle(output, center, 4, color, -1)
            label = det.label
            if det.range is not None:
                label = f"{label} {det.range:.0f}in"
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


def _nan_none(value: float) -> float | None:
    return None if math.isnan(value) else value
