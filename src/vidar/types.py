"""
Canonical ViDAR data types — field names and semantics match Java ``Vidar*`` types.

Python uses snake_case fields (PEP 8). Each type exposes camelCase read-only
properties for cross-language documentation parity (Java, sim JS, future Rust).

See ``docs/API.md`` for the language-neutral contract.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from vidar.models import Alliance, ElementDetectorType


class RangeSource(str, Enum):
    """Matches ``VidarRangeEstimate.Source``."""

    SIZE = "SIZE"
    FLOOR = "FLOOR"
    GROUND_PLANE = "GROUND_PLANE"
    PLATE_WIDTH = "PLATE_WIDTH"


@dataclass(frozen=True)
class RangeEstimate:
    source: RangeSource
    distance: float
    weight: float
    uncertainty: float
    rejection_reason: str | None = None

    @property
    def is_valid(self) -> bool:
        return (
            self.weight > 0
            and not math.isnan(self.distance)
            and self.distance > 0
            and self.rejection_reason is None
        )

    @classmethod
    def rejected(cls, source: RangeSource, reason: str) -> RangeEstimate:
        return cls(source, float("nan"), 0.0, float("nan"), reason)

    @property
    def rejectionReason(self) -> str | None:
        return self.rejection_reason


@dataclass(frozen=True)
class RangeResult:
    distance: float
    uncertainty: float
    confidence: float
    source0: RangeEstimate | None
    source1: RangeEstimate | None
    source_count: int

    @property
    def is_valid(self) -> bool:
        return (
            not math.isnan(self.distance)
            and self.distance > 0
            and self.confidence > 0
        )

    @property
    def sources(self) -> tuple[RangeEstimate, ...]:
        if self.source_count <= 0:
            return ()
        if self.source_count == 1:
            return (self.source0,) if self.source0 is not None else ()
        if self.source0 is not None and self.source1 is not None:
            return (self.source0, self.source1)
        if self.source0 is not None:
            return (self.source0,)
        if self.source1 is not None:
            return (self.source1,)
        return ()

    def source_distance(self, source: RangeSource | str) -> float:
        key = source if isinstance(source, RangeSource) else RangeSource(source)
        if self.source0 is not None and self.source0.source == key and self.source0.is_valid:
            return self.source0.distance
        if (
            self.source_count > 1
            and self.source1 is not None
            and self.source1.source == key
            and self.source1.is_valid
        ):
            return self.source1.distance
        return math.nan

    def source_weight(self, source: RangeSource | str) -> float:
        key = source if isinstance(source, RangeSource) else RangeSource(source)
        if self.source0 is not None and self.source0.source == key and self.source0.is_valid:
            return self.source0.weight
        if (
            self.source_count > 1
            and self.source1 is not None
            and self.source1.source == key
            and self.source1.is_valid
        ):
            return self.source1.weight
        return 0.0

    @classmethod
    def invalid(cls) -> RangeResult:
        return cls(float("nan"), float("nan"), 0.0, None, None, 0)

    @property
    def sourceCount(self) -> int:
        return self.source_count


@dataclass(frozen=True)
class ElementObservation:
    """Matches ``VidarElementObservation`` fields."""

    camera_name: str
    capture_time_nanos: int
    cx: float
    cy: float
    bounding_width_px: float
    bounding_height_px: float
    fitted_cx: float
    fitted_cy: float
    radius_px: float
    area_px: float
    aspect_ratio: float
    circularity: float
    fill_ratio: float
    interior_validation_score: float
    detector_type: ElementDetectorType
    confidence: float
    range: float
    range_uncertainty: float
    d_size: float
    d_floor: float
    range_result: RangeResult
    robot_x: float
    robot_y: float
    hough_votes: int = 0
    element_id: str = ""

    @property
    def cameraName(self) -> str:
        return self.camera_name

    @property
    def captureTimeNanos(self) -> int:
        return self.capture_time_nanos

    @property
    def boundingWidthPx(self) -> float:
        return self.bounding_width_px

    @property
    def boundingHeightPx(self) -> float:
        return self.bounding_height_px

    @property
    def fittedCx(self) -> float:
        return self.fitted_cx

    @property
    def fittedCy(self) -> float:
        return self.fitted_cy

    @property
    def radiusPx(self) -> float:
        return self.radius_px

    @property
    def areaPx(self) -> float:
        return self.area_px

    @property
    def aspectRatio(self) -> float:
        return self.aspect_ratio

    @property
    def fillRatio(self) -> float:
        return self.fill_ratio

    @property
    def interiorValidationScore(self) -> float:
        return self.interior_validation_score

    @property
    def detectorType(self) -> ElementDetectorType:
        return self.detector_type

    @property
    def rangeUncertainty(self) -> float:
        return self.range_uncertainty

    @property
    def dSize(self) -> float:
        return self.d_size

    @property
    def dFloor(self) -> float:
        return self.d_floor

    @property
    def rangeResult(self) -> RangeResult:
        return self.range_result

    @property
    def robotX(self) -> float:
        return self.robot_x

    @property
    def robotY(self) -> float:
        return self.robot_y

    @property
    def houghVotes(self) -> int:
        return self.hough_votes


@dataclass(frozen=True)
class PlateObservation:
    """Matches ``VidarPlateObservation``."""

    alliance: Alliance
    cx: float
    cy: float
    width_px: float
    height_px: float
    angle_deg: float
    aspect_ratio: float
    white_ratio: float
    range: float
    range_uncertainty: float
    size_based_range: float
    floor_based_range: float
    range_result: RangeResult
    viewing_angle_penalty: float
    partial_visibility_penalty: float
    confidence: float
    robot_x: float
    robot_y: float
    camera_name: str
    capture_time_nanos: int

    def is_foe(self, our_alliance: Alliance) -> bool:
        return self.alliance != our_alliance

    def is_ally(self, our_alliance: Alliance) -> bool:
        return self.alliance == our_alliance

    @property
    def widthPx(self) -> float:
        return self.width_px

    @property
    def heightPx(self) -> float:
        return self.height_px

    @property
    def angleDeg(self) -> float:
        return self.angle_deg

    @property
    def aspectRatio(self) -> float:
        return self.aspect_ratio

    @property
    def whiteRatio(self) -> float:
        return self.white_ratio

    @property
    def rangeUncertainty(self) -> float:
        return self.range_uncertainty

    @property
    def sizeBasedRange(self) -> float:
        return self.size_based_range

    @property
    def floorBasedRange(self) -> float:
        return self.floor_based_range

    @property
    def rangeResult(self) -> RangeResult:
        return self.range_result

    @property
    def viewingAnglePenalty(self) -> float:
        return self.viewing_angle_penalty

    @property
    def partialVisibilityPenalty(self) -> float:
        return self.partial_visibility_penalty

    @property
    def robotX(self) -> float:
        return self.robot_x

    @property
    def robotY(self) -> float:
        return self.robot_y

    @property
    def cameraName(self) -> str:
        return self.camera_name

    @property
    def captureTimeNanos(self) -> int:
        return self.capture_time_nanos


@dataclass(frozen=True)
class RankedElementFrame:
    """Matches ``VidarRankedElementFrame``."""

    ranked: tuple[ElementObservation, ...]
    count: int
    overflow_count: int
    capture_time_nanos: int
    camera_name: str
    capacity: int

    @classmethod
    def empty(cls, camera_name: str, capacity: int) -> RankedElementFrame:
        cap = max(1, capacity)
        return cls((), 0, 0, 0, camera_name, cap)

    def at(self, rank: int) -> ElementObservation | None:
        if rank < 0 or rank >= self.count:
            return None
        return self.ranked[rank]

    def best(self) -> ElementObservation | None:
        return self.at(0)

    def overflowCount(self) -> int:
        return self.overflow_count

    def captureTimeNanos(self) -> int:
        return self.capture_time_nanos

    def cameraName(self) -> str:
        return self.camera_name
