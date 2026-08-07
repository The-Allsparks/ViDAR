from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from vidar.units import DistanceUnit


class ElementShape(str, Enum):
    CIRCLE = "circle"
    RECT = "rect"
    BLOB = "blob"


class ElementDetectorType(str, Enum):
    COLOR_BLOB = "color_blob"
    COLOR_BLOB_WITH_LOCAL_HOUGH = "color_blob_with_local_hough"


class Alliance(str, Enum):
    RED = "red"
    BLUE = "blue"


@dataclass(frozen=True)
class HsvRange:
    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int


@dataclass(frozen=True)
class ElementSpec:
    id: str
    label: str
    shape: ElementShape
    diameter: float
    detector: ElementDetectorType
    hsv: HsvRange
    min_area_px: float = 45
    max_area_px: float = 12000
    min_width_px: float = 8
    max_width_px: float = 80
    min_height_px: float = 8
    max_height_px: float = 80
    max_aspect_ratio: float = 2.0
    min_circularity: float = 0.55
    min_rectangularity: float = 0.0
    min_aspect: float = 1.0
    max_aspect: float = 2.0
    min_fill_ratio: float = 0.55
    min_interior_score: float = 0.12
    interior_bright: int = 90
    interior_spread: int = 60
    hole_dark_max: int = 45
    morph_erode_passes: int = 0
    morph_dilate_passes: int = 0
    morph_open_passes: int = 1
    morph_close_passes: int = 2
    hough_dp: float = 1.2
    hough_min_dist: float = 24
    hough_param1: float = 80
    hough_param2: float = 11
    hough_min_radius: int = 8
    hough_max_radius: int = 36
    hough_min_interior: float = 0.14
    min_area_hough_px: float = 45


@dataclass(frozen=True)
class PlateSpec:
    alliance: Alliance
    hsv: HsvRange
    width: float = 12.0
    hsv_wrap: HsvRange | None = None
    min_area_px: float = 120
    max_area_px: float = 12000
    min_aspect: float = 1.15
    max_aspect: float = 4.5
    min_rectangularity: float = 0.45
    min_white_ratio: float = 0.12
    white_sample_grid: int = 5
    white_bright_min: int = 175
    white_spread_max: int = 55


@dataclass(frozen=True)
class FieldSpec:
    length: float = 691.2
    width: float = 317.0


@dataclass(frozen=True)
class AprilTagSpec:
    id: int
    name: str
    size: float
    x: float = float("nan")
    y: float = float("nan")
    z: float = float("nan")
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    localization: bool = True

    def has_field_position(self) -> bool:
        return not (math.isnan(self.x) or math.isnan(self.y) or math.isnan(self.z))


@dataclass(frozen=True)
class SeasonConfig:
    season_id: str
    season_name: str
    field: FieldSpec
    elements: tuple[ElementSpec, ...]
    plates: tuple[PlateSpec, ...]
    april_tags: tuple[AprilTagSpec, ...] = ()
    default_tag_size: float = 8.125
    min_element_confidence: float = 0.35
    min_plate_confidence: float = 0.35
    max_range_mismatch_ratio: float = 0.28
    distance_unit: DistanceUnit = DistanceUnit.IN

    def primary_element(self) -> ElementSpec:
        return self.elements[0]

    def plate_spec(self, alliance: Alliance) -> PlateSpec:
        for plate in self.plates:
            if plate.alliance == alliance:
                return plate
        return self.plates[0]

    def tag_by_id(self, tag_id: int) -> AprilTagSpec | None:
        for tag in self.april_tags:
            if tag.id == tag_id:
                return tag
        return None

    def localization_tags(self) -> tuple[AprilTagSpec, ...]:
        return tuple(tag for tag in self.april_tags if tag.localization)


@dataclass(frozen=True)
class CameraRoiConfig:
    element_lower_fraction: float = 0.65
    plate_start_fraction: float = 0.30
    plate_band_fraction: float = 0.40
    tag_upper_fraction: float = 0.65
    element_enabled: bool = True
    plate_enabled: bool = True
    tag_enabled: bool = True

    @classmethod
    def default(cls) -> CameraRoiConfig:
        return cls()

    def element_roi(self, frame_w: int, frame_h: int):
        from vidar.roi import element_roi

        return element_roi(self, frame_w, frame_h)

    def tag_roi(self, frame_w: int, frame_h: int):
        from vidar.roi import tag_roi

        return tag_roi(self, frame_w, frame_h)


@dataclass(frozen=True)
class CameraProfile:
    name: str
    bearing_deg: float
    horizon_row_px: int
    focal_length_px: float
    floor_cy_px: tuple[float, ...]
    floor_dist: tuple[float, ...]
    focal_length_y_px: float = 340.0
    principal_point_x: float = 320.0
    principal_point_y: float = 240.0
    horizontal_fov_deg: float = 70.0
    vertical_fov_deg: float = 55.0
    plate_width: float = 12.0
    mount_x: float = 0.0
    mount_y: float = 0.0
    mount_z: float = 0.0
    mount_yaw_deg: float = 0.0
    mount_pitch_deg: float = 0.0
    mount_roll_deg: float = 0.0
    roi_config: CameraRoiConfig = CameraRoiConfig.default()
    calibration_width: int = 0
    calibration_height: int = 0
    distortion_model: str = "none"
    distortion_coeffs: tuple[float, ...] = ()
    calibration_version: str | None = None
    calibration_date: str | None = None

    @property
    def floor_lut(self) -> tuple[tuple[float, float], ...]:
        return tuple(zip(self.floor_cy_px, self.floor_dist, strict=True))


@dataclass(frozen=True)
class CameraMount:
    webcam_name: str
    profile: CameraProfile


@dataclass(frozen=True)
class RobotDimensions:
    length: float = 13.0
    width: float = 13.0
    height: float = 18.0


@dataclass(frozen=True)
class RobotConfig:
    robot_name: str
    active_camera_index: int
    cameras: tuple[CameraMount, ...]
    dimensions: RobotDimensions = RobotDimensions()
    default_alliance: Alliance = Alliance.RED
    distance_unit_override: DistanceUnit | None = None
