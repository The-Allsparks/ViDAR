"""Compatibility shim — core logic lives in src/vidar/."""
from vidar.geometry import (
    build_size_estimate,
    build_floor_estimate,
    build_ground_plane_estimate,
    build_plate_width_estimate,
    distance_from_size,
    distance_from_width,
    fuse_range_weighted,
)
from vidar.models import CameraRoiConfig
from vidar.motion import LocalizationFusion as LocalizationFusionPure
from vidar.motion import MotionTransform
from vidar.roi import RoiRect
from vidar.types import RangeEstimate, RangeResult, RangeSource

__all__ = [
    "RoiRect",
    "CameraRoiConfig",
    "RangeSource",
    "RangeEstimate",
    "RangeResult",
    "fuse_range_weighted",
    "build_size_estimate",
    "build_floor_estimate",
    "build_ground_plane_estimate",
    "build_plate_width_estimate",
    "MotionTransform",
    "LocalizationFusionPure",
    "distance_from_size",
    "distance_from_width",
]
