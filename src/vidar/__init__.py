"""ViDAR - high-speed low-resolution vision for FTC element gathering and avoidance."""

from vidar.geometry import (
    build_size_estimate,
    build_floor_estimate,
    build_plate_width_estimate,
    distance_from_size,
    distance_from_width,
    fuse_range_weighted,
)
from vidar.types import (
    ElementObservation,
    PlateObservation,
    RangeEstimate,
    RangeResult,
    RangeSource,
    RankedElementFrame,
)
from vidar.units import DistanceUnit, convert, effective_distance_unit, format_distance

__version__ = "0.2.0"

__all__ = [
    "__version__",
    # Canonical types (see docs/API.md)
    "DistanceUnit",
    "RangeSource",
    "RangeEstimate",
    "RangeResult",
    "ElementObservation",
    "PlateObservation",
    "RankedElementFrame",
    # Units
    "convert",
    "effective_distance_unit",
    "format_distance",
    # Common geometry entry points
    "distance_from_size",
    "distance_from_width",
    "fuse_range_weighted",
    "build_size_estimate",
    "build_floor_estimate",
    "build_plate_width_estimate",
]
