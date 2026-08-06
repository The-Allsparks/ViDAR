"""Linear distance units — matches Java ``VidarDistanceUnit`` / ``VidarUnits``."""
from __future__ import annotations

from enum import Enum


class DistanceUnit(str, Enum):
    IN = "in"
    CM = "cm"
    M = "m"

    @classmethod
    def from_json(cls, raw: str | None) -> DistanceUnit:
        if raw is None or not str(raw).strip():
            return cls.IN
        key = str(raw).strip().lower()
        aliases = {
            "in": cls.IN,
            "inch": cls.IN,
            "inches": cls.IN,
            "m": cls.M,
            "meter": cls.M,
            "meters": cls.M,
            "cm": cls.CM,
            "centimeter": cls.CM,
            "centimeters": cls.CM,
        }
        if key not in aliases:
            raise ValueError(f"Unknown distanceUnit: {raw}")
        return aliases[key]

    @property
    def meters_per_unit(self) -> float:
        if self is DistanceUnit.M:
            return 1.0
        if self is DistanceUnit.CM:
            return 0.01
        return 0.0254

    @property
    def suffix(self) -> str:
        return self.value

    def to_meters(self, value: float) -> float:
        return value * self.meters_per_unit

    def from_meters(self, meters: float) -> float:
        return meters / self.meters_per_unit

    def convert(self, value: float, to: DistanceUnit) -> float:
        if to is self:
            return value
        return to.from_meters(self.to_meters(value))

    def format(self, value: float) -> str:
        import math

        if math.isnan(value):
            return "—"
        if self is DistanceUnit.M:
            return f"{value:.2f} {self.suffix}"
        if self is DistanceUnit.CM:
            return f"{value:.1f} {self.suffix}"
        return f"{value:.0f} {self.suffix}"


def convert(value: float, from_unit: DistanceUnit, to_unit: DistanceUnit) -> float:
    return from_unit.convert(value, to_unit)


def to_meters(value: float, unit: DistanceUnit) -> float:
    return unit.to_meters(value)


def from_meters(meters: float, unit: DistanceUnit) -> float:
    return unit.from_meters(meters)


def format_distance(value: float, unit: DistanceUnit) -> str:
    return unit.format(value)


def effective_distance_unit(robot: object | None, season: object | None) -> DistanceUnit:
    """Robot JSON override wins; else season; else inches."""
    if robot is not None:
        override = getattr(robot, "distance_unit_override", None)
        if override is not None:
            return override
    if season is not None:
        return getattr(season, "distance_unit", DistanceUnit.IN)
    return DistanceUnit.IN
