"""Distance unit conversion tests."""
import math

import pytest

from vidar.units import DistanceUnit, convert, effective_distance_unit, format_distance, to_meters


class TestDistanceUnit:
    def test_inch_to_meter(self):
        assert math.isclose(to_meters(12.0, DistanceUnit.IN), 0.3048, rel_tol=1e-6)

    def test_meter_round_trip(self):
        original = 2.8
        m = to_meters(original, DistanceUnit.IN)
        back = convert(m, DistanceUnit.M, DistanceUnit.IN)
        assert math.isclose(back, original, rel_tol=1e-9)

    def test_format_meters(self):
        assert format_distance(1.234, DistanceUnit.M) == "1.23 m"

    def test_format_inches(self):
        assert format_distance(48.0, DistanceUnit.IN) == "48 in"

    def test_from_json_si(self):
        assert DistanceUnit.from_json("meters") is DistanceUnit.M

    def test_from_json_unknown(self):
        with pytest.raises(ValueError):
            DistanceUnit.from_json("ft")

    def test_effective_robot_override(self):
        class Season:
            distance_unit = DistanceUnit.IN

        class Robot:
            distance_unit_override = DistanceUnit.M

        assert effective_distance_unit(Robot(), Season()) is DistanceUnit.M
