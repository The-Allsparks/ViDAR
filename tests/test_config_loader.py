from pathlib import Path

import pytest

from vidar.config_loader import load_robot, load_season, parse_season
from vidar.models import ElementDetectorType
from vidar.units import DistanceUnit


ROOT = Path(__file__).resolve().parents[1]
SEASON_FILES = sorted((ROOT / "config/seasons").glob("*.json"))


def test_load_biobuzz_season():
    season = load_season(ROOT / "config/seasons/2026-biobuzz.json")
    assert season.season_id == "2026-biobuzz"
    assert len(season.elements) == 1
    assert season.elements[0].id == "pollen"
    assert len(season.plates) == 2
    assert season.distance_unit is DistanceUnit.IN


def test_season_distance_unit_meters():
    season = parse_season(
        {
            "seasonId": "metric-test",
            "distanceUnit": "m",
            "field": {"length": 17.5, "width": 8.0},
            "elements": [
                {
                    "id": "ball",
                    "label": "Ball",
                    "diameter": 0.071,
                    "detector": "color_blob",
                    "hsv": {
                        "hMin": 0,
                        "hMax": 10,
                        "sMin": 0,
                        "sMax": 255,
                        "vMin": 0,
                        "vMax": 255,
                    },
                }
            ],
            "plates": [],
        }
    )
    assert season.distance_unit is DistanceUnit.M
    assert season.elements[0].diameter == 0.071


def test_load_example_robot():
    robot = load_robot(ROOT / "config/robots/example-robot.json")
    assert robot.robot_name == "example-robot"
    assert len(robot.cameras) == 4
    assert robot.cameras[0].profile.name == "front"
    assert robot.cameras[0].profile.floor_lut


def test_decode_season_april_tags():
    season = load_season(ROOT / "config/seasons/2025-decode.json")
    assert len(season.april_tags) == 5
    assert season.default_tag_size == pytest.approx(8.125)
    tag20 = season.tag_by_id(20)
    assert tag20 is not None
    assert tag20.localization
    assert tag20.x == pytest.approx(-58.35)
    motif = season.tag_by_id(21)
    assert motif is not None
    assert not motif.localization
    assert len(season.localization_tags()) == 2


def test_biobuzz_season_empty_april_tags():
    season = load_season(ROOT / "config/seasons/2026-biobuzz.json")
    assert season.april_tags == ()
    assert season.default_tag_size == pytest.approx(6.5)


@pytest.mark.parametrize("season_path", SEASON_FILES, ids=lambda p: p.stem)
def test_all_season_json_files_load(season_path: Path):
    season = load_season(season_path)
    assert season.season_id
    assert len(season.elements) >= 1
    assert len(season.plates) >= 1
    for element in season.elements:
        assert element.diameter > 0
        assert element.detector in ElementDetectorType
