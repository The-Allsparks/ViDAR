import json
from pathlib import Path

import pytest

from vidar.config_loader import load_robot, load_season, parse_season
from vidar.models import ElementDetectorType
from vidar.units import DistanceUnit


ROOT = Path(__file__).resolve().parents[1]
SEASON_FILES = sorted((ROOT / "config/seasons").glob("*.json"))
ROBOT_FILES = sorted((ROOT / "config/robots").glob("example*.json"))
BUNDLED_SEASON = ROOT / "teamcode/assets/vidar/default-season.json"
BUNDLED_ROBOT = ROOT / "teamcode/assets/vidar/default-robot.json"


def test_load_biobuzz_season():
    season = load_season(ROOT / "config/seasons/2026-biobuzz.json")
    assert season.season_id == "2026-biobuzz"
    assert len(season.elements) == 3
    assert season.elements[0].id == "pollen"
    by_id = {el.id: el for el in season.elements}
    assert by_id["nectar_red"].diameter == pytest.approx(3.6)
    assert by_id["nectar_blue"].diameter == pytest.approx(3.6)
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
    assert robot.cameras[0].profile.focal_length_px == pytest.approx(492)


@pytest.mark.parametrize("robot_path", ROBOT_FILES, ids=lambda p: p.stem)
def test_all_robot_json_files_load(robot_path: Path):
    robot = load_robot(robot_path)
    assert len(robot.cameras) >= 1
    assert robot.cameras[0].profile.focal_length_px > 0
    fx = robot.cameras[0].profile.focal_length_px
    if "c920" in robot_path.stem:
        assert fx == pytest.approx(680)
    elif "svpro" in robot_path.stem or robot_path.stem == "example-robot":
        assert fx == pytest.approx(492)


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


def test_biobuzz_season_hive_tags_not_landmarks():
    season = load_season(ROOT / "config/seasons/2026-biobuzz.json")
    assert season.default_tag_size == pytest.approx(3.25)
    assert len(season.april_tags) == 16
    assert [tag.id for tag in season.april_tags] == list(range(30, 46))
    assert all(not tag.localization for tag in season.april_tags)
    assert season.localization_tags() == ()
    for tag_id in range(30, 46):
        spec = season.tag_by_id(tag_id)
        assert spec is not None
        assert not spec.localization


def test_biobuzz_season_raw_json_named_poses_and_extra_keys():
    raw = json.loads((ROOT / "config/seasons/2026-biobuzz.json").read_text(encoding="utf-8"))
    assert "fixtures" not in raw
    tags = raw["apriltags"]["tags"]
    assert len(tags) == 16
    for tag in tags:
        assert tag["localization"] is False
        assert tag["size"] == pytest.approx(3.25)
        assert tag["cluster"]
        assert tag["alliance"] in {"red", "blue"}
        assert tag["cell"] in {"audience", "opposite_audience"}
        assert "positionIn" not in tag
        assert "orientationDeg" not in tag
    poses = {pose["id"]: pose for pose in raw["namedPoses"]}
    flower_ids = {
        "flower_audience",
        "flower_opposite_audience",
        "flower_red",
        "flower_blue",
    }
    assert flower_ids <= poses.keys()
    for flower_id in flower_ids:
        assert poses[flower_id]["positionIn"]["z"] == pytest.approx(21.5)
        assert "x" not in poses[flower_id]["positionIn"]
        assert "y" not in poses[flower_id]["positionIn"]
        assert "orientationDeg" not in poses[flower_id]
        assert poses[flower_id]["citation"]
    hive = poses["hive_structure"]
    assert hive["positionIn"]["x"] == pytest.approx(0)
    assert hive["positionIn"]["y"] == pytest.approx(0)
    assert hive["positionIn"]["z"] == pytest.approx(43.95)


@pytest.mark.parametrize("season_path", SEASON_FILES, ids=lambda p: p.stem)
def test_all_season_json_files_load(season_path: Path):
    season = load_season(season_path)
    assert season.season_id
    assert len(season.elements) >= 1
    assert len(season.plates) >= 1
    for element in season.elements:
        assert element.diameter > 0
        assert element.detector in ElementDetectorType


def test_bundled_default_season_matches_legacy_fusion_defaults():
    """Bundled fallback mirrors VidarConfig fusion constants, not team season templates."""
    bundled = load_season(BUNDLED_SEASON)
    decode = load_season(ROOT / "config/seasons/2025-decode.json")
    assert bundled.season_id == "2025-decode"
    assert bundled.season_id == decode.season_id
    assert bundled.elements[0].id == "pollen"
    assert bundled.min_element_confidence == decode.min_element_confidence
    assert bundled.max_range_mismatch_ratio == decode.max_range_mismatch_ratio


def test_bundled_default_robot_loads():
    robot = load_robot(BUNDLED_ROBOT)
    assert robot.robot_name == "example-robot"
    assert len(robot.cameras) >= 1
    assert robot.cameras[0].profile.focal_length_px == pytest.approx(492)
