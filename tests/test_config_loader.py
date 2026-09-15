import json
import math
from pathlib import Path

import pytest

from vidar.config_loader import load_robot, load_season, parse_season
from vidar.models import ElementDetectorType, FixtureLocalizationMode
from vidar.units import DistanceUnit


ROOT = Path(__file__).resolve().parents[1]
SEASON_FILES = sorted((ROOT / "config/seasons").glob("*.json"))
ROBOT_FILES = sorted((ROOT / "config/robots").glob("example*.json"))
BUNDLED_SEASON = ROOT / "teamcode/assets/vidar/default-season.json"
BUNDLED_ROBOT = ROOT / "teamcode/assets/vidar/default-robot.json"


BIOBUZZ_FLOWER_POSES = {
    "flower_audience": (-23.3926, -68.0416, 90),
    "flower_opposite_audience": (23.3926, 68.0416, -90),
    "flower_red": (-68.0416, 23.3926, 0),
    "flower_blue": (68.0416, -23.3926, 180),
}


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
    assert len(season.fixtures) == 4
    for fixture_id, (x, y, yaw) in BIOBUZZ_FLOWER_POSES.items():
        spec = season.fixture_by_id(fixture_id)
        assert spec is not None
        assert spec.localization is FixtureLocalizationMode.STATIC_FIELD
        assert spec.x == pytest.approx(x)
        assert spec.y == pytest.approx(y)
        assert math.isnan(spec.z)
        assert spec.yaw_deg == pytest.approx(yaw)
        assert spec.detectors == ()
        assert spec.tag_ids == ()
        assert not spec.has_field_position()


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
    fixtures = {item["id"]: item for item in raw["fixtures"]}
    assert set(fixtures) == set(BIOBUZZ_FLOWER_POSES)
    geom = raw["flowerGeometry"]
    assert geom["topOpeningHeight"] == pytest.approx(21.5)
    for flower_id, (x, y, yaw) in BIOBUZZ_FLOWER_POSES.items():
        pose = fixtures[flower_id]
        assert pose["localization"] == "static_field"
        assert pose["position"]["x"] == pytest.approx(x)
        assert pose["position"]["y"] == pytest.approx(y)
        assert "z" not in pose["position"]
        assert pose["orientationDeg"]["yaw"] == yaw
        assert "detectors" not in pose
        assert "tagIds" not in pose
        assert pose["citation"]
    poses = {pose["id"]: pose for pose in raw["namedPoses"]}
    assert set(poses) == {"hive_structure"}
    hive = poses["hive_structure"]
    assert hive["positionIn"]["x"] == pytest.approx(0)
    assert hive["positionIn"]["y"] == pytest.approx(0)
    assert hive["positionIn"]["z"] == pytest.approx(43.95)
    pipes = raw["flowerGeometry"]["pipes"]
    assert pipes["outerDiameter"] == pytest.approx(1.05)
    assert pipes["innerDiameter"] == pytest.approx(0.85)
    assert pipes["wallParallelSpacing"] == pytest.approx(3.45)
    assert pipes["depthSpacing"] == pytest.approx(3.45)
    assert pipes["count"] == 4
    assert pipes["colorRgb255"] == [95, 167, 61]
    nectar_red = next(el for el in raw["elements"] if el["id"] == "nectar_red")
    assert "hsvWrap" not in nectar_red


def _minimal_season(**extra):
    data = {
        "seasonId": "fixture-test",
        "field": {"length": 144, "width": 144},
        "elements": [
            {
                "id": "ball",
                "label": "Ball",
                "diameter": 4,
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
    data.update(extra)
    return data


def test_omitted_fixtures_key_loads_empty_tuple():
    season = parse_season(_minimal_season())
    assert season.fixtures == ()
    assert season.elements[0].id == "ball"


def test_synthetic_static_field_fixture_loads():
    season = parse_season(
        _minimal_season(
            fixtures=[
                {
                    "id": "flower_1",
                    "label": "Flower 1",
                    "localization": "static_field",
                    "position": {"x": 12.5, "y": -64.0, "z": 21.5},
                    "orientationDeg": {"yaw": 90},
                    "detectors": ["ordered_stack"],
                }
            ]
        )
    )
    spec = season.fixture_by_id("flower_1")
    assert spec is not None
    assert spec.label == "Flower 1"
    assert spec.localization is FixtureLocalizationMode.STATIC_FIELD
    assert spec.x == pytest.approx(12.5)
    assert spec.y == pytest.approx(-64.0)
    assert spec.z == pytest.approx(21.5)
    assert spec.yaw_deg == pytest.approx(90)
    assert spec.detectors == ("ordered_stack",)
    assert spec.tag_ids == ()
    assert spec.has_field_position()


def test_unknown_fixture_localization_fails_loudly():
    with pytest.raises(ValueError, match="Unknown fixture localization"):
        parse_season(_minimal_season(fixtures=[{"id": "bad", "localization": "nope"}]))


def test_blank_fixture_id_fails():
    with pytest.raises(ValueError, match="Fixture id is required"):
        parse_season(_minimal_season(fixtures=[{"id": "  ", "localization": "static_field"}]))


def test_empty_fixture_label_defaults_to_id():
    season = parse_season(
        _minimal_season(fixtures=[{"id": "flower_1", "label": "", "localization": "static_field"}])
    )
    assert season.fixture_by_id("flower_1").label == "flower_1"


@pytest.mark.parametrize("season_path", SEASON_FILES, ids=lambda p: p.stem)
def test_all_season_json_files_load(season_path: Path):
    season = load_season(season_path)
    assert season.season_id
    assert len(season.elements) >= 1
    assert len(season.plates) >= 1
    if season.season_id == "2026-biobuzz":
        assert len(season.fixtures) == 4
    else:
        assert season.fixtures == ()
    for element in season.elements:
        assert element.diameter > 0
        assert element.detector in ElementDetectorType


def test_empty_fixtures_array_loads_empty_tuple():
    season = parse_season(_minimal_season(fixtures=[]))
    assert season.fixtures == ()
    assert season.elements[0].id == "ball"


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
