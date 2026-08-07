"""Bundled Java default config assets match VidarConfig constants."""

from __future__ import annotations

from pathlib import Path

import pytest

from vidar.config_loader import load_robot, load_season

ROOT = Path(__file__).resolve().parents[1]
BUNDLED_SEASON = ROOT / "teamcode/assets/vidar/default-season.json"
BUNDLED_ROBOT = ROOT / "teamcode/assets/vidar/default-robot.json"


def test_bundled_default_season_loads():
    season = load_season(BUNDLED_SEASON)
    assert season.season_id == "2025-decode"
    assert len(season.elements) == 1
    assert season.elements[0].id == "pollen"
    assert len(season.plates) == 2
    assert season.min_element_confidence == pytest.approx(0.35)
    assert season.max_range_mismatch_ratio == pytest.approx(0.28)


def test_bundled_default_robot_loads():
    robot = load_robot(BUNDLED_ROBOT)
    assert robot.robot_name == "example-robot"
    assert len(robot.cameras) >= 1
    assert robot.cameras[0].profile.focal_length_px == pytest.approx(340)
