"""Browser sim range fusion parity with Java/Python (issue #22)."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "pure"))

from vidar_pure import (
    build_floor_estimate,
    build_ground_plane_estimate,
    build_size_estimate,
    fuse_range_weighted,
)

NODE = shutil.which("node")
GEOMETRY_TEST = ROOT / "sim" / "js" / "geometry.test.mjs"


class TestSimGeometryParity:
    def test_geometry_disagreement_keeps_ground_distance_python(self):
        size = build_size_estimate(23, 20, 0.9, False, False)
        floor = build_floor_estimate(23, 60, 0.8, False)
        ground = build_ground_plane_estimate(42, 200, 0.8, False)
        result = fuse_range_weighted(size, floor, ground)
        assert result.is_valid
        assert result.distance == pytest.approx(42.0)
        assert result.confidence < 0.5

    def test_three_way_fusion_when_geometry_agrees_python(self):
        size = build_size_estimate(24, 14, 0.9, False, False)
        floor = build_floor_estimate(25, 200, 0.8, False)
        ground = build_ground_plane_estimate(24.5, 200, 0.8, False)
        result = fuse_range_weighted(size, floor, ground)
        assert result.distance == pytest.approx(24.5)
        assert result.confidence > 0.7

    @pytest.mark.skipif(NODE is None, reason="node not installed")
    def test_sim_geometry_js_matches_contract(self):
        proc = subprocess.run(
            [NODE, str(GEOMETRY_TEST)],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0, proc.stderr or proc.stdout
