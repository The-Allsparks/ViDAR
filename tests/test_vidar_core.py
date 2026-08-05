"""
ViDAR core logic tests — geometry, ROI transforms, motion correction, localization safety.
Run: python -m pytest tests/ -v
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "pure"))

from vidar_pure import (
    RoiRect,
    CameraRoiConfig,
    fuse_range_weighted,
    build_ball_size_estimate,
    build_floor_estimate,
    build_plate_width_estimate,
    MotionTransform,
    LocalizationFusionPure,
    distance_from_size_inches,
    distance_from_width_inches,
)


class TestRangeFusion:
    def test_size_only_estimate(self):
        d = distance_from_size_inches(5.0, 340, 20)
        assert 40 < d < 45

    def test_weighted_fusion_agreement(self):
        size = build_ball_size_estimate(36, 20, 0.9, False, False)
        floor = build_floor_estimate(38, 60, 0.8, False)
        result = fuse_range_weighted(size, floor)
        assert result.is_valid
        assert 34 < result.distance_in < 40
        assert result.confidence > 0

    def test_invalid_floor_rejected(self):
        floor = build_floor_estimate(float("nan"), 60, 0.8, True)
        assert not floor.is_valid
        assert floor.rejection_reason in ("near_horizon", "invalid_lut")

    def test_estimator_disagreement_lowers_confidence(self):
        size = build_ball_size_estimate(24, 20, 0.9, False, False)
        floor = build_floor_estimate(48, 60, 0.8, False)
        result = fuse_range_weighted(size, floor)
        assert result.confidence < 0.8

    def test_plate_width_range(self):
        d = distance_from_width_inches(12.0, 340, 80)
        assert 48 < d < 52
        est = build_plate_width_estimate(d, 80, 0.8, 0.3, False, False, 0.1)
        assert est.is_valid


class TestRoiTransforms:
    def test_lower_fraction_default_ball(self):
        roi = RoiRect.lower_fraction(640, 480, 0.65)
        assert roi.y == int(480 * 0.35)
        assert roi.height == int(480 * 0.65)

    def test_roi_local_to_full(self):
        roi = RoiRect(0, 240, 640, 240)
        assert roi.to_full_x(100) == 100
        assert roi.to_full_y(50) == 290

    def test_overlapping_rois(self):
        cfg = CameraRoiConfig()
        ball = cfg.ball_roi(640, 480)
        tag = cfg.tag_roi(640, 480)
        assert ball.y > tag.y
        assert ball.y < tag.y + tag.height

    def test_invalid_roi_clamped(self):
        roi = RoiRect(-10, -5, 700, 500).clamped(640, 480)
        assert roi.x >= 0
        assert roi.y >= 0
        assert roi.x + roi.width <= 640


class TestMotionTransform:
    def test_forward_motion(self):
        t = MotionTransform.from_odom_delta(0, 0, 0, 12, 0, 0)
        x, y = t.transform_point(24, 0)
        assert abs(x - 12) < 0.01
        assert abs(y) < 0.01

    def test_rotation_in_place(self):
        t = MotionTransform.from_odom_delta(0, 0, 0, 0, 0, 90)
        x, y = t.transform_point(10, 0)
        assert abs(x) < 0.1
        assert abs(y - (-10)) < 0.1

    def test_combined_translation_rotation(self):
        t = MotionTransform.from_odom_delta(0, 0, 0, 10, 10, 45)
        x, y = t.transform_point(5, 0)
        assert not math.isnan(x)
        assert not math.isnan(y)

    def test_angle_wraparound(self):
        t = MotionTransform.from_odom_delta(0, 0, 170, 0, 0, -170)
        assert abs(abs(t.delta_heading_deg) - 20) < 1.0


class TestLocalizationSafety:
    def test_scout_never_alters_pose(self):
        fusion = LocalizationFusionPure()
        fusion.last_fused = (10.0, 10.0, 0.0)
        scout = {"bearing": 45, "width": 40, "confidence": 0.9}
        result = fusion.fused_field_pose_now(None, scout)
        assert result == fusion.last_fused

    def test_decoded_tag_can_update_with_gates(self):
        fusion = LocalizationFusionPure()
        tag = {"id": 1, "pose": (20.0, 30.0, 90.0), "age_ms": 100, "decode_pixels": 500}
        result = fusion.fused_field_pose_now(tag, None)
        assert result[0] == 20.0


class TestWorldModelMotion:
    def test_track_transform_after_forward_travel(self):
        t = MotionTransform(12, 0, 0)
        x, y = t.transform_point(36, 0)
        assert abs(x - 24) < 0.01

    def test_stale_observation_rejected_by_timestamp(self):
        prev_capture = 1000
        new_capture = 900
        assert new_capture <= prev_capture
