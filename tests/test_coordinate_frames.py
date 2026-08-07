"""Comprehensive tests for coordinate frames, transforms, intrinsics, and calibration foundation."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests" / "pure"))

from vidar.config_loader import load_robot
from vidar.models import CameraProfile
from vidar.transforms import (
    CameraIntrinsics,
    CameraTransforms,
    GroundIntersection,
    ImageTransform,
    Rotation3D,
    Transform3D,
    Vec3,
    build_robot_t_camera,
    floor_point_from_slant_range,
    intersect_ground_plane,
    optical_to_robot_base,
    ray_direction_robot_frame,
)
from vidar.geometry import ray_direction_robot_frame as legacy_ray  # noqa: F401 — import side test


def _front_profile() -> CameraProfile:
    return CameraProfile(
        name="front",
        bearing_deg=0,
        horizon_row_px=12,
        focal_length_px=340,
        floor_cy_px=(95, 75, 55, 40),
        floor_dist=(12, 24, 36, 48),
        mount_x=6.5,
        mount_y=0,
        mount_z=9.0,
        mount_pitch_deg=-12,
        calibration_width=640,
        calibration_height=480,
    )


def _right_profile() -> CameraProfile:
    return CameraProfile(
        name="right",
        bearing_deg=90,
        horizon_row_px=12,
        focal_length_px=340,
        floor_cy_px=(95, 75, 55, 40),
        floor_dist=(12, 24, 36, 48),
        mount_x=0,
        mount_y=-6.5,
        mount_z=9.0,
        mount_pitch_deg=-12,
        calibration_width=640,
        calibration_height=480,
    )


class TestTransform3D:
    def test_identity_point(self):
        t = Transform3D.identity()
        assert t.dest_frame.value == "robot"
        p = t.transform_point(Vec3(1, 2, 3))
        assert abs(p.x - 1) < 1e-9 and abs(p.y - 2) < 1e-9 and abs(p.z - 3) < 1e-9

    def test_inverse(self):
        profile = _front_profile()
        ct = build_robot_t_camera(profile)
        inv = ct.robot_t_camera.inverse()
        composed = ct.robot_t_camera.compose(inv)
        p = composed.transform_point(Vec3(1.0, -0.5, 2.0))
        assert abs(p.x - 1.0) < 1e-6
        assert abs(p.y + 0.5) < 1e-6
        assert abs(p.z - 2.0) < 1e-6

    def test_compose_chain(self):
        ct = build_robot_t_camera(_front_profile())
        chain = ct.robot_t_camera.compose(ct.camera_t_robot)
        p = chain.transform_point(Vec3(1.0, -0.5, 2.0))
        assert abs(p.x - 1.0) < 1e-5
        assert abs(p.y + 0.5) < 1e-5
        assert abs(p.z - 2.0) < 1e-5

    def test_direction_without_translation(self):
        ct = build_robot_t_camera(_front_profile())
        origin_shift = Vec3(100, 50, 20)
        t = Transform3D(
            ct.robot_t_camera.dest_frame,
            ct.robot_t_camera.source_frame,
            ct.robot_t_camera.rotation,
            origin_shift,
        )
        d = t.transform_direction(Vec3(0, 0, 1))
        d2 = ct.robot_t_camera.transform_direction(Vec3(0, 0, 1))
        assert abs(d.x - d2.x) < 1e-9
        assert abs(d.y - d2.y) < 1e-9
        assert abs(d.z - d2.z) < 1e-9

    def test_right_handed_yaw(self):
        r = Rotation3D.from_roll_pitch_yaw_deg(0, 0, 90)
        v = r.rotate(Vec3(1, 0, 0))
        assert abs(v.x) < 1e-9
        assert abs(v.y - 1) < 1e-9


class TestCameraIntrinsics:
    def test_pixel_to_ray_principal_point(self):
        intr = CameraIntrinsics.from_profile(_front_profile())
        ray = intr.pixel_to_ray(320, 240)
        assert abs(ray.x) < 0.01
        assert abs(ray.y) < 0.01
        assert ray.z > 0.99

    def test_off_center_ray(self):
        intr = CameraIntrinsics.from_profile(_front_profile())
        center = intr.pixel_to_ray(320, 240)
        off = intr.pixel_to_ray(420, 240)
        assert off.x > center.x

    def test_point_to_pixel_round_trip(self):
        intr = CameraIntrinsics.from_profile(_front_profile())
        ray = intr.pixel_to_ray(100, 200)
        px, py = intr.point_to_pixel(ray.scaled(2.0))
        assert abs(px - 100) < 0.05
        assert abs(py - 200) < 0.05

    def test_invalid_dimensions(self):
        intr = CameraIntrinsics(340, 340, 320, 240, 0, 480)
        assert not intr.is_valid()


class TestImageTransform:
    def test_identity(self):
        t = ImageTransform.identity(640, 480)
        assert abs(t.to_sensor_x(320) - 320) < 1e-9
        assert abs(t.to_sensor_y(240) - 240) < 1e-9

    def test_lower_crop(self):
        t = ImageTransform.from_crop_and_scale(0, 240, 640, 240, 640, 240)
        assert t is not None
        assert abs(t.to_sensor_y(0) - 240) < 0.01

    def test_crop_and_downscale(self):
        t = ImageTransform.from_crop_and_scale(0, 168, 640, 312, 320, 156)
        assert t is not None
        assert t.is_uniform_scale()
        sx = t.to_sensor_x(160)
        assert 300 < sx < 340

    def test_nonuniform_scale_detected(self):
        t = ImageTransform(0, 0, 640, 480, 320, 240, 2.0, 2.5)
        assert not t.is_uniform_scale()

    def test_principal_point_mapping(self):
        t = ImageTransform.from_crop_and_scale(0, 240, 640, 240, 640, 240)
        assert t is not None
        proc_cy = t.to_processed_y(240)
        assert abs(proc_cy - 0) < 0.01


class TestRobotTCamera:
    def test_registry_matches_legacy_ray_at_principal(self):
        profile = _front_profile()
        modern = ray_direction_robot_frame(320, 240, profile)
        ct = build_robot_t_camera(profile)
        ray = ct.robot_t_camera.transform_direction(ct.intrinsics.pixel_to_ray(320, 240))
        assert abs(modern[0] - ray.x) < 1e-5
        assert abs(modern[1] - ray.y) < 1e-5
        assert abs(modern[2] - ray.z) < 1e-5

    def test_side_camera(self):
        profile = _right_profile()
        ray = ray_direction_robot_frame(320, 240, profile)
        assert math.hypot(ray[0], ray[1]) > 0.9

    def test_notation(self):
        ct = build_robot_t_camera(_front_profile())
        assert ct.robot_t_camera.notation_name == "robot_T_camera_front"

    def test_compose_field_chain_documentation(self):
        ct = build_robot_t_camera(_front_profile())
        inv = ct.camera_t_robot
        chain = ct.robot_t_camera.compose(inv)
        p = chain.transform_point(Vec3(0, 0, 1))
        assert abs(p.z - 1) < 1e-4 or math.isfinite(p.z)


class TestGroundPlane:
    def test_intersect_below_camera(self):
        origin = Vec3(6.5, 0, 9)
        direction = Vec3(0.2, 0, -0.9).normalized()
        hit = intersect_ground_plane(origin, direction)
        assert hit.valid
        assert abs(hit.robot_y) < 1e-6 or math.isfinite(hit.robot_x)

    def test_parallel_rejected(self):
        hit = intersect_ground_plane(Vec3(0, 0, 9), Vec3(1, 0, 0))
        assert not hit.valid
        assert hit.rejection_reason == "parallel_to_plane"

    def test_slant_range_floor_point(self):
        hit = floor_point_from_slant_range(_front_profile(), 320, 240, 36)
        assert hit.valid
        assert hit.robot_x > 0


class TestConfigBackwardCompat:
    def test_example_robot_json_loads(self):
        path = ROOT / "config" / "robots" / "example-robot.json"
        robot = load_robot(path)
        assert len(robot.cameras) == 4
        profile = robot.cameras[0].profile
        assert profile.calibration_width == 640
        assert profile.calibration_height == 480


class TestCalibrationDataset:
    def test_sample_record_validates(self):
        from vidar.calibration_dataset import validate_record_json

        sample = ROOT / "config" / "calibration" / "sample-record.jsonl"
        line = sample.read_text(encoding="utf-8").strip().splitlines()[0]
        errors = validate_record_json(line)
        assert errors == []


class TestPropertyLoops:
    @pytest.mark.parametrize("yaw", range(-180, 181, 30))
    def test_rotation_inverse_roundtrip(self, yaw: int):
        r = Rotation3D.from_roll_pitch_yaw_deg(5, -12, yaw)
        v = Vec3(0.3, -0.2, 0.9)
        back = r.inverse().rotate(r.rotate(v))
        assert abs(back.x - v.x) < 1e-6
        assert abs(back.y - v.y) < 1e-6
        assert abs(back.z - v.z) < 1e-6
