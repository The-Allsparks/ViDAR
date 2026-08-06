"""Formal coordinate-frame, transform, and camera calibration foundation (Java parity)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

from vidar.models import CameraProfile


class FrameId(str, Enum):
    FIELD = "field"
    ROBOT = "robot"
    CAMERA_OPTICAL = "camera_optical"

    @staticmethod
    def camera_body(name: str) -> str:
        return f"camera_{name}" if name else "camera"


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def is_finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.z)

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self) -> Vec3:
        length = self.length()
        if length <= 1e-9:
            return Vec3(0.0, 0.0, 1.0)
        return Vec3(self.x / length, self.y / length, self.z / length)

    def plus(self, other: Vec3) -> Vec3:
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def scaled(self, s: float) -> Vec3:
        return Vec3(self.x * s, self.y * s, self.z * s)

    def dot(self, other: Vec3) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z


def _mat3_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    out = [0.0] * 9
    for r in range(3):
        for c in range(3):
            out[r * 3 + c] = (
                a[r * 3 + 0] * b[0 * 3 + c]
                + a[r * 3 + 1] * b[1 * 3 + c]
                + a[r * 3 + 2] * b[2 * 3 + c]
            )
    return tuple(out)


def _rotate_x(rad: float) -> tuple[float, ...]:
    c, s = math.cos(rad), math.sin(rad)
    return (1, 0, 0, 0, c, -s, 0, s, c)


def _rotate_z(rad: float) -> tuple[float, ...]:
    c, s = math.cos(rad), math.sin(rad)
    return (c, -s, 0, s, c, 0, 0, 0, 1)


def optical_to_robot_base() -> tuple[float, ...]:
    return (0, 0, 1, -1, 0, 0, 0, -1, 0)


@dataclass(frozen=True)
class Rotation3D:
    m: tuple[float, ...]

    @staticmethod
    def identity() -> Rotation3D:
        return Rotation3D((1, 0, 0, 0, 1, 0, 0, 0, 1))

    @staticmethod
    def from_roll_pitch_yaw_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Rotation3D:
        r_roll = _rotate_z(math.radians(roll_deg))
        r_pitch = _rotate_x(math.radians(pitch_deg))
        r_yaw = _rotate_z(math.radians(yaw_deg))
        return Rotation3D(_mat3_mul(_mat3_mul(r_yaw, r_pitch), r_roll))

    def times(self, other: Rotation3D) -> Rotation3D:
        return Rotation3D(_mat3_mul(self.m, other.m))

    def inverse(self) -> Rotation3D:
        m = self.m
        return Rotation3D((m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]))

    def rotate(self, v: Vec3) -> Vec3:
        m = self.m
        return Vec3(
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z,
        )


@dataclass(frozen=True)
class Transform3D:
    dest_frame: FrameId
    source_frame: FrameId
    rotation: Rotation3D
    translation: Vec3
    dest_label: str | None = None
    source_label: str | None = None

    def inverse(self) -> Transform3D:
        inv_r = self.rotation.inverse()
        inv_t = inv_r.rotate(self.translation.scaled(-1))
        return Transform3D(
            self.source_frame,
            self.dest_frame,
            inv_r,
            inv_t,
            self.source_label,
            self.dest_label,
        )

    def compose(self, other: Transform3D) -> Transform3D:
        r = self.rotation.times(other.rotation)
        t = self.rotation.rotate(other.translation).plus(self.translation)
        return Transform3D(self.dest_frame, other.source_frame, r, t, self.dest_label, other.source_label)

    def transform_point(self, p: Vec3) -> Vec3:
        return self.rotation.rotate(p).plus(self.translation)

    def transform_direction(self, d: Vec3) -> Vec3:
        return self.rotation.rotate(d).normalized()

    @property
    def notation_name(self) -> str:
        dest = self.dest_label or self.dest_frame.value
        src = self.source_label or self.source_frame.value
        return f"{dest}_T_{src}"

    @staticmethod
    def identity(frame: FrameId = FrameId.ROBOT) -> Transform3D:
        return Transform3D(frame, frame, Rotation3D.identity(), Vec3(0, 0, 0))


class DistortionModel(str, Enum):
    NONE = "none"
    BROWN_CONRADY = "brown_conrady"
    FISHEYE = "fisheye"


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    image_width: int
    image_height: int
    distortion_model: DistortionModel = DistortionModel.NONE
    distortion_coeffs: tuple[float, ...] = ()
    calibration_version: str | None = None
    calibration_date: str | None = None

    @staticmethod
    def from_profile(profile: CameraProfile, default_w: int = 640, default_h: int = 480) -> CameraIntrinsics:
        w = profile.calibration_width or default_w
        h = profile.calibration_height or default_h
        model = profile.distortion_model if hasattr(profile, "distortion_model") else DistortionModel.NONE
        coeffs = getattr(profile, "distortion_coeffs", ()) or ()
        return CameraIntrinsics(
            profile.focal_length_px,
            profile.focal_length_y_px,
            profile.principal_point_x,
            profile.principal_point_y,
            w,
            h,
            model,
            tuple(coeffs),
            getattr(profile, "calibration_version", None),
            getattr(profile, "calibration_date", None),
        )

    def is_valid(self) -> bool:
        return (
            self.fx > 0
            and self.fy > 0
            and self.image_width > 0
            and self.image_height > 0
            and self.distortion_model != DistortionModel.FISHEYE
        )

    def pixel_to_ray(self, px: float, py: float) -> Vec3:
        if not self.is_valid():
            return Vec3(math.nan, math.nan, math.nan)
        u = (px + 0.5 - self.cx) / self.fx
        v = (py + 0.5 - self.cy) / self.fy
        return Vec3(u, v, 1.0).normalized()

    def point_to_pixel(self, point: Vec3) -> tuple[float, float]:
        if not self.is_valid() or point.z <= 1e-9:
            return (math.nan, math.nan)
        u = point.x / point.z
        v = point.y / point.z
        return (u * self.fx + self.cx - 0.5, v * self.fy + self.cy - 0.5)


@dataclass(frozen=True)
class ImageTransform:
    crop_x: int
    crop_y: int
    source_width: int
    source_height: int
    processed_width: int
    processed_height: int
    scale_x: float
    scale_y: float

    @staticmethod
    def identity(width: int, height: int) -> ImageTransform:
        return ImageTransform(0, 0, width, height, width, height, 1.0, 1.0)

    @staticmethod
    def from_crop_and_scale(
        crop_x: int,
        crop_y: int,
        source_width: int,
        source_height: int,
        processed_width: int,
        processed_height: int,
    ) -> ImageTransform | None:
        if processed_width <= 0 or processed_height <= 0:
            return None
        return ImageTransform(
            crop_x,
            crop_y,
            source_width,
            source_height,
            processed_width,
            processed_height,
            source_width / processed_width,
            source_height / processed_height,
        )

    def is_uniform_scale(self) -> bool:
        return abs(self.scale_x - self.scale_y) < 1e-6

    def to_sensor_x(self, processed_x: float) -> float:
        return (processed_x + 0.5) * self.scale_x + self.crop_x - 0.5

    def to_sensor_y(self, processed_y: float) -> float:
        return (processed_y + 0.5) * self.scale_y + self.crop_y - 0.5

    def to_processed_x(self, sensor_x: float) -> float:
        return (sensor_x + 0.5 - self.crop_x) / self.scale_x - 0.5

    def to_processed_y(self, sensor_y: float) -> float:
        return (sensor_y + 0.5 - self.crop_y) / self.scale_y - 0.5


@dataclass(frozen=True)
class CameraTransforms:
    camera_name: str
    intrinsics: CameraIntrinsics
    robot_t_camera: Transform3D

    @property
    def camera_t_robot(self) -> Transform3D:
        return self.robot_t_camera.inverse()


def build_robot_t_camera(profile: CameraProfile) -> CameraTransforms:
    mount_rot = (
        Rotation3D(_rotate_z(math.radians(profile.bearing_deg + profile.mount_yaw_deg)))
        .times(Rotation3D(_rotate_x(math.radians(profile.mount_pitch_deg))))
        .times(Rotation3D(_rotate_z(math.radians(profile.mount_roll_deg))))
    )
    rot = mount_rot.times(Rotation3D(optical_to_robot_base()))
    trans = Vec3(profile.mount_x, profile.mount_y, profile.mount_z)
    robot_t_camera = Transform3D(
        FrameId.ROBOT,
        FrameId.CAMERA_OPTICAL,
        rot,
        trans,
        "robot",
        FrameId.camera_body(profile.name),
    )
    return CameraTransforms(profile.name, CameraIntrinsics.from_profile(profile), robot_t_camera)


def ray_direction_robot_frame(cx: float, cy: float, profile: CameraProfile) -> tuple[float, float, float]:
    ct = build_robot_t_camera(profile)
    ray = ct.robot_t_camera.transform_direction(ct.intrinsics.pixel_to_ray(cx, cy))
    return (ray.x, ray.y, ray.z)


@dataclass(frozen=True)
class GroundIntersection:
    robot_x: float
    robot_y: float
    slant_range: float
    horizontal_uncertainty: float
    valid: bool
    rejection_reason: str | None = None

    @staticmethod
    def rejected(reason: str) -> GroundIntersection:
        return GroundIntersection(math.nan, math.nan, math.nan, math.nan, False, reason)


def intersect_ground_plane(
    origin: Vec3, direction: Vec3, slant_uncertainty: float = math.nan, *, plane_z: float = 0.0
) -> GroundIntersection:
    if not origin.is_finite() or not direction.is_finite():
        return GroundIntersection.rejected("invalid_ray")
    dz = direction.z
    if abs(dz) < 1e-6:
        return GroundIntersection.rejected("parallel_to_plane")
    if origin.z <= plane_z and dz >= 0:
        return GroundIntersection.rejected("pointing_away_from_plane")
    t = (plane_z - origin.z) / dz
    if t <= 0:
        return GroundIntersection.rejected("behind_camera")
    hit = origin.plus(direction.scaled(t))
    horizontal = math.hypot(hit.x, hit.y)
    horiz_unc = (
        math.nan
        if math.isnan(slant_uncertainty)
        else slant_uncertainty * (horizontal / max(t, 1e-6))
    )
    return GroundIntersection(hit.x, hit.y, t, horiz_unc, True, None)


def distance_from_ground_plane(
    cx: float, cy: float, profile: CameraProfile, target_height_z: float
) -> float:
    ct = build_robot_t_camera(profile)
    origin = ct.robot_t_camera.translation
    direction = ct.robot_t_camera.transform_direction(ct.intrinsics.pixel_to_ray(cx, cy))
    hit = intersect_ground_plane(origin, direction, plane_z=target_height_z)
    return hit.slant_range if hit.valid else math.nan


def floor_point_from_slant_range(
    profile: CameraProfile, cx: float, cy: float, slant_range: float
) -> GroundIntersection:
    if math.isnan(slant_range) or slant_range <= 0:
        return GroundIntersection.rejected("invalid_range")
    ct = build_robot_t_camera(profile)
    origin = ct.robot_t_camera.translation
    direction = ct.robot_t_camera.transform_direction(ct.intrinsics.pixel_to_ray(cx, cy))
    plane = intersect_ground_plane(origin, direction)
    if not plane.valid:
        return plane
    scaled = origin.plus(direction.scaled(slant_range))
    return GroundIntersection(scaled.x, scaled.y, slant_range, math.nan, True, None)
