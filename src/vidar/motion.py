from __future__ import annotations

import math
from dataclasses import dataclass


def normalize_deg(deg: float) -> float:
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


@dataclass(frozen=True)
class MotionTransform:
    delta_x: float
    delta_y: float
    delta_heading_deg: float

    def transform_point(self, prev_x: float, prev_y: float) -> tuple[float, float]:
        rad = math.radians(-self.delta_heading_deg)
        cos_v = math.cos(rad)
        sin_v = math.sin(rad)
        rx = prev_x * cos_v - prev_y * sin_v
        ry = prev_x * sin_v + prev_y * cos_v
        return rx - self.delta_x, ry - self.delta_y

    @staticmethod
    def from_odom_delta(
        px: float, py: float, ph: float, cx: float, cy: float, ch: float
    ) -> MotionTransform:
        dx = cx - px
        dy = cy - py
        dh = normalize_deg(ch - ph)
        rad = math.radians(-ph)
        cos_v = math.cos(rad)
        sin_v = math.sin(rad)
        robot_dx = dx * cos_v - dy * sin_v
        robot_dy = dx * sin_v + dy * cos_v
        return MotionTransform(robot_dx, robot_dy, dh)


class LocalizationFusion:
    MAX_TRANSLATION = 18.0
    MAX_HEADING = 25.0
    MAX_AGE_MS = 500

    def __init__(self) -> None:
        self.last_fused: tuple[float, float, float] | None = None

    def fused_field_pose_now(
        self,
        decoded_tag: dict | None,
        scout_obs: dict | None,
    ) -> tuple[float, float, float] | None:
        if scout_obs is not None and decoded_tag is None:
            return self.last_fused
        if decoded_tag is None:
            return self.last_fused
        if decoded_tag.get("id", -1) < 0:
            return self.last_fused
        if decoded_tag.get("age_ms", 0) > self.MAX_AGE_MS:
            return self.last_fused
        pose = decoded_tag["pose"]
        if self.last_fused is not None:
            dx = pose[0] - self.last_fused[0]
            dy = pose[1] - self.last_fused[1]
            if math.hypot(dx, dy) > self.MAX_TRANSLATION:
                return self.last_fused
        self.last_fused = tuple(pose)
        return self.last_fused

    @staticmethod
    def would_scout_alter_pose(_scout: dict | None) -> bool:
        return False
