from __future__ import annotations

from dataclasses import dataclass

from vidar.models import CameraProfile, CameraRoiConfig


@dataclass(frozen=True)
class RoiRect:
    x: int
    y: int
    width: int
    height: int
    enabled: bool = True

    @staticmethod
    def lower_fraction(frame_w: int, frame_h: int, fraction: float) -> RoiRect:
        h = max(1, int(round(frame_h * fraction)))
        return RoiRect(0, frame_h - h, frame_w, h)

    @staticmethod
    def upper_fraction(frame_w: int, frame_h: int, fraction: float) -> RoiRect:
        h = max(1, int(round(frame_h * fraction)))
        return RoiRect(0, 0, frame_w, h)

    @staticmethod
    def middle_band(frame_w: int, frame_h: int, start_fraction: float, band_fraction: float) -> RoiRect:
        y = int(round(frame_h * start_fraction))
        h = max(1, int(round(frame_h * band_fraction)))
        return RoiRect(0, y, frame_w, min(h, frame_h - y))

    def to_cv_rect(self) -> tuple[int, int, int, int]:
        return (self.x, self.y, self.width, self.height)

    def touches_boundary(
        self, frame_w: int, frame_h: int, px: float, py: float, margin: float
    ) -> bool:
        return (
            px <= self.x + margin
            or px >= self.x + self.width - margin
            or py <= self.y + margin
            or py >= self.y + self.height - margin
            or px <= margin
            or py <= margin
            or px >= frame_w - margin
            or py >= frame_h - margin
        )

    def to_full_x(self, local_x: float) -> float:
        return local_x + self.x

    def to_full_y(self, local_y: float) -> float:
        return local_y + self.y

    def clamped(self, frame_w: int, frame_h: int) -> RoiRect:
        cx = max(0, min(self.x, frame_w - 1))
        cy = max(0, min(self.y, frame_h - 1))
        cw = min(self.width, frame_w - cx)
        ch = min(self.height, frame_h - cy)
        return RoiRect(cx, cy, max(1, cw), max(1, ch), self.enabled)

    def with_enabled(self, on: bool) -> RoiRect:
        return RoiRect(self.x, self.y, self.width, self.height, on)


def element_roi(config: CameraRoiConfig, frame_w: int, frame_h: int) -> RoiRect:
    roi = RoiRect.lower_fraction(frame_w, frame_h, config.element_lower_fraction)
    return roi if config.element_enabled else roi.with_enabled(False)


def plate_roi(config: CameraRoiConfig, frame_w: int, frame_h: int) -> RoiRect:
    roi = RoiRect.middle_band(frame_w, frame_h, config.plate_start_fraction, config.plate_band_fraction)
    return roi if config.plate_enabled else roi.with_enabled(False)


def tag_roi(config: CameraRoiConfig, frame_w: int, frame_h: int) -> RoiRect:
    roi = RoiRect.upper_fraction(frame_w, frame_h, config.tag_upper_fraction)
    return roi if config.tag_enabled else roi.with_enabled(False)


def detection_roi(profile: CameraProfile, frame_w: int, frame_h: int) -> RoiRect:
    """Union of element and plate bands — one crop for shared contour processing."""
    element = element_roi(profile.roi_config, frame_w, frame_h).clamped(frame_w, frame_h)
    plate = plate_roi(profile.roi_config, frame_w, frame_h).clamped(frame_w, frame_h)
    top = min(element.y, plate.y)
    bottom = max(element.y + element.height, plate.y + plate.height)
    return RoiRect(0, top, frame_w, bottom - top, True).clamped(frame_w, frame_h)
