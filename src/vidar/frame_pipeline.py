from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from vidar.models import CameraProfile
from vidar.roi import RoiRect, detection_roi


@dataclass(frozen=True)
class ScaledRoi:
    image: np.ndarray
    source_crop: tuple[int, int, int, int]
    scale: float

    def to_full_x(self, local_x: float) -> float:
        x, _, _, _ = self.source_crop
        return local_x * self.scale + x

    def to_full_y(self, local_y: float) -> float:
        _, y, _, _ = self.source_crop
        return local_y * self.scale + y


def roi_scaled(frame: np.ndarray, roi: RoiRect, scale: float) -> ScaledRoi | None:
    if frame is None or frame.size == 0 or scale <= 0 or not roi.enabled:
        return None
    clamped = roi.clamped(frame.shape[1], frame.shape[0])
    x, y, w, h = clamped.to_cv_rect()
    if w <= 0 or h <= 0:
        return None
    roi_mat = frame[y : y + h, x : x + w]
    out_w = max(32, int(round(w * scale)))
    out_h = max(24, int(round(h * scale)))
    scaled = cv2.resize(roi_mat, (out_w, out_h), interpolation=cv2.INTER_AREA)
    return ScaledRoi(image=scaled, source_crop=(x, y, w, h), scale=scale)


def detection_scaled(
    frame: np.ndarray, profile: CameraProfile, scale: float
) -> ScaledRoi | None:
    roi = detection_roi(profile, frame.shape[1], frame.shape[0])
    return roi_scaled(frame, roi, scale)
