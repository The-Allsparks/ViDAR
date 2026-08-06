from __future__ import annotations

from vidar.config import AppConfig
from vidar.contour_processor import ContourProcessor, Detection

__all__ = ["DetectionPipeline", "Detection"]


class DetectionPipeline:
    """Season-configurable contour detection aligned with Java VidarContourProcessor."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._processors = [
            ContourProcessor(
                profile=mount.profile,
                camera_name=mount.profile.name,
                season=config.season,
                camera_index=index,
                roi_scale=config.process_roi_scale,
            )
            for index, mount in enumerate(config.robot.cameras)
        ]

    def preprocess(self, frame):
        return frame

    def detect(self, frame, camera_index: int) -> list[Detection]:
        if camera_index >= len(self._processors):
            return []
        return self._processors[camera_index].detect(frame)

    def annotate(self, frame, detections: list[Detection]):
        return ContourProcessor.annotate(frame, detections)
