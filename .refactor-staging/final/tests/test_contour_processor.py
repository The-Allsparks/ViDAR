"""Contour detection pipeline tests — synthetic frames, ranking, pipeline wiring."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from vidar.config import AppConfig
from vidar.config_loader import load_robot, parse_season
from vidar.contour_detect import _interior_score, _white_digit_ratio, ContourTarget
from vidar.contour_processor import ContourProcessor, _select_top_k_scored
from vidar.detection.pipeline import DetectionPipeline
from vidar.models import CameraProfile, CameraRoiConfig

ROOT = Path(__file__).resolve().parents[1]
FRAME_W, FRAME_H = 640, 480


def _test_profile() -> CameraProfile:
    return CameraProfile(
        name="test",
        bearing_deg=0.0,
        horizon_row_px=12,
        focal_length_px=340.0,
        floor_cy_px=(95.0, 75.0, 55.0, 40.0),
        floor_dist=(12.0, 24.0, 36.0, 48.0),
        roi_config=CameraRoiConfig.default(),
    )


def _test_season(*, with_plates: bool = False, production_gates: bool = False):
    min_interior = 0.12 if production_gates else 0.0
    min_white = 0.12 if production_gates else 0.0
    s_min = 40 if production_gates else 100
    plates = []
    if with_plates:
        plates = [
            {
                "alliance": "red",
                "width": 12.0,
                "hsv": {
                    "hMin": 0,
                    "hMax": 12,
                    "sMin": 80,
                    "sMax": 255,
                    "vMin": 60,
                    "vMax": 255,
                },
                "filters": {
                    "minAreaPx": 80,
                    "maxAreaPx": 12000,
                    "minAspect": 1.15,
                    "maxAspect": 4.5,
                    "minRectangularity": 0.35,
                    "minWhiteRatio": min_white,
                },
                "whiteDigit": {"sampleGrid": 5, "brightMin": 175, "spreadMax": 55},
            }
        ]
    return parse_season(
        {
            "seasonId": "test-contour",
            "fusion": {"minElementConfidence": 0.20, "minPlateConfidence": 0.20},
            "elements": [
                {
                    "id": "marker",
                    "label": "Marker",
                    "diameter": 5.0,
                    "detector": "color_blob",
                    "hsv": {
                        "hMin": 20,
                        "hMax": 40,
                        "sMin": s_min,
                        "sMax": 255,
                        "vMin": 100,
                        "vMax": 255,
                    },
                    "filters": {
                        "minAreaPx": 50,
                        "maxAreaPx": 5000,
                        "minWidthPx": 8,
                        "maxWidthPx": 80,
                        "minHeightPx": 8,
                        "maxHeightPx": 80,
                        "maxAspectRatio": 2.0,
                        "minCircularity": 0.45,
                        "minFillRatio": 0.40,
                        "minInteriorScore": min_interior,
                    },
                    "interior": {"brightMin": 90, "spreadMax": 60, "holeDarkMax": 50},
                    "morphology": {
                        "erodePasses": 0,
                        "dilatePasses": 0,
                        "openPasses": 0,
                        "closePasses": 1,
                    },
                }
            ],
            "plates": plates,
        }
    )


def _blank_frame() -> np.ndarray:
    return np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)


def _draw_yellow_circle(frame: np.ndarray, cx: int, cy: int, radius: int, *, uniform: bool = False) -> None:
    """Draw a yellow element. Uniform fill passes production interior spread gate."""
    color = (200, 220, 220) if uniform else (0, 220, 220)
    cv2.circle(frame, (cx, cy), radius, color, -1)


def _draw_holed_circle(frame: np.ndarray, cx: int, cy: int, radius: int, hole_r: int) -> None:
    _draw_yellow_circle(frame, cx, cy, radius, uniform=True)
    cv2.circle(frame, (cx, cy), hole_r, (0, 0, 0), -1)


def _bilinear(corners: np.ndarray, u: float, v: float) -> tuple[float, float]:
    p0, p1, p2, p3 = corners
    x = (
        (1 - u) * (1 - v) * p0[0]
        + u * (1 - v) * p1[0]
        + u * v * p2[0]
        + (1 - u) * v * p3[0]
    )
    y = (
        (1 - u) * (1 - v) * p0[1]
        + u * (1 - v) * p1[1]
        + u * v * p2[1]
        + (1 - u) * v * p3[1]
    )
    return x, y


def _draw_red_plate(frame: np.ndarray, cx: int, cy: int, width: int, height: int) -> None:
    """Plate with white samples aligned to the 5×5 bilinear digit grid."""
    box = ((float(cx), float(cy)), (float(width), float(height)), 0.0)
    corners = cv2.boxPoints(box).astype(np.float64)
    cv2.fillPoly(frame, [corners.astype(np.int32)], (0, 0, 210))
    grid = 5
    for gy in range(1, grid):
        for gx in range(1, grid):
            px, py = _bilinear(corners, gx / grid, gy / grid)
            cv2.circle(frame, (int(round(px)), int(round(py))), 4, (255, 255, 255), -1)


class TestSelectTopK:
    def test_partial_sort_keeps_highest_scores(self):
        items = [(0.2, "a"), (0.9, "b"), (0.5, "c"), (0.8, "d")]
        result = _select_top_k_scored(items, 2)
        assert result == ["b", "d"]

    def test_k_larger_than_input_returns_all_sorted(self):
        items = [(0.3, 1), (0.7, 2), (0.5, 3)]
        assert _select_top_k_scored(items, 5) == [2, 3, 1]


class TestContourProcessor:
    def test_blank_frame_no_detections(self):
        season = _test_season()
        processor = ContourProcessor(_test_profile(), "test", season)
        detections = processor.detect(_blank_frame())
        assert detections == []
        assert processor.get_best_element() is None

    def test_yellow_circle_detected(self):
        season = _test_season()
        processor = ContourProcessor(_test_profile(), "test", season)
        frame = _blank_frame()
        _draw_yellow_circle(frame, 320, 350, 18)

        detections = processor.detect(frame)
        assert len(detections) >= 1
        assert detections[0].category == "element"
        assert detections[0].label == "marker"
        assert detections[0].confidence is not None
        assert detections[0].confidence > 0.2

        best = processor.get_best_element()
        assert best is not None
        assert best.element_id == "marker"
        assert processor.get_game_element("marker") is not None

    def test_ranked_elements_respects_capacity(self):
        season = _test_season()
        processor = ContourProcessor(
            _test_profile(), "test", season, max_ranked=2
        )
        frame = _blank_frame()
        _draw_yellow_circle(frame, 200, 350, 16)
        _draw_yellow_circle(frame, 440, 350, 16)
        _draw_yellow_circle(frame, 320, 380, 14)

        processor.detect(frame)
        ranked = processor.get_ranked_elements()
        assert ranked.count <= 2
        assert ranked.capacity == 2
        assert ranked.overflow_count >= 0

    def test_red_plate_detected(self):
        season = _test_season(with_plates=True)
        processor = ContourProcessor(_test_profile(), "test", season)
        frame = _blank_frame()
        _draw_red_plate(frame, 320, 300, 120, 40)

        processor.detect(frame)
        best_plate = processor.get_best_plate()
        assert best_plate is not None
        assert best_plate.alliance.value == "red"
        assert best_plate.confidence > 0.2

    def test_production_plate_white_ratio_gate(self):
        season = _test_season(with_plates=True, production_gates=True)
        processor = ContourProcessor(_test_profile(), "test", season)
        frame = _blank_frame()
        _draw_red_plate(frame, 320, 300, 120, 40)
        processor.detect(frame)
        assert processor.get_best_plate() is not None


class TestValidationGates:
    def test_interior_score_uniform_fill(self):
        season = _test_season(production_gates=True)
        target = ContourTarget.from_element(season.elements[0])
        patch = np.zeros((60, 60, 3), dtype=np.uint8)
        cv2.circle(patch, (30, 30), 18, (200, 220, 220), -1)
        score = _interior_score(patch, 30.0, 30.0, 18.0, target)
        assert score >= season.elements[0].min_interior_score

    def test_interior_score_rejects_high_spread(self):
        season = _test_season(production_gates=True)
        target = ContourTarget.from_element(season.elements[0])
        patch = np.zeros((60, 60, 3), dtype=np.uint8)
        cv2.circle(patch, (30, 30), 18, (0, 220, 220), -1)
        score = _interior_score(patch, 30.0, 30.0, 18.0, target)
        assert score < season.elements[0].min_interior_score

    def test_interior_score_holed_bonus(self):
        season = _test_season(production_gates=True)
        target = ContourTarget.from_element(season.elements[0])
        patch = np.zeros((60, 60, 3), dtype=np.uint8)
        cv2.circle(patch, (30, 30), 18, (200, 220, 220), -1)
        cv2.circle(patch, (30, 30), 8, (0, 0, 0), -1)
        score = _interior_score(patch, 30.0, 30.0, 18.0, target)
        assert score >= season.elements[0].min_interior_score

    def test_white_digit_ratio_grid(self):
        season = _test_season(with_plates=True, production_gates=True)
        target = ContourTarget.from_plate(season.plates[0])
        frame = _blank_frame()
        _draw_red_plate(frame, 320, 300, 120, 40)
        box = ((320.0, 300.0), (120.0, 40.0), 0.0)
        ratio = _white_digit_ratio(frame, box, target)
        assert ratio >= season.plates[0].min_white_ratio


class TestDetectionPipeline:
    @pytest.fixture
    def pipeline(self) -> DetectionPipeline:
        season = _test_season()
        robot = load_robot(ROOT / "config/robots/example-robot.json")
        config = AppConfig(
            season=season,
            robot=robot,
            camera_mode="mock",
            device_ids=(0,),
            stream_ports=(5555,),
            capture_width=FRAME_W,
            capture_height=FRAME_H,
            fps_target=30,
            telemetry_port=5800,
            print_fps_every=30,
            show_debug=False,
            max_workers=1,
        )
        return DetectionPipeline(config)

    def test_pipeline_detects_synthetic_element(self, pipeline: DetectionPipeline):
        frame = _blank_frame()
        _draw_yellow_circle(frame, 320, 350, 18)
        detections = pipeline.detect(frame, 0)
        assert any(det.category == "element" and det.label == "marker" for det in detections)

    def test_unknown_camera_index_returns_empty(self, pipeline: DetectionPipeline):
        assert pipeline.detect(_blank_frame(), 99) == []
