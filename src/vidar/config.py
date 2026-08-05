from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from vidar.geometry import CameraProfile, GeometryConfig


@dataclass(frozen=True)
class ColorRange:
    name: str
    hsv_low: tuple[int, int, int]
    hsv_high: tuple[int, int, int]
    min_area: int


@dataclass(frozen=True)
class AppConfig:
    camera_mode: str
    camera_count: int
    device_ids: list[int]
    stream_ports: list[int]
    capture_width: int
    capture_height: int
    process_width: int
    process_height: int
    fps_target: int
    elements: list[ColorRange]
    robots: list[ColorRange]
    blur_kernel: int
    telemetry_port: int
    print_fps_every: int
    show_debug: bool
    max_workers: int
    geometry: GeometryConfig | None
    hough_dp: float
    hough_min_dist: float
    hough_param1: float
    hough_param2: float
    hough_min_radius: int
    hough_max_radius: int
    hough_min_interior: float
    hough_interior_bright: int
    hough_interior_spread: int
    hough_min_area: float


def _color_ranges(raw: list[dict[str, Any]]) -> list[ColorRange]:
    return [
        ColorRange(
            name=item["name"],
            hsv_low=tuple(item["hsv_low"]),
            hsv_high=tuple(item["hsv_high"]),
            min_area=int(item["min_area"]),
        )
        for item in raw
    ]


def _geometry(raw: dict[str, Any] | None) -> GeometryConfig | None:
    if not raw:
        return None

    cameras = tuple(
        CameraProfile(
            name=c["name"],
            bearing_deg=float(c["bearing_deg"]),
            horizon_row_px=int(c["horizon_row_px"]),
            focal_length_px=float(c["focal_length_px"]),
            floor_lut=tuple((float(p["cy"]), float(p["dist_in"])) for p in c["floor_lut"]),
        )
        for c in raw["cameras"]
    )

    return GeometryConfig(
        ball_diameter_in=float(raw["ball_diameter_in"]),
        max_range_mismatch_ratio=float(raw.get("max_range_mismatch_ratio", 0.28)),
        min_ball_confidence=float(raw.get("min_ball_confidence", 0.35)),
        pickup_stop_in=float(raw.get("pickup_stop_in", 14)),
        seek_max_range_in=float(raw.get("seek_max_range_in", 72)),
        active_camera_index=int(raw.get("active_camera_index", 0)),
        cameras=cameras,
    )


def load_config(path: str | Path) -> AppConfig:
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    camera = data["camera"]
    detection = data["detection"]
    output = data["output"]
    runtime = data["runtime"]
    hough = detection.get("hough", {})

    return AppConfig(
        camera_mode=str(camera["mode"]),
        camera_count=int(camera["count"]),
        device_ids=[int(v) for v in camera["device_ids"]],
        stream_ports=[int(v) for v in camera["stream_ports"]],
        capture_width=int(camera["capture_width"]),
        capture_height=int(camera["capture_height"]),
        process_width=int(camera["process_width"]),
        process_height=int(camera["process_height"]),
        fps_target=int(camera["fps_target"]),
        elements=_color_ranges(detection["elements"]),
        robots=_color_ranges(detection["robots"]),
        blur_kernel=int(detection["blur_kernel"]),
        telemetry_port=int(output["telemetry_port"]),
        print_fps_every=int(output["print_fps_every"]),
        show_debug=bool(runtime["show_debug"]),
        max_workers=int(runtime["max_workers"]),
        geometry=_geometry(detection.get("geometry")),
        hough_dp=float(hough.get("dp", 1.2)),
        hough_min_dist=float(hough.get("min_dist", 24)),
        hough_param1=float(hough.get("param1", 80)),
        hough_param2=float(hough.get("param2", 11)),
        hough_min_radius=int(hough.get("min_radius", 8)),
        hough_max_radius=int(hough.get("max_radius", 36)),
        hough_min_interior=float(hough.get("min_interior", 0.14)),
        hough_interior_bright=int(hough.get("interior_bright", 90)),
        hough_interior_spread=int(hough.get("interior_spread", 60)),
        hough_min_area=float(hough.get("min_area", 45)),
    )
