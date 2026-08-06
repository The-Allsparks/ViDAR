from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from vidar.config_loader import default_robot, default_season, load_robot, load_season
from vidar.models import RobotConfig, SeasonConfig


@dataclass(frozen=True)
class AppConfig:
    season: SeasonConfig
    robot: RobotConfig
    camera_mode: str
    device_ids: tuple[int, ...]
    stream_ports: tuple[int, ...]
    capture_width: int
    capture_height: int
    fps_target: int
    telemetry_port: int
    print_fps_every: int
    show_debug: bool
    max_workers: int
    process_roi_scale: float = 0.5

    @property
    def camera_count(self) -> int:
        return len(self.robot.cameras)

    @property
    def active_camera_index(self) -> int:
        return self.robot.active_camera_index


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_json_config(
    season_path: str | Path | None = None,
    robot_path: str | Path | None = None,
) -> AppConfig:
    root = _repo_root()
    season_file = Path(season_path or os.environ.get("VIDAR_SEASON", root / "config/seasons/2026-biobuzz.json"))
    robot_file = Path(robot_path or os.environ.get("VIDAR_ROBOT", root / "config/robots/example-robot.json"))

    season = load_season(season_file) if season_file.exists() else default_season(root)
    robot = load_robot(robot_file) if robot_file.exists() else default_robot(root)

    return AppConfig(
        season=season,
        robot=robot,
        camera_mode=os.environ.get("VIDAR_CAMERA_MODE", "mock"),
        device_ids=(0, 1, 2, 3),
        stream_ports=(5555, 5556, 5557, 5558),
        capture_width=int(os.environ.get("VIDAR_CAPTURE_WIDTH", "640")),
        capture_height=int(os.environ.get("VIDAR_CAPTURE_HEIGHT", "480")),
        fps_target=int(os.environ.get("VIDAR_FPS_TARGET", "30")),
        telemetry_port=int(os.environ.get("VIDAR_TELEMETRY_PORT", "5800")),
        print_fps_every=int(os.environ.get("VIDAR_PRINT_FPS_EVERY", "30")),
        show_debug=os.environ.get("VIDAR_SHOW_DEBUG", "").lower() in {"1", "true", "yes"},
        max_workers=int(os.environ.get("VIDAR_MAX_WORKERS", "4")),
        process_roi_scale=float(os.environ.get("VIDAR_PROCESS_ROI_SCALE", "0.5")),
    )


def load_config(path: str | Path | None = None) -> AppConfig:
    if path is None:
        return load_json_config()
    path = Path(path)
    if path.parent.name == "seasons" or "season" in path.name:
        return load_json_config(season_path=path)
    if path.parent.name == "robots" or "robot" in path.name:
        return load_json_config(robot_path=path)
    return load_json_config(season_path=path)
