#!/usr/bin/env python3
"""Generate bundled default-season.json and default-robot.json from VidarConfig constants."""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG_JAVA = ROOT / "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarConfig.java"
PROFILE_JAVA = ROOT / "teamcode/org/firstinspires/ftc/teamcode/vidar/runtime/VidarCameraProfile.java"
ASSETS_DIR = ROOT / "teamcode/assets/vidar"
BUNDLED_DIR = ROOT / "teamcode/org/firstinspires/ftc/teamcode/vidar/config/bundled"


def java_number(name: str, text: str, default: float | None = None) -> float:
    m = re.search(
        rf"public static final (?:int|double) {name}\s*=\s*([\d.]+)", text)
    if m:
        return float(m.group(1))
    if default is not None:
        return default
    raise KeyError(name)


def java_int(name: str, text: str, default: int | None = None) -> int:
    return int(java_number(name, text, default))


def java_double(name: str, text: str, default: float | None = None) -> float:
    return java_number(name, text, default)


def java_bool(name: str, text: str, default: bool = False) -> bool:
    m = re.search(rf"public static final boolean {name}\s*=\s*(true|false)", text)
    if m:
        return m.group(1) == "true"
    return default


def java_string(name: str, text: str, default: str = "") -> str:
    m = re.search(rf'public static final String {name}\s*=\s*"([^"]*)"', text)
    if m:
        return m.group(1)
    return default


def build_season(cfg: str) -> dict:
    return {
        "seasonId": "2025-decode",
        "seasonName": "DECODE 2025 — game elements and team plates",
        "fusion": {
            "minElementConfidence": java_double("MIN_ELEMENT_CONFIDENCE", cfg, 0.35),
            "minPlateConfidence": java_double("MIN_PLATE_CONFIDENCE", cfg, 0.4),
            "maxRangeMismatchRatio": java_double("MAX_RANGE_MISMATCH_RATIO", cfg, 0.35),
            "maxRankedElements": java_int("FUSION_MAX_RANKED_ELEMENTS", cfg, 16),
            "defaultMaxRankedElements": java_int("DEFAULT_MAX_RANKED_ELEMENTS", cfg, 5),
        },
        "world": {
            "mergeRadius": java_double("WORLD_MERGE_RADIUS_IN", cfg, 8.0),
            "trackGateRadius": java_double("WORLD_TRACK_GATE_RADIUS_IN", cfg, 12.0),
            "trackGateRadiusFoe": java_double("WORLD_TRACK_GATE_RADIUS_FOE_IN", cfg, 18.0),
            "blockRange": java_double("WORLD_BLOCK_RANGE_IN", cfg, 36.0),
            "blockConeDeg": java_double("WORLD_BLOCK_CONE_DEG", cfg, 35.0),
        },
        "elements": [{
            "id": "pollen",
            "label": "Pollen element",
            "diameter": java_double("DEFAULT_ELEMENT_DIAMETER", cfg, 4.9),
            "detector": "color_blob_with_local_hough",
            "hsv": {
                "hMin": java_int("DEFAULT_ELEMENT_HSV_H_MIN", cfg),
                "hMax": java_int("DEFAULT_ELEMENT_HSV_H_MAX", cfg),
                "sMin": java_int("DEFAULT_ELEMENT_HSV_S_MIN", cfg),
                "sMax": java_int("DEFAULT_ELEMENT_HSV_S_MAX", cfg),
                "vMin": java_int("DEFAULT_ELEMENT_HSV_V_MIN", cfg),
                "vMax": java_int("DEFAULT_ELEMENT_HSV_V_MAX", cfg),
            },
            "filters": {
                "minAreaPx": java_int("DEFAULT_ELEMENT_MIN_AREA_PX", cfg),
                "maxAreaPx": java_int("DEFAULT_ELEMENT_MAX_AREA_PX", cfg),
                "minWidthPx": java_int("DEFAULT_ELEMENT_MIN_WIDTH_PX", cfg),
                "maxWidthPx": java_int("DEFAULT_ELEMENT_MAX_WIDTH_PX", cfg),
                "minHeightPx": java_int("DEFAULT_ELEMENT_MIN_HEIGHT_PX", cfg),
                "maxHeightPx": java_int("DEFAULT_ELEMENT_MAX_HEIGHT_PX", cfg),
                "maxAspectRatio": java_double("DEFAULT_ELEMENT_MAX_ASPECT_RATIO", cfg),
                "minCircularity": java_double("DEFAULT_ELEMENT_MIN_CIRCULARITY", cfg),
                "minFillRatio": java_double("DEFAULT_ELEMENT_MIN_FILL_RATIO", cfg),
                "minInteriorScore": java_double("DEFAULT_ELEMENT_MIN_INTERIOR_SCORE", cfg),
            },
            "interior": {
                "brightMin": java_int("DEFAULT_ELEMENT_INTERIOR_BRIGHT", cfg),
                "spreadMax": java_int("DEFAULT_ELEMENT_INTERIOR_SPREAD", cfg),
                "holeDarkMax": java_int("DEFAULT_ELEMENT_HOLE_DARK_MAX", cfg),
            },
            "morphology": {
                "erodePasses": java_int("DEFAULT_ELEMENT_MORPH_ERODE_PASSES", cfg),
                "dilatePasses": java_int("DEFAULT_ELEMENT_MORPH_DILATE_PASSES", cfg),
                "openPasses": java_int("DEFAULT_ELEMENT_MORPH_OPEN_PASSES", cfg),
                "closePasses": java_int("DEFAULT_ELEMENT_MORPH_CLOSE_PASSES", cfg),
            },
            "hough": {
                "dp": java_double("HOUGH_DP", cfg),
                "minDist": java_double("HOUGH_MIN_DIST", cfg),
                "param1": java_double("HOUGH_PARAM1", cfg),
                "param2": java_double("HOUGH_PARAM2", cfg),
                "minRadius": java_int("HOUGH_MIN_RADIUS", cfg),
                "maxRadius": java_int("HOUGH_MAX_RADIUS", cfg),
                "minInterior": java_double("HOUGH_MIN_INTERIOR", cfg),
                "minAreaPx": java_int("MIN_ELEMENT_AREA_PX", cfg),
            },
        }],
        "plates": [
            {
                "alliance": "red",
                "width": 12.0,
                "hsv": {
                    "hMin": java_int("PLATE_RED_H_MIN", cfg),
                    "hMax": java_int("PLATE_RED_H_MAX", cfg),
                    "sMin": java_int("PLATE_S_MIN", cfg),
                    "sMax": 255,
                    "vMin": java_int("PLATE_V_MIN", cfg),
                    "vMax": 255,
                },
                "hsvWrap": {
                    "hMin": java_int("PLATE_RED_WRAP_H_MIN", cfg),
                    "hMax": 179,
                    "sMin": java_int("PLATE_S_MIN", cfg),
                    "sMax": 255,
                    "vMin": java_int("PLATE_V_MIN", cfg),
                    "vMax": 255,
                },
                "filters": {
                    "minAreaPx": java_int("PLATE_MIN_AREA_PX", cfg),
                    "maxAreaPx": java_int("PLATE_MAX_AREA_PX", cfg),
                    "minAspect": java_double("PLATE_MIN_ASPECT", cfg),
                    "maxAspect": java_double("PLATE_MAX_ASPECT", cfg),
                    "minRectangularity": java_double("PLATE_MIN_RECTANGULARITY", cfg),
                    "minWhiteRatio": java_double("PLATE_MIN_WHITE_RATIO", cfg),
                },
                "whiteDigit": {
                    "sampleGrid": java_int("PLATE_WHITE_SAMPLE_GRID", cfg),
                    "brightMin": java_int("PLATE_WHITE_BRIGHT_MIN", cfg),
                    "spreadMax": java_int("PLATE_WHITE_SPREAD_MAX", cfg),
                },
            },
            {
                "alliance": "blue",
                "width": 12.0,
                "hsv": {
                    "hMin": java_int("PLATE_BLUE_H_MIN", cfg),
                    "hMax": java_int("PLATE_BLUE_H_MAX", cfg),
                    "sMin": java_int("PLATE_S_MIN", cfg),
                    "sMax": 255,
                    "vMin": java_int("PLATE_V_MIN", cfg),
                    "vMax": 255,
                },
                "filters": {
                    "minAreaPx": java_int("PLATE_MIN_AREA_PX", cfg),
                    "maxAreaPx": java_int("PLATE_MAX_AREA_PX", cfg),
                    "minAspect": java_double("PLATE_MIN_ASPECT", cfg),
                    "maxAspect": java_double("PLATE_MAX_ASPECT", cfg),
                    "minRectangularity": java_double("PLATE_MIN_RECTANGULARITY", cfg),
                    "minWhiteRatio": java_double("PLATE_MIN_WHITE_RATIO", cfg),
                },
                "whiteDigit": {
                    "sampleGrid": java_int("PLATE_WHITE_SAMPLE_GRID", cfg),
                    "brightMin": java_int("PLATE_WHITE_BRIGHT_MIN", cfg),
                    "spreadMax": java_int("PLATE_WHITE_SPREAD_MAX", cfg),
                },
            },
        ],
    }


def parse_four_sides(profile_text: str) -> list[dict]:
    pattern = re.compile(
        r'buildSide\("(\w+)"\s*,\s*([\d.]+)\s*,\s*([\d.-]+)\s*,\s*([\d.-]+)\)',
        re.MULTILINE,
    )
    mounts = []
    for m in pattern.finditer(profile_text):
        mounts.append({
            "name": m.group(1),
            "bearingDeg": float(m.group(2)),
            "mountX": float(m.group(3)),
            "mountY": float(m.group(4)),
        })
    return mounts


def build_robot(cfg: str, profile_text: str) -> dict:
    count = java_int("CAMERA_COUNT", cfg, 1)
    names = re.findall(r'"([^"]+)"', re.search(
        r"CAMERA_NAMES = \{(.*?)\};", cfg, re.DOTALL).group(1))
    sides = parse_four_sides(profile_text)
    if not sides:
        raise RuntimeError("Could not parse VidarCameraProfile.FOUR_SIDES")

    front = sides[0]
    camera_defaults = {
        "horizonRowPx": 12,
        "focalLengthPx": 340,
        "focalLengthYPx": 340,
        "principalPointX": 320,
        "principalPointY": 240,
        "calibrationWidth": 640,
        "calibrationHeight": 480,
        "horizontalFovDeg": 70,
        "verticalFovDeg": 55,
        "plateWidth": 12.0,
        "floorLut": [
            {"cy": 95, "dist": 12},
            {"cy": 75, "dist": 24},
            {"cy": 55, "dist": 36},
            {"cy": 40, "dist": 48},
        ],
        "roi": {
            "element": {"lowerFraction": 0.65, "enabled": True},
            "plate": {"startFraction": 0.30, "bandFraction": 0.40, "enabled": True},
            "tag": {"upperFraction": 0.65, "enabled": True},
        },
    }

    cameras = []
    for i in range(count):
        side = sides[min(i, len(sides) - 1)]
        cameras.append({
            "index": i,
            "webcamName": names[i] if i < len(names) else f"Webcam {i + 1}",
            "name": side["name"],
            "mount": {
                "bearingDeg": side["bearingDeg"],
                "x": side["mountX"],
                "y": side["mountY"],
                "z": 9.0,
                "pitchDeg": -12,
            },
        })

    return {
        "robotName": "example-robot",
        "activeCameraIndex": java_int("ACTIVE_CAMERA_INDEX", cfg, 0),
        "cameraCount": count,
        "dimensions": {"length": 13, "width": 13, "height": 18},
        "alliance": {
            "defaultAlliance": "red",
            "colorSensorName": java_string("ALLIANCE_COLOR_SENSOR", cfg, "alliance_color"),
            "useColorSensor": java_bool("ALLIANCE_USE_COLOR_SENSOR", cfg, True),
            "allowRuntimeToggle": java_bool("ALLIANCE_ALLOW_RUNTIME_TOGGLE", cfg, True),
        },
        "cameraDefaults": camera_defaults,
        "mountDefaults": {"yawDeg": 0, "pitchDeg": -12, "rollDeg": 0},
        "cameras": cameras,
    }


def main() -> None:
    cfg = CONFIG_JAVA.read_text(encoding="utf-8")
    profile = PROFILE_JAVA.read_text(encoding="utf-8")
    season = json.dumps(build_season(cfg), indent=2) + "\n"
    robot = json.dumps(build_robot(cfg, profile), indent=2) + "\n"
    for out_dir in (ASSETS_DIR, BUNDLED_DIR):
        out_dir.mkdir(parents=True, exist_ok=True)
        season_path = out_dir / "default-season.json"
        robot_path = out_dir / "default-robot.json"
        season_path.write_text(season, encoding="utf-8")
        robot_path.write_text(robot, encoding="utf-8")
        print(f"Wrote {season_path}")
        print(f"Wrote {robot_path}")


if __name__ == "__main__":
    main()
