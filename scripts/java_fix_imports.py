#!/usr/bin/env python3
"""Add missing imports after package reorg."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEAMCODE = ROOT / "teamcode"
BASE_PKG = "org.firstinspires.ftc.teamcode.vidar"

KEEP_AT_ROOT = {
    "VidarSpatial", "VidarMultiVision", "VidarConfig",
    "VidarTeleOp", "VidarDiscoverOpMode", "VidarAutoSeekOpMode", "VidarRoiCalibrationOpMode",
    "VidarSpatialPoint", "VidarElementObservation", "VidarPlateObservation",
    "VidarAlliance", "VidarDistanceUnit", "VidarElementDetectorType", "VidarElementShape",
    "VidarOffensiveLane", "VidarGeometry", "VidarCoordinateFrames",
}

MOVES: dict[str, str] = {
    "VidarContourProcessor": "detect", "VidarContourDetect": "detect", "VidarBlobUtil": "detect",
    "VidarContourTarget": "detect", "VidarContourWorkspace": "detect", "VidarContourWorkspacePool": "detect",
    "VidarAdaptiveTagProcessor": "tag", "VidarTagScoutRunner": "tag", "VidarTagCropDecoder": "tag",
    "VidarTagDecodeWorker": "tag", "VidarDecodeArbiter": "tag", "VidarTagGate": "tag",
    "VidarTagConfig": "tag", "VidarTagCropPlanner": "tag",
    "VidarLocalizationFusion": "fusion", "VidarTemporalFilter": "fusion", "VidarMotionCorrection": "fusion",
    "VidarMotionTransform": "fusion", "VidarPoseBackdate": "fusion", "VidarOdomHistory": "fusion",
    "VidarWorldModel": "world", "VidarTrackAssociator": "world", "VidarSpatialTrack": "world",
    "VidarTrackDetection": "world",
    "VidarObservationFrame": "frame", "VidarCorrectedFrame": "frame", "VidarCorrectedPoint": "frame",
    "VidarFramePipeline": "frame", "VidarFrameRegions": "frame", "VidarFrameMailbox": "frame",
    "VidarRankedElementFrame": "frame",
    "VidarCameraScheduler": "schedule", "VidarProcessScheduler": "schedule",
    "VidarResourceBudget": "schedule", "VidarGlobalVisionWorker": "schedule",
    "VidarRangeEstimate": "model", "VidarRangeResult": "model", "VidarTagObservation": "model",
    "VidarTagScoutObservation": "model", "VidarTagScoutResult": "model", "VidarVisionMeasurement": "model",
    "VidarElementDensityMap": "model", "VidarElementRejectionStats": "model",
    "VidarElementOccurrenceRank": "model", "VidarOffensiveLaneAnalysis": "model",
    "VidarVision": "runtime", "VidarCameraProfile": "runtime", "VidarCameraMount": "runtime",
    "VidarCameraRoiConfig": "runtime", "VidarRoiRect": "runtime", "VidarRuntimeConfig": "runtime",
    "VidarUnits": "runtime", "VidarAllianceSelector": "runtime", "VidarMetrics": "runtime",
    "VidarMetricsLogger": "runtime",
}

ALL_CLASSES = {**{k: BASE_PKG for k in KEEP_AT_ROOT}, **{k: f"{BASE_PKG}.{v}" for k, v in MOVES.items()}}


def fqcn(class_name: str) -> str:
    return ALL_CLASSES[class_name]


def file_package(text: str) -> str | None:
    m = re.search(r"^package\s+([\w.]+);", text, re.MULTILINE)
    return m.group(1) if m else None


def strip_comments_and_strings(text: str) -> str:
    text = re.sub(r"/\*\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//.*", "", text)
    return text


def fix_file(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    pkg = file_package(text)
    if not pkg:
        return False

    body = strip_comments_and_strings(text)
    imports = set(re.findall(r"^import\s+([\w.]+);", text, re.MULTILINE))
    to_add: list[str] = []

    for class_name, target_pkg in ALL_CLASSES.items():
        if not re.search(r"\b" + re.escape(class_name) + r"\b", body):
            continue
        fqn = f"{target_pkg}.{class_name}"
        if target_pkg == pkg:
            continue
        if fqn in imports:
            continue
        if f"import {fqn};" in text:
            continue
        to_add.append(fqn)

    if not to_add:
        return False

    to_add = sorted(set(to_add))
    block = "".join(f"import {fqn};\n" for fqn in to_add)

    # Insert after package line
    m = re.search(r"^package .+;\n", text, re.MULTILINE)
    if not m:
        return False
    insert_at = m.end()
    # Skip blank lines
    while insert_at < len(text) and text[insert_at] in "\r\n":
        insert_at += 1
    new_text = text[:insert_at] + block + text[insert_at:]
    path.write_text(new_text, encoding="utf-8")
    return True


def main() -> None:
    changed = 0
    for _ in range(3):
        for java in TEAMCODE.rglob("*.java"):
            if fix_file(java):
                changed += 1
    print(f"Fixed imports in {changed} file-updates across passes.")


if __name__ == "__main__":
    main()
