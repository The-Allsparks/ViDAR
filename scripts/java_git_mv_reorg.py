#!/usr/bin/env python3
"""Re-stage ViDAR package moves using git mv so file history is preserved."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIDAR = ROOT / "teamcode" / "org" / "firstinspires" / "ftc" / "teamcode" / "vidar"
BASE_PKG = "org.firstinspires.ftc.teamcode.vidar"

# Same map as java_package_reorg.py
MOVES: dict[str, str] = {
    "VidarContourProcessor": "detect",
    "VidarContourDetect": "detect",
    "VidarBlobUtil": "detect",
    "VidarContourTarget": "detect",
    "VidarContourWorkspace": "detect",
    "VidarContourWorkspacePool": "detect",
    "VidarAdaptiveTagProcessor": "tag",
    "VidarTagScoutRunner": "tag",
    "VidarTagCropDecoder": "tag",
    "VidarTagDecodeWorker": "tag",
    "VidarDecodeArbiter": "tag",
    "VidarTagGate": "tag",
    "VidarTagConfig": "tag",
    "VidarTagCropPlanner": "tag",
    "VidarLocalizationFusion": "fusion",
    "VidarTemporalFilter": "fusion",
    "VidarMotionCorrection": "fusion",
    "VidarMotionTransform": "fusion",
    "VidarPoseBackdate": "fusion",
    "VidarOdomHistory": "fusion",
    "VidarWorldModel": "world",
    "VidarTrackAssociator": "world",
    "VidarSpatialTrack": "world",
    "VidarTrackDetection": "world",
    "VidarObservationFrame": "frame",
    "VidarCorrectedFrame": "frame",
    "VidarCorrectedPoint": "frame",
    "VidarFramePipeline": "frame",
    "VidarFrameRegions": "frame",
    "VidarFrameMailbox": "frame",
    "VidarRankedElementFrame": "frame",
    "VidarCameraScheduler": "schedule",
    "VidarProcessScheduler": "schedule",
    "VidarResourceBudget": "schedule",
    "VidarGlobalVisionWorker": "schedule",
    "VidarRangeEstimate": "model",
    "VidarRangeResult": "model",
    "VidarTagObservation": "model",
    "VidarTagScoutObservation": "model",
    "VidarVisionMeasurement": "model",
    "VidarElementDensityMap": "model",
    "VidarElementRejectionStats": "model",
    "VidarElementOccurrenceRank": "model",
    "VidarOffensiveLaneAnalysis": "model",
    "VidarVision": "runtime",
    "VidarCameraProfile": "runtime",
    "VidarCameraMount": "runtime",
    "VidarCameraRoiConfig": "runtime",
    "VidarRoiRect": "runtime",
    "VidarRuntimeConfig": "runtime",
    "VidarUnits": "runtime",
    "VidarAllianceSelector": "runtime",
    "VidarMetrics": "runtime",
    "VidarMetricsLogger": "runtime",
}

# Merged into VidarTagScoutObservation in PR2 — delete, do not move.
DELETE_AT_ROOT = {"VidarTagScoutResult"}


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    rel = [str(Path(a).relative_to(ROOT)) if Path(a).is_absolute() else a for a in args]
    print("+", " ".join(["git", *rel]))
    return subprocess.run(["git", *rel], cwd=ROOT, check=check, text=True, capture_output=True)


def git_tracked(path: Path) -> bool:
    result = run("ls-files", "--error-unmatch", str(path.relative_to(ROOT)), check=False)
    return result.returncode == 0


def restore_and_git_mv(class_name: str, sub: str) -> None:
    old = VIDAR / f"{class_name}.java"
    new = VIDAR / sub / f"{class_name}.java"
    old_rel = old.relative_to(ROOT).as_posix()
    new_rel = new.relative_to(ROOT).as_posix()

    if not git_tracked(old):
        print(f"skip {class_name}: not tracked at root")
        return

    final_content = new.read_text(encoding="utf-8") if new.exists() else None
    if final_content is None:
        raise FileNotFoundError(f"Expected final content at {new}")

    if new.exists():
        new.unlink()

    run("checkout", "HEAD", "--", old_rel)

    new.parent.mkdir(parents=True, exist_ok=True)
    run("mv", old_rel, new_rel)

    new.write_text(final_content, encoding="utf-8")
    print(f"moved {class_name} -> {sub}/")


def delete_merged(class_name: str) -> None:
    old = VIDAR / f"{class_name}.java"
    old_rel = old.relative_to(ROOT).as_posix()
    if not git_tracked(old):
        return
    if old.exists():
        old.unlink()
    else:
        run("checkout", "HEAD", "--", old_rel)
    run("rm", "-f", old_rel)
    print(f"deleted merged class {class_name}")


def main() -> int:
    for class_name in sorted(DELETE_AT_ROOT):
        delete_merged(class_name)

    for class_name, sub in sorted(MOVES.items()):
        restore_and_git_mv(class_name, sub)

    # Report rename detection
    result = run("status", "-M50", "--short", "teamcode/org/firstinspires/ftc/teamcode/vidar/", check=False)
    renames = [line for line in result.stdout.splitlines() if line.startswith("R")]
    print(f"\nGit reports {len(renames)} renames (after staging moves with git mv)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
