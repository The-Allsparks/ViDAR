#!/usr/bin/env python3
"""Build four stacked refactor branches from a full-tree backup."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STAGING = ROOT / ".refactor-staging" / "final"

PR2_PATHS = [
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarCoordinateFrames.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarObservationMapper.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarSpatialOpModeBase.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarRobotPose2D.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarAutoSeekOpMode.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarDiscoverOpMode.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarTeleOp.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarMultiVision.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarSpatial.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarSpatialPoint.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/frame/VidarCorrectedFrame.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/frame/VidarObservationFrame.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/world/VidarTrackDetection.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/model/VidarTagScoutObservation.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/model/VidarElementDensityMap.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/fusion/VidarLocalizationFusion.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/fusion/VidarMotionCorrection.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/fusion/VidarMotionTransform.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/tag/VidarTagGate.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/config/VidarAprilTagSpec.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarAprilTagTransforms.java",
]

PR3_PATHS = [
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarGeometry.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarRangeFusion.java",
]

PR4_PATHS = [
    "teamcode/org/firstinspires/ftc/teamcode/vidar/config/VidarConfigLoader.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/config/bundled",
    "teamcode/assets",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarConfig.java",
    "teamcode/org/firstinspires/ftc/teamcode/VidarTeamConfig.java",
    "README.md",
    "tests/test_config_defaults.py",
    "tests/test_config_loader.py",
    "scripts/generate_default_config_assets.py",
    "docs/JAVA_PACKAGE_MAP.md",
]

PR1_DOC_PATHS = [
    "docs/JAVA_PACKAGE_MAP.md",
    "docs/TEACHING.md",
    "docs/SYSTEM_DESIGN.md",
]

PR1_SCRIPT_PATHS = [
    "scripts/java_package_reorg.py",
    "scripts/java_fix_imports.py",
    "scripts/java_git_mv_reorg.py",
]

PR1_IMPORT_ROOT = [
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarElementObservation.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarPlateObservation.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarRoiCalibrationOpMode.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/config/VidarRobotConfig.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/config/VidarSeasonConfig.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarCameraIntrinsics.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarObservationSpatial.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarPoseLookup.java",
    "teamcode/org/firstinspires/ftc/teamcode/vidar/geometry/VidarTransformRegistry.java",
]


def run(*args: str, check: bool = True) -> None:
    print("+", " ".join(args))
    subprocess.run(args, cwd=ROOT, check=check)


def copy_from_staging(rel: str) -> None:
    src = STAGING / rel
    dst = ROOT / rel
    if not src.exists():
        print(f"skip missing backup: {rel}")
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.is_dir():
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
    else:
        shutil.copy2(src, dst)


def copy_many(paths: list[str]) -> None:
    for rel in paths:
        copy_from_staging(rel)


def apply_pr1() -> None:
    copy_many(PR1_SCRIPT_PATHS)
    run("python", "scripts/java_package_reorg.py")
    run("python", "scripts/java_fix_imports.py")
    run("python", "scripts/java_fix_imports.py")
    copy_many(PR1_DOC_PATHS)
    copy_many(PR1_IMPORT_ROOT)
    readme_backup = STAGING / "README.md"
    if readme_backup.exists():
        head_readme = (ROOT / "README.md").read_text(encoding="utf-8")
        if "JAVA_PACKAGE_MAP.md" not in head_readme:
            head_readme = head_readme.replace(
                "→ `TeamCode/src/main/java/.../vidar/`.",
                "→ `TeamCode/src/main/java/.../vidar/`. See [docs/JAVA_PACKAGE_MAP.md](docs/JAVA_PACKAGE_MAP.md).",
                1,
            )
            (ROOT / "README.md").write_text(head_readme, encoding="utf-8")


def remove_tag_scout_result() -> None:
    path = ROOT / "teamcode/org/firstinspires/ftc/teamcode/vidar/VidarTagScoutResult.java"
    if path.exists():
        run("git", "rm", "-f", str(path.relative_to(ROOT)))


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "pr1"
    if cmd == "pr1":
        apply_pr1()
    elif cmd == "pr2":
        copy_many(PR2_PATHS)
        remove_tag_scout_result()
    elif cmd == "pr3":
        copy_many(PR3_PATHS)
    elif cmd == "pr4":
        copy_many(PR4_PATHS)
    else:
        print(f"unknown command: {cmd}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
