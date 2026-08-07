#!/usr/bin/env python3
"""One-shot package reorg for ViDAR Java PR1. Moves files and updates imports."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIDAR = ROOT / "teamcode" / "org" / "firstinspires" / "ftc" / "teamcode" / "vidar"
TEAMCODE = ROOT / "teamcode"
BASE_PKG = "org.firstinspires.ftc.teamcode.vidar"

KEEP_AT_ROOT = {
    "VidarSpatial",
    "VidarMultiVision",
    "VidarConfig",
    "VidarTeleOp",
    "VidarDiscoverOpMode",
    "VidarAutoSeekOpMode",
    "VidarRoiCalibrationOpMode",
    "VidarSpatialPoint",
    "VidarElementObservation",
    "VidarPlateObservation",
    "VidarAlliance",
    "VidarDistanceUnit",
    "VidarElementDetectorType",
    "VidarElementShape",
    "VidarOffensiveLane",
    "VidarGeometry",
    "VidarCoordinateFrames",
    "VidarTagScoutResult",  # removed in PR2 util dedup
}

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


def fqcn(class_name: str) -> str:
    sub = MOVES.get(class_name)
    if sub:
        return f"{BASE_PKG}.{sub}.{class_name}"
    return f"{BASE_PKG}.{class_name}"


def git_mv(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["git", "mv", str(src.relative_to(ROOT)), str(dst.relative_to(ROOT))],
        cwd=ROOT,
        check=True,
    )


def move_files() -> None:
    for class_name, sub in MOVES.items():
        src = VIDAR / f"{class_name}.java"
        dst_dir = VIDAR / sub
        dst_dir.mkdir(parents=True, exist_ok=True)
        dst = dst_dir / f"{class_name}.java"
        if dst.exists():
            continue
        if not src.exists():
            raise FileNotFoundError(src)
        git_mv(src, dst)
        text = dst.read_text(encoding="utf-8")
        new_pkg = f"{BASE_PKG}.{sub}"
        text = re.sub(
            r"^package org\.firstinspires\.ftc\.teamcode\.vidar;",
            f"package {new_pkg};",
            text,
            count=1,
            flags=re.MULTILINE,
        )
        dst.write_text(text, encoding="utf-8")


def file_package(path: Path) -> str | None:
    text = path.read_text(encoding="utf-8")
    m = re.search(r"^package\s+([\w.]+);", text, re.MULTILINE)
    return m.group(1) if m else None


def existing_imports(text: str) -> set[str]:
    return set(re.findall(r"^import\s+([\w.]+);", text, re.MULTILINE))


def uses_class(text: str, class_name: str) -> bool:
    # Word boundary match for type references
    return bool(re.search(r"\b" + re.escape(class_name) + r"\b", text))


def fix_imports_in_file(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    original = text
    pkg = file_package(path)
    if pkg is None:
        return False

    imports = existing_imports(text)

    # Rewrite old imports for moved classes
    for class_name in MOVES:
        old = f"{BASE_PKG}.{class_name}"
        new = fqcn(class_name)
        text = text.replace(f"import {old};", f"import {new};")
        if new in imports or old in imports:
            imports.discard(old)
            imports.add(new)

    needed: list[str] = []
    for class_name in sorted(MOVES.keys()):
        target = fqcn(class_name)
        if target.startswith(pkg + ".") or target == pkg:
            continue
        if class_name in KEEP_AT_ROOT and pkg == BASE_PKG:
            continue
        if uses_class(text, class_name) and target not in imports:
            # Skip if same subpackage (implicit)
            sub = MOVES[class_name]
            if pkg == f"{BASE_PKG}.{sub}":
                continue
            needed.append(target)

    if needed:
        # Insert after package declaration and before first import or class
        insert_lines = [f"import {n};" for n in sorted(set(needed))]
        if re.search(r"^import ", text, re.MULTILINE):
            text = re.sub(
                r"(^import .+\n)",
                lambda m, lines=insert_lines: "".join(l + "\n" for l in lines if l + ";" not in text) + m.group(1),
                text,
                count=1,
                flags=re.MULTILINE,
            )
            # Simpler: append all missing before first import block
            first_import = re.search(r"^import ", text, re.MULTILINE)
            if first_import:
                pos = first_import.start()
                block = "".join(f"import {n};\n" for n in sorted(set(needed)))
                # Only add imports not already present
                to_add = [n for n in sorted(set(needed)) if f"import {n};" not in text]
                if to_add:
                    block = "".join(f"import {n};\n" for n in to_add)
                    text = text[:pos] + block + text[pos:]

    # Root classes used from subpackages
    for class_name in sorted(KEEP_AT_ROOT):
        target = f"{BASE_PKG}.{class_name}"
        if pkg == BASE_PKG or pkg.endswith("." + class_name):
            continue
        if uses_class(text, class_name) and target not in existing_imports(text):
            if f"import {target};" not in text:
                first_import = re.search(r"^import ", text, re.MULTILINE)
                if first_import:
                    pos = first_import.start()
                    text = text[:pos] + f"import {target};\n" + text[pos:]

    if text != original:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def fix_all_imports() -> int:
    changed = 0
    for java in TEAMCODE.rglob("*.java"):
        if fix_imports_in_file(java):
            changed += 1
    return changed


def verify_no_stray_root_files() -> list[str]:
    stray = []
    for java in VIDAR.glob("*.java"):
        if java.stem not in KEEP_AT_ROOT:
            stray.append(java.stem)
    return stray


def main() -> None:
    move_files()
    n = fix_all_imports()
    # Second pass catches cross-references introduced by first pass
    n += fix_all_imports()
    stray = verify_no_stray_root_files()
    if stray:
        raise SystemExit(f"Unexpected files still at root: {stray}")
    print(f"Moved {len(MOVES)} classes; updated imports in {n} file passes.")


if __name__ == "__main__":
    main()
