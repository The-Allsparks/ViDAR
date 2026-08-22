"""Static guards for FTC safety and Control Hub hot-path mistakes.

These are not timing benchmarks. They fail when a known-bad pattern appears
in sources that already satisfy the rule, so new debt cannot land unnoticed.
"""

from __future__ import annotations

import re

from architecture.scan_java import VIDAR_JAVA, iter_vidar_java, read_java

MOTOR_IMPORT = re.compile(
    r"import\s+com\.qualcomm\.robotcore\.hardware\.(DcMotor|DcMotorEx|CRServo|Servo)\s*;"
)
SET_POWER = re.compile(r"\.setPower\s*\(")
SET_VELOCITY = re.compile(r"\.setVelocity\s*\(")
THREAD_SLEEP = re.compile(r"Thread\.sleep\s*\(")
VISION_PORTAL_BUILDER = re.compile(r"VisionPortal\.Builder")
FILE_IO = re.compile(r"\b(FileInputStream|FileOutputStream|Files\.newBufferedReader|new\s+FileReader)\b")

OPMODE_NAMES = (
    "VidarTeleOp.java",
    "VidarDiscoverOpMode.java",
    "VidarAutoSeekOpMode.java",
    "VidarRoiCalibrationOpMode.java",
    "VidarPedroBridgeSampleOpMode.java",
    "VidarSpatialOpModeBase.java",
)

# Thread.sleep is allowed only in background workers, never in OpModes or fusion.
SLEEP_ALLOWED = {
    "VidarObservationWorker.java",
    "VidarGlobalVisionWorker.java",
    "VidarTagDecodeWorker.java",
}

# VisionPortal construction is attach-time only.
PORTAL_BUILDER_ALLOWED = {
    "VidarVision.java",
}


def _opmode_files():
    for path in iter_vidar_java():
        if path.name in OPMODE_NAMES or path.name.endswith("OpMode.java"):
            yield path


def test_vidar_never_commands_motors_or_servos():
    hits = []
    for path in iter_vidar_java():
        text = read_java(path)
        if MOTOR_IMPORT.search(text) or SET_POWER.search(text) or SET_VELOCITY.search(text):
            hits.append(str(path.relative_to(VIDAR_JAVA)))
    assert not hits, (
        "ViDAR is a passive library and must not command drivetrain or servos:\n"
        + "\n".join(hits)
    )


def test_opmodes_do_not_sleep():
    hits = []
    for path in _opmode_files():
        if THREAD_SLEEP.search(read_java(path)):
            hits.append(path.name)
    assert not hits, "OpModes must not call Thread.sleep (blocks the robot loop):\n" + "\n".join(hits)


def test_thread_sleep_only_in_workers():
    hits = []
    for path in iter_vidar_java():
        if path.name in SLEEP_ALLOWED:
            continue
        if THREAD_SLEEP.search(read_java(path)):
            hits.append(path.name)
    assert not hits, (
        "Thread.sleep is only allowed in named worker threads, not fusion/detect/OpModes:\n"
        + "\n".join(hits)
    )


def test_visionportal_builder_only_in_vidar_vision():
    hits = []
    for path in iter_vidar_java():
        if path.name in PORTAL_BUILDER_ALLOWED:
            continue
        if VISION_PORTAL_BUILDER.search(read_java(path)):
            hits.append(path.name)
    assert not hits, (
        "VisionPortal.Builder must stay in VidarVision attach/init, not per-frame update:\n"
        + "\n".join(hits)
    )


def test_fusion_and_world_avoid_filesystem_io():
    hot = ("fusion", "world", "detect", "geometry")
    hits = []
    for path in iter_vidar_java():
        rel = path.relative_to(VIDAR_JAVA).as_posix()
        pkg = rel.split("/")[0] if "/" in rel else ""
        if pkg not in hot:
            continue
        if FILE_IO.search(read_java(path)):
            hits.append(rel)
    assert not hits, (
        "Filesystem I/O in detect/fusion/world/geometry can stall the Control Hub:\n"
        + "\n".join(hits)
    )


STATIC_VOLATILE = re.compile(r"\bstatic\s+volatile\b")


def test_tag_package_has_no_static_volatile_gate_state():
    """Tag decode gate state must live on VidarRuntime (VidarTagGateState), not process globals."""
    hits = []
    for path in iter_vidar_java():
        rel = path.relative_to(VIDAR_JAVA).as_posix()
        if not rel.startswith("tag/"):
            continue
        if STATIC_VOLATILE.search(read_java(path)):
            hits.append(rel)
    assert not hits, (
        "static volatile in tag/ leaks Auto→TeleOp / multi-camera gate state (#41):\n"
        + "\n".join(hits)
    )
