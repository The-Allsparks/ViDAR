"""Version identity and OpMode lifecycle guards for FTC packaging (#33)."""

from __future__ import annotations

import re
from pathlib import Path

from architecture.scan_java import REPO_ROOT, VIDAR_JAVA, iter_vidar_java, read_java

VERSION_PATH = REPO_ROOT / "VERSION"
README_PATH = REPO_ROOT / "README.md"
GRADLE_PATH = REPO_ROOT / "java-pure" / "build.gradle"

SPATIAL_CREATE = re.compile(
    r"VidarSpatial\.(create|createWithBundledDefaults)\s*\("
)
CLOSE_CALL = re.compile(r"\b\w+\.close\s*\(\s*\)")
GRADLE_VERSION = re.compile(r"version\s*=\s*'([^']+)'")
README_VERSION = re.compile(r"\*\*Version\s+([0-9]+\.[0-9]+\.[0-9]+)\*\*")


def test_version_files_agree():
    assert VERSION_PATH.is_file(), "repo-root VERSION is required for install identity"
    version = VERSION_PATH.read_text(encoding="utf-8").strip()
    assert re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version), f"bad VERSION: {version!r}"

    gradle = GRADLE_PATH.read_text(encoding="utf-8")
    match = GRADLE_VERSION.search(gradle)
    assert match, "java-pure/build.gradle must set version = '...'"
    assert match.group(1) == version, (
        f"java-pure version {match.group(1)!r} != VERSION {version!r}"
    )

    readme = README_PATH.read_text(encoding="utf-8")
    readme_match = README_VERSION.search(readme)
    assert readme_match, "README must contain **Version X.Y.Z**"
    assert readme_match.group(1) == version, (
        f"README version {readme_match.group(1)!r} != VERSION {version!r}"
    )


def test_install_and_lifecycle_docs_exist():
    assert (REPO_ROOT / "docs" / "INSTALL.md").is_file()
    assert (REPO_ROOT / "docs" / "LIFECYCLE.md").is_file()


def test_spatial_creating_opmodes_call_close():
    """Sample OpModes that create VidarSpatial must release portals on stop."""
    missing = []
    for path in iter_vidar_java():
        name = path.name
        if not (name.endswith("OpMode.java") or name == "VidarTeleOp.java"):
            continue
        if name == "VidarSpatialOpModeBase.java":
            continue
        text = read_java(path)
        if not SPATIAL_CREATE.search(text):
            continue
        if not CLOSE_CALL.search(text):
            missing.append(str(path.relative_to(VIDAR_JAVA)))
    assert not missing, (
        "OpModes that call VidarSpatial.create* must call .close() on the stop path:\n"
        + "\n".join(missing)
    )
