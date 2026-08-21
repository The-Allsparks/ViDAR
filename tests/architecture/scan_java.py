"""Scan ViDAR TeamCode Java sources for package edges and hot-path patterns.

Used by architecture tests. Does not compile Java — FTC TeamCode is source-copied
into the SDK, and java-pure excludes most perception classes.
"""

from __future__ import annotations

import re
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VIDAR_JAVA = REPO_ROOT / "teamcode" / "org" / "firstinspires" / "ftc" / "teamcode" / "vidar"
VIDAR_PREFIX = "org.firstinspires.ftc.teamcode.vidar"

IMPORT_RE = re.compile(
    r"^import\s+(org\.firstinspires\.ftc\.teamcode\.vidar(?:\.[A-Za-z0-9_]+)*)\s*;",
    re.MULTILINE,
)

PACKAGES = (
    "root",
    "api",
    "config",
    "detect",
    "frame",
    "fusion",
    "geometry",
    "integration",
    "model",
    "runtime",
    "schedule",
    "tag",
    "world",
)


def package_of(path: Path) -> str:
    rel = path.relative_to(VIDAR_JAVA).as_posix()
    parts = rel.split("/")
    if len(parts) == 1:
        return "root"
    return parts[0]


def dest_package(imported: str) -> str:
    rest = imported[len(VIDAR_PREFIX) :]
    if not rest:
        return "root"
    segs = rest.lstrip(".").split(".")
    if segs and segs[0] and segs[0][0].islower() and segs[0] in PACKAGES:
        return segs[0]
    return "root"


def iter_vidar_java() -> list[Path]:
    return sorted(VIDAR_JAVA.rglob("*.java"))


def package_edges() -> dict[str, set[str]]:
    edges: dict[str, set[str]] = defaultdict(set)
    for path in iter_vidar_java():
        src = package_of(path)
        text = path.read_text(encoding="utf-8")
        for match in IMPORT_RE.finditer(text):
            dst = dest_package(match.group(1))
            if dst != src:
                edges[src].add(dst)
    return edges


def read_java(path: Path) -> str:
    return path.read_text(encoding="utf-8")
