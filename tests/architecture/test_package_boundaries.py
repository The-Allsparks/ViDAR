"""Architecture ratchet: no new vidar package edges; never add hard-forbidden edges.

These tests parse TeamCode sources (not bytecode) so they cover classes that
java-pure excludes. ArchUnit is not used: it cannot see excluded sources, and
the competition path is Android/FTC TeamCode rather than a JVM library module.
"""

from __future__ import annotations

import json
from pathlib import Path

from architecture.scan_java import package_edges

ALLOWED_PATH = Path(__file__).with_name("allowed_package_edges.json")

# Edges that must never appear, even if someone edits the freeze file.
# These currently do not exist; they would invert the intended layering.
HARD_FORBIDDEN = {
    ("geometry", "detect"),
    ("geometry", "tag"),
    ("geometry", "schedule"),
    ("geometry", "world"),
    ("world", "detect"),
    ("world", "tag"),
    ("world", "schedule"),
    ("world", "runtime"),
    ("integration", "detect"),
    ("integration", "tag"),
    ("integration", "schedule"),
    ("integration", "runtime"),
    ("integration", "world"),
    ("config", "detect"),
    ("config", "fusion"),
    ("config", "world"),
    ("config", "tag"),
    ("config", "schedule"),
    ("api", "detect"),
    ("api", "fusion"),
    ("api", "world"),
    ("api", "tag"),
    ("api", "schedule"),
    ("api", "frame"),
}


def _allowed_graph() -> dict[str, set[str]]:
    raw = json.loads(ALLOWED_PATH.read_text(encoding="utf-8"))
    return {src: set(dsts) for src, dsts in raw.items() if not src.startswith("_")}


def test_no_new_package_edges():
    allowed = _allowed_graph()
    actual = package_edges()
    unexpected = []
    for src, dests in sorted(actual.items()):
        permitted = allowed.get(src, set())
        extra = sorted(dests - permitted)
        for dst in extra:
            unexpected.append(f"{src} -> {dst}")
    assert not unexpected, (
        "New vidar package dependencies are not allowed without updating "
        "tests/architecture/allowed_package_edges.json in the same PR.\n"
        + "\n".join(unexpected)
    )


def test_hard_forbidden_package_edges_absent():
    actual = package_edges()
    violations = []
    for src, dst in sorted(HARD_FORBIDDEN):
        if dst in actual.get(src, set()):
            violations.append(f"{src} -> {dst}")
    assert not violations, (
        "Forbidden layering inversion:\n" + "\n".join(violations)
    )


def test_freeze_file_does_not_permit_hard_forbidden_edges():
    allowed = _allowed_graph()
    bad = []
    for src, dst in sorted(HARD_FORBIDDEN):
        if dst in allowed.get(src, set()):
            bad.append(f"{src} -> {dst}")
    assert not bad, (
        "allowed_package_edges.json must not whitelist hard-forbidden edges:\n"
        + "\n".join(bad)
    )


def test_integration_stays_an_adapter():
    dests = package_edges().get("integration", set())
    assert dests <= {"fusion"}, (
        "vidar.integration may only depend on fusion (Pedro adapter). "
        f"Actual: {sorted(dests)}"
    )
