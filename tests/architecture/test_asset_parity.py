"""Keep duplicated default JSON assets byte-identical until they are consolidated."""

from __future__ import annotations

from pathlib import Path

from architecture.scan_java import REPO_ROOT

PAIRS = (
    (
        REPO_ROOT / "teamcode" / "assets" / "vidar" / "default-season.json",
        REPO_ROOT / "teamcode" / "org" / "firstinspires" / "ftc" / "teamcode" / "vidar" / "config" / "bundled" / "default-season.json",
    ),
    (
        REPO_ROOT / "teamcode" / "assets" / "vidar" / "default-robot.json",
        REPO_ROOT / "teamcode" / "org" / "firstinspires" / "ftc" / "teamcode" / "vidar" / "config" / "bundled" / "default-robot.json",
    ),
)


def test_bundled_default_json_copies_match():
    mismatches = []
    for left, right in PAIRS:
        assert left.is_file(), f"missing {left}"
        assert right.is_file(), f"missing {right}"
        if left.read_bytes() != right.read_bytes():
            mismatches.append(f"{left.name}: {left} != {right}")
    assert not mismatches, (
        "Default JSON copies drifted. Keep them identical or complete the "
        "single-source consolidation issue.\n" + "\n".join(mismatches)
    )
