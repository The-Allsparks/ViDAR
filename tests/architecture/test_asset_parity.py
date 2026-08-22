"""Keep FTC asset copies identical to the authoritative bundled defaults (#45).

Authoritative: `vidar/config/bundled/default-*.json` (Java classpath / VidarConfigLoader).
Generated copy: `teamcode/assets/vidar/default-*.json` (TeamCode install surface).

Regenerate both with:
  python scripts/generate_default_config_assets.py
"""

from __future__ import annotations

from pathlib import Path

from architecture.scan_java import REPO_ROOT

BUNDLED = (
    REPO_ROOT
    / "teamcode"
    / "org"
    / "firstinspires"
    / "ftc"
    / "teamcode"
    / "vidar"
    / "config"
    / "bundled"
)
ASSETS = REPO_ROOT / "teamcode" / "assets" / "vidar"

NAMES = ("default-season.json", "default-robot.json")


def test_asset_copies_match_authoritative_bundled_defaults():
    mismatches = []
    for name in NAMES:
        authoritative = BUNDLED / name
        copy = ASSETS / name
        assert authoritative.is_file(), f"missing authoritative {authoritative}"
        assert copy.is_file(), (
            f"missing generated asset {copy}; run "
            "python scripts/generate_default_config_assets.py"
        )
        if authoritative.read_bytes() != copy.read_bytes():
            mismatches.append(name)
    assert not mismatches, (
        "teamcode/assets/vidar/default-*.json drifted from config/bundled/. "
        "Do not hand-edit the assets copies. Regenerate with "
        "`python scripts/generate_default_config_assets.py`.\n"
        + ", ".join(mismatches)
    )
