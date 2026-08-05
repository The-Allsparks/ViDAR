#!/usr/bin/env python3
"""Run ViDAR offline tests."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    try:
        import pytest  # noqa: F401
    except ImportError:
        print("pytest is not installed. Run: pip install -r requirements-dev.txt", file=sys.stderr)
        sys.exit(1)

    rc = subprocess.call([sys.executable, "-m", "pytest", str(ROOT / "tests"), "-v"])
    sys.exit(rc)


if __name__ == "__main__":
    main()
