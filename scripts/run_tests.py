#!/usr/bin/env python3
"""Run ViDAR offline tests (Python parity + java-pure JVM tests)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run_pytest() -> int:
    try:
        import pytest  # noqa: F401
    except ImportError:
        print("pytest is not installed. Run: pip install -r requirements-dev.txt", file=sys.stderr)
        return 1
    return subprocess.call([sys.executable, "-m", "pytest", str(ROOT / "tests"), "-v"])


def run_java_pure() -> int:
    gradlew = ROOT / "java-pure" / "gradlew.bat"
    if not gradlew.exists():
        print("java-pure/gradlew.bat not found — skipping JVM tests", file=sys.stderr)
        return 0
    return subprocess.call([str(gradlew), "test", "--no-daemon"], cwd=ROOT / "java-pure")


def main():
    rc = run_pytest()
    if rc != 0:
        sys.exit(rc)
    rc = run_java_pure()
    sys.exit(rc)


if __name__ == "__main__":
    main()
