# Contributing to ViDAR

## Setup

```powershell
git clone <repo-url>
cd ViDAR
pip install -r requirements-dev.txt
```

## Running tests

```powershell
python -m pytest tests/ -v
# or
python scripts/run_tests.py
```

Tests use a pure-Python mirror of core Java logic in `tests/pure/` — no Control Hub or OpenCV required.

`tests/architecture/` is part of the same pytest run. It freezes the Java package import graph, forbids drivetrain commands, and blocks `Thread.sleep` / `VisionPortal.Builder` on hot paths. See [AGENTS.md](AGENTS.md).

### Java unit tests (java-pure)

JVM tests compile selected TeamCode classes with Android/FTC stubs:

```powershell
cd java-pure
.\gradlew.bat test
```

Or run everything:

```powershell
python scripts/run_tests.py
```

`java-pure` runs in GitHub Actions. Make it a **required** check on `main` (issue #24) so a red JVM suite cannot merge. Today branch protection requires `test (ubuntu-latest)`, `test (windows-latest)`, and `java-compile` against FTC SDK `v11.2.1`.

## Browser simulator

```powershell
.\scripts\serve_sim.ps1
```

Open http://127.0.0.1:8765 and click **Start**.

## Java on Control Hub

ViDAR's competition path is Java. Copy the library into your FTC SDK workspace:

```
teamcode/org/firstinspires/ftc/teamcode/vidar/
  → TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vidar/
```

See [README.md](README.md) for OpMode names and configuration.

### Supported FTC SDK

CI `java-compile` clones [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) at a **pinned tag**, not `main` tip.

| Field | Value |
|-------|--------|
| **Supported tag** | `v11.2.1` |
| **Workflow env** | `FTC_SDK_REF` in `.github/workflows/test.yml` |

Bump the pin deliberately when validating a newer SDK; do not treat an unpinned clone as compatibility proof. Desktop `java-pure` tests do **not** claim Control Hub readiness.

## Quality and performance

- [AGENTS.md](AGENTS.md) — architecture, tests, hot-path rules for humans and coding agents
- [docs/JAVA_PACKAGE_MAP.md](docs/JAVA_PACKAGE_MAP.md) — packages and allowed dependency direction
- [docs/PERFORMANCE.md](docs/PERFORMANCE.md) — Hub vs desktop measurement; what CI can and cannot gate

Do not add Java/Android static-analysis plugins to TeamCode without checking FTC SDK compatibility. Architecture is enforced with source tests, not ArchUnit.

## Do not commit

- `.env` or credentials
- `captures/*` (runtime sim captures)
- `__pycache__/`, `.pytest_cache/`, `.venv/`, `venv/`

## Line endings

The repository stores LF line endings (see [.gitattributes](.gitattributes)). Git will normalize on commit; shell scripts must remain LF for Docker/Linux.

## Field validation

Record hardware test results in [docs/validation-log.md](docs/validation-log.md) as you complete roadmap checklist items.
