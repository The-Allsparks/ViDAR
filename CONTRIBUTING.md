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

## Do not commit

- `.env` or credentials
- `captures/*` (runtime sim captures)
- `__pycache__/`, `.pytest_cache/`, `.venv/`, `venv/`

## Line endings

The repository stores LF line endings (see [.gitattributes](.gitattributes)). Git will normalize on commit; shell scripts must remain LF for Docker/Linux.

## Field validation

Record hardware test results in [docs/validation-log.md](docs/validation-log.md) as you complete roadmap checklist items.
