# Agent and contributor expectations

This file is for humans and coding agents working in The-Allsparks/ViDAR.

Treat ViDAR as production FTC library software. Features that work in isolation are not enough: new code must stay organized, testable, and hard to accidentally degrade.

## What ViDAR is

ViDAR converts USB webcam detections into timestamped **robot-space** observations (elements, alliance plates, sparse AprilTags). It is **passive**: it must never command motors or servos.

Competition path: Java TeamCode on the Control Hub (`teamcode/.../vidar/`). Python, Docker, and the browser sim are off-robot only.

## Module responsibilities

| Area | Responsibility | Must not |
|------|----------------|----------|
| `vidar` root | Public facade (`VidarSpatial`), enums, sample OpModes | Own hardware threads or OpenCV loops |
| `vidar.runtime` | Process singleton, camera attach/detach, workers | Pathing / motor APIs |
| `vidar.detect` | Contour / plate OpenCV pipelines | Field pose ownership |
| `vidar.tag` | Scout + budgeted AprilTag decode | Alter pose from scout observations |
| `vidar.fusion` | Multi-camera pick, localization gates | Direct VisionPortal construction |
| `vidar.world` | Short-term tracks, TTL, association | HardwareMap / OpenCV |
| `vidar.geometry` | Transforms, ground plane, range fusion | Detect/tag implementations |
| `vidar.integration` | Optional Pedro adapters | Compile dependency on Pedro |
| `vidar.config` | Season/robot JSON | Detection algorithms |
| `sim/` + `src/vidar/` | Parity / teaching | Claim Control Hub performance |

Allowed dependency direction and the frozen import graph: [docs/JAVA_PACKAGE_MAP.md](docs/JAVA_PACKAGE_MAP.md) and `tests/architecture/allowed_package_edges.json`.

**New package edges fail CI.** If a dependency is truly required, update the freeze file in the same PR and explain why. Hard-forbidden inversions (`geometry` → `detect`, `integration` → `runtime`, etc.) cannot be waived via the freeze file.

## Commands

```powershell
pip install -r requirements-dev.txt
python -m pytest tests/ -v
# architecture guards live under tests/architecture/ and run with pytest

cd java-pure
.\gradlew.bat test --no-daemon

python scripts/run_tests.py
python scripts/bench_metrics.py
```

There is no repo-wide Java formatter yet (Spotless deferred — formatting the existing tree is a dedicated change). Match surrounding TeamCode style: 4-space indent, existing class/method conventions.

## Tests for new functionality

- Logic that does not need OpenCV, VisionPortal, or HardwareMap belongs in `java-pure` JVM tests.
- Config / geometry / fusion math also needs a Python parity test when the browser sim or `src/vidar` mirrors it.
- Lifecycle, TTL, and worker behavior must not be “tested only on a hub.”
- Do not add Android emulator or EasyOpenCV jobs without an issue.

## Performance

See [docs/PERFORMANCE.md](docs/PERFORMANCE.md).

Critical paths: VisionPortal callback, contour/tag process, `VidarObservationWorker` (~1 ms tick), `VidarRuntime.observationTick()`, OpMode `loop()`.

On those paths:

- Do not add `Thread.sleep`, filesystem I/O, or network calls.
- Do not construct `VisionPortal.Builder` outside attach/init (`VidarVision`).
- Do not allocate unbounded queues. Mailboxes are latest-wins.
- Prefer reused Mats / scratch buffers over per-frame `new ArrayList` / `String.format` in workers. OpMode telemetry formatting is allowed but keep it modest.
- Do not “optimize” Control Hub behavior without measurement ([issue #27](https://github.com/The-Allsparks/ViDAR/issues/27)).

CI may only apply **generous** JVM ceilings (algorithmic catastrophe), never millisecond Hub budgets.

## Dependencies

- Prefer no new Maven/pip dependencies. java-pure is Java 11 bytecode; CI compiles TeamCode against FTC SDK **v11.2.1**.
- Do not add a DI framework.
- Do not add ArchUnit to TeamCode (Android/FTC). Architecture is enforced by source tests under `tests/architecture/`.
- Pedro Pathing stays an optional consumer in `vidar.integration` with no Maven dependency.

## Public API

Teams are expected to call `VidarSpatial`, snapshot types, diagnostics, JSON config, and optional Pedro adapters.

Do not widen `VidarSpatial.runtime()` or other internals without an issue. Do not break existing `create` / `update` / `snapshot` / `close` signatures without a documented migration.

## Architectural changes

If you change package boundaries, threading, or snapshot publication, update `docs/JAVA_PACKAGE_MAP.md`, `docs/SYSTEM_DESIGN.md`, and architecture tests in the same PR.
