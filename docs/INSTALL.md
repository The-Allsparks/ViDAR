# Installing ViDAR into an FTC project

This is the **canonical student install** for ViDAR **0.2.0** (see repo-root [`VERSION`](../VERSION)). Source-copy remains supported; the version file is how you know which revision you copied.

**Supported FTC SDK:** `v11.2.1` (CI `java-compile` pin — see [CONTRIBUTING.md](../CONTRIBUTING.md)).

Desktop Python, Docker, and the browser sim are **off-robot only**. Do not deploy them to the Robot Controller.

## Source-copy audit (current)

| Path | Role | Goes on Control Hub? |
|------|------|----------------------|
| `teamcode/org/.../vidar/` | Competition library + sample OpModes | **Yes** → TeamCode `src/main/java/.../vidar/` |
| `teamcode/org/.../VidarTeamConfig.java` | Optional team config entry | **Yes** if used |
| `teamcode/assets/vidar/` | Default season/robot JSON | **Yes** → TeamCode `src/main/assets/vidar/` |
| `config/robots/*.json` | Templates | Copy one into assets as `robot.json` |
| `java-pure/` | Desktop JVM tests + stubs | **No** |
| `sim/`, `src/vidar/`, `tests/`, `docker/` | Sim / parity / CI | **No** |

ViDAR does **not** command motors. Your OpMode (or pathing library) remains the only drivetrain owner.

## Install steps

1. Clone or download this repository at a known tag/commit. Confirm `VERSION` matches the release you intended.
2. Open [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) at tag **`v11.2.1`** (or newer only after ViDAR CI is updated).
3. Copy `teamcode/org/firstinspires/ftc/teamcode/vidar/` → `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vidar/`.
4. Copy `teamcode/assets/vidar/` → `TeamCode/src/main/assets/vidar/`.
5. Optional: copy `VidarTeamConfig.java` beside the `vidar` package if you use it.
6. Copy a robot template from `config/robots/` to `assets/vidar/robot.json` and calibrate.
7. Name webcams `Webcam 1` … `Webcam 4` as needed; set camera count in JSON / `VidarConfig`.
8. Run **ViDAR: Discover**, then call `spatial.close()` in every OpMode `stop` path (see [LIFECYCLE.md](LIFECYCLE.md)).

## Version identity

| File | Field |
|------|--------|
| `VERSION` | Single-line semver (authoritative for this repo) |
| `java-pure/build.gradle` | `version = '…'` — must match `VERSION` |
| README “Current status” | Must match `VERSION` |

CI (`tests/architecture/test_version_identity.py`) fails if these drift.

Upgrades: replace the copied `vidar/` tree and assets with a newer ViDAR checkout; do not mix files from two versions.

## What “versioned install” means today

There is **no** Maven/composite Gradle publish of ViDAR yet. “Versioned” means:

1. You install from a tagged git revision or a release whose `VERSION` you record.
2. CI compiles that tree against a **pinned** FTC SDK.
3. You can answer “which ViDAR is on this robot?” by reading `VERSION` from the checkout you copied.

A future composite-Gradle or published AAR path would be an additive alternative; source-copy remains valid for students.

## Related

- [LIFECYCLE.md](LIFECYCLE.md) — VisionPortal ownership and `stop()`
- [CONFIGURATION.md](CONFIGURATION.md) — season/robot JSON
- [#33](https://github.com/The-Allsparks/ViDAR/issues/33) — packaging epic
