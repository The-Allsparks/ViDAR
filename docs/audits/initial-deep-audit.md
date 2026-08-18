# ViDAR initial deep audit

| Field | Value |
|-------|--------|
| **Date of audit** | 2026-08-17 |
| **Audited commit SHA** | `74747de7a52ccbfae8a47b057e44afb79814abc9` (`feature/geometry-authoritative-ranging`) |
| **Default-branch SHA** | `699953f6cddb5b8b872a84551eda1a761e5468b9` (`main`, merge of PR #17) |
| **Auditor identity** | `TA-C-GHill` |
| **Repository** | [The-Allsparks/ViDAR](https://github.com/The-Allsparks/ViDAR) |
| **Automatic merge** | **false** (not explicitly authorized) |

This audit covers the current worktree (geometry-authoritative ranging, open PR #18) and `main`. Findings cite files, GitHub objects, or commands. Speculative defects without evidence are omitted.

---

## Executive summary

ViDAR is a **passive** FTC vision library: USB webcam detections become timestamped robot-space observations. It does **not** command motors. Architecture (runtime singleton, per-OpMode camera attach, snapshot-first student API, sparse AprilTag decode, optional Pedro bridge) matches the stated purpose.

Maturity is **0.2.0, simulation-tested, not hardware-validated**. The empty [docs/validation-log.md](../validation-log.md) is the honest readiness signal: do not treat ViDAR as competition-ready.

The highest-value in-flight work is **issue #13 / PR #18** (ground-plane range authoritative). Required CI on that PR is green. Remaining product risk is not “missing features”; it is **stale last-detections**, **silent worker failures**, **sim/Java range-fusion drift**, and **untested runtime lifecycle**.

No **BLOCKER** prevents continued development. No finding shows ViDAR can energize hardware. Replay/sim cannot reach physical outputs (sim is browser/Python only).

---

## Project purpose

**Problem:** Convert 1–4 UVC webcam images into calibrated robot-frame positions for game elements, alliance plates, and sparse AprilTags on an FTC Control Hub.

**Intended users:** Beginning and advanced FTC students, mentors, teams adopting one feature or the full facade, and downstream Allsparks libraries (TRACE/HELM consumers later).

**Explicit responsibilities:** Detection, ranging, multi-camera fusion, short-term world memory, diagnostics, optional pose *observations* for a pathing library.

**Not ViDAR:** Continuous field pose, path following, motor commands, power management (AMPER), mechanism coordination (MIMIC), radio/link safety (BEACON), recording/replay product (TRACE), task selection (HELM).

---

## Current maturity

| Area | Status |
|------|--------|
| Product version | 0.2.0 (`java-pure/build.gradle`) |
| Implementation phase | Passive observability + sparse tag fixes |
| Simulation | Browser sim + Python tests + `java-pure` JVM tests |
| Hardware | **Not validated** (empty validation log) |
| Releases | None |
| Readiness gate (trustworthy timestamped observations) | **Partial** — timestamps exist; last-processor stickiness and unvalidated extrinsics remain |

---

## Implemented capabilities

- Unified contour element + plate pipeline (`VidarContourProcessor`)
- Per-camera overlapping ROIs + alliance selector
- Explicit frames/transforms (`vidar.geometry`)
- Multi-camera fusion + `VidarSpatial` facade
- Process singleton `VidarRuntime` + `VidarVisionAttachment` (Auto → TeleOp camera recycle)
- Snapshot-first API; OpModes listed as no-motor
- AprilTag scout + async crop decode with global 1 Hz budget
- Pedro pose bridge (no Pedro Maven dependency) — PR #12 merged
- Vitelli XYZ mount rotation — PR #17 merged
- Geometry-authoritative range fusion — **implemented on PR #18, not on `main`**
- Browser simulator, JSON season/robot config, teaching OpModes

## Documented but unimplemented capabilities

| Item | Evidence |
|------|----------|
| Offline Ceres extrinsic optimizer | ROADMAP “Planned”; issue #15 |
| Intrinsics quality metadata (estimated vs calibrated) | Issue #14; FOV defaults look first-class in robot JSON |
| Rich calibration-axis harness | Issue #16; sim overlay is basic |
| Optional Brown-Conrady runtime undistort | ROADMAP; correctly deferred |
| 4-camera USB stress / match lighting / Pedro auto jerk tests | Empty validation log |
| Sim multi-cam preview | ROADMAP backlog, deferred |

---

## Architecture findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| A1 | INFORMATIONAL | ARCHITECTURE | Module map in `docs/JAVA_PACKAGE_MAP.md` matches packages. Dependency direction is sound: detect → fusion → world → snapshot → `VidarSpatial`. Pedro is an adapter (`vidar.integration`), not a compile dependency. |
| A2 | HIGH | ARCHITECTURE | `VidarObservationWorker` runs `engine.update()` + `world.update()` every **1 ms**. World association therefore clocks off the worker, not new camera frames. Last processor outputs are re-consumed until overwritten. Evidence: `VidarObservationWorker.sleepQuiet(1)`, `VidarRuntime.observationTick()`, `VidarVision.update()` copies `contourProcessor.getBestElement()` with no generation/age gate. |
| A3 | MEDIUM | ARCHITECTURE | `VidarWorldModel.update` returns immediately when `vision == null`. Tracks neither coast nor TTL-expire during `detachVision()`. `publishDetachedSnapshot()` still builds a snapshot from the frozen world. Evidence: `VidarWorldModel.java` lines 72–75; `VidarRuntime.publishDetachedSnapshot()`. |
| A4 | MEDIUM | ARCHITECTURE | `java-pure/build.gradle` excludes runtime, world, detect, schedule, most frame/fusion types. JVM tests cannot catch A2/A3. |
| A5 | LOW | ARCHITECTURE | `VidarConfig` remains a large constants class alongside JSON `VidarSettings`. Documented as fallback; still easy to edit the wrong layer. |
| A6 | INFORMATIONAL | ARCHITECTURE | No circular compile-time dependency with Pedro/AMPER/MIMIC/BEACON/TRACE/HELM in this repo. |

God-object risk: `VidarFusionEngine` is large but bounded. Hidden global state: `VidarRuntime` singleton and `VidarTagGate` static camera bearing — acceptable for FTC process lifetime if shutdown is used; `shutdown()` is documented as optional.

---

## Correctness findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| C1 | HIGH | CORRECTNESS | **In flight (PR #18 / #13):** Java/Python fusion now keeps `GROUND_PLANE` distance and uses heuristics as confidence/fallback. Tests cover agree / disagree / geometry-only / heuristic-only. |
| C2 | HIGH | CORRECTNESS | **Browser sim did not receive C1.** `sim/js/geometry.js` `fuseRangeWeighted(dSize, dFloor)` still inverse-variance averages size and floor; no ground plane. `sim/README-SIM.md` still says “size + floor LUT”. Students tuning in sim will not see robot behavior. |
| C3 | MEDIUM | CORRECTNESS | `docs/API.md` still documents `sourceCount: 0–2` while `VidarRangeResult` clamps 0–3 and three-way fusion tests assert `sourceCount == 3`. Left in PR #18 docs. |
| C4 | HIGH | CORRECTNESS | Last-detection stickiness (A2): if the contour processor retains the last blob across frames or camera freeze, `lastSeenNanos` refreshes every worker tick with `missCount == 0`, so TTL never fires. Not reproduced on hardware; invariant is missing in tests. |
| C5 | MEDIUM | CORRECTNESS | README claims “26 tests”; worktree has **51** `@Test` methods under `java-pure/src/test`. |
| C6 | INFORMATIONAL | CORRECTNESS | Scout-never-alters-pose is tested (`LocalizationFusionTest`). Pedro event-id gating is tested (`VidarPedroCorrectionTrackerTest`). |

---

## Safety findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| S1 | INFORMATIONAL | SAFETY | Built-in OpModes (`VidarTeleOp`, Discover, Spatial Map, ROI calibrate, Pedro sample) do not call `DcMotor` / `setPower`. `VidarSpatial` Javadoc: “never commands motors.” |
| S2 | INFORMATIONAL | SAFETY | Replay/sim cannot produce physical outputs. `sim/js/geometry.js` `rangeDrivePower` is a **sim-only** helper and must not be copied into OpModes. |
| S3 | HIGH | SAFETY | `VidarObservationWorker.run` catches all `RuntimeException` and continues. Perception can fail silently while snapshots remain last-good or empty without student-visible diagnostics. Evidence: worker lines 30–34. |
| S4 | MEDIUM | SAFETY | Camera `FAILED` health exists (`VidarMetrics`, fusion skip of `isFailed()` for some loops). Fusion still begins by calling `camera.update()` on failed cameras. Confirm failed cameras cannot keep feeding last plates into world tracks (related to C4). |
| S5 | INFORMATIONAL | SAFETY | Passive library: unsafe *use* is a consumer problem (Pedro `setPose` with fuse-time pose). Docs in `PEDRO_INTEGRATION.md` already warn. |

ViDAR cannot bypass a lower-layer motor safety interlock because it has no motor API.

---

## Performance findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| P1 | HIGH | PERFORMANCE | **Unmeasured on Control Hub.** ROADMAP Phase 6 targets (15 FPS/cam, tag decode &lt; 400 ms) have no recorded results. `scripts/bench_metrics.py` exists for desktop only. |
| P2 | MEDIUM | PERFORMANCE | Observation worker at ~1 kHz plus VisionPortal callbacks, `VidarGlobalVisionWorker`, and optional `VidarTagDecodeWorker` is **four** persistent threads when tags are enabled. Predicted CPU/GC risk; not profiled. |
| P3 | INFORMATIONAL | PERFORMANCE | Resource budget ladder and camera scheduler (processor disable before stream stop) are implemented; not hardware-validated. |
| P4 | INFORMATIONAL | PERFORMANCE | `java-pure` and Python tests are fast in CI (~1 min). Not a Control Hub measurement. |

---

## API / usability findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| U1 | LOW | USABILITY | Primary path `VidarSpatial.create` + `update()` + `snapshot()` is discoverable. `createWithBundledDefaults` is a good first-use escape hatch. |
| U2 | MEDIUM | USABILITY | Two config layers (`VidarConfig` constants vs JSON assets) increase first-use mistakes. |
| U3 | LOW | USABILITY | `spatial.vision()` deprecated in favor of `diagnostics()` + `lastFrame()` — good. |
| U4 | MEDIUM | DOCUMENTATION | Cross-language API doc drifted from implementation (`sourceCount`, sim fusion). |

---

## Testing findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| T1 | HIGH | TESTING | No JVM tests for `VidarRuntime` attach/detach, observation worker, world TTL, or camera-loss snapshots (`java-pure` excludes those sources). |
| T2 | MEDIUM | TESTING | `java-pure` job runs in CI but is **not** a required status check. Required: `test (ubuntu-latest)`, `test (windows-latest)`, `java-compile` only. |
| T3 | INFORMATIONAL | TESTING | Python tests cover config, contour mirror, coordinate frames, units, core geometry. |
| T4 | INFORMATIONAL | TESTING | No disabled/flaky tests found (`TODO`/`FIXME`/`@Disabled` absent in tracked sources). |
| T5 | HIGH | TESTING | Hardware-required tests are documented but **never run** (empty log). Do not claim coverage they do not provide. |

---

## Documentation findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| D1 | MEDIUM | DOCUMENTATION | README, SYSTEM_DESIGN, ROADMAP, API, COORDINATE_FRAMES, PEDRO_INTEGRATION, TEACHING, CALIBRATION are substantial and mostly aligned. |
| D2 | MEDIUM | DOCUMENTATION | Sim README parity table is stale vs PR #18 Java/Python fusion. |
| D3 | LOW | DOCUMENTATION | README “26 tests” is stale (51 `@Test` methods). |
| D4 | MEDIUM | DOCUMENTATION | No `SECURITY.md`, `CODE_OF_CONDUCT.md`, issue templates, or PR template. CONTRIBUTING exists. LICENSE is MIT. |
| D5 | INFORMATIONAL | DOCUMENTATION | No `AGENTS.md` / `.agents/` / `.codex/`. No CODEOWNERS. |

---

## Dependency findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| Dep1 | MEDIUM | SECURITY | `.github/workflows/test.yml` uses floating tags (`actions/checkout@v4`, `setup-python@v5`, `setup-java@v4`). CodeQL alerts **#1–#3**: missing `permissions:` block (`actions/missing-workflow-permissions`). |
| Dep2 | MEDIUM | COMPATIBILITY | `java-compile` clones `FtcRobotController` at `--depth 1` default branch (unpinned SDK revision). |
| Dep3 | LOW | SECURITY | No Dependabot config. Python pins are lower-bounds (`opencv-python-headless>=4.9.0`). |
| Dep4 | INFORMATIONAL | COMPATIBILITY | `java-pure` uses `org.json:json:20240303` and JUnit 5.10.2; Gradle wrapper 8.7. MIT license compatible with typical FTC TeamCode. |
| Dep5 | INFORMATIONAL | SECURITY | No secrets in workflows. `captures/` gitignored via CONTRIBUTING. |

Do not perform broad dependency upgrades without FTC SDK compatibility analysis.

---

## Repository-health findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| R1 | INFORMATIONAL | ARCHITECTURE | Default branch `main`. Protections: strict required checks, dismiss stale reviews, **0** required approving reviews, `enforce_admins: false`, force-push disabled. Merge commit/squash/rebase all allowed. Delete-on-merge **false**. |
| R2 | MEDIUM | USABILITY | Labels are GitHub defaults only. No milestones, no releases, no changelog. |
| R3 | INFORMATIONAL | INTEGRATION | Open product issues: #13, #14, #15, #16. Open PR: **#18** (Fixes #13). Recent merges: #17 mount rotation, #12 Pedro bridge. Recent CI on `main` and PR #18: **success**. |
| R4 | INFORMATIONAL | INTEGRATION | Stale local branches exist (`refactor/*`, `feature/*` already merged). Remote still lists them; cleanup is optional. |
| R5 | MEDIUM | DOCUMENTATION | Issues #13–#16 lack the team’s full acceptance-criteria template (Problem / Evidence / Validation / Safety / Rollback). |

---

## Cross-project integration findings

| ID | Severity | Type | Finding |
|----|----------|------|---------|
| X1 | INFORMATIONAL | INTEGRATION | ViDAR → optional Pedro consumer via `vidar.integration`. No compile-time edge into TRACE/HELM. |
| X2 | INFORMATIONAL | INTEGRATION | FTC maintainer MCP in this environment is scoped to **ftc-dev-tools**, not ViDAR. GitHub operations for this repo used `gh` against `The-Allsparks/ViDAR`. |
| X3 | LOW | INTEGRATION | Conceptual stack `ViDAR → TRACE → HELM` is not implemented here. Do not add those compile dependencies. |

---

## Readiness assessment

**ViDAR must produce trustworthy timestamped observations before advanced fusion or autonomous use.**

| Gate | Met? |
|------|------|
| Passive (no motors) | Yes |
| Explicit frames + Vitelli XYZ mounts | Yes (PR #17 on `main`) |
| Geometry-authoritative ranging | Yes in PR #18; **not on `main` until merge** |
| Sim/Java ranging parity | **No** (C2) |
| Observation age / camera-loss invariants tested | **No** (C4, T1) |
| Worker failure visible | **No** (S3) |
| Hardware validation | **No** |
| Active control / HELM-style tasking | Not applicable (must stay out of scope) |

**Verdict:** Safe to continue library development. **Not** safe to claim match-ready spatial memory or to enable later “active” Allsparks phases on top of unvalidated tracks.

---

## Recommended work order

1. Finish **#13 / PR #18** (human merge; `AUTOMATIC_MERGE=false`).
2. **Safety:** surface observation-worker failures (S3).
3. **Correctness seam:** associate/TTL on **new frame generation**, not 1 ms ticks (A2/C4).
4. **Parity:** browser sim ground-plane fusion (C2).
5. **Tests:** java-pure lifecycle + world-model tests; make `java-pure` required (T1/T2).
6. **CI hygiene:** workflow `permissions` + pin actions (Dep1).
7. **#16** calibration visualization (catches mount mistakes).
8. **#14** estimated vs calibrated intrinsics.
9. Hardware validation log (blocked on robots/hubs).
10. **#15** Ceres optimizer — keep later.
11. Docs/test-count/API `sourceCount` cleanup.

---

## Deferred or rejected ideas

| Idea | Decision |
|------|----------|
| On-hub checkerboard intrinsic OpMode | Already closed on ROADMAP — keep closed |
| TensorFlow core detector | ROADMAP reject — keep rejected for competition path |
| Digit OCR for plates | Closed — white-digit ratio sufficient |
| Runtime auto-switch detector under CPU load | Closed — metrics only |
| Broad dependency upgrades | Deferred pending SDK pin |
| Active drive/intake from ViDAR | Rejected — responsibility of robot app / HELM later |

---

## Evidence and references

- README, `docs/SYSTEM_DESIGN.md`, `docs/ROADMAP.md`, `docs/API.md`, `docs/PEDRO_INTEGRATION.md`, `docs/validation-log.md`
- `teamcode/.../vidar/runtime/VidarRuntime.java`, `VidarObservationWorker.java`, `VidarVision.java`
- `teamcode/.../vidar/geometry/VidarRangeFusion.java`, `sim/js/geometry.js`, `src/vidar/geometry.py`
- `teamcode/.../vidar/world/VidarWorldModel.java`, `VidarTrackAssociator.java`
- `java-pure/build.gradle`, `.github/workflows/test.yml`
- GitHub: issues #13–#16, PR #18, branch protection, CodeQL alerts #1–#3
- Commands: `gh api user` → `TA-C-GHill`; `gh pr checks 18` all pass; `gh issue list --repo The-Allsparks/ViDAR`

---

## Finding index (severity × type)

| ID | Severity | Type |
|----|----------|------|
| C1 / #13 | HIGH | CORRECTNESS |
| C2 | HIGH | CORRECTNESS |
| C4 | HIGH | CORRECTNESS |
| S3 | HIGH | SAFETY |
| T1 | HIGH | TESTING |
| P1 | HIGH | PERFORMANCE |
| A2 | HIGH | ARCHITECTURE |
| A3 | MEDIUM | ARCHITECTURE |
| A4 / T2 | MEDIUM | TESTING |
| C3 / D2 / D3 | MEDIUM / LOW | DOCUMENTATION |
| S4 | MEDIUM | SAFETY |
| Dep1 | MEDIUM | SECURITY |
| Dep2 | MEDIUM | COMPATIBILITY |
| R2 / R5 / D4 | MEDIUM | DOCUMENTATION |
| U2 / U4 | MEDIUM | USABILITY |
| P2 | MEDIUM | PERFORMANCE |
| Remaining | LOW / INFORMATIONAL | various |
