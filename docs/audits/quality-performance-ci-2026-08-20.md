# ViDAR quality, performance, and CI audit

| Field | Value |
|-------|--------|
| **Date** | 2026-08-20 |
| **Audited commit** | `b4d4f00` (`main` at start) plus this hardening branch |
| **Repository** | [The-Allsparks/ViDAR](https://github.com/The-Allsparks/ViDAR) |
| **Java** | TeamCode Java 11 bytecode; CI JDK 17; Gradle 8.7 |
| **FTC SDK** | `v11.2.1` (`java-compile`) |
| **Application refactors** | None (issues only) |

This audit does not repeat the 2026-08-17 product/correctness audit. It scores **structure**, **performance readiness**, and **CI enforcement**, then adds source-level architecture guards.

---

## Build / test baseline (before CI edits)

| Suite | Result |
|-------|--------|
| `python -m pytest tests/` | **PASS** — 109 tests, ~4 s |
| `cd java-pure && ./gradlew test` | **PASS** — 62 `@Test` methods |
| `java-compile` vs FTC SDK | Not re-run locally (clones FtcRobotController). Last merge on `main` (PR #36) was green. |

After this branch: **119** pytest tests (10 architecture guards) and **64** JVM tests (two generous throughput ceilings).

Compiler: `-Xlint:unchecked,deprecation` reports **3 warnings** (`WORLD_MERGE_RADIUS_IN` deprecated without `@Deprecated`, two call sites). Not `-Werror`.

---

## Overall structural score: **58 / 100**

| Category | Score | Evidence | Strengths | Weaknesses | Most important improvement |
|----------|------:|----------|-----------|------------|----------------------------|
| Architecture & boundaries | 11/20 | 13 packages match `JAVA_PACKAGE_MAP`; cycles frame↔detect, schedule↔runtime/detect, geometry↔fusion/runtime; `config`→`runtime`; root dumps facade+OpModes+constants | Clear runtime vs attach split; Pedro is an adapter | Boundaries are documentation, not enforced (until this branch); camera profiles live under runtime but feed geometry | Shrink cycles; move mount/ROI types out of runtime |
| Maintainability & readability | 8/15 | `VidarContourProcessor` ~1038 physical lines; `VidarFusionEngine.update` ~124 lines; `VidarContourTarget` ~40-arg ctor | Names generally match roles; little commented-out code | God methods; dual `VidarGeometry` / `VidarRangeFusion`; `VidarConfig` vs JSON | Split fusion tick and contour pass |
| Dependency/state management | 8/15 | `VidarRuntime` singleton; `VidarTagGate` static volatiles; observation worker starts in runtime ctor | Atomic snapshot publish; attach/detach documented | Hidden init order; global tag gate; `runtime()` exposes internals | Runtime-owned tag gate; document/hide `runtime()` |
| Testing & testability | 9/15 | World TTL + worker tests exist (#23 largely done); java-pure still excludes detect, FusionEngine, Spatial, most runtime | Strong geometry/fusion unit tests; Python parity | Perception stack untested on JVM; no Hub tests | Expand java-pure or seams for FusionEngine |
| Build/repository hygiene | 7/10 | Gradle wrapper 8.7; pinned SDK; duplicate default JSON; stale excludes (removed here) | MIT; gitattributes LF | Duplicate assets; two config layers | Single JSON source of truth |
| Static-analysis coverage | 3/10 → 6/10 after this branch | No Checkstyle/PMD/Spotless/ArchUnit. Source architecture tests added. | Targeted guards beat a noisy linter blast | No formatter; 3 deprecation warnings | Spotless later; keep architecture tests |
| Documentation & contributor guidance | 4/5 | Excellent SYSTEM_DESIGN/API/frames; no AGENTS.md before this branch | Honest hardware-not-validated status | Stale README test count (fixed) | Keep docs matched to freeze file |
| CI enforcement | 5/10 → 8/10 after this branch | Required: pytest + java-compile. java-pure ran but was optional (#24). No permissions/SHA pins (#25). | FTC compile job exists | Optional JVM tests; floating action tags | Require java-pure; pin actions |

**Current Structural Score: 58 / 100** (functional architecture with real package cycles, weak static enforcement before this work, good docs).

Classification: **Functional but quality debt exists.** A passing build does not imply isolated modules.

---

## Performance score: **47 / 100**

| Category | Score | Evidence | Weak points | Likely impact | Confidence | Recommended measurement |
|----------|------:|----------|-------------|---------------|------------|-------------------------|
| Algorithmic efficiency | 14/20 | Range fusion O(1) per call; associator O(tracks×detections) with small N; MultiCameraFusion sorts small lists | FusionEngine.update does too much per 1 ms tick | Medium if track counts grow | Medium | Hub: tick time vs track count |
| Allocation / GC | 8/20 | Pooled Mats (good). Per-frame: mailbox `copyTo`, ElementDetector ArrayList/HashMap, Scalars, snapshot ArrayLists at worker rate | Snapshot build ~1 kHz; portal-thread memcpy | **High** GC/jitter on Hub | High (static) | Allocation traces on Hub; drop worker rate |
| Critical-path latency | 7/20 | `synchronized (VidarRuntime)` around fusion+world; OpMode `pinSnapshot` contends; 1-cam processFrame on portal thread | Unbounded wait = fusion duration | **High** loop jitter | High (static) | p50/p95 observationTick + OpMode loop |
| Hardware / I/O | 9/15 | Scheduler disables processors before stream stop; latest-wins mailbox; tag decode budgeted | Full-frame copy every capture; DEEP_IDLE stream stop spikes | High USB/CPU | Medium | USB 2 vs 4 cam FPS; droppedFrames |
| Concurrency | 6/10 | 3–4 threads; bounded mailboxes; workers joined on shutdown | Lock across fusion; static TagGate; 1 ms sleep wakeup | Medium CPU + contention | Medium | Thread traces; lock hold time |
| Performance observability | 3/10 | Metrics: FPS, drops, health. No p50/p95/max tick. validation-log empty | Cannot regress what is not recorded | Blocks all Hub optimization | High | Add percentile fields |
| Regression protection | 1/5 → 3/5 after this branch | No Hub CI. Generous JVM ceilings added. Pattern guards for sleep/portal/motors | Timing CI would be flaky | N/A | High | Keep ceilings generous |

**Current Performance Score: 47 / 100** — **Significant performance risk** (unmeasured Hub path + strong static evidence of allocation/lock issues).

Do not treat desktop `bench_metrics.py` ≥15 FPS as Hub proof.

### Highest-risk performance findings

| ID | Class | Finding |
|----|--------|---------|
| P-A | STRONG STATIC | `VidarFrameMailbox.publish` full-frame `copyTo` on portal thread |
| P-B | STRONG STATIC | `observationTick` holds `synchronized (this)` for `engine.update()` + `world.update()` |
| P-C | STRONG STATIC | Observation worker `sleepQuiet(1)` plus snapshot `build` every tick |
| P-D | STRONG STATIC | ElementDetector / ContourDetect per-pass lists, Scalars, hit objects |
| P-E | STRONG STATIC | OpMode `String.format` + telemetry every loop |
| P-F | SUSPECTED | DEEP_IDLE stream stop/resume multi-10 ms USB spikes |
| P-G | MEASURED | Desktop Python bench exists; Hub rows in validation-log are **empty** |

---

## CI coverage added (this branch)

| Check | Prevents |
|-------|----------|
| `tests/architecture/test_package_boundaries.py` | New vidar package edges; hard-forbidden layering inversions |
| `tests/architecture/test_hotpath_guards.py` | Motors/servos in ViDAR; `Thread.sleep` outside workers; `VisionPortal.Builder` outside `VidarVision`; File I/O in detect/fusion/world/geometry |
| `tests/architecture/test_asset_parity.py` | Silent drift of duplicated default JSON |
| `RangeFusionBudgetTest` / `WorldModelBudgetTest` | Catastrophic JVM algorithmic regression (5 s ceiling, not ms Hub budgets) |
| Workflow `permissions: contents: read` + SHA-pinned actions | Over-privileged token; tag-moving supply chain (#25) |
| Dependabot (actions, pip, gradle) | Stale GitHub Actions / deps without a human bump |
| java-pure exclude comments; removed dead excludes | Mystery excludes; stale filenames |
| `-Xlint` without `-Werror` | Visibility of deprecation/unchecked (3 existing warnings) |

### Intentionally not added

| Tool | Why |
|------|-----|
| ArchUnit | java-pure excludes most sources; TeamCode is Android/FTC |
| Spotless / google-java-format | Would reformat ~120 files; separate baseline issue |
| PMD / SpotBugs / Checkstyle | Noisy without a baseline; Android compatibility unclear |
| `-Werror` | 3 existing warnings + stubs |
| Hub millisecond performance gates | Runner noise; not a Control Hub |
| CodeQL workflow | Repo already has CodeQL alerts; do not duplicate |

### Cannot reliably test in CI

Hub USB, GC, VisionPortal jitter, AprilTag decode time, 4-cam contention, DS telemetry cost. Profile on hardware; instrument `VidarMetrics`; fill validation-log.

---

## GitHub issues

| Issue | Priority | Category | Problem | CI Detectable? |
|-------|----------|----------|---------|----------------|
| [#37](https://github.com/The-Allsparks/ViDAR/issues/37) | P1 | epic | Quality / performance / CI parent | Partial (guards in this PR) |
| [#38](https://github.com/The-Allsparks/ViDAR/issues/38) | P1 | architecture | Package cycles | Yes — freeze; shrink is manual |
| [#39](https://github.com/The-Allsparks/ViDAR/issues/39) | P2 | architecture | God methods | No (review) |
| [#40](https://github.com/The-Allsparks/ViDAR/issues/40) | P1 | performance | Tick lock, 1 kHz snapshots, mailbox copy | Pattern only; Hub timing no |
| [#41](https://github.com/The-Allsparks/ViDAR/issues/41) | P2 | architecture | TagGate statics | Future unit test |
| [#42](https://github.com/The-Allsparks/ViDAR/issues/42) | P2 | architecture | Dual config surface | `-Xlint` today; more later |
| [#43](https://github.com/The-Allsparks/ViDAR/issues/43) | P2 | testing | java-pure FusionEngine/Spatial | Yes once tests exist |
| [#44](https://github.com/The-Allsparks/ViDAR/issues/44) | P1 | performance | Metrics percentiles | Helper unit test; not Hub |
| [#45](https://github.com/The-Allsparks/ViDAR/issues/45) | P3 | hygiene | Duplicate default JSON | Yes — parity test now |
| [#46](https://github.com/The-Allsparks/ViDAR/issues/46) | P3 | api | `runtime()` exposure | Optional later |
| [#47](https://github.com/The-Allsparks/ViDAR/issues/47) | P3 | ci | Spotless baseline | Yes after format PR |
| [#24](https://github.com/The-Allsparks/ViDAR/issues/24) | P2 | ci | Require java-pure check | Settings (not applied here) |
| [#25](https://github.com/The-Allsparks/ViDAR/issues/25) | P2 | ci | Pin Actions + permissions | Implemented this branch |

Existing issues **not duplicated**: #22 sim fusion, #23 world TTL (mostly done → #43), #26/#27 hardware benches, #28 templates, #33 packaging.
