# ViDAR performance expectations

ViDAR must keep Control Hub loop time **predictable**. Throughput on a desktop JVM is not the product.

## Critical paths

| Path | Thread | Notes |
|------|--------|-------|
| VisionPortal callback → `VidarFrameMailbox.publish` | Portal | Full-frame `copyTo` today |
| Contour / tag scout `processFrame` | Portal (1-cam sync) or `VidarGlobalVisionWorker` | OpenCV; pooled Mats |
| AprilTag crop decode | `VidarTagDecodeWorker` | Budgeted ≤ 1 Hz globally |
| Fusion + world + snapshot | `VidarObservationWorker` (~1 ms sleep) | Holds `synchronized (VidarRuntime)` during `engine.update()` + `world.update()` |
| Student OpMode `loop()` | Robot | Must only read snapshots; telemetry `String.format` can dominate DS loop time |

## Budgets (targets, not CI gates)

Record results in [validation-log.md](validation-log.md). Empty cells mean **unmeasured**.

| Metric | Target | Where |
|--------|--------|-------|
| Element FPS / camera | ≥ 15 | Discover telemetry / `scripts/bench_metrics.py` (desktop only) |
| Tag decode | < 400 ms when it runs | Manual OpMode |
| Worst-case OpMode loop | Team-defined; record p50/p95/max | OpMode telemetry |
| Observation worker tick | Bound fusion+world; avoid 1 kHz snapshot churn if Hub CPU is tight | Discover **Tick ms** (`diagnostics().observationTickP50Ms` / `P95` / `Max`) |

## How to measure

Desktop (not Hub):

```powershell
python scripts/bench_metrics.py
```

On the Control Hub:

1. Run **ViDAR: Discover** with the intended camera count.
2. Read **Tick ms** — `p50`, `p95`, `max`, and sample count `n` over a ~1024-sample ring (`VidarLatencyWindow`).
3. Also note portal FPS, dropped frames, and decode drops from `VidarMetrics`.
4. Paste numbers into [validation-log.md](validation-log.md) with firmware, camera count, and resolution.

Do not enable FTC Dashboard streaming during a MATCH (manual R704).

The same tick percentiles are on `VidarSpatial.diagnostics()` after each `update()`:

```java
VidarDiagnostics d = spatial.diagnostics();
// d.observationTickP50Ms / P95Ms / MaxMs / Samples
```

Hot-path code changes: [#40](https://github.com/The-Allsparks/ViDAR/issues/40). Measure first ([#27](https://github.com/The-Allsparks/ViDAR/issues/27)).

## What CI can test

- Architecture / hot-path **pattern** guards (`tests/architecture/`)
- Generous JVM fusion/association ceilings (`RangeFusionBudgetTest`, `WorldModelBudgetTest`) — fail only on catastrophic algorithmic regression
- Python contour sanity / desktop bench script existence

## What CI cannot test

Control Hub USB, GC pauses, VisionPortal callback jitter, AprilTag decode spikes, multi-camera hub contention, DS telemetry cost.

Those require hardware profiling ([issue #27](https://github.com/The-Allsparks/ViDAR/issues/27), [issue #26](https://github.com/The-Allsparks/ViDAR/issues/26)).
