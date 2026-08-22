# ViDAR Field Validation Log

Record pass/fail results from [ROADMAP.md](ROADMAP.md) Phase 6 before trusting ViDAR in competition auto.

| Date | Hub firmware | Camera count | Test row | Pass/Fail | Notes |
|------|--------------|--------------|----------|-----------|-------|
| | | | | | |

---

## 0 — Bench metrics (no robot)

### Desktop (recorded)

Run `python scripts/bench_metrics.py` on a dev machine. Full write-up: [benches/desktop-2026-08-22.md](benches/desktop-2026-08-22.md).

| Date | Host | Element FPS | Sanity hits | Pass/Fail | Notes |
|------|------|-------------|-------------|-----------|-------|
| 2026-08-22 | Win11 / Python 3.12.10 | 648.8 | 1 | **PASS** (desktop only) | Not Hub; see bench artifact |

| Test row | Target | Pass/Fail | Notes |
|----------|--------|-----------|-------|
| Element FPS per camera | ≥ 15 | **PASS** (desktop) | `python scripts/bench_metrics.py` → 648.8 FPS |
| Tag decode latency | < 400 ms | | Manual OpMode on Hub |
| Tag decode CPU spike | acceptable at 2 s interval | | Manual OpMode (Dashboard off-match only per R704) |
| Plate false positives | < 1/min on empty field | | Manual / sim |

### Control Hub procedure (fill when hardware exists)

Do **not** invent numbers. When a Hub is available:

1. Flash/deploy TeamCode with ViDAR; configure USB webcams in the Driver Station.
2. Run **ViDAR: Discover** at the intended camera count and resolution (typically 640×480).
3. After ~30 s of steady detections, read telemetry:
   - **Tick ms** — `p50`, `p95`, `max`, `n` (`VidarLatencyWindow` / `diagnostics().observationTick*`)
   - Portal FPS / dropped frames / decode drops from camera metrics
4. Paste a row into the top table and Phase 0 Hub cells below with: date, Hub firmware, camera count, resolution, Tick ms, FPS.
5. Optional: FTC Dashboard **off-match only** (manual R704) for graphs — never during a MATCH.

| Date | Hub firmware | Cams | Res | Tick p50/p95/max ms | Portal FPS | Pass/Fail | Notes |
|------|--------------|------|-----|---------------------|------------|-----------|-------|
| | | | | | | | *empty — hardware blocked* |

## 1 — Single-camera calibration

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| `DEFAULT_ELEMENT_DIAMETER` measured | | |
| Floor LUT within ±3″ at 12/24/36/48″ | | |
| `focalLengthPx` within ±10% of tape measure | | |
| Plate detects real alliance panel, rejects red tape on floor | | |

## 2 — Multi-camera USB stress

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| 2 cams: 10 min no disconnect | | |
| 4 cams: 10 min no disconnect | | |
| Wi‑Fi DS link stable during vision (USB 3.0 preferred) | | |
| Record worst-case loop time in OpMode | | |

## 3 — Match lighting

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| Venue lighting: elements + plates at 24″ and 48″ | | |
| Glare: perforated elements still detected | | |
| Motion blur: slow drive-by detection rate | | |

## 4 — Integration

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| `VidarWorldModel` foe memory survives 1 s occlusion | | |
| Spatial Map lists match live element position | | |
| Same `trackId` survives 0.5 s element occlusion with odom | | |
| `elementId#0` / `#1` per type ranked by distance | | |
| Static element field velocity ≈ 0 after settle | | |
| Tag fix + odom backdating vs known field dimension | | |

## 5 — Pedro auto routines

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| Localization module provides stable `Pose2D` at 20 Hz | | |
| ViDAR tag correction every 2+ s does not jerk path | | |
| Assisted intake uses `nearestElement()` without fighting Pedro turn | | |
