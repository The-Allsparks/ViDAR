# ViDAR Field Validation Log

Record pass/fail results from [ROADMAP.md](ROADMAP.md) Phase 6 before trusting ViDAR in competition auto.

| Date | Hub firmware | Camera count | Test row | Pass/Fail | Notes |
|------|--------------|--------------|----------|-----------|-------|
| | | | | | |

---

## 0 — Bench metrics (no robot)

Run `python scripts/bench_metrics.py` on your dev machine to fill the rows below.
Re-run on Control Hub hardware before competition.

| Test row | Target | Pass/Fail | Notes |
|----------|--------|-----------|-------|
| Element FPS per camera | ≥ 15 | | `python scripts/bench_metrics.py` |
| Tag decode latency | < 400 ms | | Manual OpMode |
| Tag decode CPU spike | acceptable at 2 s interval | | Manual OpMode (Dashboard off-match only per R704) |
| Plate false positives | < 1/min on empty field | | Manual / sim |

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
| Auto Seek stops at `PICKUP_STOP` | | |
| Tag fix + odom backdating vs known field dimension | | |

## 5 — Pedro auto routines

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| Localization module provides stable `Pose2D` at 20 Hz | | |
| ViDAR tag correction every 2+ s does not jerk path | | |
| Assisted intake uses `nearestElement()` without fighting Pedro turn | | |
