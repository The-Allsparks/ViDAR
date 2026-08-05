# ViDAR Field Validation Log

Record pass/fail results from [ROADMAP.md](ROADMAP.md) Phase 6 before trusting ViDAR in competition auto.

| Date | Hub firmware | Camera count | Test row | Pass/Fail | Notes |
|------|--------------|--------------|----------|-----------|-------|
| | | | | | |

---

## 0 — Bench metrics (no robot)

| Test row | Target | Pass/Fail | Notes |
|----------|--------|-----------|-------|
| Ball FPS per camera | ≥ 15 | | |
| Tag decode latency | < 400 ms | | |
| Tag decode CPU spike | acceptable at 2 s interval | | |
| Plate false positives | < 1/min on empty field | | |

## 1 — Single-camera calibration

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| `BALL_DIAMETER_IN` measured | | |
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
| Venue lighting: balls + plates at 24″ and 48″ | | |
| Glare: white ball holes still detected | | |
| Motion blur: slow drive-by detection rate | | |

## 4 — Integration

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| `VidarWorldModel` foe memory survives 1 s occlusion | | |
| Auto Seek stops at `PICKUP_STOP_IN` | | |
| Tag fix + odom backdating vs known field dimension | | |

## 5 — Pedro auto routines

| Test row | Pass/Fail | Notes |
|----------|-----------|-------|
| Localization module provides stable `Pose2D` at 20 Hz | | |
| ViDAR tag correction every 2+ s does not jerk path | | |
| Assisted intake uses `nearestBall()` without fighting Pedro turn | | |
