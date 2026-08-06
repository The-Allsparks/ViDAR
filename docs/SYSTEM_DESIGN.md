# ViDAR System Design

Status labels used throughout ViDAR documentation:

| Label | Meaning |
|-------|---------|
| **Implemented** | Code exists in `teamcode/.../vidar/` |
| **Tested in simulation** | Covered by browser sim or Python tests |
| **Tested on one camera** | Manual OpMode validation on a single webcam |
| **Tested on multiple cameras** | Not yet validated at scale |
| **Hardware validated** | Sustained Control Hub USB + CPU stress test passed |
| **Planned** | Documented but not implemented |

## Architecture (preserved)

```
Per-camera detection → Multi-camera fusion → Short-term world model
                              ↓
                    AprilTag observations (localization separate)
                              ↓
                    Pedro Pathing via external pose provider
```

## Element detection — **Implemented**, **Tested in simulation**

Unified pipeline: `VidarContourProcessor` (season `elements[]` + `plates[]` in one scaled ROI pass).

1. Crop to per-camera element ROI (default: lower 65% of frame)
2. HSV threshold per configured element or plate
3. Configurable morphology (open/close/erode/dilate)
4. Circle elements: contour filter → `minEnclosingCircle` → interior validation → optional local Hough
5. Plate elements: `minAreaRect` → white-digit ratio gate → width-based range

Detector mode per element: `COLOR_BLOB` or `COLOR_BLOB_WITH_LOCAL_HOUGH` (see season JSON / `VidarElementSpec`).

## Regions of interest — **Implemented**, **Tested in simulation**

Per-camera via `VidarCameraProfile.roiConfig` (`VidarCameraRoiConfig`):

| ROI | Default | Overlaps |
|-----|---------|----------|
| Element | Lower 65% | Yes |
| Plate | Middle 40% starting at 30% | Yes |
| AprilTag | Upper 65% | Yes |

Horizon is per-camera (`horizonRowPx` in process coordinates). ROI calibration OpMode: `VidarRoiCalibrationOpMode`.

## Camera scheduling — **Implemented**, not **Hardware validated**

States (`VidarCameraScheduler.State`):

| State | Streaming | Processors |
|-------|-----------|------------|
| PRIMARY | On | Element + plate + tag schedule |
| SECONDARY | On | Lightweight element only |
| IDLE | On | All disabled |
| DEEP_IDLE | Off after 2.5 s debounced | All disabled |

Cameras normally remain streaming while processors are disabled. Stream stop is delayed and debounced.

Health tracking: `VidarMetrics.CameraHealth` (CONFIGURED → CONNECTED → STREAMING → PROCESSING → HEALTHY / FAILED).

## AprilTag — **Implemented**, **Tested in simulation**

- Official FTC `AprilTagProcessor` via `VidarTagCropDecoder`
- Scout (`VidarTagScoutRunner`) identifies probable tag regions on a dedicated worker thread
- Scout observations (`VidarTagScoutObservation`) **never alter absolute pose**
- Async crop decode via `VidarTagDecodeWorker` + `VidarFrameMailbox` buffer swap
- Decode budget: `VidarDecodeArbiter` (1 decode / second global)
- Pose gates in `VidarLocalizationFusion`

## Plate ranging — **Implemented**, **Tested in simulation**

Primary: `distance = plateWidth × focalLengthPx / observedPixelWidth`

Floor LUT used only as secondary consistency check when geometry supports it.

## Range fusion — **Implemented**, **Tested in simulation**

`VidarRangeResult` with uncertainty-weighted fusion (up to **three** estimates: elements = SIZE + FLOOR + GROUND_PLANE; plates = width + floor + ground @ z=0):

`fusedDistance = Σ(weight × distance) / Σ(weight)`

Component estimates exposed as `source0`, `source1`, and `sourceCount` (up to 3) for telemetry. **ViDAR: Discover** logs `size` / `floor` / `ground` on the element detail line.

## World model — **Implemented**, **Tested in simulation**

`VidarWorldModel` motion-corrects tracks using odom delta (translation + rotation) or field-pose reprojection when available.

## Spatial facade — **Implemented**, **Tested in simulation**

`VidarSpatial` is the Pedro-style entry point: one `update()` per loop, no motor output.

| Output | Role |
|--------|------|
| `fieldPose()` / `robotPose()` | Pose estimates (tag fusion + optional odom supplier) |
| `elements()` | Season game elements (ranked live + remembered when motion tracking active) |
| `allies()` | Friendly alliance plates |
| `foes()` | Opponent plates |
| `intakeBlocked()` | Spatial hint — foe in intake cone |

Motion-corrected tracks run only when `isMotionTrackingActive()` (odom supplier + `WORLD_MOTION_TRACKING_ENABLED`). Without odom, queries return live detections only.

**Track associator:** each cycle predicts field position (`pos + velocity × dt`), reprojects to robot frame, gates detections within `WORLD_TRACK_GATE_RADIUS_IN`, updates velocity with EMA, coasts on miss. Elements classify as `STATIC` after stable low velocity; foes/allies as `MOVING` when speed exceeds threshold.

## Multi-camera — **Implemented**, not **Hardware validated**

Architecturally supports 1–4 cameras. Four simultaneous cameras require USB hub validation on the actual Control Hub — configuration success does not guarantee USB stability.

## Resource budgeting — **Implemented**

`VidarResourceBudget` degrades tag frequency, plates, and secondary cameras based on measured loop CPU.

## Simulator parity — **Tested in simulation**

Browser sim (`sim/`) mirrors Java logic for ROIs, color-blob detection, weighted range fusion, and non-localizing scout observations. Cross-language field names and method contracts are defined in [API.md](API.md). Intentional differences documented in `sim/README-SIM.md`.

## Competition legality

Teams must verify final implementation against the current-season FTC manual. ViDAR does not claim competition readiness without team validation.
