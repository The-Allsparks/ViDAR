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

## Ball detection — **Implemented**, **Tested in simulation**

Default pipeline: `COLOR_BLOB_WITH_LOCAL_HOUGH` (`VidarBallProcessor`).

1. Crop to per-camera ball ROI (default: lower 65% of frame)
2. HSV threshold for pollen color
3. Configurable morphology (open/close)
4. Contour filtering: area, aspect, circularity, fill ratio, interior validation
5. Optional local Hough validation inside candidate patches
6. Legacy full-ROI Hough available via `VidarConfig.BALL_DETECTOR_TYPE = LEGACY_HOUGH`

Rejection telemetry: `VidarBallRejectionStats` exposed in Discover OpMode.

## Regions of interest — **Implemented**, **Tested in simulation**

Per-camera via `VidarCameraProfile.roiConfig` (`VidarCameraRoiConfig`):

| ROI | Default | Overlaps |
|-----|---------|----------|
| Ball | Lower 65% | Yes |
| Plate | Middle 40% starting at 30% | Yes |
| AprilTag | Upper 65% | Yes |

Horizon is per-camera (`horizonRowPx` in process coordinates). ROI calibration OpMode: `VidarRoiCalibrationOpMode`.

## Camera scheduling — **Implemented**, not **Hardware validated**

States (`VidarCameraScheduler.State`):

| State | Streaming | Processors |
|-------|-----------|------------|
| PRIMARY | On | Ball + plate + tag schedule |
| SECONDARY | On | Lightweight ball only |
| IDLE | On | All disabled |
| DEEP_IDLE | Off after 2.5 s debounced | All disabled |

Cameras normally remain streaming while processors are disabled. Stream stop is delayed and debounced.

Health tracking: `VidarMetrics.CameraHealth` (CONFIGURED → CONNECTED → STREAMING → PROCESSING → HEALTHY / FAILED).

## AprilTag — **Implemented**, **Tested in simulation**

- Official FTC `AprilTagProcessor` via `VidarTagCropDecoder`
- Scout (`VidarTagScout`) identifies probable tag regions
- Scout observations (`VidarTagScoutObservation`) **never alter absolute pose**
- Decode budget: `VidarDecodeArbiter` (1 decode / second global)
- Pose gates in `VidarLocalizationFusion`

## Plate ranging — **Implemented**, **Tested in simulation**

Primary: `distance = plateWidthIn × focalLengthPx / observedPixelWidth`

Floor LUT used only as secondary consistency check when geometry supports it.

## Range fusion — **Implemented**, **Tested in simulation**

`VidarRangeResult` with uncertainty-weighted fusion:

`fusedDistance = Σ(weight × distance) / Σ(weight)`

Component estimates and weights exposed in telemetry.

## World model — **Implemented**, **Tested in simulation**

`VidarWorldModel` motion-corrects tracks using odom delta (translation + rotation) or field-pose reprojection when available.

## Multi-camera — **Implemented**, not **Hardware validated**

Architecturally supports 1–4 cameras. Four simultaneous cameras require USB hub validation on the actual Control Hub — configuration success does not guarantee USB stability.

## Resource budgeting — **Implemented**

`VidarResourceBudget` degrades tag frequency, plates, and secondary cameras based on measured loop CPU.

## Simulator parity — **Tested in simulation**

Browser sim (`sim/`) mirrors Java logic for ROIs, color-blob detection, weighted range fusion, and non-localizing scout observations. Intentional differences documented in `sim/README-SIM.md`.

## Competition legality

Teams must verify final implementation against the current-season FTC manual. ViDAR does not claim competition readiness without team validation.
