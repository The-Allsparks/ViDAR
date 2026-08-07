# ViDAR System Design

ViDAR turns USB webcam detections into **calibrated positions in robot space**. Vision pipelines (color blobs, plate geometry, sparse AprilTags) feed explicit coordinate frames, range fusion, multi-camera fusion, and a short-term world model. Your OpMode reads robot-frame observations and tracks; field pose and autonomous pathing stay separate and optional.

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
Per-camera detection → Multi-camera fusion → Short-term world model (robot space)
                              ↓
                    AprilTag observations (localization separate)
                              ↓
                    Your OpMode / optional auto stack (Pedro, Road Runner, custom)
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

`VidarRangeResult` with uncertainty-weighted fusion (up to two contributing estimates, no list allocation):

`fusedDistance = Σ(weight × distance) / Σ(weight)`

Component estimates exposed as `source0`, `source1`, and `sourceCount` for telemetry.

## World model — **Implemented**, **Tested in simulation**

`VidarWorldModel` motion-corrects tracks using odom delta (translation + rotation) or field-pose reprojection when available.

## Multi-camera — **Implemented**, not **Hardware validated**

Architecturally supports 1–4 cameras. Four simultaneous cameras require USB hub validation on the actual Control Hub — configuration success does not guarantee USB stability.

## Resource budgeting — **Implemented**

`VidarResourceBudget` degrades tag frequency, plates, and secondary cameras based on measured loop CPU.

## Simulator parity — **Tested in simulation**

Browser sim (`sim/`) mirrors Java logic for ROIs, color-blob detection, weighted range fusion, and non-localizing scout observations. Cross-language field names and method contracts are defined in [API.md](API.md). Intentional differences documented in `sim/README-SIM.md`.

## Competition legality

Teams must verify final implementation against the current-season FTC manual. ViDAR does not claim competition readiness without team validation.

Reviewed against **BIOBUZZ Competition Manual V0** (Jul 2026). Nothing in V0 appears to prohibit ViDAR's architecture (UVC webcams + custom Java on the Control Hub). Re-check after kickoff when game rules and expansion limits (R105) are published.

| Topic | BIOBUZZ V0 rule | ViDAR note |
|-------|-----------------|------------|
| On-robot vision | R708 | UVC webcams via Robot Controller app; each camera must be natively supported (validate SVPRO or other models on your hub) |
| USB wiring | R707, R611, R602 | Hub + cameras on USB; hub power from +5V aux or legal USB pack |
| Custom software | R304 | ViDAR TeamCode is allowed |
| Match networking | R704 | No FTC Dashboard or continuous streaming during MATCH play; Driver Station telemetry only |
| Fair play | R202 | Do not put 36h11 AprilTag-like graphics on the robot |
| Expansion / game rules | R105, Section 11 | **TBD at kickoff** (Sep 12, 2026) |

Off-robot tools (browser sim, Python tests, Docker) are for development only and must not run vision during matches.

Manual: [ftc-resources.firstinspires.org/ftc/game/manual](https://ftc-resources.firstinspires.org/ftc/game/manual)
