# ViDAR Java package map

| Class | Role |
|-------|------|
| `VidarSpatial` | Primary integration API — `update()` pins snapshot each loop |
| `VidarRuntime` | Process singleton — world model, fusion, workers, snapshots |
| `VidarVisionAttachment` | Ephemeral FTC cameras (attach/detach per OpMode) |
| `VidarFusionEngine` | Internal multi-camera poll + fuse (not constructed by teams) |
| `VidarConfig` | Hardware / legacy constants (prefer `VidarSettings` + JSON) |
| `VidarSpatialPoint`, `VidarElementObservation`, `VidarPlateObservation` | Public return types |
| `VidarAlliance`, `VidarDistanceUnit`, `VidarElementDetectorType`, `VidarElementShape`, `VidarOffensiveLane` | Enums |
| `VidarGeometry`, `VidarCoordinateFrames` | Range and field/robot transforms |
| `VidarSpatialOpModeBase` | Shared OpMode helpers (alliance init, telemetry) |
| `VidarTeleOp`, `VidarDiscoverOpMode`, `VidarAutoSeekOpMode`, `VidarRoiCalibrationOpMode`, `VidarPedroBridgeSampleOpMode` | Built-in OpModes |

## Subpackages

| Package | Responsibility | Key classes |
|---------|----------------|-------------|
| `vidar.runtime` | Process runtime + camera attachment | `VidarRuntime`, `VidarVisionAttachment`, `VidarObservationWorker`, `VidarLatencyWindow`, `RuntimeBootstrap`, `VidarVision`, `VidarRuntimeConfig` |
| `vidar.api` | Student diagnostics | `VidarDiagnostics` |
| `vidar.integration` | Optional pathing bridges (no hard deps) | `VidarPedroPose`, `VidarPedroPoseBridge`, `VidarPedroCorrectionTracker` |
| `vidar.detect` | Contour / color-blob pipeline | `VidarContourProcessor`, `ElementDetector`, `PlateDetector` |
| `vidar.tag` | AprilTag scout, crop decode, gates | `VidarAdaptiveTagProcessor`, `VidarTagDecodeWorker`, `TagDecodeBudget` |
| `vidar.fusion` | Localization, temporal filter, multi-camera fusion | `VidarFusionEngine`, `VidarLocalizationFusion`, `MultiCameraFusion`, `FieldPoseContext`, `VidarVisionFusion` |
| `vidar.world` | Short-term track memory | `VidarWorldModel`, `VidarTrackAssociator` |
| `vidar.frame` | Per-cycle immutable snapshots | `VidarObservationFrame`, `VidarSpatialSnapshot` |
| `vidar.schedule` | CPU / camera scheduling | `VidarCameraScheduler`, `VidarResourceBudget`, `VidarGlobalVisionWorker` |
| `vidar.model` | Shared DTOs and measurements | `VidarRangeResult`, `VidarTagObservation` |
| `vidar.config` | JSON season / robot specs | `VidarConfigLoader`, `VidarSettings`, bundled `default-*.json` |
| `vidar.geometry` | 3D transforms, ground plane, range fusion | `VidarTransformRegistry`, `VidarGroundPlane`, `VidarRangeFusion` |

## Data flow

```
OpMode → VidarSpatial → VidarRuntime (singleton)
                              ↓
         VidarObservationWorker (background)
                              ↓
         VidarVisionAttachment → VidarVision (runtime) → detect / tag processors
                              ↓
         fusion + VidarSpatialSnapshot + world tracks
```

**Lifecycle:** `VidarSpatial.create()` → `attachVision()`; `spatial.close()` → `detachVision()`; `VidarRuntime.shutdown()` on RC exit.

## Dependency direction

Intended flow (high → low): OpMode / `VidarSpatial` → runtime → fusion/world → geometry/model → config types.

`vidar.integration` is an adapter and may depend only on `fusion`.

`vidar.geometry` and `vidar.world` must not depend on `detect`, `tag`, or `schedule`.

The **current** import graph is frozen in `tests/architecture/allowed_package_edges.json`. CI fails if a new edge appears. Known cycles (frame↔detect, schedule↔runtime/detect, geometry↔fusion/runtime) are documented debt — shrink the freeze, do not grow it.

See [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) for pipeline detail.
