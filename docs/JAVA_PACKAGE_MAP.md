# ViDAR Java package map

Java source lives under `teamcode/org/firstinspires/ftc/teamcode/vidar/`. Copy the entire `vidar/` tree into your FTC SDK `TeamCode` module.

## Stable team imports (`vidar` root)

These stay at `org.firstinspires.ftc.teamcode.vidar` so existing OpModes keep working:

| Class | Role |
|-------|------|
| `VidarSpatial` | Primary integration API |
| `VidarSession` | Instance-scoped orchestrator (vision + world + workers) |
| `VidarMultiVision` | Multi-camera vision coordination |
| `VidarConfig` | Hardware / legacy constants (prefer `VidarSettings` + JSON) |
| `VidarSpatialPoint`, `VidarElementObservation`, `VidarPlateObservation` | Public return types |
| `VidarAlliance`, `VidarDistanceUnit`, `VidarElementDetectorType`, `VidarElementShape`, `VidarOffensiveLane` | Enums |
| `VidarGeometry`, `VidarCoordinateFrames` | Range and field/robot transforms |
| `VidarSpatialOpModeBase` | Shared OpMode helpers (alliance init, telemetry) |
| `VidarTeleOp`, `VidarDiscoverOpMode`, `VidarAutoSeekOpMode`, `VidarRoiCalibrationOpMode` | Built-in OpModes |

## Subpackages

| Package | Responsibility | Key classes |
|---------|----------------|-------------|
| `vidar.api` | Student diagnostics | `VidarDiagnostics` |
| `vidar.detect` | Contour / color-blob pipeline | `VidarContourProcessor`, `ElementDetector`, `PlateDetector` |
| `vidar.tag` | AprilTag scout, crop decode, gates | `VidarAdaptiveTagProcessor`, `VidarTagDecodeWorker`, `TagDecodeBudget` |
| `vidar.fusion` | Localization, temporal filter, multi-camera fusion | `VidarLocalizationFusion`, `MultiCameraFusion`, `FieldPoseContext` |
| `vidar.world` | Short-term track memory | `VidarWorldModel`, `VidarTrackAssociator` |
| `vidar.frame` | Per-cycle immutable snapshots | `VidarObservationFrame`, `VidarSpatialSnapshot` |
| `vidar.schedule` | CPU / camera scheduling | `VidarCameraScheduler`, `VidarResourceBudget` |
| `vidar.model` | Shared DTOs and measurements | `VidarRangeResult`, `VidarTagObservation` |
| `vidar.runtime` | Per-camera vision unit | `VidarVision`, `CameraPipelineConfig` |
| `vidar.config` | JSON season / robot specs | `VidarConfigLoader`, `VidarSettings`, bundled `default-*.json` |
| `vidar.geometry` | 3D transforms, ground plane, range fusion | `VidarTransformRegistry`, `VidarGroundPlane`, `VidarRangeFusion` |

## Data flow

```
OpMode → VidarSpatial → VidarSession → VidarMultiVision → VidarVision (runtime)
                              ↓
         detect / tag processors → frame + spatial snapshots → world tracks
```

See [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) for pipeline detail.
