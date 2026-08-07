# ViDAR Java package map

Java source lives under `teamcode/org/firstinspires/ftc/teamcode/vidar/`. Copy the entire `vidar/` tree into your FTC SDK `TeamCode` module.

## Stable team imports (`vidar` root)

These stay at `org.firstinspires.ftc.teamcode.vidar` so existing OpModes keep working:

| Class | Role |
|-------|------|
| `VidarSpatial` | Primary integration API |
| `VidarMultiVision` | Multi-camera orchestration |
| `VidarConfig` | Hardware / legacy constants |
| `VidarSpatialPoint`, `VidarElementObservation`, `VidarPlateObservation` | Public return types |
| `VidarAlliance`, `VidarDistanceUnit`, `VidarElementDetectorType`, `VidarElementShape`, `VidarOffensiveLane` | Enums |
| `VidarGeometry`, `VidarCoordinateFrames` | Range and field/robot transforms |
| `VidarObservationMapper` | Maps detections to `VidarSpatialPoint` / track DTOs |
| `VidarSpatialOpModeBase` | Shared OpMode helpers (alliance init, telemetry) |
| `VidarTeleOp`, `VidarDiscoverOpMode`, `VidarAutoSeekOpMode`, `VidarRoiCalibrationOpMode` | Built-in OpModes |

## Subpackages

| Package | Responsibility | Key classes |
|---------|----------------|-------------|
| `vidar.detect` | Contour / color-blob pipeline | `VidarContourProcessor`, `VidarBlobUtil` |
| `vidar.tag` | AprilTag scout, crop decode, gates | `VidarAdaptiveTagProcessor`, `VidarTagDecodeWorker` |
| `vidar.fusion` | Localization, temporal filter, motion correction | `VidarLocalizationFusion`, `VidarMotionCorrection` |
| `vidar.world` | Short-term track memory | `VidarWorldModel`, `VidarTrackAssociator` |
| `vidar.frame` | Per-cycle immutable snapshots | `VidarObservationFrame`, `VidarCorrectedFrame` |
| `vidar.schedule` | CPU / camera scheduling | `VidarCameraScheduler`, `VidarResourceBudget` |
| `vidar.model` | Shared DTOs and measurements | `VidarRangeResult`, `VidarTagObservation` |
| `vidar.runtime` | Per-camera vision unit | `VidarVision`, `VidarCameraProfile` |
| `vidar.config` | JSON season / robot specs | `VidarConfigLoader`, `VidarSeasonConfig`, bundled `default-*.json` |
| `vidar.geometry` | 3D transforms, ground plane, range fusion | `VidarTransformRegistry`, `VidarGroundPlane`, `VidarRangeFusion` |

## Data flow

```
OpMode → VidarSpatial → VidarMultiVision → VidarVision (runtime)
                              ↓
         detect / tag processors → frame snapshots → world tracks
```

See [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) for pipeline detail.
