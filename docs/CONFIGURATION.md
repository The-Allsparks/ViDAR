# ViDAR Configuration Guide

All competition tuning lives in Java constants under `teamcode/org/firstinspires/ftc/teamcode/vidar/`.

## Primary files

| File | Purpose |
|------|---------|
| `VidarConfig.java` | Camera count, ball HSV/Hough, morphology, scheduling, world model |
| `VidarCameraProfile.java` | Per-camera intrinsics, mount, ROIs, floor LUT, plate width |
| `VidarTagConfig.java` | AprilTag decode schedule, pose gates, decimation |

## Camera count

```java
public static final int CAMERA_COUNT = 1;  // 1–4
public static final String[] CAMERA_NAMES = { "Webcam 1", ... };
```

Driver Station webcam names must match. MJPEG is auto-selected for multi-camera (`MJPEG_MULTI_CAMERA`).

## Ball detector selection

```java
public static final VidarBallDetectorType BALL_DETECTOR_TYPE =
        VidarBallDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH;
```

Options: `COLOR_BLOB`, `COLOR_BLOB_WITH_LOCAL_HOUGH`, `LEGACY_HOUGH`.

## Per-camera ROIs

Edit `VidarCameraProfile.FOUR_SIDES[i].roiConfig` or build custom `VidarCameraRoiConfig`:

```java
new VidarCameraRoiConfig(
    0.65,  // ball lower fraction
    0.30,  // plate start fraction
    0.40,  // plate band height fraction
    0.65,  // tag upper fraction
    true, true, true  // enable flags
);
```

## Pollen color (HSV)

Tune in `VidarConfig`: `BALL_HSV_H_MIN/MAX`, `BALL_HSV_S_MIN/MAX`, `BALL_HSV_V_MIN/MAX`.

Browser sim: edit `sim/vidar-tuning.json` element HSV ranges, then copy values to Java.

## Localization pose gates

In `VidarTagConfig`:

| Constant | Default | Purpose |
|----------|---------|---------|
| `MAX_TRANSLATION_RESIDUAL_IN` | 18 | Reject tag if jump too large |
| `MAX_HEADING_RESIDUAL_DEG` | 25 | Reject heading mismatch |
| `MAX_OBSERVATION_AGE_MS` | 500 | Stale tag rejection |
| `MAX_CORRECTION_MAGNITUDE_IN` | 12 | Cap single correction |
| `CORRECTION_COOLDOWN_MS` | 750 | Minimum time between fixes |

Scout observations never pass these gates — they do not localize.

## Simulator alignment

`sim/vidar-tuning.json` mirrors Java tuning. Configuration names intentionally align where practical. Sim uses HSV for color-blob; Java default matches.

## Startup validation

`VidarCameraProfile.validate(frameW, frameH)` reports incomplete calibration at startup. Wire into your init sequence or use `VidarRoiCalibrationOpMode`.
