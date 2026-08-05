# ViDAR Calibration Guide

Calibrate each camera independently. Store results in `VidarCameraProfile.FOUR_SIDES[index]`.

## 1. Camera intrinsic calibration — **Planned** (manual constants today)

Measure or estimate at 640×480:

| Field | Location |
|-------|----------|
| `focalLengthPx` | Horizontal focal length |
| `focalLengthYPx` | Vertical (defaults to horizontal) |
| `principalPointX/Y` | Usually ~320, 240 |
| `horizontalFovDeg` / `verticalFovDeg` | From datasheet or measurement |

OpMode for full checkerboard calibration: **Planned**.

## 2. Camera mount calibration — **Implemented** (manual)

Set per camera in `VidarCameraProfile`:

- `mountXIn`, `mountYIn` — lens position from robot center (+X forward, +Y left)
- `mountYawDeg`, `mountPitchDeg`, `mountRollDeg`
- `bearingDeg` — compass direction camera faces (0=front, 90=right, …)

## 3. ROI and horizon — **Implemented**

Use `VidarRoiCalibrationOpMode` to visualize:

- Ball, plate, and tag ROIs
- Calibrated horizon row in full-frame coordinates
- Rejection stats while tuning HSV

Adjust `VidarCameraRoiConfig` fractions until incorrect mounting is visually obvious.

## 4. Ball-size range calibration — **Implemented**

1. Place ball at known distances (12, 24, 36, 48 in)
2. Record pixel radius at each distance
3. Verify `VidarConfig.BALL_DIAMETER_IN` and `focalLengthPx`
4. Tune HSV until detection is stable

## 5. Floor LUT calibration — **Implemented**

Update `floorCyPx[]` and `floorDistIn[]` in process-frame coordinates:

```java
new double[] {95, 75, 55, 40},  // cy rows
new double[] {12, 24, 36, 48}   // inches
```

Floor LUT is a **secondary** range check for balls and plates — not primary for elevated plates.

## 6. Plate-width range calibration — **Implemented**

Set `plateWidthIn` to measured alliance plate width. Primary range formula:

`distance = plateWidthIn × focalLengthPx / observedPixelWidth`

Validate at multiple distances and viewing angles.

## 7. Multi-camera bearing alignment — **Implemented** (manual)

Ensure each camera's `bearingDeg` matches physical mounting. Overlapping ROIs should show consistent robot-frame positions for the same object — tune until handoff is smooth.

## Validation checklist

- [ ] Ball detected at expected HSV range under venue lighting
- [ ] Range within ±15% at 12–48 in
- [ ] Plate width ranging stable when plate is rotated <30°
- [ ] Tag decode succeeds at expected distances
- [ ] Scout observations appear in telemetry but never shift field pose alone
- [ ] World model tracks stay stable during 90° rotation in place
- [ ] Multi-camera: no USB disconnects over 2-minute sustained run (**Hardware validated** required)

Record results in a team validation log (see `docs/ROADMAP.md` Phase 6).
