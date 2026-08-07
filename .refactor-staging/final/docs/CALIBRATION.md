# ViDAR Calibration Guide

Calibrate each camera independently. Store results in robot JSON (`config/robots/*.json`) — see [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md) for `robot_T_camera`, intrinsics, and crop mapping.

**Acknowledgement:** Coordinate-frame and calibration architecture was substantially informed by Matt Vitelli’s [*How Robots Understand Space*](https://vivalosmentors.org/wp-content/uploads/2026/08/How_Robots_Understand_Space-Vitelli.pdf) (Viva Los Mentors). Conceptual credit only — see [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md).

## Team camera — SVPRO 8MP USB module

**Model:** [SVPRO SV-USB8MP05AF-FF105](https://www.amazon.com/dp/B0DCF8WW6V) (ASIN B0DCF8WW6V)

| Spec | Value | ViDAR note |
|------|-------|------------|
| Sensor | Sony IMX179, 1/3.2″ CMOS | UVC; native up to 3264×2448 |
| HFOV | ~105° (manufacturer) | Wide **rectilinear**, not fisheye — pinhole + `distortionModel: "none"` is appropriate |
| Focus | Fixed | No autofocus hunting on field |
| Interface | USB 2.0 UVC | Plug-and-play on Control Hub; use a **powered hub** for 3–4 cameras |
| ViDAR stream | **640×480** | `VidarConfig.PORTAL_RESOLUTION` — calibrate intrinsics at this size, not native 8MP |

**Starting intrinsics @ 640×480** (pinhole from 105° HFOV; refine on-field):

```
focalLengthPx ≈ 246     // (640/2) / tan(105°/2)
focalLengthYPx ≈ 246    // ~88° VFOV at 4:3
principalPointX/Y ≈ 320, 240
horizontalFovDeg = 105
verticalFovDeg = 88
distortionModel = "none"
```

Wider FOV than a C920 (~78°): same physical object occupies fewer pixels at a given range — re-run floor LUT and size-based ranging after swapping cameras. Template JSON files include `"floorLutStatus": "example-placeholder-recalibrate-on-robot"` until you measure on-field.

**Robot templates:** [`config/robots/README.md`](../config/robots/README.md) compares SVPRO vs C920 example files.

--- — **Implemented** (manual + validation; checkerboard OpMode **Planned**)

Measure or estimate at calibration resolution (`calibrationWidth` × `calibrationHeight`, default 640×480):

| Field | Location |
|-------|----------|
| `focalLengthPx` | Horizontal focal length |
| `focalLengthYPx` | Vertical (defaults to horizontal) |
| `principalPointX/Y` | Usually ~320, 240 |
| `horizontalFovDeg` / `verticalFovDeg` | From datasheet or measurement |

OpMode for full checkerboard calibration: **Planned**. Pinhole `pixelToRay` / `pointToPixel` and sim axis overlay: **Implemented**, **Tested in simulation**.

## 2. Camera mount calibration — **Implemented** (manual)

Set per camera in `VidarCameraProfile`:

- `mountX`, `mountY` — lens position from robot center (+X forward, +Y left)
- `mountYawDeg`, `mountPitchDeg`, `mountRollDeg`
- `bearingDeg` — compass direction camera faces (0=front, 90=right, …)

## 3. ROI and horizon — **Implemented**

Use `VidarRoiCalibrationOpMode` to visualize:

- Ball, plate, and tag ROIs
- Calibrated horizon row in full-frame coordinates
- Rejection stats while tuning HSV

Adjust `VidarCameraRoiConfig` fractions until incorrect mounting is visually obvious.

## 4. Element-size range calibration — **Implemented**

1. Place element at known distances (12, 24, 36, 48 in)
2. Record pixel radius at each distance
3. Verify `VidarConfig.DEFAULT_ELEMENT_DIAMETER` and `focalLengthPx`
4. Tune HSV until detection is stable

## 5. Floor LUT calibration — **Implemented**

Update `floorCyPx[]` and `floorDist[]` in process-frame coordinates:

```java
new double[] {95, 75, 55, 40},  // cy rows
new double[] {12, 24, 36, 48}   // inches
```

Floor LUT is a **secondary** range check for elements and plates — not primary for elevated plates.

## 6. Plate-width range calibration — **Implemented**

Set `plateWidth` to measured alliance plate width. Primary range formula:

`distance = plateWidth × focalLengthPx / observedPixelWidth`

Validate at multiple distances and viewing angles.

## 7. Multi-camera bearing alignment — **Implemented** (manual)

Ensure each camera's `bearingDeg` matches physical mounting. Overlapping ROIs should show consistent robot-frame positions for the same object — tune until handoff is smooth.

## Validation checklist

- [ ] Element detected at expected HSV range under venue lighting
- [ ] Range within ±15% at 12–48 in
- [ ] Plate width ranging stable when plate is rotated <30°
- [ ] Tag decode succeeds at expected distances
- [ ] Scout observations appear in telemetry but never shift field pose alone
- [ ] World model tracks stay stable during 90° rotation in place
- [ ] Multi-camera: no USB disconnects over 2-minute sustained run (**Hardware validated** required)

Record results in a team validation log (see `docs/ROADMAP.md` Phase 6).
