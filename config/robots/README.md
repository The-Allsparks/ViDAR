# Robot configuration examples

Copy **one** file to `TeamCode/src/main/assets/vidar/robot.json` and tune mounts + floor LUT on your physical robot.

All templates stream at **640×480** (`VidarConfig.PORTAL_RESOLUTION`). Intrinsics must match that resolution, not the camera’s native 8MP mode.

## Which file to use

| File | Cameras | Hardware | `focalLengthPx` | HFOV | When to use |
|------|---------|----------|-----------------|------|-------------|
| [`example-robot.json`](example-robot.json) | 4 | **SVPRO 8MP** (team default) | 246 | 105° | Competition robot with four SVPRO modules |
| [`example-robot-svpro-4cam.json`](example-robot-svpro-4cam.json) | 4 | SVPRO 8MP | 246 | 105° | Same as `example-robot.json` (explicit name) |
| [`example-robot-svpro-1cam.json`](example-robot-svpro-1cam.json) | 1 | SVPRO 8MP | 246 | 105° | First Control Hub bring-up, ranging validation |
| [`example-robot-c920-4cam.json`](example-robot-c920-4cam.json) | 4 | Logitech C920 / C270 class | 340 | ~70° | Legacy/reference layout for narrow-FOV USB webcams |

**Amazon (team camera):** [SVPRO 8MP USB module B0DCF8WW6V](https://www.amazon.com/dp/B0DCF8WW6V) — model SV-USB8MP05AF-FF105, Sony IMX179, fixed focus, wide **rectilinear** lens (not fisheye).

## Camera selection impact on ranging

ViDAR fuses up to **three** slant-range sources for game elements:

| Source | Depends on |
|--------|------------|
| **SIZE** | Season `diameter` + `focalLengthPx` + detected pixel radius |
| **FLOOR** | Per-row **floor LUT** (`floorLut`) — must be measured on-robot |
| **GROUND_PLANE** | `focalLengthPx`, mount pose (`mountX/Y/Z`, pitch), pixel `(cx, cy)` |

Plates fuse **width + floor LUT + ground plane (z = 0)**.

Wrong `focalLengthPx` skews SIZE (e.g. C920 defaults on SVPRO ≈ **38% error**). Wrong or placeholder floor LUT makes FLOOR disagree with SIZE/GEOMETRY and **lowers fusion confidence** — that is expected until you calibrate.

## Required after copying a template

See **[docs/CALIBRATION_CHECKLIST.md](../../docs/CALIBRATION_CHECKLIST.md)** for the one-page setup list. Summary:

1. Match `webcamName` to Driver Station camera names (`Webcam 1` … `Webcam 4`).
2. Set `cameraCount` to the number of physical cameras.
3. **Re-run floor LUT** at 12 / 24 / 36 / 48 in (or your check distances) — templates mark `"floorLutStatus": "example-placeholder-recalibrate-on-robot"`.
4. Verify mount offsets and pitch with `VidarRoiCalibrationOpMode` + sim **Calibration axes** overlay.
5. Use **ViDAR: Discover** telemetry: `size=`, `floor=`, `ground=` on element detail line.

## Library defaults vs robot JSON

If a field is omitted from JSON, loaders fall back to **C920-class** defaults (`focalLengthPx = 340`, tag scout FOV 70°). Always ship a complete `robot.json` for your actual camera model — do not rely on code defaults on the hub.
