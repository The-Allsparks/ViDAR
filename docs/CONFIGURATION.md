# ViDAR Configuration Guide

Teams configure ViDAR with **JSON files in team code** (similar to Pedro Pathing robot constants). The library loads them at OpMode init.

## Team JSON files

| File (in `TeamCode/src/main/assets/vidar/`) | Purpose |
|---------------------------------------------|---------|
| `season.json` | Game pieces, field size, AprilTag world map, fusion thresholds |
| `robot.json` | Robot dimensions, camera names, 3D mount pose, floor LUT, ROIs, alliance sensor |

Templates live in the repo:

| File | Season | Game pieces |
|------|--------|-------------|
| `config/seasons/2026-biobuzz.json` | BIOBUZZ 2026-27 | Yellow POLLEN (2.8 in) |
| `config/seasons/2025-decode.json` | DECODE 2025-26 | Purple + green ARTIFACTS (4.9 in) |
| `config/seasons/2024-intothedeep.json` | INTO THE DEEP 2024-25 | Yellow / red / blue SAMPLES (3.5 × 1.5 × 1.5 in prism) |
| `config/seasons/2023-centerstage.json` | CENTERSTAGE 2023-24 | White / purple / yellow / green PIXELS (hex 3 in) |
| `config/seasons/2022-powerplay.json` | POWERPLAY 2022-23 | Red + blue CONES (4 in base) |
| `config/seasons/2021-freightfrenzy.json` | FREIGHT FRENZY 2021-22 | CARGO wiffle balls (2.75 in), BOXES, DUCKS |
| `config/seasons/2020-ultimategoal.json` | ULTIMATE GOAL 2020-21 | Red + blue RINGS (5 in OD) |
| `config/robots/example-robot.json` | 4× SVPRO | Team default — same as `example-robot-svpro-4cam.json` |
| `config/robots/example-robot-svpro-4cam.json` | 4× SVPRO | Explicit four-camera SVPRO template |
| `config/robots/example-robot-svpro-1cam.json` | 1× SVPRO | Bench / first hub validation |
| `config/robots/example-robot-c920-4cam.json` | 4× C920-class | ~70° HFOV reference (fx ≈ 340) |

See [`config/robots/README.md`](../config/robots/README.md) for camera selection, ranging impact, and floor LUT notes.

Copy the file for your active season to `TeamCode/src/main/assets/vidar/season.json`. HSV and filter values in older-season templates are **starting points** — tune on field before competition.

## Bundled defaults (no team JSON required)

When teams omit custom assets, `VidarConfigLoader.defaultSeason()` and `defaultRobot()` load built-in JSON:

| Location | Used by |
|----------|---------|
| `vidar/config/bundled/default-*.json` (Java classpath) | On-robot `VidarConfigLoader` |
| `teamcode/assets/vidar/default-*.json` | Python tests (`tests/test_config_defaults.py`) |

Both copies must stay identical. Regenerate from `VidarConfig` constants:

```bash
python scripts/generate_default_config_assets.py
```

The script writes both paths. Run it after changing defaults in `VidarConfig.java` or `VidarCameraProfile.java`.

## Distance units

ViDAR supports **inches (default)**, **meters (SI)**, and **centimeters** via JSON `"distanceUnit"`.

| Location | Key | Behavior |
|----------|-----|----------|
| `season.json` | `"distanceUnit": "in" \| "m" \| "cm"` | Unit for every distance in that file. Omit = `"in"`. |
| `robot.json` | `"distanceUnit"` (optional) | Overrides season for this robot's runtime observations. |

All distance numbers in a file use plain names (`diameter`, `length`, `mountX`, floor LUT `dist`, …) in that file's unit. Loaders still accept legacy `*In` and `*Dist` keys. Ranging math is unit-agnostic (pinhole formula works in any consistent unit).

**Inches (default, existing templates):**

```json
{
  "distanceUnit": "in",
  "elements": [{ "diameter": 2.8 }]
}
```

**SI meters (recommended for sim / Rust / export):**

```json
{
  "distanceUnit": "m",
  "elements": [{ "diameter": 0.0711 }],
  "field": { "length": 17.556, "width": 8.052 }
}
```

Convert at runtime when needed:

```java
VidarDistanceUnit u = vision.distanceUnit();
double meters = VidarUnits.toMeters(obs.range, u);
telemetry.addData("Range", VidarUnits.format(obs.range, u));
```

Python: `from vidar.units import effective_distance_unit, format_distance, to_meters`.

### Loading in an OpMode

**Recommended** — Pedro-style spatial facade:

```java
VidarAllianceSelector alliance = new VidarAllianceSelector(hardwareMap);
VidarSpatial spatial = VidarSpatial.create(hardwareMap, () -> odomPose, alliance::get);
// each loop:
spatial.update();
VidarSpatialPoint ball = spatial.bestElement();  // robotX, robotY in inches
```

See [CALIBRATION_CHECKLIST.md](CALIBRATION_CHECKLIST.md) before first match. Advanced / direct config access:

```java
VidarSeasonConfig season = VidarTeamConfig.loadSeason(hardwareMap);
VidarRobotConfig robot = VidarTeamConfig.loadRobot(hardwareMap);
// Dimensions and camera layout come from robot JSON — no separate vision constructor.
double lengthIn = robot.dimensions.length;
```

Use `VidarTeamConfig.defaultSeason()` / `defaultRobot()` until assets are copied.

## Legacy Java constants

Element/plate tuning constants in `VidarConfig.java` remain as library defaults and seed `VidarConfigLoader.defaultSeason()`. Prefer JSON for season swaps and per-robot camera layout.

## Primary files

| File | Purpose |
|------|---------|
| `config/VidarConfigLoader.java` | Parse season/robot JSON |
| `config/VidarSeasonConfig.java` | Game-piece definitions |
| `config/VidarRobotConfig.java` | Camera mounts and calibration |
| `VidarConfig.java` | Library defaults, portal/USB settings |
| `VidarTagConfig.java` | AprilTag decode schedule, pose gates |

## Robot frame and dimensions

Detections are reported in **robot frame** on the floor plane:

- **+X** forward (robot front)
- **+Y** left
- **+Z** up (mount height; floor contact points have z ≈ 0)

Define outer body size so downstream code can reason about reach and bounds:

```json
"dimensions": {
  "length": 13.0,
  "width": 13.0,
  "height": 18.0
}
```

Access via `VidarTeamConfig.loadRobot(hardwareMap).dimensions`.

## Camera count and placement

`cameras[]` is **zero-based**. `webcamName` must match Driver Station config (`"Webcam 1"` … `"Webcam 4"`).

Robot JSON splits **intrinsics** from **mounting**:

| Block | Purpose |
|-------|---------|
| `dimensions` | Robot body size (length/width/height in inches) |
| `cameraDefaults` | Shared lens calibration: focal length, floor LUT, ROI, plate width |
| `mountDefaults` | Shared mount orientation defaults (yaw/pitch/roll) |
| `cameras[].camera` | Optional per-camera intrinsics overrides |
| `cameras[].mount` | Per-camera 3D pose (see below) |
| `cameras[].name` | Logical side label (`front`, `right`, …) |

### Mount pose (3D)

Each camera mount is a position plus orientation in robot frame:

| Field | Meaning |
|-------|---------|
| `bearingDeg` | Horizontal look direction: 0 = front, 90 = right, 180 = back, 270 = left |
| `xIn`, `yIn`, `zIn` | Lens position from robot center (+X forward, +Y left, +Z up from floor) |
| `pitchDeg` | Inclination; **negative = looking down** (typical field view) |
| `rollDeg` | Roll about optical axis (bumps / sideways tilt) |
| `yawDeg` | Small twist about optical axis (usually 0) |

Nested form is also accepted:

```json
"mount": {
  "bearingDeg": 0,
  "position": { "x": 6.5, "y": 0, "z": 9.0 },
  "orientationDeg": { "pitch": -12, "roll": 0, "yaw": 0 }
}
```

Slant range from the floor LUT or size-based ranging is applied along the pixel ray through `(cx, cy)`, so mount height and tilt affect `robotX` / `robotY` on fused observations.

ROI bands use season-neutral names — **element** (scoring piece), **plate**, **tag**:

```json
"roi": {
  "element": { "lowerFraction": 0.65, "enabled": true },
  "plate":   { "startFraction": 0.30, "bandFraction": 0.40, "enabled": true },
  "tag":     { "upperFraction": 0.65, "enabled": true }
}
```

Legacy flat `profile` keys on a camera entry still load for backward compatibility.

```json
"cameraCount": 2,
"cameraDefaults": {
  "focalLengthPx": 246,
  "horizontalFovDeg": 105,
  "floorLutStatus": "recalibrate-on-robot",
  "floorLut": [...],
  "roi": { ... }
},
"cameras": [
  { "index": 0, "webcamName": "Webcam 1", "name": "front",
    "mount": { "bearingDeg": 0, "x": 6.5, "y": 0, "z": 9.0, "pitchDeg": -12 } }
]
```

## Element detector and season tuning

Set in `season.json` under `elements[]`: `shape` (`circle`, `rect`, `blob`), `detector`, `hsv`, `diameter`, filters, morphology, and optional local `hough` tuning. Each entry is a scorable game piece (pollen, block, cone, ring, etc.). Fusion threshold: `fusion.minElementConfidence`.

`VidarContourProcessor` runs **one shared pass** per frame: a single scaled ROI + HSV conversion, then a mask/contour loop for every entry in `elements[]` and `plates[]`. DECODE purple and green artifacts are both found in that one pass; use `getGameElement("artifact_purple")` / `getGameElement("artifact_green")` for per-color results.

Options for `detector`: `color_blob`, `color_blob_with_local_hough`.

Tune HSV on field, then update `season.json` (browser sim: `sim/vidar-tuning.json` mirrors the same values).

## AprilTags in world space

Season JSON defines where AprilTags live on the field so ViDAR can build an FTC
`AprilTagLibrary`, compute field-relative robot pose, and gate decode sampling.

**Field frame** (matches FTC SDK): origin at field center, +X right, +Y forward, +Z up.

```json
"field": { "length": 691.2, "width": 317.0 },
"apriltags": {
  "defaultSize": 8.125,
  "tags": [
    {
      "id": 20,
      "name": "blue_goal",
      "size": 8.125,
      "position": { "x": -58.35, "y": -55.63, "z": 29.49 },
      "orientationDeg": { "yaw": 54, "pitch": 0, "roll": 0 },
      "localization": true
    }
  ]
}
```

| Field | Meaning |
|-------|---------|
| `defaultSize` | Black square size when a tag omits `size` |
| `position` | Tag center in inches (optional for decode-only tags) |
| `orientationDeg` | Tag facing on the field (yaw/pitch/roll) |
| `localization` | `false` = motif/targeting only; excluded from pose fusion |

Flat keys (`xIn`, `yawDeg`, …) are also accepted. When `apriltags.tags` is empty,
ViDAR falls back to `AprilTagGameDatabase.getCurrentGameTagLibrary()`.

Access at runtime: `season.aprilTagLibrary()`, `season.tagById(20)`,
`season.useTagForLocalization(tagId)`.

## Per-camera ROIs

Edit `VidarCameraProfile.FOUR_SIDES[i].roiConfig` or build custom `VidarCameraRoiConfig`:

```java
new VidarCameraRoiConfig(
    0.65,  // element lower fraction
    0.30,  // plate start fraction
    0.40,  // plate band height fraction
    0.65,  // tag upper fraction
    true, true, true  // element, plate, tag enabled
);
```

## Pollen color (HSV)

Tune in `season.json` → `elements[0].hsv`. Legacy constants: `VidarConfig.DEFAULT_ELEMENT_HSV_*`.

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

## Do we need a Limelight-style web pipeline UI?

**Short answer: no — not for the same job.** Limelight’s web UI edits *camera pipelines* (thresholds, fiducials, Python) on a coprocessor. ViDAR’s JSON is mostly **season geometry + robot mounts**, which a webcam cannot invent by itself.

| Config piece | Source of truth | Autoconfigure from camera? |
|--------------|-----------------|----------------------------|
| Season elements / plates (size, HSV, shape gates) | Game manual + field lighting | **Partial** — HSV can be assisted from a sample frame; diameter/aspect cannot |
| AprilTag field map | Season / FIRST map | **No** — world coordinates are external |
| Camera `webcamName` / count | Driver Station config | **Yes** — discover connected UVC names at INIT |
| Mount pose (`x/y/z`, pitch, bearing) | Tape measure / CAD | **No** — not in the image without a known target ritual |
| Intrinsics (`focalLengthPx`) / floor LUT | Known-size or known-distance targets | **Assisted** — existing Discover / ROI / floor calibration OpModes |
| ROIs / horizon | Mount geometry + FOV | **Assisted** — `VidarRoiCalibrationOpMode` |

**Recommended path (current):**

1. Copy season + robot templates → `assets/vidar/*.json` (or use bundled defaults).
2. Measure mounts once; tune HSV / floor LUT with **ViDAR: Discover** and ROI calibration OpModes (Driver Station + Camera Stream — not a separate web app).
3. Optional later: a **tuning OpMode that writes JSON** (or a laptop helper that samples frames) — still not Limelight’s pipeline-as-code model.

**What cameras can autoconfigure today / soon:** enumerate webcams and warn on name mismatches; propose focal length from a known-diameter element at a measured distance; refresh floor LUT from marked distances. **What they cannot:** replace `robot.json` mount poses or invent season piece definitions.

## Simulator alignment

`sim/vidar-tuning.json` mirrors Java tuning. Configuration names intentionally align where practical. Sim uses HSV for color-blob; Java default matches.

## Startup validation

`VidarCameraProfile.validate(frameW, frameH)` reports incomplete calibration at startup. Wire into your init sequence or use `VidarRoiCalibrationOpMode`.
