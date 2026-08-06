# ViDAR Cross-Language API

This document is the **language-neutral contract** for ViDAR's outer layer. Implementations should match these names, field semantics, parameter order, and units so that integration docs, telemetry schemas, and future bindings (Rust, C++, etc.) stay equivalent.

## Naming rules

| Rule | Example |
|------|---------|
| **Canonical field/method names** | camelCase: `distance`, `getBestElement()` |
| **Python implementation** | snake_case fields + camelCase read-only property aliases |
| **JSON config keys** | camelCase (unchanged): `diameter`, `minElementConfidence` |
| **Distance fields** | Plain names (`distance`, `range`, `diameter`); unit from `distanceUnit`. Legacy `*In` and `*Dist` JSON keys still load. `Px`, `Deg`, `Nanos`, `Ms` unchanged. |
| **Robot frame** | +X forward, +Y left, in the active distance unit |
| **Timestamps** | VisionPortal `frameCaptureNanos` (`captureTimeNanos` on observations) — same mailbox path for 1–4 cameras |

Type prefix `Vidar` is used in Java (`VidarRangeResult`). Other languages may drop the prefix but must preserve field names.

---

## Core enums

### `RangeSource` / `VidarRangeEstimate.Source`

| Value | Meaning |
|-------|---------|
| `SIZE` | Known physical diameter → pixel radius |
| `FLOOR` | Floor-row LUT from image Y |
| `GROUND_PLANE` | Mount + intrinsics ray intersect at ball-center height (element diameter / 2) |
| `PLATE_WIDTH` | Known plate width → pixel width |

### `ElementDetectorType` / `VidarElementDetectorType`

| Value | Meaning |
|-------|---------|
| `COLOR_BLOB` | HSV mask + contour geometry |
| `COLOR_BLOB_WITH_LOCAL_HOUGH` | Above + local Hough validation on blob ROI |

### `Alliance` / `VidarAlliance`

`RED`, `BLUE`, (`UNKNOWN` in Java only today)

### `DistanceUnit` / `VidarDistanceUnit`

| Value | JSON | Notes |
|-------|------|-------|
| `IN` | `"in"` | Default; backward compatible with existing configs |
| `M` | `"m"` | **SI** — recommended for new cross-language projects |
| `CM` | `"cm"` | Optional display/calibration unit |

**Rules:**

1. Set `"distanceUnit"` on **season.json** (required semantics for that file's distances).
2. Optional `"distanceUnit"` on **robot.json** overrides season for runtime.
3. All distance fields in JSON (`diameter`, `length`, floor LUT `dist`, mount offsets, …) are expressed in that file's `distanceUnit`.
4. Observation fields (`range`, `robotX`, `dSize`, …) use the **effective** unit: `robot.distanceUnitOverride ?? season.distanceUnit ?? IN`.
5. **Meters** is the SI interchange unit: `VidarUnits.toMeters(value, unit)` / `DistanceUnit.toMeters()` for cross-language export.

```java
VidarDistanceUnit unit = vision.distanceUnit();
double rangeM = VidarUnits.toMeters(obs.range, unit);
telemetry.addLine(VidarUnits.format(obs.range, unit));
```

---

## Range fusion

### `RangeEstimate` / `VidarRangeEstimate`

```
source: RangeSource
distance: float
weight: float           // inverse-variance; 0 = rejected
uncertainty: float
rejectionReason: string | null

isValid(): bool         // weight > 0, distance > 0, no rejectionReason
rejected(source, reason): RangeEstimate
```

### `RangeResult` / `VidarRangeResult`

```
distance: float
uncertainty: float
confidence: float       // 0–1
source0: RangeEstimate | null
source1: RangeEstimate | null
sourceCount: int        // 0–2 valid slots used

isValid(): bool         // distance > 0 && confidence > 0
invalid(): RangeResult
sourceDistance(source: RangeSource): float
sourceWeight(source: RangeSource): float
```

**Fusion** (up to three estimates for elements: SIZE + FLOOR LUT + GROUND_PLANE, no list allocation):

```
fuseRangeWeighted(maxRangeMismatchRatio, ...estimates): RangeResult
fusedDistance = Σ(weight × distance) / Σ(weight)
```

Default `maxRangeMismatchRatio = 0.28`.

---

## Geometry functions

All static in Java `VidarGeometry`; module functions in Python `vidar.geometry`.

| Function | Parameters | Returns |
|----------|------------|---------|
| `distanceFromSize` | `diameter, focalPx, radiusPx` | slant range (in) |
| `distanceFromWidth` | `physicalWidthIn, focalPx, pixelWidth` | slant range (in) |
| `distanceFromFloor` | `cyPx, profile` | range from LUT (in) |
| `buildSizeEstimate` | `dSize, radiusPx, circleFitQuality, partialOcclusion, touchesBoundary` | `RangeEstimate` |
| `buildFloorEstimate` | `dFloor, cyPx, horizonConfidence, nearHorizon` | `RangeEstimate` |
| `distanceFromGroundPlane` | `cx, cy, profile, targetHeightZ` | slant range from mount + intrinsics |
| `buildGroundPlaneEstimate` | `dGround, cyPx, horizonConfidence, nearHorizon` | `RangeEstimate` |
| `buildPlateWidthEstimate` | `dWidth, pixelWidth, rectangularity, whiteRatio, partialVisibility, touchesRoiBoundary, rotationPenalty` | `RangeEstimate` |
| `fuseRangeWeighted` | `maxRangeMismatchRatio?, ...estimates` | `RangeResult` |
| `robotX` | `range, bearingDeg, profile?` | robot X (in) |
| `robotY` | `range, bearingDeg, profile?` | robot Y (in) |
| `floorPointInRobot` | `cx, cy, slantRange, profile` | `(robotX, robotY, robotZ)` |
| `rayDirectionRobotFrame` | `cx, cy, profile` | unit ray `(x, y, z)` |
| `composeElementConfidence` | shape metrics + `rangeResult` + `elementSpec` | 0–1 |
| `composePlateConfidence` | plate metrics + `rangeResult` + `plateSpec` | 0–1 |
| `fuseElementObservation` | detection metrics + `profile` + `cameraName` + `elementSpec` + `seasonConfig` | `ElementObservation` |

Python also exposes **camelCase aliases** (`distanceFromSize = distance_from_size`, etc.).

---

## Coordinate frames and transforms

**Status:** **Implemented**, **Tested in simulation**. See [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md). Distortion: pinhole on-robot (`distortionModel: "none"` default); FTC USB cameras are not fisheye — mild Brown-Conrady at edges is optional future work.

Java package `vidar.geometry`; Python module `vidar.transforms`.

| Type / function | Role |
|-----------------|------|
| `VidarFrameId` / `FrameId` | `FIELD`, `ROBOT`, `CAMERA_OPTICAL` |
| `VidarTransform3D` / `Transform3D` | `destination_T_source`; compose, inverse, point vs direction |
| `VidarCameraIntrinsics` / `CameraIntrinsics` | `fx`, `fy`, `cx`, `cy`, `pixelToRay`, `pointToPixel` |
| `VidarImageTransform` / `ImageTransform` | processed pixel → calibrated sensor pixel |
| `VidarTransformRegistry` / `build_robot_t_camera` | cached `robot_T_camera` from profile |
| `VidarGroundPlane` / `intersect_ground_plane` | floor z=0 intersection |
| `VidarAprilTagTransforms` | documented `field_T_robot` chain |
| `VidarCalibrationDataset` / `calibration_dataset` | offline JSONL schema validation |

Notation: **`robot_T_camera`** maps points from camera optical frame into robot frame.

---

## Observations

### `ElementObservation` / `VidarElementObservation`

Immutable fused game-piece detection.

| Field | Type | Notes |
|-------|------|-------|
| `cameraName` | string | |
| `captureTimeNanos` | int64 | VisionPortal `frameCaptureNanos` at mailbox publish |
| `cx`, `cy` | float | image center, full-frame px |
| `boundingWidthPx`, `boundingHeightPx` | float | axis-aligned box |
| `fittedCx`, `fittedCy`, `radiusPx` | float | circle fit |
| `areaPx` | float | contour area |
| `aspectRatio`, `circularity`, `fillRatio` | float | shape metrics |
| `interiorValidationScore` | float | color interior check |
| `detectorType` | `ElementDetectorType` | |
| `confidence` | float | 0–1 composite |
| `range`, `rangeUncertainty` | float | fused slant range |
| `dSize`, `dFloor` | float | component ranges |
| `rangeResult` | `RangeResult` | full fusion detail |
| `robotX`, `robotY` | float | robot-frame floor point |
| `houghVotes` | int | 0 for color-blob path |

Python extension: `elementId` string (map key in Java is external via `getGameElement(id)`).

### `PlateObservation` / `VidarPlateObservation`

| Field | Type |
|-------|------|
| `alliance` | `Alliance` |
| `cx`, `cy` | float |
| `widthPx`, `heightPx`, `angleDeg` | float |
| `aspectRatio`, `whiteRatio` | float |
| `range`, `rangeUncertainty` | float |
| `sizeBasedRange`, `floorBasedRange` | float |
| `rangeResult` | `RangeResult` |
| `viewingAnglePenalty`, `partialVisibilityPenalty` | float |
| `confidence` | float |
| `robotX`, `robotY` | float |
| `cameraName` | string |
| `captureTimeNanos` | int64 |

Methods: `isFoe(ourAlliance)`, `isAlly(ourAlliance)`.

### `RankedElementFrame` / `VidarRankedElementFrame`

Fixed-capacity ranked list from one camera frame.

```
ranked: ElementObservation[]   // length <= capacity
count: int
overflowCount: int             // scored but not retained
captureTimeNanos: int64
cameraName: string
capacity: int

empty(cameraName, capacity): RankedElementFrame
at(rank): ElementObservation | null
best(): ElementObservation | null   // rank 0
count(), capacity(), overflowCount(), captureTimeNanos(), cameraName()
```

---

## Per-camera processor

### `ContourProcessor` / `VidarContourProcessor`

One scaled ROI pass per frame; loops season `elements[]` and `plates[]`.

**Construction:**

```
ContourProcessor(profile, cameraName, seasonConfig, cameraIndex?, roiScale?, maxRanked?)
```

**Configuration:**

```
setMaxRankedElements(n: int): void
maxRankedElements(): int
```

**Processing:**

```
detect(frame) -> Detection[]          // Python/sim lightweight list
processFrame(frame, captureTimeNanos) // Java VisionProcessor entry
```

**Queries (after process):**

| Method | Returns |
|--------|---------|
| `getBestElement()` | best `ElementObservation` or null |
| `getRankedElements()` | `RankedElementFrame` |
| `getGameElement(elementId)` | best observation for that season element id |
| `getGameElements()` | `Map<string, ElementObservation>` |
| `getBestPlate()` | best `PlateObservation` or null |

---

## Multi-camera fusion (Java reference)

Python/sim implement subsets; full fusion lives in Java today.

### `VidarMultiVision` integration loop

```
vision.recordOdom(odomPose)
frame = vision.update()                    // VidarObservationFrame
corrected = vision.updateCorrected()       // VidarCorrectedFrame

frame.bestElement
frame.bestPlate / bestFoe / bestAlly
frame.bestTag / bestScout
frame.rankedElements
frame.rankedForCamera(index)
```

### `VidarObservationFrame`

| Field | Type |
|-------|------|
| `updateTimeNanos` | int64 |
| `rankedElements` | `RankedElementFrame` |
| `bestElement` | `ElementObservation` |
| `bestPlate`, `bestFoe`, `bestAlly` | `PlateObservation` |
| `bestTag` | `VidarTagObservation` |
| `bestScout` | `VidarTagScoutObservation` |
| `bestScoutResult` | `VidarTagScoutResult` |
| `rankedByCamera` | `RankedElementFrame[]` |
| `tagsByCamera` | `VidarTagObservation[]` |

---

## Config types

JSON files under `config/seasons/` and `config/robots/` use camelCase keys. Loaders:

| Java | Python |
|------|--------|
| `VidarConfigLoader.loadSeason(json)` | `config_loader.load_season(path)` |
| `VidarConfigLoader.loadRobot(json)` | `config_loader.load_robot(path)` |

Core specs: `SeasonConfig`, `ElementSpec`, `PlateSpec`, `AprilTagSpec`, `RobotConfig`, `CameraProfile` — field mapping in [CONFIGURATION.md](CONFIGURATION.md).

---

## Language mapping cheat sheet

| Concept | Java | Python field | Python alias |
|---------|------|--------------|--------------|
| Slant range | `range` | `range` | `range` |
| Fusion result | `VidarRangeResult` | `RangeResult` | same properties |
| Element type | `VidarElementObservation` | `ElementObservation` | camelCase `@property` |
| Processor query | `getBestElement()` | `get_best_element()` | — |
| Geometry fn | `fuseRangeWeighted(...)` | `fuse_range_weighted(...)` | `fuseRangeWeighted` |

When adding a new language, implement this document first, then wire platform-specific I/O (FTC SDK, OpenCV, etc.).
