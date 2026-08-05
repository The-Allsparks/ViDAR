# ViDAR — Java teaching path

End-to-end Java on the **Control Hub** using the official FTC SDK vision APIs. One language, one device, competition-legal.

## Prerequisites

1. [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) cloned and opening in Android Studio (Ladybug or later).
2. REV Control Hub + **one or more UVC USB webcams** (powered USB hub for 3–4 cameras — see [ROADMAP.md](ROADMAP.md)).
3. Robot configured in Driver Station with:
   - USB cameras named **`Webcam 1`** … **`Webcam 4`** (match `VidarConfig.CAMERA_NAMES`)
   - Set `VidarConfig.CAMERA_COUNT` to the number installed
   - Optional: REV Color Sensor named **`alliance_color`** on your own ROBOT SIGN
   - For TeleOp lesson: drive motors **`left`** and **`right`**

## Install ViDAR into your robot project

Copy the whole package folder into your TeamCode tree:

```
FtcRobotController/
└── TeamCode/
    └── src/main/java/org/firstinspires/ftc/teamcode/
        └── vidar/          ← copy from this repo's teamcode/.../vidar/
            ├── VidarConfig.java
            ├── VidarCameraProfile.java
            ├── VidarGeometry.java
            ├── VidarHoughBallProcessor.java
            ├── VidarBallObservation.java
            ├── VidarPlateProcessor.java
            ├── VidarMultiVision.java
            ├── VidarWorldModel.java
            ├── VidarVision.java
            ├── VidarBlobUtil.java
            ├── VidarDiscoverOpMode.java
            ├── VidarTeleOp.java
            └── VidarAutoSeekOpMode.java
```

Build and deploy to the Control Hub like any other OpMode.

## Lesson plan

### Lesson 1 — `VidarDiscoverOpMode` (no robot required)

**Goal:** See Hough ball detections with fused range on telemetry and Camera Stream.

1. Configure webcam on Control Hub.
2. Select **ViDAR: Discover** on Driver Station → INIT.
3. Open **Camera Stream** — circle overlay shows ball + inches estimate.
4. Calibrate `VidarConfig.BALL_DIAMETER_IN`, `VidarCameraProfile.focalLengthPx`, and floor LUT on the field.
5. Telemetry shows **size** vs **floor** range and **confidence** when they cross-check.

**Concepts:** VisionProcessor, HoughCircles, known-size ranging, floor-row LUT, multi-camera profiles.

### Lesson 2 — `VidarTeleOp` (drive + crude avoidance)

**Goal:** Gamepad tank drive + turn away when a red blob is near center of view.

1. Wire a simple two-motor drive; names must match `VidarConfig`.
2. Run **ViDAR: TeleOp**.
3. Hold a red object in front of the camera — robot should nudge aside.

**Concepts:** Reading `getBestRobot()`, simple proportional avoidance, combining manual + automatic inputs.

### Lesson 3 — `VidarAutoSeekOpMode` (autonomous seek)

**Goal:** Turn toward ball, drive with power scaled by fused range, stop at pickup distance.

Try the same logic in the **browser sim** first — sidebar shows range, size/floor cross-check, and SEARCH / TURN / DRIVE / AT PICKUP states.

### Lesson 4 — Browser simulator (no robot)

**Goal:** Understand overlays before hardware.

1. `.\scripts\serve_sim.ps1` (or `python scripts/serve_sim.py`)
2. Open http://127.0.0.1:8765
3. Start with **Mock scene**, then try **Webcam** with yellow/red objects
4. Compare sim sidebar to Driver Station telemetry from `VidarDiscoverOpMode`

Tune `sim/vidar-tuning.json` alongside `VidarConfig.java` when colors drift.

### Lesson 5 — Extend (student project)

Ideas in order of difficulty:

| Project | What to change |
|---------|----------------|
| Drive toward yellow blob | Use `getBestElement()` center X → `turn` toward target |
| Autonomous grab line | New `@Autonomous` OpMode, no gamepad |
| Custom HSV color | `new ColorRange(ColorSpace.HSV, lower, upper)` in `VidarVision` |
| Second camera | Second `VisionPortal` + second `VidarVision` instance (ports 5555-style not needed — USB on hub) |

## File roles (teach students this map)

```
VidarConfig.java          ← tune camera, Hough, ball diameter, floor LUT
VidarCameraProfile.java   ← per-side bearing + horizon + calibration
VidarHoughBallProcessor   ← Hough + interior check + range overlay
VidarGeometry.java        ← size/floor fusion math
VidarVision.java          ← portal wiring (Hough ball + plate blobs)
VidarDiscoverOpMode       ← read-only test OpMode
VidarAutoSeekOpMode.java  ← range-based autonomous approach
VidarTeleOp.java          ← OpMode that uses vision to affect motors
```

## SDK samples to compare

In Android Studio, also open FTC's built-in samples (same ideas, more comments):

- `ConceptVisionColorLocator_Rectangle`
- `ConceptVisionColorLocator_Circle`

Docs: [FTC Color Processing](https://ftc-docs.firstinspires.org/color-processing/)

## Scaling to 4 cameras

Each USB webcam needs its own `VisionPortal`. Pass a different `VidarCameraProfile` per side:

```java
VidarVision front = new VidarVision(hardwareMap, "Webcam 1", VidarCameraProfile.FRONT);
VidarVision right = new VidarVision(hardwareMap, "Webcam 2", VidarCameraProfile.FOUR_SIDES[1]);
// call update() on each every loop; pick nearest confirmed ball across cameras
```

Calibrate `focalLengthPx` and `floorCyPx`/`floorDistIn` per camera after mounting.

## Python / Docker folder

The `src/vidar/` Python code and Docker setup are optional for off-robot algorithm experiments. **Competition path is this Java package.**
