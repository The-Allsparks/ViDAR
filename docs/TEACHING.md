# ViDAR Java teaching path

End-to-end Java on the **Control Hub** using the official FTC SDK vision APIs. The goal is **robot-space awareness**: calibrated range, bearing, and tracks your OpMode can use, not raw pixels. One language, one device, competition-legal.

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
            ├── VidarContourProcessor.java
            ├── VidarElementObservation.java
            ├── VidarMultiVision.java
            ├── VidarWorldModel.java
            ├── VidarVision.java
            ├── VidarDiscoverOpMode.java
            ├── VidarTeleOp.java
            └── VidarAutoSeekOpMode.java
```

Build and deploy to the Control Hub like any other OpMode.

## Lesson plan

### Lesson 1 — `VidarDiscoverOpMode` (no robot required)

**Goal:** See element detections with fused range on telemetry and Camera Stream.

1. Configure webcam on Control Hub.
2. Select **ViDAR: Discover** on Driver Station → INIT.
3. Open **Camera Stream** — circle overlay shows game element + inches estimate.
4. Calibrate season JSON / `VidarConfig` (element diameter, HSV, floor LUT) and `VidarCameraProfile.focalLengthPx` on the field.
5. Telemetry shows **size** vs **floor** range and **confidence** when they cross-check.

**Concepts:** VisionProcessor, color contour + `minEnclosingCircle`, known-size ranging, floor-row LUT, multi-camera profiles.

### Lesson 2 — `VidarTeleOp` (drive + crude avoidance)

**Goal:** Gamepad tank drive + turn away when a detected element is near center of view.

1. Wire a simple two-motor drive; names must match `VidarConfig`.
2. Run **ViDAR: TeleOp**.
3. Hold a colored object in front of the camera — robot should nudge aside.

**Concepts:** Reading `getBestElement()` / alliance plates, simple proportional avoidance, combining manual + automatic inputs.

### Lesson 3 — `VidarAutoSeekOpMode` (autonomous seek)

**Goal:** Turn toward element, drive with power scaled by fused range, stop at pickup distance.

Try the same logic in the **browser sim** first — sidebar shows range, size/floor cross-check, and SEARCH / TURN / DRIVE / AT PICKUP states.

### Lesson 4 — Browser simulator (no robot)

**Goal:** Understand overlays before hardware.

1. `.\scripts\serve_sim.ps1` (or `python scripts/serve_sim.py`)
2. Open http://127.0.0.1:8765
3. Start with **Mock scene**, then try **Webcam** with season-colored objects
4. Compare sim sidebar to Driver Station telemetry from `VidarDiscoverOpMode`

Tune `sim/vidar-tuning.json` alongside season config / `VidarConfig.java` when colors drift.

### Lesson 5 — Extend (student project)

Ideas in order of difficulty:

| Project | What to change |
|---------|----------------|
| Drive toward a specific element | Use `getBestElement()` center X → `turn` toward target |
| Autonomous grab line | New `@Autonomous` OpMode, no gamepad |
| Custom HSV color | Add or edit an entry in `config/seasons/*.json` |
| Second camera | Second `VisionPortal` + second `VidarVision` instance (ports 5555-style not needed — USB on hub) |

## File roles (teach students this map)

```
VidarConfig.java / config/     ← tune camera, season elements, floor LUT
VidarCameraProfile.java      ← per-side bearing + horizon + calibration
VidarContourProcessor        ← unified element + plate detection + range overlay
VidarGeometry.java           ← size/floor fusion math (VidarRangeResult)
VidarVision.java             ← portal wiring (contour + tag processors)
VidarDiscoverOpMode          ← read-only test OpMode
VidarAutoSeekOpMode.java     ← range-based autonomous approach
VidarTeleOp.java             ← OpMode that uses vision to affect motors
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
```

See [ROADMAP.md](ROADMAP.md) for USB hub wiring and validation checklist.
