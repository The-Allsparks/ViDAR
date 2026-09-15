# ViDAR: Robot-Space Situational Awareness for FIRST Tech Challenge

**Technical white paper**


|              |                                                                                                   |
| ------------ | ------------------------------------------------------------------------------------------------- |
| **Project**  | [ViDAR](https://github.com/The-Allsparks/ViDAR)                                                   |
| **Authors**  | The Allsparks (FTC Team **#36117**), Las Vegas, Nevada                                            |
| **Version**  | **0.2.0** (`VidarVersion.SEMVER`)                                                                 |
| **Date**     | 14 September 2026                                                                                 |
| **Audience** | Students, mentors, other FTC teams, and design reviewers                                          |
| **License**  | MIT (software). This paper describes the public library; it is not an official FIRST publication. |


**Disclaimer.** ViDAR is community-developed and unofficial. It is **not** affiliated with or endorsed by FIRST, REV Robotics, Pedro Pathing, or other referenced vendors. Teams must verify legality and performance against the current-season FTC Game Manual. Matt Vitelli did not author ViDAR code, endorse ViDAR, or license code to this project; see [§12](#12-related-work-and-acknowledgements).

---



## Abstract

FTC robots see the field through USB webcams, but **pixels are not a plan**. A blob centroid, a Camera Stream overlay, or a once-per-second AprilTag decode does not tell an OpMode where a game piece sits relative to the intake, whether an opponent plate is in the way, or whether a remembered track is still trustworthy.

**ViDAR** (Vision Detection And Ranging) is a Control Hub library that converts 1–4 UVC webcam streams into **timestamped observations in robot space**: calibrated range, bearing, and short-term tracks for season elements, alliance plates, and sparse AprilTags. Vision is the sensor. Spatial understanding is the product.

The library is **passive**. It never commands motors. Pathing (Pedro Pathing, Road Runner, or a hand-written auto) and strategy (TeamCode, HELM) are optional consumers. Perception lives in a process singleton with attach/detach VisionPortals so Auto → TeleOp does not rebuild the world model. AprilTag scout hits never rewrite pose; official decode is crop-only and globally budgeted. Range fusion treats **ground-plane geometry as authoritative** when the ray is valid; size and floor-LUT estimates raise or lower confidence rather than averaging away from physics.

This paper states the problem ViDAR is built to solve, the architectural contracts that keep it teachable on Android 7 Control Hubs, what is implemented as of 0.2.0, and what is **not** yet field-proven. Four-camera USB stress and Control Hub loop-time rows remain **unmeasured**. Do not treat desktop simulation FPS as match readiness.

---



## 1. Introduction

FIRST Tech Challenge robots run custom Java on a REV Control Hub: a single Android 7 computer that also owns drivetrain control, USB cameras, and Driver Station telemetry. There is no coprocessor in the legal competition path (Game Manual R708). Teams who want vision therefore share CPU, USB bandwidth, and GC pauses with the rest of the match.

Common FTC vision patterns fail in predictable ways:


| Pattern                        | What it gives you        | What it hides                                   |
| ------------------------------ | ------------------------ | ----------------------------------------------- |
| Color blob + centroid          | A pixel in an image      | Frame, range, capture time, which camera        |
| Full-frame AprilTag every loop | Occasional field pose    | USB + CPU cost; scout false positives as “pose” |
| One webcam, hard-coded HSV     | A demo on the shop floor | Lighting, mount, multi-cam handoff              |
| Vision that `setPower`s        | A “smart” OpMode         | Untestable coupling; cannot reuse across autos  |


ViDAR exists so The Allsparks — and any team that copies the library — can treat cameras like other sensors: calibrated, framed, timestamped, and **read-only** from the OpMode loop.

The competition path is **Java TeamCode** on FTC SDK **v12.0**. A browser simulator and Python tests exist so students can learn geometry and ROIs on a laptop; they do not run during matches.

---



## 2. Problem statement



### 2.1 What an OpMode actually needs

A driver or autonomous routine needs answers in **robot frame** (+X forward, +Y left, +Z up):

- Where is the nearest season element, in inches (or the configured distance unit), and how old is that observation?
- Is a foe plate in the intake cone?
- Which forward lane is least occupied?
- When a camera is briefly occluded, did we **remember** a track or invent one from a stale blob?

Those questions require **range**, **bearing**, **identity** (element vs plate vs tag), **uncertainty**, and **time**. They do not require ViDAR to own field pose or to follow a path.

### 2.2 Constraints unique to FTC

1. **One computer.** Vision, fusion, and drive share the Hub. Unbounded OpenCV or full-frame AprilTag decode will starve the control loop.
2. **USB is the capture tax.** Portals run **1280×720 MJPEG**. The Hub pays for every pixel on the wire even if AprilTag only decodes a crop.
3. **Match networking (R704).** FTC Dashboard and similar streams are illegal during MATCH play. Telemetry and Camera Stream at INIT are the legal windows.
4. **OpMode lifecycle.** Auto STOP must release VisionPortals; TeleOp INIT must re-attach without discarding useful world state.
5. **Season rules change.** Element colors, plate geometry, and AprilTag layouts belong in JSON — not `if (seasonId == "2026-biobuzz")` inside detect.



### 2.3 What ViDAR refuses to be

- A drivetrain.
- A localization system of record. Canonical fused pose stays with odometry / Pinpoint / Pedro / TeamCode.
- A season-specific BIOBUZZ interpreter. Physical measurements (range, plate color, tag id when decoded) are ViDAR; nectar ownership and auto strategy are TeamCode.
- A claim of four-camera match readiness without Hub rows in [validation-log.md](validation-log.md).

---



## 3. Design principles

These rules are load-bearing. Implementation that violates them should be rejected, not “tuned around.”

1. **Vision is a sensor; space is the product.** Every published observation names a frame, a unit, and a capture timestamp (`frameCaptureNanos` → `captureTimeNanos`).
2. **Passive.** No `DcMotor`, no `Servo`, no path follower inside `vidar.`*. Built-in OpModes are telemetry and overlays only.
3. **Snapshot-first.** Perception advances on background workers. The OpMode calls `VidarSpatial.update()` once per loop and reads an immutable snapshot. The robot loop must not run OpenCV.
4. **Runtime vs attachment.** `VidarRuntime` is a process singleton (world model, fusion, workers). `VidarVisionAttachment` owns VisionPortals for one OpMode. `close()` detaches cameras; it does not destroy memory.
5. **Geometry over heuristics.** When a ground-plane ray intersection is valid, fused slant range **is** that distance. Size and floor LUT cross-check confidence or fill in when geometry is rejected. They do not pull range toward a midpoint.
6. **Scout is not pose.** AprilTag *scout* observations never alter absolute pose. Decode is crop-only and globally rate-limited (1 Hz in 0.2.0).
7. **Capture-time pose.** Projection and fusion must use odometry at `captureTimeNanos`, not the newest pose on the worker tick.
8. **Stale is a first-class state.** World association clocks on **new vision capture times**, not a 1 ms worker metronome. Replaying the last blob must not keep a track “alive.” High `missCount` or old `lastSeenNanos` means the memory is stale.
9. **Sim first, Hub honest.** Browser sim and `java-pure` tests freeze contracts. Desktop FPS is not Control Hub evidence.
10. **Season JSON is data.** Detection code stays generic. Copy-out TeamCode samples may interpret a season; the library must not auto-load them.

---



## 4. System architecture



### 4.1 Two lifetimes

```
Robot Controller process
├── VidarRuntime          (singleton: world, fusion, tag worker, observation worker, snapshots)
└── OpMode
    └── VidarVisionAttachment   (VisionPortal × N, processors, mailboxes)
            ↑ attach / detach per OpMode
```

`VidarSpatial.create(...)` attaches vision and rebinds odometry / alliance suppliers so Auto and TeleOp may pass different callbacks. `spatial.close()` detaches portals. Optional `VidarRuntime.shutdown()` tears down the singleton at RC exit.

### 4.2 Threads


| Thread                           | Allowed to do                              | Must not                                        |
| -------------------------------- | ------------------------------------------ | ----------------------------------------------- |
| VisionPortal callback            | Latest-wins mailbox publish (`copyTo`)     | Fusion, world update, `setPower`                |
| Global vision worker (multi-cam) | Round-robin contour + tag scout            | Own field pose                                  |
| Tag decode worker                | Official FTC AprilTag on a **crop**, ≤ 1/s | Full 720p decode every frame                    |
| Observation worker               | Poll, fuse, world update, publish snapshot | Block the OpMode loop                           |
| OpMode `loop()`                  | `update()` + read snapshot / diagnostics   | Construct `VisionPortal.Builder`, sleep, OpenCV |


Tick failures on the observation worker are recorded (`VidarDiagnostics`) and the worker **stays alive**. Snapshots may go stale; cameras are not restart-looped from a thrown tick.

### 4.3 Package map (responsibility, not a tour)


| Package             | Owns                                      | Forbidden                        |
| ------------------- | ----------------------------------------- | -------------------------------- |
| `vidar` (root)      | `VidarSpatial`, enums, teaching OpModes   | Hardware threads, OpenCV loops   |
| `vidar.runtime`     | Singleton, attach/detach, workers         | Pathing / motors                 |
| `vidar.detect`      | Contour / plate OpenCV                    | Field pose ownership             |
| `vidar.tag`         | Scout + budgeted decode                   | Pose from scout                  |
| `vidar.fusion`      | Multi-camera pick, localization **gates** | Direct VisionPortal construction |
| `vidar.world`       | Short-term tracks, TTL, association       | HardwareMap / OpenCV             |
| `vidar.geometry`    | Transforms, ground plane, range fusion    | Detect/tag implementations       |
| `vidar.frame`       | Immutable snapshots                       | Advancing perception             |
| `vidar.schedule`    | Camera CPU / stream policy                | Student API                      |
| `vidar.config`      | Season / robot JSON                       | Algorithms                       |
| `vidar.integration` | Optional Pedro adapters                   | Maven dependency on Pedro        |


CI freezes allowed import edges (`tests/architecture/allowed_package_edges.json`). New package edges fail the build.

### 4.4 Student API (the only loop most teams need)

```java
VidarSpatial vidar = VidarSpatial.create(hardwareMap, odom::getPose, alliance::get);
try {
    waitForStart();
    while (opModeIsActive()) {
        vidar.update();
        VidarSpatialSnapshot snap = vidar.snapshot();
        // drive from snap — never from live OpenCV
    }
} finally {
    vidar.close();
}
```

Published groups: `elements()`, `allies()`, `foes()`, plus pose accessors, `intakeBlocked()`, `recommendOffensiveLane()`, and `diagnostics()`. ViDAR does not bundle a pathing library.

---



## 5. Perception



### 5.1 One scaled pass, overlapping ROIs

`VidarContourProcessor` runs **one** downscaled ROI pass per camera for season `elements[]` and `plates[]` — not full-frame Hough.

Default overlapping bands (per camera, process coordinates):


| ROI       | Default               | Role                      |
| --------- | --------------------- | ------------------------- |
| Elements  | Lower 65%             | Floor-contact game pieces |
| Plates    | Middle 40% (from 30%) | Alliance signs            |
| AprilTags | Upper 65%             | Scout + crop decode       |


Horizon is per-camera (`horizonRowPx`). Students calibrate with **ViDAR ROI Calibrate**, not by guessing pixel rows in Java.

### 5.2 Elements

Pipeline: HSV mask → morphology → contour geometry → `minEnclosingCircle` → interior color check → optional **local** Hough on the blob ROI (`COLOR_BLOB` vs `COLOR_BLOB_WITH_LOCAL_HOUGH`). Physical diameter in season JSON feeds size-based range.

### 5.3 Plates (friend / foe)

Pipeline: color mask → `minAreaRect` → **white-digit ratio** gate (reject solid colored tape) → width-based primary range. Alliance is **runtime**, not a match-constant:

- REV Color Sensor on the team’s own robot sign (`alliance_color`), and/or
- Gamepad **Y** = RED, **B** = BLUE at INIT (optional Back toggle in match).

`PlateObservation.isFoe(ourAlliance)` / `isAlly(ourAlliance)` keep friend/foe out of HSV.

### 5.4 Camera scheduling

Cameras stay streaming while processors disable (PRIMARY / SECONDARY / IDLE). Stream stop is delayed and debounced (**DEEP_IDLE** after 2.5 s). `VidarResourceBudget` can shed tag frequency, plates, and secondary cameras from measured loop CPU — **metrics-driven degradation**, not a second “vision mode” API.

### 5.5 What 0.2.0 does not detect yet

There is **no** `vidar.fixture` package. Loaders do not yet parse `fixtures[]`. Season JSON now cites BIOBUZZ HIVE tag IDs and FLOWER geometry ([#77](https://github.com/The-Allsparks/ViDAR/issues/77)); fixture **state** and fixture-assisted localization remain gated on ([#75](https://github.com/The-Allsparks/ViDAR/issues/75), [#96](https://github.com/The-Allsparks/ViDAR/issues/96)) and are **out of scope for this version**. This paper does not treat that backlog as shipped.

---



## 6. Geometry and ranging



### 6.1 Frames

A pixel is not a robot position. ViDAR names frames explicitly ([COORDINATE_FRAMES.md](COORDINATE_FRAMES.md)):


| Frame              | Convention                                                                           |
| ------------------ | ------------------------------------------------------------------------------------ |
| **FIELD**          | FTC / SDK: origin field center, +X right, +Y forward, +Z up                          |
| **ROBOT**          | +X forward, +Y left, +Z up from the floor                                            |
| **CAMERA_OPTICAL** | OpenCV: +X image-right, +Y image-down, +Z into the scene                             |
| **IMAGE**          | Full-frame vs process-frame pixels; `VidarImageTransform` maps before `pixelToRay()` |


Transforms use `destination_T_source`: `p_robot = robot_T_camera * p_camera`. Points use rotation and translation; rays use rotation only. Mounts are configured once in `robot.json` (`mount.x/y/z`, `bearingDeg`, `pitchDeg`, `rollDeg`, `yawDeg`). `VidarTransformRegistry` caches `robot_T_cameraOptical` at init.

On-robot intrinsics are **pinhole** (`distortionModel: "none"`). Typical FTC USB cameras are not fisheye. Brown-Conrady at the edges is optional future / offline work, not a Hub hot path.

### 6.2 Three range estimates, one fused distance

For elements, up to three slant-range estimates:


| Source           | Meaning                                                               |
| ---------------- | --------------------------------------------------------------------- |
| **SIZE**         | Known diameter and fitted pixel radius                                |
| **FLOOR**        | Image-row lookup table (field calibration)                            |
| **GROUND_PLANE** | Mount + intrinsics ray ∩ plane at ball-center height (`diameter / 2`) |


Plates use **PLATE_WIDTH** + FLOOR + ground at **z = 0**.

**Fusion rule:** if GROUND_PLANE is valid, fused `distance` **equals** the ground-plane slant range. Heuristics that agree raise `confidence`; heuristics that disagree lower it. Without a valid ground plane, inverse-variance weighting among remaining estimates (legacy path). Default mismatch gate: `maxRangeMismatchRatio = 0.28`.

Telemetry on **ViDAR: Discover** exposes `size=` / `floor=` / `ground=` so students can see the argument, not only the winner.

### 6.3 Why this matters on the field

Size ranging fails under partial occlusion and at the ROI boundary. Floor LUT fails when the horizon is wrong or the piece is not on the floor. Ground plane fails on parallel / skyward rays and bad extrinsics. Publishing all three — with rejection reasons — is how a team debugs calibration instead of “turning HSV until it works.”

---



## 7. Multi-camera fusion and the world model



### 7.1 Fusion

`VidarFusionEngine` / `MultiCameraFusion` pick a **global** best element, plate, and tag each observation tick from 1–4 cameras. Per-camera `VidarCameraProfile` (bearing 0° / 90° / 180° / 270° plus mount offsets) makes “which camera saw it” a geometry problem, not a naming accident. Driver Station webcam names must be `Webcam 1` **…** `Webcam 4`.

Architecturally this is implemented. **Four simultaneous UVC cameras on a Control Hub are not hardware-validated.** Configuration success ≠ USB stability.

### 7.2 World model

`VidarWorldModel` is short-term spatial memory owned by the runtime:

- Tracks: elements, allies, foes, with TTL.
- Association: predict (`pos + velocity × dt`) → gate in robot frame → EMA velocity.
- Clock: **observation capture time**, not the ~1 ms worker tick.
- Detach: `update(null, now)` coasts so tracks age when cameras are gone; after `WORLD_*_TTL_SEC` they prune and `intakeBlocked()` goes false.

Motion-corrected tracks require an odometry supplier and `WORLD_MOTION_TRACKING_ENABLED`. Without odom, queries return live detections only. Elements can classify `STATIC` after low velocity; plates `MOVING` above a speed threshold.

**Teaching rule:** covering the lens is a valid failure injection. Stale tracks must not be treated as current field truth.

---



## 8. AprilTags and the localization boundary



### 8.1 Two products that must not be confused


| Product                                                           | Rate                                  | May change field pose?                        |
| ----------------------------------------------------------------- | ------------------------------------- | --------------------------------------------- |
| **Scout** (`VidarTagScoutRunner`)                                 | Every processed frame (ROI)           | **Never**                                     |
| **Decode** (`VidarTagCropDecoder` + official `AprilTagProcessor`) | ≤ **1 Hz** global (`TagDecodeBudget`) | Only if localization **gates** accept the fix |


USB still captures 1280×720. Decode CPU pays for an upper-band crop, not a second full-resolution pipeline.

Decoded tags supply `field_T_robot` at **capture** time (SDK pose). Documented chain:

```
field_T_robot = field_T_cameraOptical * cameraOptical_T_robot
```

ViDAR uses the SDK pose on `VidarTagObservation.fieldPoseAtCapture`. `VidarLocalizationFusion.wouldScoutAlterPose()` is required to stay false.

### 8.2 Optional Pedro (and any other auto)

`vidar.integration` offers `VidarPedroPoseBridge` and `VidarPedroCorrectionTracker` with **no** Pedro Maven dependency. Typical split:

- Continuous localizer (Pinpoint / odom / IMU) drives path following.
- Sparse ViDAR tag fixes, gated on correction event id, re-propagate to current odom when **TeamCode** calls `follower.setPose`.
- Assisted TeleOp reads `elements()` / `foes()` for slow / abort / lane hints.

Do not run full AprilTag decode inside a pathing control loop. Do not treat ViDAR as the pose source of record.

### 8.3 BIOBUZZ note (season context, not a feature)

Kickoff-era BIOBUZZ tags can **move**. Season JSON lists HIVE IDs 30-45 with `localization: false` so they are **not** static SDK landmarks. Planned localization work must not rely on SDK `robotPose` metadata. Until Hub validation exists, AprilTag localization is a **gated optional consumer**, not a promised match behavior.

---



## 9. Control Hub constraints



### 9.1 Performance contract

ViDAR’s product metric is **predictable Hub loop time**, not desktop throughput. A recorded desktop bench (2026-08-22) showed hundreds of element FPS on a Windows JVM; that number is **explicitly not Hub**.

Targets (for field logs, not CI gates):


| Metric                        | Target                                                           |
| ----------------------------- | ---------------------------------------------------------------- |
| Element process rate / camera | ≥ 15 FPS                                                         |
| Tag decode when it runs       | < 400 ms                                                         |
| Observation tick              | Bound fusion + world; watch p50/p95/max via `VidarLatencyWindow` |


CI may only apply **generous** JVM ceilings (algorithmic catastrophe). It cannot see USB drops, GC pauses, VisionPortal jitter, or Driver Station `String.format` cost.

Hot-path rules: no `Thread.sleep` in OpModes, no filesystem/network on workers, latest-wins mailboxes, pooled Mats, no `VisionPortal.Builder` outside attach.

### 9.2 USB wiring (legal envelope)

Reviewed against **BIOBUZZ Competition Manual V0** (July 2026); **re-check Team Updates** after kickoff. Nothing in V0 appeared to prohibit UVC webcams + custom Java. Expansion limits (R105) and game-specific vision rules can still change.


| Rule (V0)          | ViDAR implication                                                   |
| ------------------ | ------------------------------------------------------------------- |
| R708               | On-robot UVC via Robot Controller; natively supported cameras only  |
| R707 / R611 / R602 | Powered USB hub from +5 V aux (2 A/port) or legal USB pack ≤ 100 Wh |
| R304               | Custom TeamCode allowed                                             |
| R704               | No Dashboard / continuous streaming during MATCH                    |
| R202               | Do not put 36h11-like graphics on the robot                         |


Preferred Hub layout: powered **USB 2.0 hub on the USB 3.0 port**, hub power from REV +5 V aux. Four 720p MJPEG streams are **not yet Hub-measured**. If streams drop, reduce camera count at 720p rather than silently dropping a useful side to 480p.

Off-robot tools (sim, pytest, Docker) must not run vision during matches.

---



## 10. Integration, teaching, and the rest of the stack



### 10.1 Teaching path (no motors)


| Lesson | OpMode / tool                 | Idea                                         |
| ------ | ----------------------------- | -------------------------------------------- |
| 1      | **ViDAR: Discover**           | Fused range on telemetry + Camera Stream     |
| 2      | **ViDAR: Spatial**            | Pose, nearest element/foe, `intakeBlocked()` |
| 3      | **ViDAR: Spatial Map**        | Full remembered lists; stale vs live         |
| 4      | Browser sim (`serve_sim.ps1`) | ROIs and colors before USB                   |
| 5      | Student project               | Map `snapshot()` onto *their* drivetrain     |


FORGE treats ViDAR as **optional**. Preseason: sim and one-camera Discover. Multi-camera fusion is not an early-season build goal until one camera is boringly reliable.

### 10.2 Sibling libraries

ViDAR publishes physical observations. Other Allsparks libraries consume them without taking ownership of OpenCV:


| Consumer             | Role                                                                                                           |
| -------------------- | -------------------------------------------------------------------------------------------------------------- |
| **TeamCode / Pedro** | Drive and auto; optional pose correction tracker                                                               |
| **ECHO**             | Sonification of a **single** selected target (`vidar-echo.v0`); adapter-side frame conversion; feature-flagged |
| **HELM**             | Strategy / fixture *meaning* (planned; not ViDAR)                                                              |
| **TRACE**            | Observe-only logging when adapters exist                                                                       |


ECHO will not scan ViDAR’s full track list. HELM or the driver chooses `targetId`. Missing ViDAR is silence, not a crash.

### 10.3 Install

Versioned **source-copy** into TeamCode (see [INSTALL.md](INSTALL.md)): the `vidar/` tree, `assets/vidar/` JSON, robot template, SDK pin **v12.0**. Hub-visible version is `VidarVersion.SEMVER` (`0.2.0`), not a Maven coordinate.

---



## 11. Status and evidence (0.2.0)

Honesty is part of the architecture. Labels mean what [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) says they mean.


| Capability                                        | Status                                                                              |
| ------------------------------------------------- | ----------------------------------------------------------------------------------- |
| Unified element + plate contour pipeline          | Implemented; **tested in simulation**                                               |
| Per-camera overlapping ROIs + alliance selector   | Implemented; **tested in simulation**                                               |
| Explicit frames, transforms, ground-plane ranging | Implemented; **tested in simulation**; on-robot compile; **not hardware-validated** |
| Multi-camera fusion + world model                 | Implemented; **not hardware-validated at 4 cameras**                                |
| `VidarSpatial` snapshot facade                    | Implemented; motion tracking needs **field** validation                             |
| AprilTag scout + async crop decode                | Implemented; **tested in simulation**                                               |
| Runtime / attachment split, Auto→TeleOp rebind    | Implemented; **JVM session tests**; not Hub USB proof                               |
| Browser sim + Python parity + `java-pure` CI      | Available                                                                           |
| Packaging / `VidarVersion` / INSTALL + LIFECYCLE  | Done                                                                                |
| Control Hub 1–4 camera USB + Tick-ms log          | **Open** — Hub table empty                                                          |
| Fixture state / fixture-assisted localization     | **Not implemented** (design review)                                                 |


**Evidence that counts:** Camera Stream overlays, Discover telemetry (`size`/`floor`/`ground`, tick percentiles), calibration checklist photos, `java-pure` tests, architecture guards, dated [validation-log.md](validation-log.md) rows with firmware and camera count.

**Evidence that does not count:** desktop element FPS, green CI, or “it worked once in the shop” without a log row.

---



## 12. Related work and acknowledgements

ViDAR’s coordinate-frame and calibration architecture was **substantially informed** by Matt Vitelli’s presentation *[How Robots Understand Space](https://vivalosmentors.org/wp-content/uploads/2026/08/How_Robots_Understand_Space-Vitelli.pdf)* (Viva Los Mentors, accessed 2026-08-05): explicit frames, `destination_T_source` chains, practical intrinsic/extrinsic calibration, visualization-first validation, and offline pose refinement — adapted here for FTC USB cameras on a Control Hub.

**Conceptual credit only.** Vitelli did not author, endorse, or license ViDAR.

FTC SDK VisionPortal / EasyOpenCV / `AprilTagProcessor` are the on-robot I/O. Pedro Pathing is an optional consumer. Official color-processing samples (`ConceptVisionColorLocator_*`) are the teaching comparison, not a substitute for framed ranging.

---



## 13. Limitations and future work

**Limitations (now):**

- No runtime Brown-Conrady undistort on the Hub.
- Observation worker still ticks with a 1 ms sleep; fusion still holds the runtime lock during `engine.update()` + `world.update()` (quality issue [#40](https://github.com/The-Allsparks/ViDAR/issues/40), waiting on Hub numbers).
- `VidarContourProcessor` remains a large unified pass; a split is a prerequisite before fixture OpenCV.
- Four-camera 720p MJPEG unmeasured.
- Season AprilTag list may be empty until transcribed from the current manual.
- Offline Ceres-style extrinsic refinement is documented, not implemented.

**Directed future work** (GitHub is authoritative; do not treat this list as a promise):

1. Fill Hub validation rows before claiming match vision.
2. Accept fixture-state and fixture-localization design reviews, then implement generic landmark geometry — not season `if` branches.
3. Capture-time camera-to-world projection as a published contract.
4. Opportunistic tag decode (longer cooldown after a trusted fix) if Hub CPU requires it.
5. Narrow adapters for TRACE / ECHO / HELM that preserve passivity.

---



## 14. Conclusion

ViDAR is an attempt to make FTC vision **boring in the right ways**: named frames, capture timestamps, geometry-authoritative range, a world model that can go stale, AprilTags that cannot lie via scout, and an OpMode API that only reads snapshots.

It is version **0.2.0**. The software architecture for 1–4 cameras is in tree and tested off-robot. The Control Hub still owes the project a USB and loop-time log. Until those rows exist, ViDAR is a **spatial library you can install and teach**, not a claim that the hive of cameras will survive a qualifying match.

The correct next experiment is not more detectors. It is one calibrated camera, Discover telemetry that matches the sim, and a dated Hub row.

---



## References (project documents)


| Document                                     | Use                                       |
| -------------------------------------------- | ----------------------------------------- |
| [README.md](../README.md)                    | Product overview and quick start          |
| [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md)         | Living architecture and status labels     |
| [API.md](API.md)                             | Cross-language outer contract             |
| [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md) | Frames, transforms, calibration           |
| [PERFORMANCE.md](PERFORMANCE.md)             | Budgets and what CI cannot test           |
| [LIFECYCLE.md](LIFECYCLE.md)                 | VisionPortal ownership and `stop()`       |
| [INSTALL.md](INSTALL.md)                     | Versioned TeamCode copy                   |
| [TEACHING.md](TEACHING.md)                   | Java lesson path                          |
| [PEDRO_INTEGRATION.md](PEDRO_INTEGRATION.md) | Optional pose bridge                      |
| [ROADMAP.md](ROADMAP.md)                     | Phases and USB wiring                     |
| [validation-log.md](validation-log.md)       | Field evidence (Hub rows currently empty) |
| [AGENTS.md](../AGENTS.md)                    | Invariants for humans and coding agents   |
| [JAVA_PACKAGE_MAP.md](JAVA_PACKAGE_MAP.md)   | Package responsibilities                  |


Repository: [github.com/The-Allsparks/ViDAR](https://github.com/The-Allsparks/ViDAR)

---



## How to cite

The Allsparks, *ViDAR: Robot-Space Situational Awareness for FIRST Tech Challenge*, technical white paper, library version 0.2.0, 14 September 2026. [https://github.com/The-Allsparks/ViDAR](https://github.com/The-Allsparks/ViDAR)