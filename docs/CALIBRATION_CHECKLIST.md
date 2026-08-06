# ViDAR calibration checklist

One-page setup before relying on `robotX` / `robotY` or assisted driving. Full detail: [CALIBRATION.md](CALIBRATION.md) and [config/robots/README.md](../config/robots/README.md).

## 0 — Copy config

| Source (repo) | Destination (Android project) |
|---------------|-------------------------------|
| `config/robots/<your-template>.json` | `TeamCode/src/main/assets/vidar/robot.json` |
| `config/seasons/<year>-<game>.json` | `TeamCode/src/main/assets/vidar/season.json` |

Pick the template that matches **camera count** and **hardware** (SVPRO vs C920). See the selection table in `config/robots/README.md`.

## 1 — Hardware names

- [ ] `webcamName` matches Driver Station camera names (`Webcam 1` … `Webcam 4`)
- [ ] `cameraCount` equals physical USB cameras plugged in
- [ ] `activeCameraIndex` points at your primary forward camera (0-based)

## 2 — Intrinsics (one-time per camera model)

- [ ] `focalLengthPx` matches your camera at **640×480** (SVPRO ≈ 246, C920-class ≈ 340)
- [ ] `horizontalFovDeg` / `verticalFovDeg` match product spec (sanity check only)

## 3 — Mount pose

- [ ] `mount.x`, `mount.y`, `mount.z` measured from robot center (inches)
- [ ] `bearingDeg` matches physical aim (0 = forward)
- [ ] `pitchDeg` set — run **ViDAR: ROI Calibrate** + sim **Calibration axes** overlay to verify

## 4 — Floor LUT (required on-robot)

Templates ship with `"floorLutStatus": "example-placeholder-recalibrate-on-robot"`.

- [ ] Run **ViDAR: Discover** on the Control Hub
- [ ] Place a game element on the floor at **12 / 24 / 36 / 48 in** (or your check distances) centered in the active camera
- [ ] Read `size=`, `floor=`, `ground=` on the element detail line — note pixel row (`cy`) and true distance
- [ ] Update `floorLut` rows in `robot.json` for each camera (repeat per camera if mounts differ)
- [ ] Re-deploy and confirm fusion confidence rises (three sources agree)

## 5 — Validate ranging

- [ ] `robotX` / `robotY` move in the expected direction when you slide the target left/right/forward
- [ ] Fused range within ~15% of tape measure at 24–48 in
- [ ] `elements()` / `foes()` bearing matches physical left/right/forward motion

## 6 — Optional: field pose (Pedro / localization)

ViDAR does **not** require Pedro. If you use path following:

- [ ] Supply odom via `VidarSpatial.create(hardwareMap, odom::getPose, alliance::get)`
- [ ] `spatial.setFieldPosePrior(startPose)` at auto init
- [ ] For Pedro-primary pose: `spatial.setFieldPoseSupplier(follower::getPose)` so world tracks stay in field frame
- [ ] Read `spatial.elements()` / `allies()` / `foes()` in team drivetrain code — ViDAR does not command motors

## 7 — Motion tracking (requires odom)

Run **ViDAR: Spatial Map** with Pinpoint / Pedro / team odom wired at `create()`.

- [ ] Telemetry shows `Motion tracks: true`
- [ ] Discover element line shows season `elementId` (e.g. `artifact_purple · …`)
- [ ] `elements()` shows `elementId#0`, `#1`, … per type ranked by distance
- [ ] Same `trackId` survives 0.5 s occlusion while robot moves slowly
- [ ] Static ball: field velocity on track telemetry ≈ 0 in/s after ~5 frames
- [ ] Moving foe: non-zero field velocity when opponent crosses FOV
- [ ] `spatial.setMotionTrackingEnabled(false)` → `Motion tracks: false`, live-only lists

## Done?

When steps 1–5 pass on hardware, mark your team config calibrated and remove or update `floorLutStatus` in `robot.json`. Complete step 7 before competition auto that relies on remembered targets.
