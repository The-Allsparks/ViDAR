# Pedro Pathing + ViDAR

ViDAR does **not** depend on Pedro Pathing. Pedro owns continuous pose and path following; ViDAR owns robot-space awareness and sparse AprilTag fixes. This doc shows how to wire them with the bridge in `vidar.integration`.

## Classes (no Pedro Maven required)

| Class | Role |
|-------|------|
| `VidarPedroPose` | Inches + heading **radians** (Pedro `Pose` shape) |
| `VidarPedroPoseBridge` | `Pose2D` ↔ `VidarPedroPose` / suppliers |
| `VidarPedroCorrectionTracker` | Gate on correction **event id**; return pose **corrected to now** |
| `VidarPedroBridgeSampleOpMode` | Compilable loop sample with an in-memory follower stand-in |

## Latency: why `fieldPose()` alone is wrong for `setPose`

AprilTag imagery is old by the time a correction reaches Pedro:

```
capture ──decode/fuse (≤1 Hz)──► fused anchor ──OpMode lag──► setPose
   │                                    │                         │
   fieldPoseAtCapture          backdated to odom@fuse      must re-propagate
                                                         to Pedro pose @ now
```

| Step | What ViDAR does |
|------|-----------------|
| 1. Capture | Tag stores `fieldPoseAtCapture` + `captureTimeNanos` |
| 2. Fuse | `odomHistory.at(capture)` + gates → stamp **odom-at-fuse once**; `lastTagCorrectionNanos()` advances |
| 3. Snapshot | `spatial.fusedFieldPose()` is the fuse-time anchor (not Pedro) |
| 4. setPose | `tagCorrectedFieldPoseNow()` re-propagates fuse → current odom |

**Do not** gate novelty on `spatial.fieldPose()` when Pedro is wired via `setFieldPoseSupplier` — that value is the live Pedro pose and will re-fire `setPose` as you drive.

**Do not** call `follower.setPose` with raw `fusedFieldPose()` — that plants the robot where it was at fuse time.

## Loop contract

```
follower.update();
spatial.update();
correction = tracker.poll(
        spatial.lastTagCorrectionNanos(),     // novelty: event id (0 = none)
        spatial.tagCorrectedFieldPoseNow());  // inject: re-propagated to now
if (correction != null) {
    follower.setPose(new Pose(correction.x, correction.y, correction.headingRad));
}
```

Cooldown lives in localization gates only — the tracker does not add a second timer.

## Team wiring (real Pedro)

```java
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.vidar.VidarSpatial;
import org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroCorrectionTracker;
import org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroPose;
import org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroPoseBridge;

Follower follower = Constants.createFollower(hardwareMap);
follower.setStartingPose(startPose);

VidarSpatial spatial = VidarSpatial.create(
        hardwareMap,
        VidarPedroPoseBridge.asPose2DSupplier(
                () -> follower.getPose().getX(),
                () -> follower.getPose().getY(),
                () -> follower.getPose().getHeading()),
        alliance::get);
// Continuous field pose for world tracks only — not the fused-tag anchor
spatial.setFieldPoseSupplier(
        VidarPedroPoseBridge.asPose2DSupplier(
                () -> follower.getPose().getX(),
                () -> follower.getPose().getY(),
                () -> follower.getPose().getHeading()));

VidarPedroCorrectionTracker corrections = new VidarPedroCorrectionTracker();

while (opModeIsActive()) {
    follower.update();
    spatial.update();

    if (spatial.intakeBlocked()) {
        // slow / hold — team FSM
    }

    VidarPedroPose fix = corrections.poll(
            spatial.lastTagCorrectionNanos(),
            spatial.tagCorrectedFieldPoseNow());
    if (fix != null) {
        follower.setPose(new Pose(fix.x, fix.y, fix.headingRad));
    }
}
spatial.close();
```

### API cheat sheet

| Method | Meaning |
|--------|---------|
| `fieldPose()` | Snapshot field pose (Pedro if `setFieldPoseSupplier` set) |
| `fusedFieldPose()` | Last gate-accepted tag fix at fuse-time (never Pedro; pinned) |
| `lastTagCorrectionNanos()` | Event id for tracker novelty (0 = none yet; pinned) |
| `tagCorrectedFieldPoseNow()` | Fused fix re-propagated to odom-at-publish (pinned) |

### Manual backdate (advanced)

```java
Pose2D atCapture = tag.fieldPoseAtCapture;
Pose2D odomThen = /* pose at tag.captureTimeNanos */;
Pose2D odomNow = VidarPedroPoseBridge.toPose2D(
        follower.getPose().getX(),
        follower.getPose().getY(),
        follower.getPose().getHeading());
Pose2D forSetPose = VidarPedroCorrectionTracker.backdateToNow(atCapture, odomThen, odomNow);
```

## Frames and units

| | ViDAR `Pose2D` | Pedro `Pose` |
|--|----------------|--------------|
| Position | inches | inches (typical) |
| Heading | degrees on the object (query with `AngleUnit`) | **radians** |
| Field axes | FTC center origin (+X right, +Y forward) | Team Pedro coordinate system |

The bridge assumes **the same field frame** your Pedro constants already use. If you rely on Pedro `CoordinateSystem` converters, convert before wrapping into `VidarPedroPose` / after reading `follower.getPose()`.

## Run the sample

1. Deploy and open **ViDAR: Pedro Bridge Sample**.
2. Confirm telemetry: fused anchor vs corrected-now (Δ), correction id.
3. Copy the loop into your Pedro auto; replace the stand-in with `Follower`.

Correction fields (`fusedFieldPose`, `lastTagCorrectionNanos`, `tagCorrectedFieldPoseNow`) are pinned on `spatial.update()` under the runtime lock, using the odom sample at pin time.

## Related

- [ROADMAP.md](ROADMAP.md) Phase 4 — localization split
- [API.md](API.md) — `VidarSpatial` contract
- [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) — runtime / attachment lifecycle
- `VidarMotionCorrection` / `VidarOdomHistory` — capture→now math
