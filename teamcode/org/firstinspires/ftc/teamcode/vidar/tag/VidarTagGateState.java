package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarCoordinateFrames;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;

/**
 * Instance-scoped AprilTag decode gate state (driver request + pose cone).
 *
 * <p>Owned by {@link org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntime} so Auto→TeleOp
 * reuse and sequential create cannot leak {@code driverRequested} across logical lifetimes.
 */
public final class VidarTagGateState {

    private volatile boolean driverRequested;
    private volatile boolean autoEnabled = true;
    private volatile double expectedTagBearingDeg = Double.NaN;
    private volatile double cameraBearingDeg = 0;

    /** Call from gamepad edge (e.g. A button) to force one decode window. */
    public void requestSample() {
        driverRequested = true;
    }

    public void clearDriverRequest() {
        driverRequested = false;
    }

    public boolean consumeDriverRequest() {
        if (!driverRequested) {
            return false;
        }
        driverRequested = false;
        return true;
    }

    public void setAutoEnabled(boolean enabled) {
        autoEnabled = enabled;
    }

    /** Field bearing where the tag should appear (degrees). NaN disables pose gate. */
    public void setExpectedTagBearingDeg(double bearingDeg) {
        expectedTagBearingDeg = bearingDeg;
    }

    /** Mount bearing of this camera on the robot (degrees). */
    public void setCameraBearingDeg(double bearingDeg) {
        cameraBearingDeg = bearingDeg;
    }

    /** Set pose-gate bearing from season tag map and a field pose prior. */
    public void updateExpectedBearingFromFieldPose(Pose2D fieldPose, VidarSeasonConfig season) {
        if (season == null || season.aprilTags.length == 0) {
            return;
        }
        double bearing = season.nearestLocalizationTagFieldBearing(fieldPose);
        if (!Double.isNaN(bearing)) {
            setExpectedTagBearingDeg(bearing);
        }
    }

    public boolean shouldSample(VidarTagScoutObservation scout, int frameCols) {
        if (!VidarTagConfig.ENABLED || scout == null) {
            return false;
        }
        if (consumeDriverRequest()) {
            return true;
        }
        if (!autoEnabled) {
            return false;
        }
        if (Double.isNaN(expectedTagBearingDeg) || VidarTagConfig.POSE_GATE_DEG <= 0) {
            return scout.apparentWidthPx >= VidarTagConfig.SCOUT_MIN_WIDTH_PX;
        }
        double centerErrPx = scout.cx - frameCols / 2.0;
        double halfFovDeg = 35;
        double bearingErr = centerErrPx / Math.max(1, frameCols) * halfFovDeg * 2;
        double robotToTag = VidarCoordinateFrames.normalizeDeg(expectedTagBearingDeg - cameraBearingDeg);
        return Math.abs(VidarCoordinateFrames.normalizeDeg(robotToTag - bearingErr))
                <= VidarTagConfig.POSE_GATE_DEG;
    }

    /** Clear driver request and pose-gate knobs (attach/detach / match reset). */
    public void reset() {
        driverRequested = false;
        autoEnabled = true;
        expectedTagBearingDeg = Double.NaN;
        cameraBearingDeg = 0;
    }
}
