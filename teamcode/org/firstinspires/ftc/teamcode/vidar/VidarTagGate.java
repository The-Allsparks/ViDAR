package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

/**
 * When to run an AprilTag decode pass (driver trigger and/or pose cone).
 */
public final class VidarTagGate {

    private static volatile boolean driverRequested;
    private static volatile boolean autoEnabled = true;
    private static volatile double expectedTagBearingDeg = Double.NaN;
    private static volatile double cameraBearingDeg = 0;

    private VidarTagGate() {}

    /** Call from gamepad edge (e.g. A button) to force one decode window. */
    public static void requestSample() {
        driverRequested = true;
    }

    public static void clearDriverRequest() {
        driverRequested = false;
    }

    public static boolean consumeDriverRequest() {
        if (!driverRequested) {
            return false;
        }
        driverRequested = false;
        return true;
    }

    public static void setAutoEnabled(boolean enabled) {
        autoEnabled = enabled;
    }

    /** Field bearing where the tag should appear (degrees). NaN disables pose gate. */
    public static void setExpectedTagBearingDeg(double bearingDeg) {
        expectedTagBearingDeg = bearingDeg;
    }

    /** Mount bearing of this camera on the robot (degrees). */
    public static void setCameraBearingDeg(double bearingDeg) {
        cameraBearingDeg = bearingDeg;
    }

    /** Set pose-gate bearing from season tag map and a field pose prior. */
    public static void updateExpectedBearingFromFieldPose(
            Pose2D fieldPose,
            VidarSeasonConfig season) {
        if (season == null || season.aprilTags.length == 0) {
            return;
        }
        double bearing = season.nearestLocalizationTagFieldBearing(fieldPose);
        if (!Double.isNaN(bearing)) {
            setExpectedTagBearingDeg(bearing);
        }
    }

    public static boolean shouldSample(VidarTagScoutResult scout, int frameCols) {
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
            return scout.widthPx >= VidarTagConfig.SCOUT_MIN_WIDTH_PX;
        }
        double centerErrPx = scout.cx - frameCols / 2.0;
        double halfFovDeg = 35;
        double bearingErr = centerErrPx / Math.max(1, frameCols) * halfFovDeg * 2;
        double robotToTag = normalizeDegrees(expectedTagBearingDeg - cameraBearingDeg);
        return Math.abs(normalizeDegrees(robotToTag - bearingErr)) <= VidarTagConfig.POSE_GATE_DEG;
    }

    private static double normalizeDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
