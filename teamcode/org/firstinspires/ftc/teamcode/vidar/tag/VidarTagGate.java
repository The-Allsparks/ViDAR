package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntime;

/**
 * Student-facing static facade for AprilTag decode gating.
 *
 * <p>Mutable state lives on the live {@link VidarRuntime} ({@link VidarTagGateState}).
 * Calls are no-ops / false when no runtime is active.
 */
public final class VidarTagGate {

    private VidarTagGate() {}

    private static VidarTagGateState state() {
        VidarRuntime runtime = VidarRuntime.getInstance();
        return runtime == null ? null : runtime.tagGate();
    }

    /** Call from gamepad edge (e.g. A button) to force one decode window. */
    public static void requestSample() {
        VidarTagGateState s = state();
        if (s != null) {
            s.requestSample();
        }
    }

    public static void clearDriverRequest() {
        VidarTagGateState s = state();
        if (s != null) {
            s.clearDriverRequest();
        }
    }

    public static boolean consumeDriverRequest() {
        VidarTagGateState s = state();
        return s != null && s.consumeDriverRequest();
    }

    public static void setAutoEnabled(boolean enabled) {
        VidarTagGateState s = state();
        if (s != null) {
            s.setAutoEnabled(enabled);
        }
    }

    /** Field bearing where the tag should appear (degrees). NaN disables pose gate. */
    public static void setExpectedTagBearingDeg(double bearingDeg) {
        VidarTagGateState s = state();
        if (s != null) {
            s.setExpectedTagBearingDeg(bearingDeg);
        }
    }

    /** Mount bearing of this camera on the robot (degrees). */
    public static void setCameraBearingDeg(double bearingDeg) {
        VidarTagGateState s = state();
        if (s != null) {
            s.setCameraBearingDeg(bearingDeg);
        }
    }

    /** Set pose-gate bearing from season tag map and a field pose prior. */
    public static void updateExpectedBearingFromFieldPose(
            Pose2D fieldPose,
            VidarSeasonConfig season) {
        VidarTagGateState s = state();
        if (s != null) {
            s.updateExpectedBearingFromFieldPose(fieldPose, season);
        }
    }

    public static boolean shouldSample(VidarTagScoutObservation scout, int frameCols) {
        VidarTagGateState s = state();
        return s != null && s.shouldSample(scout, frameCols);
    }
}
