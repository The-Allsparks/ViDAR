package org.firstinspires.ftc.teamcode.vidar;

/**
 * Per-camera regions of interest and calibration. ROIs may overlap.
 * Defaults: ball lower 65%, plate middle 40%, tag upper 65%.
 */
public final class VidarCameraRoiConfig {

    /** Lower fraction of frame for ball search (from bottom). */
    public final double ballLowerFraction;
    /** Middle band start (fraction from top) and height for plate search. */
    public final double plateStartFraction;
    public final double plateBandFraction;
    /** Upper fraction of frame for AprilTag search (from top). */
    public final double tagUpperFraction;

    public final boolean ballEnabled;
    public final boolean plateEnabled;
    public final boolean tagEnabled;

    public VidarCameraRoiConfig(
            double ballLowerFraction,
            double plateStartFraction,
            double plateBandFraction,
            double tagUpperFraction,
            boolean ballEnabled,
            boolean plateEnabled,
            boolean tagEnabled) {
        this.ballLowerFraction = ballLowerFraction;
        this.plateStartFraction = plateStartFraction;
        this.plateBandFraction = plateBandFraction;
        this.tagUpperFraction = tagUpperFraction;
        this.ballEnabled = ballEnabled;
        this.plateEnabled = plateEnabled;
        this.tagEnabled = tagEnabled;
    }

    public static final VidarCameraRoiConfig DEFAULT = new VidarCameraRoiConfig(
            0.65, 0.30, 0.40, 0.65, true, true, true);

    public VidarRoiRect ballRoi(int frameW, int frameH) {
        VidarRoiRect roi = VidarRoiRect.lowerFraction(frameW, frameH, ballLowerFraction);
        return ballEnabled ? roi : roi.withEnabled(false);
    }

    public VidarRoiRect plateRoi(int frameW, int frameH) {
        VidarRoiRect roi = VidarRoiRect.middleBand(frameW, frameH, plateStartFraction, plateBandFraction);
        return plateEnabled ? roi : roi.withEnabled(false);
    }

    public VidarRoiRect tagRoi(int frameW, int frameH) {
        VidarRoiRect roi = VidarRoiRect.upperFraction(frameW, frameH, tagUpperFraction);
        return tagEnabled ? roi : roi.withEnabled(false);
    }

    /** Horizon row in full-frame coordinates (smaller y = farther). */
    public int horizonRowFullFrame(int frameH, int processHorizonRowPx) {
        VidarRoiRect ball = ballRoi(frameW(frameH), frameH);
        if (!ball.enabled) {
            return (int) Math.round(frameH * (1.0 - ballLowerFraction));
        }
        return ball.y + processHorizonRowPx;
    }

    private static int frameW(int frameH) {
        return (int) Math.round(frameH * 4.0 / 3.0);
    }
}
