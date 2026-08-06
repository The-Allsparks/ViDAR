package org.firstinspires.ftc.teamcode.vidar;

/**
 * Per-camera regions of interest and calibration. ROIs may overlap.
 * Defaults: element lower 65%, plate middle 40%, tag upper 65%.
 */
public final class VidarCameraRoiConfig {

    /** Lower fraction of frame for season element search (from bottom). */
    public final double elementLowerFraction;
    /** Middle band start (fraction from top) and height for plate search. */
    public final double plateStartFraction;
    public final double plateBandFraction;
    /** Upper fraction of frame for AprilTag search (from top). */
    public final double tagUpperFraction;

    public final boolean elementEnabled;
    public final boolean plateEnabled;
    public final boolean tagEnabled;

    public VidarCameraRoiConfig(
            double elementLowerFraction,
            double plateStartFraction,
            double plateBandFraction,
            double tagUpperFraction,
            boolean elementEnabled,
            boolean plateEnabled,
            boolean tagEnabled) {
        this.elementLowerFraction = elementLowerFraction;
        this.plateStartFraction = plateStartFraction;
        this.plateBandFraction = plateBandFraction;
        this.tagUpperFraction = tagUpperFraction;
        this.elementEnabled = elementEnabled;
        this.plateEnabled = plateEnabled;
        this.tagEnabled = tagEnabled;
    }

    public static final VidarCameraRoiConfig DEFAULT = new VidarCameraRoiConfig(
            0.65, 0.30, 0.40, 0.65, true, true, true);

    public VidarRoiRect elementRoi(int frameW, int frameH) {
        VidarRoiRect roi = VidarRoiRect.lowerFraction(frameW, frameH, elementLowerFraction);
        return elementEnabled ? roi : roi.withEnabled(false);
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
        VidarRoiRect element = elementRoi(frameW(frameH), frameH);
        if (!element.enabled) {
            return (int) Math.round(frameH * (1.0 - elementLowerFraction));
        }
        return element.y + processHorizonRowPx;
    }

    private static int frameW(int frameH) {
        return (int) Math.round(frameH * 4.0 / 3.0);
    }
}
