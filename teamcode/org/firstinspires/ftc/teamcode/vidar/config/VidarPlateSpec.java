package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;

/**
 * Alliance plate appearance for the active FTC season.
 */
public final class VidarPlateSpec {

    public final VidarAlliance alliance;
    public final VidarHsvRange hsv;
    /** Optional hue wrap for red; null when not used. */
    public final VidarHsvRange hsvWrap;
    /** Nominal plate width in inches (for documentation; ranging uses camera profile). */
    public final double width;
    public final double minAreaPx;
    public final double maxAreaPx;
    public final double minAspect;
    public final double maxAspect;
    public final double minRectangularity;
    public final double minWhiteRatio;
    public final int whiteSampleGrid;
    public final int whiteBrightMin;
    public final int whiteSpreadMax;

    public VidarPlateSpec(
            VidarAlliance alliance,
            VidarHsvRange hsv,
            VidarHsvRange hsvWrap,
            double width,
            double minAreaPx,
            double maxAreaPx,
            double minAspect,
            double maxAspect,
            double minRectangularity,
            double minWhiteRatio,
            int whiteSampleGrid,
            int whiteBrightMin,
            int whiteSpreadMax) {
        this.alliance = alliance;
        this.hsv = hsv;
        this.hsvWrap = hsvWrap;
        this.width = width;
        this.minAreaPx = minAreaPx;
        this.maxAreaPx = maxAreaPx;
        this.minAspect = minAspect;
        this.maxAspect = maxAspect;
        this.minRectangularity = minRectangularity;
        this.minWhiteRatio = minWhiteRatio;
        this.whiteSampleGrid = whiteSampleGrid;
        this.whiteBrightMin = whiteBrightMin;
        this.whiteSpreadMax = whiteSpreadMax;
    }
}
