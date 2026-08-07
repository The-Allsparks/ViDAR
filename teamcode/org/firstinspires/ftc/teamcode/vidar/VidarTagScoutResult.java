package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;

/**
 * Cheap top-half scout hit used to pick horizontal decode band and decimation.
 */
public final class VidarTagScoutResult {

    public final double cx;
    public final double cy;
    public final double widthPx;
    public final VidarFrameRegions.HorizontalBand band;

    public VidarTagScoutResult(
            double cx,
            double cy,
            double widthPx,
            VidarFrameRegions.HorizontalBand band) {
        this.cx = cx;
        this.cy = cy;
        this.widthPx = widthPx;
        this.band = band;
    }
}
