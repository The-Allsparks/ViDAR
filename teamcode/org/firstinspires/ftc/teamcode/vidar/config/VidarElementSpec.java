package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementShape;

/**
 * One scorable game element for the active FTC season (ball, block, cone, ring, etc.).
 */
public final class VidarElementSpec {

    public final String id;
    public final String label;
    public final VidarElementShape shape;
    public final double diameter;
    public final VidarElementDetectorType detector;
    public final VidarHsvRange hsv;
    public final double minAreaPx;
    public final double maxAreaPx;
    public final double minWidthPx;
    public final double maxWidthPx;
    public final double minHeightPx;
    public final double maxHeightPx;
    public final double maxAspectRatio;
    public final double minCircularity;
    public final double minRectangularity;
    public final double minAspect;
    public final double maxAspect;
    public final double minFillRatio;
    public final double minInteriorScore;
    public final int interiorBright;
    public final int interiorSpread;
    public final int holeDarkMax;
    public final int morphErodePasses;
    public final int morphDilatePasses;
    public final int morphOpenPasses;
    public final int morphClosePasses;
    public final double houghDp;
    public final double houghMinDist;
    public final double houghParam1;
    public final double houghParam2;
    public final int houghMinRadius;
    public final int houghMaxRadius;
    public final double houghMinInterior;
    public final double minAreaHoughPx;

    public VidarElementSpec(
            String id,
            String label,
            VidarElementShape shape,
            double diameter,
            VidarElementDetectorType detector,
            VidarHsvRange hsv,
            double minAreaPx,
            double maxAreaPx,
            double minWidthPx,
            double maxWidthPx,
            double minHeightPx,
            double maxHeightPx,
            double maxAspectRatio,
            double minCircularity,
            double minRectangularity,
            double minAspect,
            double maxAspect,
            double minFillRatio,
            double minInteriorScore,
            int interiorBright,
            int interiorSpread,
            int holeDarkMax,
            int morphErodePasses,
            int morphDilatePasses,
            int morphOpenPasses,
            int morphClosePasses,
            double houghDp,
            double houghMinDist,
            double houghParam1,
            double houghParam2,
            int houghMinRadius,
            int houghMaxRadius,
            double houghMinInterior,
            double minAreaHoughPx) {
        this.id = id;
        this.label = label;
        this.shape = shape == null ? VidarElementShape.CIRCLE : shape;
        this.diameter = diameter;
        this.detector = detector;
        this.hsv = hsv;
        this.minAreaPx = minAreaPx;
        this.maxAreaPx = maxAreaPx;
        this.minWidthPx = minWidthPx;
        this.maxWidthPx = maxWidthPx;
        this.minHeightPx = minHeightPx;
        this.maxHeightPx = maxHeightPx;
        this.maxAspectRatio = maxAspectRatio;
        this.minCircularity = minCircularity;
        this.minRectangularity = minRectangularity;
        this.minAspect = minAspect;
        this.maxAspect = maxAspect;
        this.minFillRatio = minFillRatio;
        this.minInteriorScore = minInteriorScore;
        this.interiorBright = interiorBright;
        this.interiorSpread = interiorSpread;
        this.holeDarkMax = holeDarkMax;
        this.morphErodePasses = morphErodePasses;
        this.morphDilatePasses = morphDilatePasses;
        this.morphOpenPasses = morphOpenPasses;
        this.morphClosePasses = morphClosePasses;
        this.houghDp = houghDp;
        this.houghMinDist = houghMinDist;
        this.houghParam1 = houghParam1;
        this.houghParam2 = houghParam2;
        this.houghMinRadius = houghMinRadius;
        this.houghMaxRadius = houghMaxRadius;
        this.houghMinInterior = houghMinInterior;
        this.minAreaHoughPx = minAreaHoughPx;
    }
}
