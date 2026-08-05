package org.firstinspires.ftc.teamcode.vidar;

import org.opencv.core.RotatedRect;

/**
 * Alliance plate detection with size-based primary ranging and optional floor cross-check.
 */
public final class VidarPlateObservation {

    public final VidarAlliance alliance;
    public final double cx;
    public final double cy;
    public final double widthPx;
    public final double heightPx;
    public final double angleDeg;
    public final double aspectRatio;
    public final double whiteRatio;
    public final double rangeIn;
    public final double rangeUncertaintyIn;
    public final double sizeBasedRangeIn;
    public final double floorBasedRangeIn;
    public final VidarRangeResult rangeResult;
    public final double viewingAnglePenalty;
    public final double partialVisibilityPenalty;
    public final double confidence;
    public final double robotXIn;
    public final double robotYIn;
    public final String cameraName;
    public final long captureTimeNanos;

    public VidarPlateObservation(
            VidarAlliance alliance,
            double cx,
            double cy,
            double widthPx,
            double heightPx,
            double angleDeg,
            double aspectRatio,
            double whiteRatio,
            double rangeIn,
            double confidence,
            double robotXIn,
            double robotYIn,
            String cameraName) {
        this(alliance, cx, cy, widthPx, heightPx, angleDeg, aspectRatio, whiteRatio,
                rangeIn, Double.NaN, Double.NaN, Double.NaN, null,
                1.0, 1.0, confidence, robotXIn, robotYIn, cameraName, 0);
    }

    public VidarPlateObservation(
            VidarAlliance alliance,
            double cx,
            double cy,
            double widthPx,
            double heightPx,
            double angleDeg,
            double aspectRatio,
            double whiteRatio,
            double rangeIn,
            double rangeUncertaintyIn,
            double sizeBasedRangeIn,
            double floorBasedRangeIn,
            VidarRangeResult rangeResult,
            double viewingAnglePenalty,
            double partialVisibilityPenalty,
            double confidence,
            double robotXIn,
            double robotYIn,
            String cameraName,
            long captureTimeNanos) {
        this.alliance = alliance;
        this.cx = cx;
        this.cy = cy;
        this.widthPx = widthPx;
        this.heightPx = heightPx;
        this.angleDeg = angleDeg;
        this.aspectRatio = aspectRatio;
        this.whiteRatio = whiteRatio;
        this.rangeIn = rangeIn;
        this.rangeUncertaintyIn = rangeUncertaintyIn;
        this.sizeBasedRangeIn = sizeBasedRangeIn;
        this.floorBasedRangeIn = floorBasedRangeIn;
        this.rangeResult = rangeResult;
        this.viewingAnglePenalty = viewingAnglePenalty;
        this.partialVisibilityPenalty = partialVisibilityPenalty;
        this.confidence = confidence;
        this.robotXIn = robotXIn;
        this.robotYIn = robotYIn;
        this.cameraName = cameraName;
        this.captureTimeNanos = captureTimeNanos;
    }

    public RotatedRect toRotatedRect() {
        return new RotatedRect(
                new org.opencv.core.Point(cx, cy),
                new org.opencv.core.Size(widthPx, heightPx),
                (float) angleDeg);
    }

    public boolean isFoe() {
        return isFoe(VidarConfig.DEFAULT_ALLIANCE);
    }

    public boolean isAlly() {
        return isAlly(VidarConfig.DEFAULT_ALLIANCE);
    }

    public boolean isFoe(VidarAlliance ourAlliance) {
        return alliance.isFoe(ourAlliance);
    }

    public boolean isAlly(VidarAlliance ourAlliance) {
        return alliance.isAlly(ourAlliance);
    }
}
