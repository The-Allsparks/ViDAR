package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
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
    public final double range;
    public final double rangeUncertainty;
    public final double sizeBasedRange;
    public final double floorBasedRange;
    public final double groundBasedRange;
    public final VidarRangeResult rangeResult;
    public final double viewingAnglePenalty;
    public final double partialVisibilityPenalty;
    public final double confidence;
    public final double robotX;
    public final double robotY;
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
            double range,
            double confidence,
            double robotX,
            double robotY,
            String cameraName) {
        this(alliance, cx, cy, widthPx, heightPx, angleDeg, aspectRatio, whiteRatio,
                range, Double.NaN, Double.NaN, Double.NaN, Double.NaN, null,
                1.0, 1.0, confidence, robotX, robotY, cameraName, 0);
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
            double range,
            double rangeUncertainty,
            double sizeBasedRange,
            double floorBasedRange,
            double groundBasedRange,
            VidarRangeResult rangeResult,
            double viewingAnglePenalty,
            double partialVisibilityPenalty,
            double confidence,
            double robotX,
            double robotY,
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
        this.range = range;
        this.rangeUncertainty = rangeUncertainty;
        this.sizeBasedRange = sizeBasedRange;
        this.floorBasedRange = floorBasedRange;
        this.groundBasedRange = groundBasedRange;
        this.rangeResult = rangeResult;
        this.viewingAnglePenalty = viewingAnglePenalty;
        this.partialVisibilityPenalty = partialVisibilityPenalty;
        this.confidence = confidence;
        this.robotX = robotX;
        this.robotY = robotY;
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
