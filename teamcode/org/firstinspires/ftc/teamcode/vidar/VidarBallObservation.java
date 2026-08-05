package org.firstinspires.ftc.teamcode.vidar;

/**
 * Fused ball detection with color-blob pipeline outputs and range uncertainty.
 */
public final class VidarBallObservation {

    public final String cameraName;
    public final long captureTimeNanos;
    /** Image center in full-frame coordinates. */
    public final double cx;
    public final double cy;
    public final double boundingWidthPx;
    public final double boundingHeightPx;
    public final double fittedCx;
    public final double fittedCy;
    public final double radiusPx;
    public final double areaPx;
    public final double aspectRatio;
    public final double circularity;
    public final double fillRatio;
    public final double interiorValidationScore;
    public final VidarBallDetectorType detectorType;
    /** 0–1 composite confidence. */
    public final double confidence;
    public final double rangeIn;
    public final double rangeUncertaintyIn;
    public final double dSizeIn;
    public final double dFloorIn;
    public final VidarRangeResult rangeResult;
    /** Robot-frame floor position (inches): +X forward, +Y left. */
    public final double robotXIn;
    public final double robotYIn;
    /** Legacy Hough vote count (0 for color-blob detections). */
    public final int houghVotes;

    public VidarBallObservation(
            String cameraName,
            long captureTimeNanos,
            double cx, double cy,
            double boundingWidthPx, double boundingHeightPx,
            double fittedCx, double fittedCy, double radiusPx,
            double areaPx, double aspectRatio, double circularity, double fillRatio,
            double interiorValidationScore,
            VidarBallDetectorType detectorType,
            double confidence,
            double rangeIn, double rangeUncertaintyIn,
            double dSizeIn, double dFloorIn,
            VidarRangeResult rangeResult,
            double robotXIn, double robotYIn) {
        this(cameraName, captureTimeNanos, cx, cy, boundingWidthPx, boundingHeightPx,
                fittedCx, fittedCy, radiusPx, areaPx, aspectRatio, circularity, fillRatio,
                interiorValidationScore, detectorType, confidence, rangeIn, rangeUncertaintyIn,
                dSizeIn, dFloorIn, rangeResult, robotXIn, robotYIn, 0);
    }

    /** @deprecated Legacy constructor — use full constructor. */
    @Deprecated
    public VidarBallObservation(
            double cx,
            double cy,
            double radiusPx,
            double rangeIn,
            double dSizeIn,
            double dFloorIn,
            double confidence,
            double robotXIn,
            double robotYIn,
            int houghVotes,
            String cameraName) {
        this(cameraName, 0, cx, cy, radiusPx * 2, radiusPx * 2,
                cx, cy, radiusPx, Math.PI * radiusPx * radiusPx,
                1.0, 1.0, 0.85, 0.5,
                VidarBallDetectorType.LEGACY_HOUGH, confidence,
                rangeIn, Double.NaN, dSizeIn, dFloorIn, null,
                robotXIn, robotYIn, houghVotes);
    }

    public VidarBallObservation(
            String cameraName,
            long captureTimeNanos,
            double cx, double cy,
            double boundingWidthPx, double boundingHeightPx,
            double fittedCx, double fittedCy, double radiusPx,
            double areaPx, double aspectRatio, double circularity, double fillRatio,
            double interiorValidationScore,
            VidarBallDetectorType detectorType,
            double confidence,
            double rangeIn, double rangeUncertaintyIn,
            double dSizeIn, double dFloorIn,
            VidarRangeResult rangeResult,
            double robotXIn, double robotYIn,
            int houghVotes) {
        this.cameraName = cameraName;
        this.captureTimeNanos = captureTimeNanos;
        this.cx = cx;
        this.cy = cy;
        this.boundingWidthPx = boundingWidthPx;
        this.boundingHeightPx = boundingHeightPx;
        this.fittedCx = fittedCx;
        this.fittedCy = fittedCy;
        this.radiusPx = radiusPx;
        this.areaPx = areaPx;
        this.aspectRatio = aspectRatio;
        this.circularity = circularity;
        this.fillRatio = fillRatio;
        this.interiorValidationScore = interiorValidationScore;
        this.detectorType = detectorType;
        this.confidence = confidence;
        this.rangeIn = rangeIn;
        this.rangeUncertaintyIn = rangeUncertaintyIn;
        this.dSizeIn = dSizeIn;
        this.dFloorIn = dFloorIn;
        this.rangeResult = rangeResult;
        this.robotXIn = robotXIn;
        this.robotYIn = robotYIn;
        this.houghVotes = houghVotes;
    }
}
