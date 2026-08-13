package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
/**
 * Fused season game-element detection (ball, block, ring, pixel, …) with range and
 * robot-frame position.
 */
public final class VidarElementObservation {

    /** Season element id from {@code season.json} (e.g. {@code artifact_purple}). */
    public final String elementId;
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
    public final VidarElementDetectorType detectorType;
    /** 0–1 composite confidence. */
    public final double confidence;
    public final double range;
    public final double rangeUncertainty;
    public final double dSize;
    public final double dFloor;
    /** Geometric slant range from mount + intrinsics (ball-center height). */
    public final double dGround;
    public final VidarRangeResult rangeResult;
    /** Robot-frame floor position: +X forward, +Y left (active distance unit). */
    public final double robotX;
    public final double robotY;
    /** Legacy Hough vote count (0 for color-blob detections). */
    public final int houghVotes;

    public VidarElementObservation(
            String cameraName,
            long captureTimeNanos,
            double cx, double cy,
            double boundingWidthPx, double boundingHeightPx,
            double fittedCx, double fittedCy, double radiusPx,
            double areaPx, double aspectRatio, double circularity, double fillRatio,
            double interiorValidationScore,
            VidarElementDetectorType detectorType,
            double confidence,
            double range, double rangeUncertainty,
            double dSize, double dFloor,
            VidarRangeResult rangeResult,
            double robotX, double robotY) {
        this("", cameraName, captureTimeNanos, cx, cy, boundingWidthPx, boundingHeightPx,
                fittedCx, fittedCy, radiusPx, areaPx, aspectRatio, circularity, fillRatio,
                interiorValidationScore, detectorType, confidence, range, rangeUncertainty,
                dSize, dFloor, Double.NaN, rangeResult, robotX, robotY, 0);
    }

    public VidarElementObservation(
            String elementId,
            String cameraName,
            long captureTimeNanos,
            double cx, double cy,
            double boundingWidthPx, double boundingHeightPx,
            double fittedCx, double fittedCy, double radiusPx,
            double areaPx, double aspectRatio, double circularity, double fillRatio,
            double interiorValidationScore,
            VidarElementDetectorType detectorType,
            double confidence,
            double range, double rangeUncertainty,
            double dSize, double dFloor, double dGround,
            VidarRangeResult rangeResult,
            double robotX, double robotY,
            int houghVotes) {
        this.elementId = elementId == null ? "" : elementId;
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
        this.range = range;
        this.rangeUncertainty = rangeUncertainty;
        this.dSize = dSize;
        this.dFloor = dFloor;
        this.dGround = dGround;
        this.rangeResult = rangeResult;
        this.robotX = robotX;
        this.robotY = robotY;
        this.houghVotes = houghVotes;
    }
}
