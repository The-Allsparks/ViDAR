package org.firstinspires.ftc.teamcode.vidar;

/**
 * Rejection telemetry for element-detection pipeline stages (student-debuggable).
 */
public final class VidarElementRejectionStats {

    public int maskPixels;
    public int contourCount;
    public int rejectedArea;
    public int rejectedAspect;
    public int rejectedCircularity;
    public int rejectedFillRatio;
    public int rejectedInterior;
    public int rejectedBoundary;
    public int rejectedRange;
    public int rejectedConfidence;
    public int accepted;

    public void reset() {
        maskPixels = 0;
        contourCount = 0;
        rejectedArea = 0;
        rejectedAspect = 0;
        rejectedCircularity = 0;
        rejectedFillRatio = 0;
        rejectedInterior = 0;
        rejectedBoundary = 0;
        rejectedRange = 0;
        rejectedConfidence = 0;
        accepted = 0;
    }

    public String summary() {
        return String.format(
                "mask=%d cnt=%d acc=%d rej[a=%d ar=%d circ=%d fill=%d int=%d bnd=%d rng=%d conf=%d]",
                maskPixels, contourCount, accepted,
                rejectedArea, rejectedAspect, rejectedCircularity, rejectedFillRatio,
                rejectedInterior, rejectedBoundary, rejectedRange, rejectedConfidence);
    }
}
