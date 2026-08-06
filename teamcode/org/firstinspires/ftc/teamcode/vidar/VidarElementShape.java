package org.firstinspires.ftc.teamcode.vidar;

/**
 * Expected silhouette for contour fitting and ranging.
 */
public enum VidarElementShape {
    /** Circle fit + optional Hough validation (balls, rings, cone bases). */
    CIRCLE,
    /** {@link org.opencv.imgproc.Imgproc#minAreaRect} fit (plates, prisms, boxes). */
    RECT,
    /** Color blob only — centroid and bounding box, no strict shape gate. */
    BLOB
}
