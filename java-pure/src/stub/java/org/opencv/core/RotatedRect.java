package org.opencv.core;

/** Minimal stub for JVM unit tests. */
public final class RotatedRect {
    public final Point center;
    public final Size size;
    public final double angle;

    public RotatedRect(Point center, Size size, double angle) {
        this.center = center;
        this.size = size;
        this.angle = angle;
    }
}
