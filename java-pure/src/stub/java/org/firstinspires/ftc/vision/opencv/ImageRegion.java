package org.firstinspires.ftc.vision.opencv;

/** Minimal stub for JVM unit tests. */
public final class ImageRegion {

    public final int x;
    public final int y;
    public final int width;
    public final int height;

    private ImageRegion(int x, int y, int width, int height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public static ImageRegion asImageCoordinates(int x, int y, int width, int height) {
        return new ImageRegion(x, y, width, height);
    }
}
