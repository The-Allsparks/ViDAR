package org.firstinspires.ftc.teamcode.vidar.config;

/**
 * OpenCV HSV bounds for color segmentation (H 0–179, S/V 0–255).
 */
public final class VidarHsvRange {

    public final int hMin;
    public final int hMax;
    public final int sMin;
    public final int sMax;
    public final int vMin;
    public final int vMax;

    public VidarHsvRange(int hMin, int hMax, int sMin, int sMax, int vMin, int vMax) {
        this.hMin = hMin;
        this.hMax = hMax;
        this.sMin = sMin;
        this.sMax = sMax;
        this.vMin = vMin;
        this.vMax = vMax;
    }
}
