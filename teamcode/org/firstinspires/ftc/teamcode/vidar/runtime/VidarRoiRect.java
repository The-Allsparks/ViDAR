package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.opencv.core.Rect;

/**
 * Axis-aligned region of interest in full-frame pixel coordinates.
 * Supports overlapping ROIs and enable/disable without removing configuration.
 */
public final class VidarRoiRect {

    public final int x;
    public final int y;
    public final int width;
    public final int height;
    public final boolean enabled;

    public VidarRoiRect(int x, int y, int width, int height, boolean enabled) {
        this.x = x;
        this.y = y;
        this.width = Math.max(1, width);
        this.height = Math.max(1, height);
        this.enabled = enabled;
    }

    public VidarRoiRect(int x, int y, int width, int height) {
        this(x, y, width, height, true);
    }

    /** Default element: lower 65%. */
    public static VidarRoiRect lowerFraction(int frameW, int frameH, double fraction) {
        int h = Math.max(1, (int) Math.round(frameH * fraction));
        return new VidarRoiRect(0, frameH - h, frameW, h);
    }

    /** Fraction of frame height from top (0–1). Default tag: upper 65%. */
    public static VidarRoiRect upperFraction(int frameW, int frameH, double fraction) {
        int h = Math.max(1, (int) Math.round(frameH * fraction));
        return new VidarRoiRect(0, 0, frameW, h);
    }

    /** Middle band: starts at startFraction from top, height = bandFraction of frame. */
    public static VidarRoiRect middleBand(int frameW, int frameH, double startFraction, double bandFraction) {
        int y = (int) Math.round(frameH * startFraction);
        int h = Math.max(1, (int) Math.round(frameH * bandFraction));
        return new VidarRoiRect(0, y, frameW, Math.min(h, frameH - y));
    }

    public Rect toOpenCvRect() {
        return new Rect(x, y, width, height);
    }

    public boolean contains(double px, double py) {
        return px >= x && px < x + width && py >= y && py < y + height;
    }

    public boolean touchesBoundary(int frameW, int frameH, double px, double py, double margin) {
        return px <= x + margin || px >= x + width - margin
                || py <= y + margin || py >= y + height - margin
                || px <= margin || py <= margin
                || px >= frameW - margin || py >= frameH - margin;
    }

    /** Map ROI-local coordinates to full-frame coordinates. */
    public double toFullX(double localX) {
        return localX + x;
    }

    public double toFullY(double localY) {
        return localY + y;
    }

    public VidarRoiRect clamped(int frameW, int frameH) {
        int cx = Math.max(0, Math.min(x, frameW - 1));
        int cy = Math.max(0, Math.min(y, frameH - 1));
        int cw = Math.min(width, frameW - cx);
        int ch = Math.min(height, frameH - cy);
        return new VidarRoiRect(cx, cy, Math.max(1, cw), Math.max(1, ch), enabled);
    }

    public VidarRoiRect withEnabled(boolean on) {
        return new VidarRoiRect(x, y, width, height, on);
    }
}
