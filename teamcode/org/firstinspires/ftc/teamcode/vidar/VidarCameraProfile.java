package org.firstinspires.ftc.teamcode.vidar;

/**
 * Fixed mount geometry and per-camera calibration for one side camera.
 * Calibrate intrinsics, horizon, floor LUT, and ROIs on the field.
 */
public final class VidarCameraProfile {

    public final String name;
    /** Compass bearing from robot front: 0 = front, 90 = right, 180 = back, 270 = left. */
    public final double bearingDeg;
    /** Horizon row in process-frame coordinates (smaller y = farther). */
    public final int horizonRowPx;
    /** Horizontal focal length in pixels (primary for width-based ranging). */
    public final double focalLengthPx;
    /** Vertical focal length — defaults to horizontal when not separately calibrated. */
    public final double focalLengthYPx;
    public final double principalPointX;
    public final double principalPointY;
    public final double horizontalFovDeg;
    public final double verticalFovDeg;
    /** Floor distance LUT: row cy in process frame → slant range in inches. */
    public final double[] floorCyPx;
    public final double[] floorDistIn;
    /** Camera lens position in robot frame (inches): +X forward, +Y left from robot center. */
    public final double mountXIn;
    public final double mountYIn;
    public final double mountYawDeg;
    public final double mountPitchDeg;
    public final double mountRollDeg;
    /** Known physical plate width for size-based ranging (inches). */
    public final double plateWidthIn;
    public final VidarCameraRoiConfig roiConfig;

    public VidarCameraProfile(
            String name,
            double bearingDeg,
            int horizonRowPx,
            double focalLengthPx,
            double[] floorCyPx,
            double[] floorDistIn) {
        this(name, bearingDeg, horizonRowPx, focalLengthPx, floorCyPx, floorDistIn, 0, 0);
    }

    public VidarCameraProfile(
            String name,
            double bearingDeg,
            int horizonRowPx,
            double focalLengthPx,
            double[] floorCyPx,
            double[] floorDistIn,
            double mountXIn,
            double mountYIn) {
        this(name, bearingDeg, horizonRowPx, focalLengthPx, focalLengthPx,
                320, 240, 70, 55, floorCyPx, floorDistIn,
                mountXIn, mountYIn, 0, 0, 0, 12.0, VidarCameraRoiConfig.DEFAULT);
    }

    public VidarCameraProfile(
            String name,
            double bearingDeg,
            int horizonRowPx,
            double focalLengthPx,
            double focalLengthYPx,
            double principalPointX,
            double principalPointY,
            double horizontalFovDeg,
            double verticalFovDeg,
            double[] floorCyPx,
            double[] floorDistIn,
            double mountXIn,
            double mountYIn,
            double mountYawDeg,
            double mountPitchDeg,
            double mountRollDeg,
            double plateWidthIn,
            VidarCameraRoiConfig roiConfig) {
        this.name = name;
        this.bearingDeg = bearingDeg;
        this.horizonRowPx = horizonRowPx;
        this.focalLengthPx = focalLengthPx;
        this.focalLengthYPx = focalLengthYPx;
        this.principalPointX = principalPointX;
        this.principalPointY = principalPointY;
        this.horizontalFovDeg = horizontalFovDeg;
        this.verticalFovDeg = verticalFovDeg;
        this.floorCyPx = floorCyPx;
        this.floorDistIn = floorDistIn;
        this.mountXIn = mountXIn;
        this.mountYIn = mountYIn;
        this.mountYawDeg = mountYawDeg;
        this.mountPitchDeg = mountPitchDeg;
        this.mountRollDeg = mountRollDeg;
        this.plateWidthIn = plateWidthIn;
        this.roiConfig = roiConfig == null ? VidarCameraRoiConfig.DEFAULT : roiConfig;
    }

    /** Single-camera teaching default — front-facing, matches sim/vidar-tuning.json. */
    public static final VidarCameraProfile FRONT = FOUR_SIDES[0];

    /**
     * Four side cameras with identical tilt; calibrate each LUT on the field.
     * Mount offsets are example values for a ~13×13 in square robot — measure yours.
     */
    public static final VidarCameraProfile[] FOUR_SIDES = {
            buildSide("front", 0, 6.5, 0),
            buildSide("right", 90, 0, -6.5),
            buildSide("back", 180, -6.5, 0),
            buildSide("left", 270, 0, 6.5),
    };

    private static VidarCameraProfile buildSide(String name, double bearing, double mountX, double mountY) {
        return new VidarCameraProfile(
                name, bearing, 12, 340, 340, 320, 240, 70, 55,
                new double[] {95, 75, 55, 40},
                new double[] {12, 24, 36, 48},
                mountX, mountY, 0, 0, 0, 12.0,
                VidarCameraRoiConfig.DEFAULT);
    }

    public static VidarCameraProfile forIndex(int index) {
        if (index < 0 || index >= FOUR_SIDES.length) {
            return FOUR_SIDES[0];
        }
        return FOUR_SIDES[index];
    }

    /** Validate calibration completeness at startup. Returns warning messages (empty = OK). */
    public java.util.List<String> validate(int frameW, int frameH) {
        java.util.List<String> warnings = new java.util.ArrayList<>();
        if (focalLengthPx <= 0) {
            warnings.add(name + ": focalLengthPx must be positive");
        }
        if (floorCyPx == null || floorDistIn == null || floorCyPx.length == 0) {
            warnings.add(name + ": floor LUT missing");
        }
        if (plateWidthIn <= 0) {
            warnings.add(name + ": plateWidthIn not configured");
        }
        VidarRoiRect ball = roiConfig.ballRoi(frameW, frameH);
        if (ball.width <= 0 || ball.height <= 0) {
            warnings.add(name + ": invalid ball ROI");
        }
        return warnings;
    }

    public int horizonRowFullFrame(int frameH) {
        VidarRoiRect ball = roiConfig.ballRoi((int) Math.round(frameH * 4.0 / 3.0), frameH);
        return ball.y + horizonRowPx;
    }
}
