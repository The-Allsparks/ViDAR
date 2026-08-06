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
    public final double[] floorDist;
    /** Camera lens position in robot frame (inches): +X forward, +Y left, +Z up from floor. */
    public final double mountX;
    public final double mountY;
    public final double mountZ;
    /** Mount orientation: yaw about optical axis, pitch (negative = inclined down), roll. */
    public final double mountYawDeg;
    public final double mountPitchDeg;
    public final double mountRollDeg;
    /** Known physical plate width for size-based ranging (inches). */
    public final double plateWidth;
    public final VidarCameraRoiConfig roiConfig;

    public VidarCameraProfile(
            String name,
            double bearingDeg,
            int horizonRowPx,
            double focalLengthPx,
            double[] floorCyPx,
            double[] floorDist) {
        this(name, bearingDeg, horizonRowPx, focalLengthPx, floorCyPx, floorDist, 0, 0);
    }

    public VidarCameraProfile(
            String name,
            double bearingDeg,
            int horizonRowPx,
            double focalLengthPx,
            double[] floorCyPx,
            double[] floorDist,
            double mountX,
            double mountY) {
        this(name, bearingDeg, horizonRowPx, focalLengthPx, focalLengthPx,
                320, 240, 70, 55, floorCyPx, floorDist,
                mountX, mountY, 0, 0, 0, 0, 12.0, VidarCameraRoiConfig.DEFAULT);
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
            double[] floorDist,
            double mountX,
            double mountY,
            double mountZ,
            double mountYawDeg,
            double mountPitchDeg,
            double mountRollDeg,
            double plateWidth,
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
        this.floorDist = floorDist;
        this.mountX = mountX;
        this.mountY = mountY;
        this.mountZ = mountZ;
        this.mountYawDeg = mountYawDeg;
        this.mountPitchDeg = mountPitchDeg;
        this.mountRollDeg = mountRollDeg;
        this.plateWidth = plateWidth;
        this.roiConfig = roiConfig == null ? VidarCameraRoiConfig.DEFAULT : roiConfig;
    }

    /** Single-camera teaching default — front-facing, matches sim/vidar-tuning.json. */
    public static final VidarCameraProfile FRONT;

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

    static {
        FRONT = FOUR_SIDES[0];
    }

    private static VidarCameraProfile buildSide(String name, double bearing, double mountX, double mountY) {
        return new VidarCameraProfile(
                name, bearing, 12, 340, 340, 320, 240, 70, 55,
                new double[] {95, 75, 55, 40},
                new double[] {12, 24, 36, 48},
                mountX, mountY, 9.0, 0, -12, 0, 12.0,
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
        if (floorCyPx == null || floorDist == null || floorCyPx.length == 0) {
            warnings.add(name + ": floor LUT missing");
        }
        if (plateWidth <= 0) {
            warnings.add(name + ": plateWidth not configured");
        }
        VidarRoiRect element = roiConfig.elementRoi(frameW, frameH);
        if (element.width <= 0 || element.height <= 0) {
            warnings.add(name + ": invalid element ROI");
        }
        return warnings;
    }

    public int horizonRowFullFrame(int frameH) {
        VidarRoiRect element = roiConfig.elementRoi((int) Math.round(frameH * 4.0 / 3.0), frameH);
        return element.y + horizonRowPx;
    }
}
