package org.firstinspires.ftc.teamcode.vidar;

/**
 * Map scout tag width to {@link org.firstinspires.ftc.vision.apriltag.AprilTagProcessor} decimation.
 * Higher decimation = faster / less CPU (official SDK downsample).
 */
public final class VidarTagCropPlanner {

    private VidarTagCropPlanner() {}

    public static int chooseDecimation(double scoutWidthPx, int scoutFrameWidth, int fullFrameWidth) {
        if (scoutFrameWidth <= 0) {
            return VidarTagConfig.DECIMATION_MAX;
        }
        double scale = (double) fullFrameWidth / scoutFrameWidth;
        double fullWidthEst = scoutWidthPx * scale;
        if (fullWidthEst >= 80) {
            return VidarTagConfig.DECIMATION_MAX;
        }
        if (fullWidthEst >= 45) {
            return Math.max(VidarTagConfig.DECIMATION_MIN, 2);
        }
        return VidarTagConfig.DECIMATION_MIN;
    }

    public static boolean worthDecode(double scoutWidthPx, int scoutFrameWidth, int fullFrameWidth) {
        if (scoutFrameWidth <= 0) {
            return false;
        }
        double scale = (double) fullFrameWidth / scoutFrameWidth;
        return scoutWidthPx * scale >= VidarTagConfig.DECODE_MIN_TAG_WIDTH_PX;
    }
}
