package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * AprilTag fix at image capture time in field coordinates.
 * Use {@link VidarCoordinateFrames} and team odometry to relate to the robot.
 *
 * <p>Localization uses {@code field_T_robot} from the FTC SDK at capture time. Equivalent chain:
 * {@code field_T_robot = field_T_cameraOptical * cameraOptical_T_robot} — see
 * {@link org.firstinspires.ftc.teamcode.vidar.geometry.VidarAprilTagTransforms}.
 */
public final class VidarTagObservation {

    public final int tagId;
    /** Robot pose on the field from SDK AprilTagProcessor at capture time ({@code field_T_robot}). */
    public final Pose2D fieldPoseAtCapture;
    public final long captureTimeNanos;
    /** Source camera logical name ({@code front}, {@code right}, …). */
    public final String cameraName;
    public final double centerX;
    public final double centerY;
    public final VidarFrameRegions.HorizontalBand band;
    public final int decimationUsed;
    /** Pixels fed to AprilTag on the last decode (crop area / decimation²). */
    public final int decodePixels;

    public VidarTagObservation(
            int tagId,
            Pose2D fieldPoseAtCapture,
            long captureTimeNanos,
            double centerX,
            double centerY,
            VidarFrameRegions.HorizontalBand band,
            int decimationUsed,
            int decodePixels) {
        this(tagId, fieldPoseAtCapture, captureTimeNanos, null,
                centerX, centerY, band, decimationUsed, decodePixels);
    }

    public VidarTagObservation(
            int tagId,
            Pose2D fieldPoseAtCapture,
            long captureTimeNanos,
            String cameraName,
            double centerX,
            double centerY,
            VidarFrameRegions.HorizontalBand band,
            int decimationUsed,
            int decodePixels) {
        this.tagId = tagId;
        this.fieldPoseAtCapture = fieldPoseAtCapture;
        this.captureTimeNanos = captureTimeNanos;
        this.cameraName = cameraName;
        this.centerX = centerX;
        this.centerY = centerY;
        this.band = band;
        this.decimationUsed = decimationUsed;
        this.decodePixels = decodePixels;
    }
}
