package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * AprilTag fix at image capture time. Use {@link VidarPoseBackdate} before applying to odometry.
 */
public final class VidarTagObservation {

    public final int tagId;
    /** Robot pose on the field from SDK AprilTagProcessor at capture time. */
    public final Pose2D fieldPoseAtCapture;
    /** Dead-reckoning pose recorded at the same instant (optional). */
    public final Pose2D odomPoseAtCapture;
    public final long captureTimeNanos;
    public final double centerX;
    public final double centerY;
    public final VidarFrameRegions.HorizontalBand band;
    public final int decimationUsed;
    /** Pixels fed to AprilTag on the last decode (crop area / decimation²). */
    public final int decodePixels;

    public VidarTagObservation(
            int tagId,
            Pose2D fieldPoseAtCapture,
            Pose2D odomPoseAtCapture,
            long captureTimeNanos,
            double centerX,
            double centerY,
            VidarFrameRegions.HorizontalBand band,
            int decimationUsed,
            int decodePixels) {
        this.tagId = tagId;
        this.fieldPoseAtCapture = fieldPoseAtCapture;
        this.odomPoseAtCapture = odomPoseAtCapture;
        this.captureTimeNanos = captureTimeNanos;
        this.centerX = centerX;
        this.centerY = centerY;
        this.band = band;
        this.decimationUsed = decimationUsed;
        this.decodePixels = decodePixels;
    }
}
