package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Weak field pose from scout geometry + known tag map (no AprilTag decode).
 */
public final class VidarScoutLandmarkObservation {

    public final int tagId;
    public final Pose2D fieldPoseAtCapture;
    public final Pose2D odomPoseAtCapture;
    public final long captureTimeNanos;
    public final String cameraName;
    public final double confidence;
    public final double rangeIn;
    public final double bearingDeg;

    public VidarScoutLandmarkObservation(
            int tagId,
            Pose2D fieldPoseAtCapture,
            Pose2D odomPoseAtCapture,
            long captureTimeNanos,
            String cameraName,
            double confidence,
            double rangeIn,
            double bearingDeg) {
        this.tagId = tagId;
        this.fieldPoseAtCapture = fieldPoseAtCapture;
        this.odomPoseAtCapture = odomPoseAtCapture;
        this.captureTimeNanos = captureTimeNanos;
        this.cameraName = cameraName;
        this.confidence = confidence;
        this.rangeIn = rangeIn;
        this.bearingDeg = bearingDeg;
    }
}
