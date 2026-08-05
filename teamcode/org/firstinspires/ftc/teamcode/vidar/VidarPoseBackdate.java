package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Extrapolate a field pose measured at image capture to the current time using odometry delta.
 */
public final class VidarPoseBackdate {

    private VidarPoseBackdate() {}

    /**
     * Apply dead-reckoning change since capture so localization uses the tag timestamp, not decode time.
     *
     * <p>field_now ≈ field_at_capture + (odom_now - odom_at_capture) in field axes.
     */
    public static Pose2D fieldPoseNow(
            VidarTagObservation tag,
            Pose2D odomNow,
            DistanceUnit distanceUnit,
            AngleUnit angleUnit) {
        if (tag == null || tag.fieldPoseAtCapture == null) {
            return null;
        }
        if (tag.odomPoseAtCapture == null || odomNow == null) {
            return tag.fieldPoseAtCapture;
        }

        double dx = odomNow.getX(distanceUnit) - tag.odomPoseAtCapture.getX(distanceUnit);
        double dy = odomNow.getY(distanceUnit) - tag.odomPoseAtCapture.getY(distanceUnit);
        double dh = odomNow.getHeading(angleUnit) - tag.odomPoseAtCapture.getHeading(angleUnit);

        return new Pose2D(
                distanceUnit,
                tag.fieldPoseAtCapture.getX(distanceUnit) + dx,
                tag.fieldPoseAtCapture.getY(distanceUnit) + dy,
                angleUnit,
                tag.fieldPoseAtCapture.getHeading(angleUnit) + dh);
    }

    /** Nanoseconds elapsed since the tag frame was captured. */
    public static double ageSeconds(VidarTagObservation tag) {
        if (tag == null) {
            return Double.NaN;
        }
        return (System.nanoTime() - tag.captureTimeNanos) / 1e9;
    }

    public static Pose2D scoutLandmarkNow(
            VidarScoutLandmarkObservation scout,
            Pose2D odomNow,
            DistanceUnit distanceUnit,
            AngleUnit angleUnit) {
        if (scout == null || scout.fieldPoseAtCapture == null) {
            return null;
        }
        if (scout.odomPoseAtCapture == null || odomNow == null) {
            return scout.fieldPoseAtCapture;
        }

        double dx = odomNow.getX(distanceUnit) - scout.odomPoseAtCapture.getX(distanceUnit);
        double dy = odomNow.getY(distanceUnit) - scout.odomPoseAtCapture.getY(distanceUnit);
        double dh = odomNow.getHeading(angleUnit) - scout.odomPoseAtCapture.getHeading(angleUnit);

        return new Pose2D(
                distanceUnit,
                scout.fieldPoseAtCapture.getX(distanceUnit) + dx,
                scout.fieldPoseAtCapture.getY(distanceUnit) + dy,
                angleUnit,
                scout.fieldPoseAtCapture.getHeading(angleUnit) + dh);
    }

    public static double ageSeconds(VidarScoutLandmarkObservation scout) {
        if (scout == null) {
            return Double.NaN;
        }
        return (System.nanoTime() - scout.captureTimeNanos) / 1e9;
    }
}
