package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Extrapolate backdated observations using explicit capture-time and current odometry.
 *
 * @see VidarMotionCorrection
 */
public final class VidarPoseBackdate {

    private VidarPoseBackdate() {}

    /**
     * Tag field pose at {@code odomNow}, corrected from the capture-time fix.
     *
     * @param odomAtCapture team odometry sample at {@code tag.captureTimeNanos}
     */
    public static Pose2D fieldPoseNow(
            VidarTagObservation tag,
            Pose2D odomAtCapture,
            Pose2D odomNow,
            DistanceUnit distanceUnit,
            AngleUnit angleUnit) {
        Pose2D corrected = VidarMotionCorrection.tagFieldNow(tag, odomAtCapture, odomNow);
        if (corrected == null) {
            return null;
        }
        return new Pose2D(
                distanceUnit,
                corrected.getX(DistanceUnit.INCH),
                corrected.getY(DistanceUnit.INCH),
                angleUnit,
                corrected.getHeading(AngleUnit.DEGREES));
    }

    public static double ageSeconds(VidarTagObservation tag) {
        if (tag == null) {
            return Double.NaN;
        }
        return (System.nanoTime() - tag.captureTimeNanos) / 1e9;
    }
}
