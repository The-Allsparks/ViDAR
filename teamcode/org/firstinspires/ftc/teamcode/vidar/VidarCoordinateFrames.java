package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Helpers for converting ViDAR robot-frame observations to field/world space and back.
 *
 * <p>Observations are backdated — use {@link VidarMotionCorrection} with team odom at capture
 * and odom now before converting to field frame at the current instant.
 */
public final class VidarCoordinateFrames {

    private VidarCoordinateFrames() {}

    /** Robot-frame floor point → field-frame floor point (inches, degrees). */
    public static double[] robotToField(
            double robotX,
            double robotY,
            Pose2D robotFieldPose) {
        if (robotFieldPose == null || Double.isNaN(robotX) || Double.isNaN(robotY)) {
            return new double[] { Double.NaN, Double.NaN };
        }
        double headingRad = Math.toRadians(robotFieldPose.getHeading(AngleUnit.DEGREES));
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double fieldX = robotFieldPose.getX(DistanceUnit.INCH) + robotX * cos - robotY * sin;
        double fieldY = robotFieldPose.getY(DistanceUnit.INCH) + robotX * sin + robotY * cos;
        return new double[] { fieldX, fieldY };
    }

    /** Field-frame floor point → robot-frame floor point. */
    public static double[] fieldToRobot(
            double fieldXIn,
            double fieldYIn,
            Pose2D robotFieldPose) {
        if (robotFieldPose == null || Double.isNaN(fieldXIn) || Double.isNaN(fieldYIn)) {
            return new double[] { Double.NaN, Double.NaN };
        }
        double dx = fieldXIn - robotFieldPose.getX(DistanceUnit.INCH);
        double dy = fieldYIn - robotFieldPose.getY(DistanceUnit.INCH);
        double headingRad = Math.toRadians(robotFieldPose.getHeading(AngleUnit.DEGREES));
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        return new double[] {
                dx * cos + dy * sin,
                -dx * sin + dy * cos
        };
    }

    public static Pose2D robotToFieldPose(Pose2D robotRelativePose, Pose2D robotFieldPose) {
        if (robotRelativePose == null || robotFieldPose == null) {
            return null;
        }
        double[] field = robotToField(
                robotRelativePose.getX(DistanceUnit.INCH),
                robotRelativePose.getY(DistanceUnit.INCH),
                robotFieldPose);
        double heading = robotFieldPose.getHeading(AngleUnit.DEGREES)
                + robotRelativePose.getHeading(AngleUnit.DEGREES);
        return new Pose2D(
                DistanceUnit.INCH, field[0], field[1],
                AngleUnit.DEGREES, heading);
    }

    /** Normalize heading to (-180, 180]. */
    public static double normalizeDeg(double deg) {
        while (deg > 180) {
            deg -= 360;
        }
        while (deg < -180) {
            deg += 360;
        }
        return deg;
    }

    /** Age of an observation relative to {@link System#nanoTime()}. */
    public static double observationAgeMs(long captureTimeNanos) {
        if (captureTimeNanos <= 0) {
            return Double.NaN;
        }
        return (System.nanoTime() - captureTimeNanos) / 1_000_000.0;
    }
}
