package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarCoordinateFrames;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Extrapolate backdated ViDAR observations to the current robot/field frame.
 *
 * <p>ViDAR outputs detections in robot-centric space at {@code captureTimeNanos}. To use them
 * at the current instant, supply <b>both</b> odometry at capture and odometry now — ViDAR never
 * reads odom internally; the team owns pose history.
 *
 * <p>Prefer {@link VidarObservationFrame#toRobotNow(VidarOdomHistory, Pose2D)} for batch correction.
 * Use these static helpers only for custom fusion or one-off transforms.
 */
public final class VidarMotionCorrection {

    private VidarMotionCorrection() {}

    /**
     * AprilTag field pose at {@code odomNow}, corrected from capture-time fix.
     * field_now ≈ field_at_capture + (odom_now − odom_at_capture).
     */
    public static Pose2D tagFieldNow(
            VidarTagObservation tag,
            Pose2D odomAtCapture,
            Pose2D odomNow) {
        if (tag == null || tag.fieldPoseAtCapture == null) {
            return null;
        }
        return robotFieldPoseNow(tag.fieldPoseAtCapture, odomAtCapture, odomNow);
    }

    /** Robot field pose at {@code odomNow} from a known field pose at capture. */
    public static Pose2D robotFieldPoseNow(
            Pose2D fieldPoseAtCapture,
            Pose2D odomAtCapture,
            Pose2D odomNow) {
        if (fieldPoseAtCapture == null) {
            return null;
        }
        if (odomAtCapture == null || odomNow == null) {
            return fieldPoseAtCapture;
        }
        double dx = odomNow.getX(DistanceUnit.INCH) - odomAtCapture.getX(DistanceUnit.INCH);
        double dy = odomNow.getY(DistanceUnit.INCH) - odomAtCapture.getY(DistanceUnit.INCH);
        double dh = VidarCoordinateFrames.normalizeDeg(
                odomNow.getHeading(AngleUnit.DEGREES) - odomAtCapture.getHeading(AngleUnit.DEGREES));
        return new Pose2D(
                DistanceUnit.INCH,
                fieldPoseAtCapture.getX(DistanceUnit.INCH) + dx,
                fieldPoseAtCapture.getY(DistanceUnit.INCH) + dy,
                AngleUnit.DEGREES,
                fieldPoseAtCapture.getHeading(AngleUnit.DEGREES) + dh);
    }

    /** Game element floor point in the current robot frame (+X forward, +Y left). */
    public static double[] elementRobotNow(
            VidarElementObservation obs,
            Pose2D odomAtCapture,
            Pose2D odomNow) {
        if (obs == null) {
            return new double[] { Double.NaN, Double.NaN };
        }
        return robotPointNow(obs.robotX, obs.robotY, odomAtCapture, odomNow);
    }

    /** Alliance plate floor point in the current robot frame. */
    public static double[] plateRobotNow(
            VidarPlateObservation plate,
            Pose2D odomAtCapture,
            Pose2D odomNow) {
        if (plate == null) {
            return new double[] { Double.NaN, Double.NaN };
        }
        return robotPointNow(plate.robotX, plate.robotY, odomAtCapture, odomNow);
    }

    /**
     * Element on the field at {@code odomNow}.
     *
     * @param fieldPoseAtCapture robot field pose when the element frame was captured — often from
     *                           {@link #tagFieldNow} on a recent tag, or fused localization
     */
    public static double[] elementFieldNow(
            VidarElementObservation obs,
            Pose2D odomAtCapture,
            Pose2D odomNow,
            Pose2D fieldPoseAtCapture) {
        double[] robotNow = elementRobotNow(obs, odomAtCapture, odomNow);
        Pose2D robotFieldNow = robotFieldPoseNow(fieldPoseAtCapture, odomAtCapture, odomNow);
        return VidarCoordinateFrames.robotToField(robotNow[0], robotNow[1], robotFieldNow);
    }

    public static double[] plateFieldNow(
            VidarPlateObservation plate,
            Pose2D odomAtCapture,
            Pose2D odomNow,
            Pose2D fieldPoseAtCapture) {
        double[] robotNow = plateRobotNow(plate, odomAtCapture, odomNow);
        Pose2D robotFieldNow = robotFieldPoseNow(fieldPoseAtCapture, odomAtCapture, odomNow);
        return VidarCoordinateFrames.robotToField(robotNow[0], robotNow[1], robotFieldNow);
    }

    /** Motion-correct a robot-frame floor point from capture-time odom to now. */
    public static double[] robotPointNow(
            double robotXAtCapture,
            double robotYAtCapture,
            Pose2D odomAtCapture,
            Pose2D odomNow) {
        if (Double.isNaN(robotXAtCapture) || Double.isNaN(robotYAtCapture)) {
            return new double[] { Double.NaN, Double.NaN };
        }
        if (odomAtCapture == null || odomNow == null) {
            return new double[] { robotXAtCapture, robotYAtCapture };
        }
        VidarMotionTransform motion = VidarMotionTransform.fromOdomDelta(
                odomAtCapture.getX(DistanceUnit.INCH),
                odomAtCapture.getY(DistanceUnit.INCH),
                odomAtCapture.getHeading(AngleUnit.DEGREES),
                odomNow.getX(DistanceUnit.INCH),
                odomNow.getY(DistanceUnit.INCH),
                odomNow.getHeading(AngleUnit.DEGREES));
        return motion.transformPoint(robotXAtCapture, robotYAtCapture);
    }

    public static VidarMotionTransform motionSinceCapture(Pose2D odomAtCapture, Pose2D odomNow) {
        if (odomAtCapture == null || odomNow == null) {
            return VidarMotionTransform.identity();
        }
        return VidarMotionTransform.fromOdomDelta(
                odomAtCapture.getX(DistanceUnit.INCH),
                odomAtCapture.getY(DistanceUnit.INCH),
                odomAtCapture.getHeading(AngleUnit.DEGREES),
                odomNow.getX(DistanceUnit.INCH),
                odomNow.getY(DistanceUnit.INCH),
                odomNow.getHeading(AngleUnit.DEGREES));
    }

}
