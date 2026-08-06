package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarTagObservation;

/**
 * Explicit AprilTag localization transform chain.
 *
 * <p>The FTC SDK {@link org.firstinspires.ftc.vision.apriltag.AprilTagProcessor} reports
 * {@code field_T_robot} at capture time. ViDAR documents the equivalent chain:
 *
 * <pre>
 * field_T_robot = field_T_cameraOptical * cameraOptical_T_robot
 * </pre>
 *
 * <p>On-robot, {@code field_T_robot} comes directly from the SDK decode. This helper validates
 * consistency with configured {@code robot_T_camera} when a synthetic camera pose is known.
 */
public final class VidarAprilTagTransforms {

    public static final class ChainResult {
        public final Pose2D fieldTRobotFromSdk;
        public final Pose2D fieldTRobotFromChain;
        public final double translationErrorIn;
        public final double headingErrorDeg;
        public final boolean consistent;

        ChainResult(
                Pose2D fieldTRobotFromSdk,
                Pose2D fieldTRobotFromChain,
                double translationErrorIn,
                double headingErrorDeg,
                boolean consistent) {
            this.fieldTRobotFromSdk = fieldTRobotFromSdk;
            this.fieldTRobotFromChain = fieldTRobotFromChain;
            this.translationErrorIn = translationErrorIn;
            this.headingErrorDeg = headingErrorDeg;
            this.consistent = consistent;
        }
    }

    private VidarAprilTagTransforms() {}

    /**
     * Documented chain for tests: given {@code field_T_camera} (3D pose of camera in field) and
     * {@code robot_T_camera}, derive {@code field_T_robot} for a planar robot on the floor.
     *
     * <p>Simplified 2D floor projection: robot pose is the camera pose adjusted by mount offset
     * in the horizontal plane.
     */
    public static Pose2D fieldTRobotFromChain(
            Pose2D fieldTCamera2d,
            VidarTransform3D robotTCamera) {
        if (fieldTCamera2d == null || robotTCamera == null) {
            return null;
        }
        // camera origin in robot frame
        double ox = robotTCamera.translation.x;
        double oy = robotTCamera.translation.y;
        double headingRad = Math.toRadians(fieldTCamera2d.getHeading(AngleUnit.DEGREES));
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        // robot origin relative to camera in field frame (inverse horizontal offset)
        double robotFx = fieldTCamera2d.getX(DistanceUnit.INCH)
                - (ox * cos - oy * sin);
        double robotFy = fieldTCamera2d.getY(DistanceUnit.INCH)
                - (ox * sin + oy * cos);
        return new Pose2D(
                DistanceUnit.INCH,
                robotFx,
                robotFy,
                AngleUnit.DEGREES,
                fieldTCamera2d.getHeading(AngleUnit.DEGREES));
    }

    public static ChainResult validateChain(
            VidarTagObservation decoded,
            Pose2D fieldTCameraEstimate,
            VidarTransformRegistry.CameraTransforms transforms,
            double maxTranslationErrorIn,
            double maxHeadingErrorDeg) {
        if (decoded == null || decoded.fieldPoseAtCapture == null) {
            return new ChainResult(null, null, Double.NaN, Double.NaN, false);
        }
        Pose2D fromChain = fieldTRobotFromChain(fieldTCameraEstimate, transforms.robotTCamera);
        if (fromChain == null) {
            return new ChainResult(decoded.fieldPoseAtCapture, null, Double.NaN, Double.NaN, false);
        }
        double dx = decoded.fieldPoseAtCapture.getX(DistanceUnit.INCH)
                - fromChain.getX(DistanceUnit.INCH);
        double dy = decoded.fieldPoseAtCapture.getY(DistanceUnit.INCH)
                - fromChain.getY(DistanceUnit.INCH);
        double transErr = Math.hypot(dx, dy);
        double headErr = Math.abs(normalizeDeg(
                decoded.fieldPoseAtCapture.getHeading(AngleUnit.DEGREES)
                        - fromChain.getHeading(AngleUnit.DEGREES)));
        boolean ok = transErr <= maxTranslationErrorIn && headErr <= maxHeadingErrorDeg;
        return new ChainResult(decoded.fieldPoseAtCapture, fromChain, transErr, headErr, ok);
    }

    /** Returns SDK field pose — scout observations must never be passed here. */
    public static Pose2D localizationPoseFromDecode(VidarTagObservation decoded) {
        if (decoded == null) {
            return null;
        }
        return decoded.fieldPoseAtCapture;
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
