package org.firstinspires.ftc.teamcode.vidar.integration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Converts between FTC {@link Pose2D} (ViDAR) and Pedro-native components ({@link VidarPedroPose}).
 *
 * <p>ViDAR stores headings with {@link AngleUnit#DEGREES} on {@link Pose2D}. Pedro {@code Pose}
 * uses radians. Positions are inches on both sides.
 *
 * <p>No Pedro Maven dependency is required - wrap results into {@code com.pedropathing.geometry.Pose}
 * in team code that already depends on Pedro Pathing.
 */
public final class VidarPedroPoseBridge {

    private VidarPedroPoseBridge() {}

    public static VidarPedroPose fromPose2D(Pose2D pose) {
        if (pose == null) {
            return null;
        }
        return new VidarPedroPose(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS));
    }

    public static Pose2D toPose2D(VidarPedroPose pose) {
        if (pose == null) {
            return null;
        }
        return toPose2D(pose.x, pose.y, pose.headingRad);
    }

    public static Pose2D toPose2D(double xIn, double yIn, double headingRad) {
        return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.RADIANS, headingRad);
    }

    /**
     * Adapts a Pedro pose supplier (inches + radians) into a ViDAR {@link Pose2D} supplier for
     * {@link org.firstinspires.ftc.teamcode.vidar.VidarSpatial#create} odom / field priors.
     */
    public static Supplier<Pose2D> asPose2DSupplier(Supplier<VidarPedroPose> pedroPoseSupplier) {
        if (pedroPoseSupplier == null) {
            return null;
        }
        return () -> toPose2D(pedroPoseSupplier.get());
    }

    /**
     * Convenience when team code already exposes Pedro {@code getX()/getY()/getHeading()} via lambdas.
     */
    public static Supplier<Pose2D> asPose2DSupplier(
            DoubleSupplier xIn,
            DoubleSupplier yIn,
            DoubleSupplier headingRad) {
        if (xIn == null || yIn == null || headingRad == null) {
            return null;
        }
        return () -> toPose2D(xIn.getAsDouble(), yIn.getAsDouble(), headingRad.getAsDouble());
    }
}
