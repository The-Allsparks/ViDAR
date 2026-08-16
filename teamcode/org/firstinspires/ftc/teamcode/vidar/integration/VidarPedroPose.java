package org.firstinspires.ftc.teamcode.vidar.integration;

/**
 * Pedro-native pose components without a Pedro Pathing classpath dependency.
 *
 * <p>Matches {@code com.pedropathing.geometry.Pose} conventions used by most FTC teams:
 * field {@code x}/{@code y} in <strong>inches</strong>, heading in <strong>radians</strong>.
 *
 * <p>Construct a Pedro pose in team code with:
 * <pre>{@code
 * Pose pedro = new Pose(vidarPedro.x, vidarPedro.y, vidarPedro.headingRad);
 * }</pre>
 *
 * <p>Assumes the team uses the same field origin/axes for ViDAR {@link
 * org.firstinspires.ftc.robotcore.external.navigation.Pose2D} and Pedro. If you use Pedro
 * coordinate-system converters, convert before/after this DTO.
 */
public final class VidarPedroPose {

    public final double x;
    public final double y;
    /** Heading in radians (Pedro {@code Pose.getHeading()}). */
    public final double headingRad;

    public VidarPedroPose(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    public double headingDeg() {
        return Math.toDegrees(headingRad);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f) %.1f°", x, y, headingDeg());
    }
}
