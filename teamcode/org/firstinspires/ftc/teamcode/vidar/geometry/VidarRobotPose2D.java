package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * Shared robot-frame floor helpers (+X forward, +Y left).
 */
public final class VidarRobotPose2D {

    private VidarRobotPose2D() {}

    public static double bearingDeg(double robotX, double robotY) {
        return Math.toDegrees(Math.atan2(robotY, robotX));
    }

    public static double distance(double robotX, double robotY) {
        return Math.hypot(robotX, robotY);
    }

    public static double separation(double x1, double y1, double x2, double y2) {
        return Math.hypot(x1 - x2, y1 - y2);
    }

    public static boolean withinRadius(
            double x1, double y1, double x2, double y2, double radius) {
        return separation(x1, y1, x2, y2) <= radius;
    }

    /** True if {@code candidate} is within {@code radius} of any point in {@code kept}. */
    public static boolean isDuplicate(
            double candidateX,
            double candidateY,
            Iterable<? extends RobotPoint> kept,
            double radius) {
        for (RobotPoint other : kept) {
            if (withinRadius(candidateX, candidateY, other.robotX(), other.robotY(), radius)) {
                return true;
            }
        }
        return false;
    }

    /** Minimal robot-frame position for proximity checks. */
    public interface RobotPoint {
        double robotX();
        double robotY();
    }
}
