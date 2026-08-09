package org.firstinspires.ftc.robotcore.external.navigation;

/** Minimal stub for JVM unit tests. */
public final class Pose2D {
    private final double x;
    private final double y;
    private final double heading;

    public Pose2D(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX(DistanceUnit unit) {
        return x;
    }

    public double getY(DistanceUnit unit) {
        return y;
    }

    public double getHeading(AngleUnit unit) {
        return heading;
    }
}
