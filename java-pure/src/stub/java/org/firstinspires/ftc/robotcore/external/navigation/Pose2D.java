package org.firstinspires.ftc.robotcore.external.navigation;

/** Minimal stub for JVM unit tests — stores inches and radians internally. */
public final class Pose2D {
    private final double xInches;
    private final double yInches;
    private final double headingRadians;

    public Pose2D(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double heading) {
        this.xInches = toInches(distanceUnit, x);
        this.yInches = toInches(distanceUnit, y);
        this.headingRadians = toRadians(angleUnit, heading);
    }

    public double getX(DistanceUnit unit) {
        return fromInches(unit, xInches);
    }

    public double getY(DistanceUnit unit) {
        return fromInches(unit, yInches);
    }

    public double getHeading(AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            return Math.toDegrees(headingRadians);
        }
        return headingRadians;
    }

    private static double toInches(DistanceUnit unit, double value) {
        if (unit == DistanceUnit.MM) {
            return value / 25.4;
        }
        if (unit == DistanceUnit.CM) {
            return value / 2.54;
        }
        if (unit == DistanceUnit.METER) {
            return value / 0.0254;
        }
        return value; // INCH
    }

    private static double fromInches(DistanceUnit unit, double inches) {
        if (unit == DistanceUnit.MM) {
            return inches * 25.4;
        }
        if (unit == DistanceUnit.CM) {
            return inches * 2.54;
        }
        if (unit == DistanceUnit.METER) {
            return inches * 0.0254;
        }
        return inches;
    }

    private static double toRadians(AngleUnit unit, double heading) {
        if (unit == AngleUnit.DEGREES) {
            return Math.toRadians(heading);
        }
        return heading;
    }
}
