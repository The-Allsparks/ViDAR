package org.firstinspires.ftc.teamcode.vidar;

/**
 * Distance conversion helpers. SI (meters) is the interchange unit for cross-language APIs.
 */
public final class VidarUnits {

    private VidarUnits() {}

    public static double convert(double value, VidarDistanceUnit from, VidarDistanceUnit to) {
        if (from == null) {
            from = VidarDistanceUnit.IN;
        }
        if (to == null) {
            to = VidarDistanceUnit.IN;
        }
        return from.convert(value, to);
    }

    public static double toMeters(double value, VidarDistanceUnit unit) {
        return (unit == null ? VidarDistanceUnit.IN : unit).toMeters(value);
    }

    public static double fromMeters(double meters, VidarDistanceUnit unit) {
        return (unit == null ? VidarDistanceUnit.IN : unit).fromMeters(meters);
    }

    public static String format(double value, VidarDistanceUnit unit) {
        return (unit == null ? VidarDistanceUnit.IN : unit).format(value);
    }
}
