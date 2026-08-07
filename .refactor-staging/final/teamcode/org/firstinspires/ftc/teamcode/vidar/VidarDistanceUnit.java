package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

/**
 * Linear distance unit for ViDAR config and observations.
 *
 * <p>All distance fields in JSON and observation DTOs use the <em>active</em> unit
 * ({@link #effective(VidarRobotConfig, VidarSeasonConfig)}). Set {@code distanceUnit} in JSON
 * for SI/metric; field names carry no unit suffix.
 */
public enum VidarDistanceUnit {

    IN(DistanceUnit.INCH, "in", 0.0254),
    CM(DistanceUnit.CM, "cm", 0.01),
    M(DistanceUnit.METER, "m", 1.0);

    public final DistanceUnit ftcUnit;
    public final String suffix;
    private final double metersPerUnit;

    VidarDistanceUnit(DistanceUnit ftcUnit, String suffix, double metersPerUnit) {
        this.ftcUnit = ftcUnit;
        this.suffix = suffix;
        this.metersPerUnit = metersPerUnit;
    }

    /** Parse JSON {@code distanceUnit} value (default {@link #IN} if null/blank). */
    public static VidarDistanceUnit fromJson(String raw) {
        if (raw == null || raw.trim().isEmpty()) {
            return IN;
        }
        switch (raw.trim().toLowerCase()) {
            case "in":
            case "inch":
            case "inches":
                return IN;
            case "m":
            case "meter":
            case "meters":
                return M;
            case "cm":
            case "centimeter":
            case "centimeters":
                return CM;
            default:
                throw new IllegalArgumentException("Unknown distanceUnit: " + raw);
        }
    }

    public double toMeters(double value) {
        return value * metersPerUnit;
    }

    public double fromMeters(double meters) {
        return meters / metersPerUnit;
    }

    public double convert(double value, VidarDistanceUnit to) {
        if (to == null || to == this) {
            return value;
        }
        return to.fromMeters(toMeters(value));
    }

    /** Robot JSON override wins when set; otherwise season; otherwise inches. */
    public static VidarDistanceUnit effective(VidarRobotConfig robot, VidarSeasonConfig season) {
        if (robot != null && robot.distanceUnitOverride != null) {
            return robot.distanceUnitOverride;
        }
        if (season != null) {
            return season.distanceUnit;
        }
        return IN;
    }

    public String format(double value) {
        if (Double.isNaN(value)) {
            return "—";
        }
        if (this == M) {
            return String.format("%.2f %s", value, suffix);
        }
        if (this == CM) {
            return String.format("%.1f %s", value, suffix);
        }
        return String.format("%.0f %s", value, suffix);
    }
}
