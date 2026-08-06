package org.firstinspires.ftc.teamcode.vidar;

/**
 * One range estimate with uncertainty weight for fusion.
 */
public final class VidarRangeEstimate {

    public enum Source {
        SIZE,
        FLOOR,
        /** Geometric slant range from mount + intrinsics to a horizontal plane at target height. */
        GROUND_PLANE,
        PLATE_WIDTH
    }

    public final Source source;
    public final double distance;
    /** Inverse variance weight (higher = more trusted). 0 = rejected. */
    public final double weight;
    public final double uncertainty;
    public final String rejectionReason;

    public VidarRangeEstimate(Source source, double distance, double weight, double uncertainty) {
        this(source, distance, weight, uncertainty, null);
    }

    public VidarRangeEstimate(Source source, double distance, double weight,
                              double uncertainty, String rejectionReason) {
        this.source = source;
        this.distance = distance;
        this.weight = weight;
        this.uncertainty = uncertainty;
        this.rejectionReason = rejectionReason;
    }

    public boolean isValid() {
        return weight > 0 && !Double.isNaN(distance) && distance > 0
                && rejectionReason == null;
    }

    public static VidarRangeEstimate rejected(Source source, String reason) {
        return new VidarRangeEstimate(source, Double.NaN, 0, Double.NaN, reason);
    }
}
