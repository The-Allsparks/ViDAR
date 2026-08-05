package org.firstinspires.ftc.teamcode.vidar;

/**
 * One range estimate with uncertainty weight for fusion.
 */
public final class VidarRangeEstimate {

    public enum Source {
        SIZE,
        FLOOR,
        PLATE_WIDTH
    }

    public final Source source;
    public final double distanceIn;
    /** Inverse variance weight (higher = more trusted). 0 = rejected. */
    public final double weight;
    public final double uncertaintyIn;
    public final String rejectionReason;

    public VidarRangeEstimate(Source source, double distanceIn, double weight, double uncertaintyIn) {
        this(source, distanceIn, weight, uncertaintyIn, null);
    }

    public VidarRangeEstimate(Source source, double distanceIn, double weight,
                              double uncertaintyIn, String rejectionReason) {
        this.source = source;
        this.distanceIn = distanceIn;
        this.weight = weight;
        this.uncertaintyIn = uncertaintyIn;
        this.rejectionReason = rejectionReason;
    }

    public boolean isValid() {
        return weight > 0 && !Double.isNaN(distanceIn) && distanceIn > 0
                && rejectionReason == null;
    }

    public static VidarRangeEstimate rejected(Source source, String reason) {
        return new VidarRangeEstimate(source, Double.NaN, 0, Double.NaN, reason);
    }
}
