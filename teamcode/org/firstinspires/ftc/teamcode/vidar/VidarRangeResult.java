package org.firstinspires.ftc.teamcode.vidar;

/**
 * Uncertainty-weighted range fusion result with component estimates exposed for telemetry.
 * Stores up to two contributing estimates without list allocation.
 */
public final class VidarRangeResult {

    public final double distance;
    public final double uncertainty;
    public final double confidence;
    public final VidarRangeEstimate source0;
    public final VidarRangeEstimate source1;
    public final int sourceCount;

    public VidarRangeResult(
            double distance,
            double uncertainty,
            double confidence,
            VidarRangeEstimate source0,
            VidarRangeEstimate source1,
            int sourceCount) {
        this.distance = distance;
        this.uncertainty = uncertainty;
        this.confidence = confidence;
        this.source0 = source0;
        this.source1 = source1;
        this.sourceCount = Math.max(0, Math.min(sourceCount, 2));
    }

    public static VidarRangeResult invalid() {
        return new VidarRangeResult(Double.NaN, Double.NaN, 0, null, null, 0);
    }

    public boolean isValid() {
        return !Double.isNaN(distance) && distance > 0 && confidence > 0;
    }

    public double sourceDistance(VidarRangeEstimate.Source source) {
        if (source0 != null && source0.source == source && source0.isValid()) {
            return source0.distance;
        }
        if (sourceCount > 1 && source1 != null && source1.source == source && source1.isValid()) {
            return source1.distance;
        }
        return Double.NaN;
    }

    public double sourceWeight(VidarRangeEstimate.Source source) {
        if (source0 != null && source0.source == source && source0.isValid()) {
            return source0.weight;
        }
        if (sourceCount > 1 && source1 != null && source1.source == source && source1.isValid()) {
            return source1.weight;
        }
        return 0;
    }
}
