package org.firstinspires.ftc.teamcode.vidar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Uncertainty-weighted range fusion result with component estimates exposed for telemetry.
 */
public final class VidarRangeResult {

    public final double distanceIn;
    public final double uncertaintyIn;
    public final double confidence;
    public final List<VidarRangeEstimate> sources;

    public VidarRangeResult(double distanceIn, double uncertaintyIn, double confidence,
                            List<VidarRangeEstimate> sources) {
        this.distanceIn = distanceIn;
        this.uncertaintyIn = uncertaintyIn;
        this.confidence = confidence;
        this.sources = Collections.unmodifiableList(new ArrayList<>(sources));
    }

    public static VidarRangeResult invalid() {
        return new VidarRangeResult(Double.NaN, Double.NaN, 0, Collections.emptyList());
    }

    public boolean isValid() {
        return !Double.isNaN(distanceIn) && distanceIn > 0 && confidence > 0;
    }

    public double sourceDistance(VidarRangeEstimate.Source source) {
        for (VidarRangeEstimate est : sources) {
            if (est.source == source && est.isValid()) {
                return est.distanceIn;
            }
        }
        return Double.NaN;
    }

    public double sourceWeight(VidarRangeEstimate.Source source) {
        for (VidarRangeEstimate est : sources) {
            if (est.source == source && est.isValid()) {
                return est.weight;
            }
        }
        return 0;
    }
}
