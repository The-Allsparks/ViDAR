package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarOdomHistory;

/**
 * Timestamped robot field-pose lookup with explicit stale-sample behavior.
 *
 * <p>Wraps {@link VidarOdomHistory} for transform-at-capture queries. Interpolation is used only
 * when bracketing samples exist; otherwise nearest sample is returned.
 */
public final class VidarPoseLookup {

    public static final class LookupResult {
        public final Pose2D pose;
        public final boolean interpolated;
        public final boolean stale;
        public final double ageMs;
        public final String failureReason;

        LookupResult(Pose2D pose, boolean interpolated, boolean stale, double ageMs, String failureReason) {
            this.pose = pose;
            this.interpolated = interpolated;
            this.stale = stale;
            this.ageMs = ageMs;
            this.failureReason = failureReason;
        }

        public boolean isValid() {
            return pose != null && failureReason == null;
        }
    }

    private final VidarOdomHistory history;
    private final double maxAgeMs;
    private int lookupFailures;
    private int staleRejections;

    public VidarPoseLookup(VidarOdomHistory history) {
        this(history, 500.0);
    }

    public VidarPoseLookup(VidarOdomHistory history, double maxAgeMs) {
        this.history = history;
        this.maxAgeMs = maxAgeMs;
    }

    /**
     * Lookup robot field pose at {@code captureTimeNanos} (VisionPortal callback/receipt time —
     * not exposure start unless the platform provides that).
     */
    public LookupResult atCapture(long captureTimeNanos) {
        if (history == null || captureTimeNanos <= 0) {
            lookupFailures++;
            return new LookupResult(null, false, false, Double.NaN, "no_history_or_time");
        }
        double ageMs = (System.nanoTime() - captureTimeNanos) / 1_000_000.0;
        if (ageMs > maxAgeMs) {
            staleRejections++;
            return new LookupResult(null, false, true, ageMs, "stale_observation");
        }
        Pose2D pose = history.at(captureTimeNanos);
        if (pose == null) {
            lookupFailures++;
            return new LookupResult(null, false, ageMs > maxAgeMs * 0.8, ageMs, "no_pose_sample");
        }
        boolean interpolated = history.size() >= 2;
        return new LookupResult(pose, interpolated, false, ageMs, null);
    }

    public int lookupFailures() {
        return lookupFailures;
    }

    public int staleRejections() {
        return staleRejections;
    }

    public void resetDiagnostics() {
        lookupFailures = 0;
        staleRejections = 0;
    }
}
