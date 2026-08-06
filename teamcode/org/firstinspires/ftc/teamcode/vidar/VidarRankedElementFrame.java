package org.firstinspires.ftc.teamcode.vidar;

/**
 * Up to N element observations from one camera frame, ranked 0 (best) … n−1.
 * {@link #overflowCount()} counts detections scored but not retained.
 */
public final class VidarRankedElementFrame {

    private final VidarElementObservation[] ranked;
    private final int count;
    private final int overflowCount;
    private final long captureTimeNanos;
    private final String cameraName;
    private final int capacity;

    public VidarRankedElementFrame(
            VidarElementObservation[] ranked,
            int count,
            int overflowCount,
            long captureTimeNanos,
            String cameraName) {
        this(ranked, count, overflowCount, captureTimeNanos, cameraName,
                ranked == null ? 0 : ranked.length);
    }

    public VidarRankedElementFrame(
            VidarElementObservation[] ranked,
            int count,
            int overflowCount,
            long captureTimeNanos,
            String cameraName,
            int capacity) {
        this.ranked = ranked;
        this.capacity = Math.max(0, capacity);
        this.count = Math.max(0, Math.min(count, this.capacity));
        this.overflowCount = Math.max(0, overflowCount);
        this.captureTimeNanos = captureTimeNanos;
        this.cameraName = cameraName == null ? "" : cameraName;
    }

    public static VidarRankedElementFrame empty(String cameraName, int capacity) {
        int cap = Math.max(1, capacity);
        return new VidarRankedElementFrame(
                new VidarElementObservation[cap], 0, 0, 0, cameraName, cap);
    }

    public VidarElementObservation at(int rank) {
        if (rank < 0 || rank >= count || ranked == null) {
            return null;
        }
        return ranked[rank];
    }

    public int count() {
        return count;
    }

    public int capacity() {
        return capacity;
    }

    public int overflowCount() {
        return overflowCount;
    }

    public long captureTimeNanos() {
        return captureTimeNanos;
    }

    public String cameraName() {
        return cameraName;
    }

    public VidarElementObservation best() {
        return at(0);
    }
}
