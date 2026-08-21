package org.firstinspires.ftc.teamcode.vidar.runtime;

import java.util.Arrays;

/**
 * Fixed-capacity ring of latency samples with O(1) record and reused scratch for percentiles.
 *
 * <p>No per-sample allocation. Query methods sort a preallocated scratch copy — call them at
 * telemetry rates, not inside every vision callback.
 */
public final class VidarLatencyWindow {

    private final long[] samples;
    private final long[] scratch;
    private int next;
    private int count;

    public VidarLatencyWindow(int capacity) {
        if (capacity < 1) {
            throw new IllegalArgumentException("capacity must be >= 1");
        }
        this.samples = new long[capacity];
        this.scratch = new long[capacity];
    }

    /** Record a duration in nanoseconds (negative values are clamped to 0). */
    public synchronized void record(long durationNanos) {
        long value = durationNanos < 0L ? 0L : durationNanos;
        samples[next] = value;
        next = (next + 1) % samples.length;
        if (count < samples.length) {
            count++;
        }
    }

    public synchronized int sampleCount() {
        return count;
    }

    public synchronized void clear() {
        next = 0;
        count = 0;
        Arrays.fill(samples, 0L);
    }

    /** Median of the current window in milliseconds, or 0 if empty. */
    public double p50Ms() {
        return percentileMs(0.50);
    }

    /** 95th percentile of the current window in milliseconds, or 0 if empty. */
    public double p95Ms() {
        return percentileMs(0.95);
    }

    /** Maximum sample in the current window in milliseconds, or 0 if empty. */
    public synchronized double maxMs() {
        if (count == 0) {
            return 0.0;
        }
        long max = samples[0];
        for (int i = 1; i < count; i++) {
            if (samples[i] > max) {
                max = samples[i];
            }
        }
        return max / 1_000_000.0;
    }

    /**
     * Percentile in {@code [0, 1]} over the current window, returned in milliseconds.
     * Empty window returns 0.
     */
    public synchronized double percentileMs(double percentile) {
        if (count == 0) {
            return 0.0;
        }
        double p = percentile;
        if (p < 0.0) {
            p = 0.0;
        } else if (p > 1.0) {
            p = 1.0;
        }
        System.arraycopy(samples, 0, scratch, 0, count);
        Arrays.sort(scratch, 0, count);
        int index = (int) Math.ceil(p * count) - 1;
        if (index < 0) {
            index = 0;
        } else if (index >= count) {
            index = count - 1;
        }
        return scratch[index] / 1_000_000.0;
    }
}
