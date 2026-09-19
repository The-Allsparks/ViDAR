package org.firstinspires.ftc.teamcode.vidar.runtime;

/**
 * Optional observer of per-camera ViDAR metrics. TRACE or tests implement this.
 * ViDAR does not import TRACE. Default is {@link #NOOP}.
 *
 * <p>Called from {@link VidarMetricsLogger#recordCycle} on the vision/OpMode
 * thread. Must not block, write files, or command motors.
 */
public interface VidarMetricsSink {
    void onMetrics(VidarMetrics metrics);

    VidarMetricsSink NOOP = new VidarMetricsSink() {
        @Override
        public void onMetrics(VidarMetrics metrics) {
            // intentionally empty
        }
    };
}
