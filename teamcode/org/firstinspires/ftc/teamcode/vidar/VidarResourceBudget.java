package org.firstinspires.ftc.teamcode.vidar;

/**
 * Graceful degradation based on measured vision-loop CPU and frame age.
 */
public final class VidarResourceBudget {

    public enum DegradationLevel {
        NONE,
        REDUCE_TAG,
        DISABLE_LOCAL_HOUGH,
        REDUCE_SECONDARY,
        REDUCE_RESOLUTION,
        DISABLE_PLATES,
        IDLE_REAR_CAMERAS
    }

    private DegradationLevel level = DegradationLevel.NONE;
    private double lastLoopCpuMs;
    private double lastFrameAgeMs;
    private int connectedCameras;

    public void update(VidarMetrics[] metrics, int connected) {
        connectedCameras = connected;
        double maxLoop = 0;
        double maxAge = 0;
        if (metrics != null) {
            for (VidarMetrics m : metrics) {
                if (m == null) continue;
                maxLoop = Math.max(maxLoop, m.lastLoopCpuMs());
                maxAge = Math.max(maxAge, m.lastFrameAgeMs());
            }
        }
        lastLoopCpuMs = maxLoop;
        lastFrameAgeMs = maxAge;

        if (!VidarConfig.RESOURCE_BUDGET_ENABLED) {
            level = DegradationLevel.NONE;
            return;
        }

        if (maxLoop > VidarConfig.DEGRADATION_LOOP_BUDGET_MS * 1.5) {
            level = DegradationLevel.IDLE_REAR_CAMERAS;
        } else if (maxLoop > VidarConfig.DEGRADATION_LOOP_BUDGET_MS * 1.25) {
            level = DegradationLevel.DISABLE_PLATES;
        } else if (maxLoop > VidarConfig.DEGRADATION_LOOP_BUDGET_MS) {
            level = DegradationLevel.REDUCE_TAG;
        } else if (maxAge > 200) {
            level = DegradationLevel.REDUCE_SECONDARY;
        } else {
            level = DegradationLevel.NONE;
        }
    }

    public DegradationLevel level() {
        return level;
    }

    public double lastLoopCpuMs() {
        return lastLoopCpuMs;
    }

    public double lastFrameAgeMs() {
        return lastFrameAgeMs;
    }

    public int connectedCameras() {
        return connectedCameras;
    }

    public boolean shouldDisableLocalHough() {
        return level.ordinal() >= DegradationLevel.DISABLE_LOCAL_HOUGH.ordinal();
    }

    public boolean shouldReduceTagFrequency() {
        return level.ordinal() >= DegradationLevel.REDUCE_TAG.ordinal();
    }
}
