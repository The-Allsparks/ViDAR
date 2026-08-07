package org.firstinspires.ftc.teamcode.vidar.schedule;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;
/**
 * Graceful degradation based on measured vision-loop CPU and frame age.
 *
 * <p>Ladder (when {@link VidarConfig#RESOURCE_BUDGET_ENABLED}):
 * <ol>
 *   <li>{@link DegradationLevel#REDUCE_SECONDARY} — high frame age → smaller ROI</li>
 *   <li>{@link DegradationLevel#DISABLE_LOCAL_HOUGH} — higher frame age → skip local Hough</li>
 *   <li>{@link DegradationLevel#REDUCE_TAG} — loop CPU over budget → no tag decode</li>
 *   <li>{@link DegradationLevel#REDUCE_RESOLUTION} — loop CPU 1.1× budget → minimum ROI</li>
 *   <li>{@link DegradationLevel#DISABLE_PLATES} — loop CPU 1.25× budget → element-only contour pass</li>
 *   <li>{@link DegradationLevel#IDLE_REAR_CAMERAS} — loop CPU 1.5× budget → optional rear idle
 *       (requires {@link VidarConfig#RESOURCE_BUDGET_AUTO_IDLE_REAR})</li>
 * </ol>
 */
public final class VidarResourceBudget {

    public enum DegradationLevel {
        NONE,
        REDUCE_SECONDARY,
        DISABLE_LOCAL_HOUGH,
        REDUCE_TAG,
        REDUCE_RESOLUTION,
        DISABLE_PLATES,
        IDLE_REAR_CAMERAS
    }

    private volatile DegradationLevel level = DegradationLevel.NONE;
    private double lastLoopCpuMs;
    private double lastFrameAgeMs;
    private int connectedCameras;

    public void update(VidarMetrics[] metrics, int connected) {
        connectedCameras = connected;
        double maxLoop = 0;
        double maxAge = 0;
        if (metrics != null) {
            for (VidarMetrics m : metrics) {
                if (m == null) {
                    continue;
                }
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

        double budget = VidarConfig.DEGRADATION_LOOP_BUDGET_MS;
        if (maxLoop > budget * 1.5) {
            level = DegradationLevel.IDLE_REAR_CAMERAS;
        } else if (maxLoop > budget * 1.25) {
            level = DegradationLevel.DISABLE_PLATES;
        } else if (maxLoop > budget * 1.1) {
            level = DegradationLevel.REDUCE_RESOLUTION;
        } else if (maxLoop > budget) {
            level = DegradationLevel.REDUCE_TAG;
        } else if (maxAge > 180) {
            level = DegradationLevel.DISABLE_LOCAL_HOUGH;
        } else if (maxAge > 120) {
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

    public boolean shouldDisablePlates() {
        return level.ordinal() >= DegradationLevel.DISABLE_PLATES.ordinal();
    }

    public boolean shouldIdleRearCameras() {
        return level == DegradationLevel.IDLE_REAR_CAMERAS
                && VidarConfig.RESOURCE_BUDGET_AUTO_IDLE_REAR;
    }

    public double processingRoiScale() {
        if (level.ordinal() >= DegradationLevel.REDUCE_RESOLUTION.ordinal()) {
            return VidarConfig.DEGRADED_PROCESS_ROI_SCALE;
        }
        if (level.ordinal() >= DegradationLevel.REDUCE_SECONDARY.ordinal()) {
            return VidarConfig.MEDIUM_PROCESS_ROI_SCALE;
        }
        return VidarConfig.PROCESS_ROI_SCALE;
    }
}
