package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.firstinspires.ftc.teamcode.vidar.VidarMultiVision;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Cumulative run metrics for post-match review and bottleneck identification.
 * Call {@link #recordCycle(VidarMetrics[])} once per {@link VidarMultiVision#update()}.
 */
public final class VidarMetricsLogger {

    private long cycleCount;
    private long totalDroppedFrames;
    private long totalStaleFrames;
    private long totalSkippedSlots;
    private long totalElementOverflow;
    private long totalDroppedDecodeJobs;
    private long totalProcessedElementFrames;
    private double peakLoopCpuMs;
    private double peakFrameAgeMs;
    private double peakTagDecodeMs;
    private long failedHealthSamples;

    public void recordCycle(VidarMetrics[] metrics) {
        cycleCount++;
        if (metrics == null) {
            return;
        }
        for (VidarMetrics m : metrics) {
            if (m == null) {
                continue;
            }
            totalDroppedFrames += m.droppedFramesDelta();
            totalStaleFrames += m.staleFramesDelta();
            totalSkippedSlots += m.skippedSlotsDelta();
            totalElementOverflow += m.elementOverflowLast();
            totalDroppedDecodeJobs += m.droppedDecodeJobsDelta();
            totalProcessedElementFrames += m.processedElementFramesDelta();
            peakLoopCpuMs = Math.max(peakLoopCpuMs, m.lastLoopCpuMs());
            peakFrameAgeMs = Math.max(peakFrameAgeMs, m.lastFrameAgeMs());
            peakTagDecodeMs = Math.max(peakTagDecodeMs, m.lastTagDecodeMs());
            if (m.health() == VidarMetrics.CameraHealth.FAILED) {
                failedHealthSamples++;
            }
        }
    }

    public long cycleCount() {
        return cycleCount;
    }

    /** Summary map for telemetry, EventLog, or post-run export. */
    public Map<String, Object> runSummary() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("cycles", cycleCount);
        m.put("droppedFrames", totalDroppedFrames);
        m.put("staleFrames", totalStaleFrames);
        m.put("skippedSlots", totalSkippedSlots);
        m.put("elemOverflow", totalElementOverflow);
        m.put("droppedDecodes", totalDroppedDecodeJobs);
        m.put("elemFrames", totalProcessedElementFrames);
        m.put("peakLoopMs", peakLoopCpuMs);
        m.put("peakFrameAgeMs", peakFrameAgeMs);
        m.put("peakTagDecodeMs", peakTagDecodeMs);
        m.put("failedSamples", failedHealthSamples);
        if (cycleCount > 0) {
            m.put("avgDroppedPerCycle", (double) totalDroppedFrames / cycleCount);
            m.put("avgStalePerCycle", (double) totalStaleFrames / cycleCount);
        }
        return m;
    }

    public void reset() {
        cycleCount = 0;
        totalDroppedFrames = 0;
        totalStaleFrames = 0;
        totalSkippedSlots = 0;
        totalElementOverflow = 0;
        totalDroppedDecodeJobs = 0;
        totalProcessedElementFrames = 0;
        peakLoopCpuMs = 0;
        peakFrameAgeMs = 0;
        peakTagDecodeMs = 0;
        failedHealthSamples = 0;
    }
}
