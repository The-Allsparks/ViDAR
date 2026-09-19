package org.firstinspires.ftc.teamcode.vidar.runtime;

import java.util.Objects;

/**
 * Maps {@link VidarMetrics} onto TRACE channel names from TRACE schema.md.
 * ViDAR does not import TRACE. TeamCode supplies the {@link Emitter} that
 * calls TRACE when BumbleBee actually runs ViDAR.
 */
public final class TraceVidarAdapter implements VidarMetricsSink {
    public interface Emitter {
        void record(String name, double value);
    }

    private final Emitter emitter;

    public TraceVidarAdapter(Emitter emitter) {
        this.emitter = Objects.requireNonNull(emitter, "emitter");
    }

    @Override
    public void onMetrics(VidarMetrics metrics) {
        if (metrics == null) {
            return;
        }
        String camera = sanitize(metrics.cameraName());
        String prefix = "ViDAR/Camera/" + camera;
        emitter.record(prefix + "/FrameAge", metrics.lastFrameAgeMs());
        emitter.record(prefix + "/LoopCpuMs", metrics.lastLoopCpuMs());
        emitter.record(prefix + "/DroppedFrames", metrics.droppedFrames());
        emitter.record(prefix + "/StaleFrames", metrics.staleFrames());
        emitter.record(prefix + "/SkippedSlots", metrics.skippedSlots());
        emitter.record(prefix + "/Fps", metrics.portalFps());
        emitter.record(prefix + "/Healthy", metrics.health() == VidarMetrics.CameraHealth.HEALTHY ? 1.0 : 0.0);
        emitter.record("ViDAR/Fusion/ProcessedElementFrames", metrics.processedElementFrames());
    }

    static String sanitize(String raw) {
        if (raw == null || raw.trim().isEmpty()) {
            return "Camera";
        }
        StringBuilder sb = new StringBuilder();
        String trimmed = raw.trim();
        for (int i = 0; i < trimmed.length(); i++) {
            char ch = trimmed.charAt(i);
            if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_') {
                sb.append(ch);
            } else if (sb.length() == 0 || sb.charAt(sb.length() - 1) != '_') {
                sb.append('_');
            }
        }
        if (sb.length() == 0) {
            return "Camera";
        }
        if (sb.charAt(0) >= '0' && sb.charAt(0) <= '9') {
            sb.insert(0, 'C');
        }
        return sb.toString();
    }
}
