package org.firstinspires.ftc.teamcode.vidar.runtime;

import java.util.HashMap;
import java.util.Map;
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

    private static final String FUSION_PROCESSED = "ViDAR/Fusion/ProcessedElementFrames";

    private final Emitter emitter;
    /** Camera name -> TRACE channels. Filled once per camera, then reused. */
    private final Map<String, String[]> channelsByCamera = new HashMap<>();

    public TraceVidarAdapter(Emitter emitter) {
        this.emitter = Objects.requireNonNull(emitter, "emitter");
    }

    @Override
    public void onMetrics(VidarMetrics metrics) {
        if (metrics == null) {
            return;
        }
        String[] channels = channelsFor(metrics.cameraName());
        emitter.record(channels[0], metrics.lastFrameAgeMs());
        emitter.record(channels[1], metrics.lastLoopCpuMs());
        emitter.record(channels[2], metrics.droppedFrames());
        emitter.record(channels[3], metrics.staleFrames());
        emitter.record(channels[4], metrics.skippedSlots());
        emitter.record(channels[5], metrics.portalFps());
        emitter.record(channels[6], metrics.health() == VidarMetrics.CameraHealth.HEALTHY ? 1.0 : 0.0);
        emitter.record(channels[7], metrics.processedElementFrames());
    }

    private String[] channelsFor(String rawName) {
        String key = rawName == null ? "" : rawName;
        String[] cached = channelsByCamera.get(key);
        if (cached != null) {
            return cached;
        }
        String camera = sanitize(rawName);
        String prefix = "ViDAR/Camera/" + camera;
        cached = new String[] {
            prefix + "/FrameAge",
            prefix + "/LoopCpuMs",
            prefix + "/DroppedFrames",
            prefix + "/StaleFrames",
            prefix + "/SkippedSlots",
            prefix + "/Fps",
            prefix + "/Healthy",
            FUSION_PROCESSED
        };
        channelsByCamera.put(key, cached);
        return cached;
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
