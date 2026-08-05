package org.firstinspires.ftc.teamcode.vidar;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Per-camera and global vision-loop instrumentation for CPU/USB budgeting.
 */
public final class VidarMetrics {

    public enum CameraHealth {
        CONFIGURED,
        CONNECTED,
        STREAMING,
        PROCESSING,
        HEALTHY,
        FAILED
    }

    private final String cameraName;
    private VidarCameraScheduler.State cameraState = VidarCameraScheduler.State.PRIMARY;
    private CameraHealth health = CameraHealth.CONFIGURED;
    private long stateEnteredNanos = System.nanoTime();
    private long timeInStateMs;
    private int streamTransitionCount;
    private long lastStreamRestartDurationMs;
    private int droppedFrames;
    private int staleFrames;
    private double lastBallProcessorMs;
    private double lastPlateProcessorMs;
    private double lastTagProcessorMs;
    private double lastFrameAgeMs;
    private double lastLoopCpuMs;
    private float portalFps;
    private int activeProcessors;
    private String lastError;

    public VidarMetrics(String cameraName) {
        this.cameraName = cameraName;
    }

    public void setCameraState(VidarCameraScheduler.State state) {
        if (state != cameraState) {
            timeInStateMs += (System.nanoTime() - stateEnteredNanos) / 1_000_000L;
            cameraState = state;
            stateEnteredNanos = System.nanoTime();
        }
    }

    public void recordStreamTransition(long restartDurationMs) {
        streamTransitionCount++;
        lastStreamRestartDurationMs = restartDurationMs;
    }

    public void recordProcessorTime(String processor, double ms) {
        switch (processor) {
            case "ball":
                lastBallProcessorMs = ms;
                break;
            case "plate":
                lastPlateProcessorMs = ms;
                break;
            case "tag":
                lastTagProcessorMs = ms;
                break;
            default:
                break;
        }
    }

    public void recordFrameAge(double ageMs) {
        lastFrameAgeMs = ageMs;
    }

    public void recordLoopCpu(double ms) {
        lastLoopCpuMs = ms;
    }

    public void setPortalFps(float fps) {
        portalFps = fps;
    }

    public void setActiveProcessors(int count) {
        activeProcessors = count;
    }

    public void setHealth(CameraHealth h) {
        health = h;
    }

    public void incrementDroppedFrames() {
        droppedFrames++;
    }

    public void incrementStaleFrames() {
        staleFrames++;
    }

    public void setLastError(String error) {
        lastError = error;
    }

    public String cameraName() {
        return cameraName;
    }

    public VidarCameraScheduler.State cameraState() {
        return cameraState;
    }

    public CameraHealth health() {
        return health;
    }

    public long timeInStateMs() {
        return timeInStateMs + (System.nanoTime() - stateEnteredNanos) / 1_000_000L;
    }

    public int streamTransitionCount() {
        return streamTransitionCount;
    }

    public long lastStreamRestartDurationMs() {
        return lastStreamRestartDurationMs;
    }

    public int droppedFrames() {
        return droppedFrames;
    }

    public int staleFrames() {
        return staleFrames;
    }

    public double lastBallProcessorMs() {
        return lastBallProcessorMs;
    }

    public double lastPlateProcessorMs() {
        return lastPlateProcessorMs;
    }

    public double lastTagProcessorMs() {
        return lastTagProcessorMs;
    }

    public double lastFrameAgeMs() {
        return lastFrameAgeMs;
    }

    public double lastLoopCpuMs() {
        return lastLoopCpuMs;
    }

    public float portalFps() {
        return portalFps;
    }

    public int activeProcessors() {
        return activeProcessors;
    }

    public String lastError() {
        return lastError;
    }

    public Map<String, Object> toTelemetryMap() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("camera", cameraName);
        m.put("state", cameraState.name());
        m.put("health", health.name());
        m.put("stateMs", timeInStateMs());
        m.put("streamX", streamTransitionCount);
        m.put("fps", portalFps);
        m.put("ballMs", lastBallProcessorMs);
        m.put("plateMs", lastPlateProcessorMs);
        m.put("tagMs", lastTagProcessorMs);
        m.put("frameAgeMs", lastFrameAgeMs);
        m.put("loopMs", lastLoopCpuMs);
        m.put("processors", activeProcessors);
        m.put("dropped", droppedFrames);
        m.put("stale", staleFrames);
        if (lastError != null) {
            m.put("error", lastError);
        }
        return m;
    }
}
