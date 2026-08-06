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
    private int skippedSlots;
    private int processedElementFrames;
    private int droppedDecodeJobs;
    private int elementOverflowLast;
    private double lastElementProcessorMs;
    private double lastPlateProcessorMs;
    private double lastTagProcessorMs;
    private double lastTagDecodeMs;
    private double lastFrameAgeMs;
    private double lastLoopCpuMs;
    private float portalFps;
    private int activeProcessors;
    private String lastError;

    private int cycleStartDropped;
    private int cycleStartStale;
    private int cycleStartSkipped;
    private int cycleStartProcessedElement;
    private int cycleStartDroppedDecode;

    public VidarMetrics(String cameraName) {
        this.cameraName = cameraName;
    }

    /** Call at the start of each OpMode vision update cycle for delta metrics. */
    public void beginCycle() {
        cycleStartDropped = droppedFrames;
        cycleStartStale = staleFrames;
        cycleStartSkipped = skippedSlots;
        cycleStartProcessedElement = processedElementFrames;
        cycleStartDroppedDecode = droppedDecodeJobs;
    }

    public void incrementDroppedDecodeJobs() {
        droppedDecodeJobs++;
    }

    public int droppedFramesDelta() {
        return droppedFrames - cycleStartDropped;
    }

    public int staleFramesDelta() {
        return staleFrames - cycleStartStale;
    }

    public int skippedSlotsDelta() {
        return skippedSlots - cycleStartSkipped;
    }

    public int processedElementFramesDelta() {
        return processedElementFrames - cycleStartProcessedElement;
    }

    public int droppedDecodeJobsDelta() {
        return droppedDecodeJobs - cycleStartDroppedDecode;
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
            case "element":
            case "contour":
                lastElementProcessorMs = ms;
                lastElementProcessorMs = ms;
                break;
            case "element":
                lastElementProcessorMs = ms;
                lastElementProcessorMs = ms;
                break;
            case "plate":
                lastPlateProcessorMs = ms;
                break;
            case "tag":
                lastTagProcessorMs = ms;
                break;
            case "tagDecode":
                lastTagDecodeMs = ms;
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

    public void incrementSkippedSlots() {
        skippedSlots++;
    }

    public void incrementProcessedElementFrames() {
        processedElementFrames++;
    }

    public void recordElementOverflow(int overflowCount) {
        elementOverflowLast = Math.max(0, overflowCount);
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

    public int skippedSlots() {
        return skippedSlots;
    }

    public int processedElementFrames() {
        return processedElementFrames;
    }

    public int droppedDecodeJobs() {
        return droppedDecodeJobs;
    }

    public int elementOverflowLast() {
        return elementOverflowLast;
    }

    public double lastElementProcessorMs() {
        return lastElementProcessorMs;
    }

    public double lastPlateProcessorMs() {
        return lastPlateProcessorMs;
    }

    public double lastTagProcessorMs() {
        return lastTagProcessorMs;
    }

    public double lastTagDecodeMs() {
        return lastTagDecodeMs;
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
        m.put("elementMs", lastElementProcessorMs);
        m.put("plateMs", lastPlateProcessorMs);
        m.put("tagMs", lastTagProcessorMs);
        m.put("tagDecodeMs", lastTagDecodeMs);
        m.put("frameAgeMs", lastFrameAgeMs);
        m.put("loopMs", lastLoopCpuMs);
        m.put("processors", activeProcessors);
        m.put("dropped", droppedFrames);
        m.put("stale", staleFrames);
        m.put("skipped", skippedSlots);
        m.put("elemFrames", processedElementFrames);
        m.put("elemOver", elementOverflowLast);
        m.put("droppedDecode", droppedDecodeJobs);
        if (lastError != null) {
            m.put("error", lastError);
        }
        return m;
    }
}
