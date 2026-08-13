package org.firstinspires.ftc.teamcode.vidar.tag;

/**
 * Per-session AprilTag decode rate limit (replaces the former static decode arbiter).
 */
public final class TagDecodeBudget {

    private long lastDecodeNanos;
    private String lastDecodeCamera = "";

    public boolean tryAcquire(long captureTimeNanos, String cameraName) {
        if (!VidarTagConfig.ENABLED) {
            return false;
        }
        long intervalNs = VidarTagConfig.DECODE_INTERVAL_MS * 1_000_000L;
        if (captureTimeNanos - lastDecodeNanos < intervalNs) {
            return false;
        }
        lastDecodeNanos = captureTimeNanos;
        lastDecodeCamera = cameraName == null ? "" : cameraName;
        return true;
    }

    public String lastDecodeCamera() {
        return lastDecodeCamera;
    }

    public void reset() {
        lastDecodeNanos = 0;
        lastDecodeCamera = "";
    }
}
