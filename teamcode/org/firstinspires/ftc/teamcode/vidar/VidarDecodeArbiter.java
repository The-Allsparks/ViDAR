package org.firstinspires.ftc.teamcode.vidar;

/**
 * Limits full AprilTag decode to about once per second across all cameras.
 */
public final class VidarDecodeArbiter {

    private static final Object LOCK = new Object();
    private static long lastDecodeNanos;
    private static String lastDecodeCamera = "";

    private VidarDecodeArbiter() {}

    public static boolean tryAcquire(long captureTimeNanos, String cameraName) {
        if (!VidarTagConfig.ENABLED) {
            return false;
        }
        synchronized (LOCK) {
            long intervalNs = VidarTagConfig.DECODE_INTERVAL_MS * 1_000_000L;
            if (captureTimeNanos - lastDecodeNanos < intervalNs) {
                return false;
            }
            lastDecodeNanos = captureTimeNanos;
            lastDecodeCamera = cameraName;
            return true;
        }
    }

    public static String lastDecodeCamera() {
        synchronized (LOCK) {
            return lastDecodeCamera;
        }
    }

    public static void reset() {
        synchronized (LOCK) {
            lastDecodeNanos = 0;
            lastDecodeCamera = "";
        }
    }
}
