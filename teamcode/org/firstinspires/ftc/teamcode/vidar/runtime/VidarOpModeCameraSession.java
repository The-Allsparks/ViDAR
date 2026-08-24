package org.firstinspires.ftc.teamcode.vidar.runtime;

/**
 * Process-level bookkeeping for Auto → stop → TeleOp camera sessions.
 *
 * <p>{@link VidarVisionAttachment} owns VisionPortals. This class records generation and
 * whether a session is open so attach/detach/close is idempotent and testable on the JVM
 * without HardwareMap.
 */
public final class VidarOpModeCameraSession {

    private boolean attached;
    private int generation;
    private int releaseCount;

    /**
     * Open a camera session. If one is already open, it is released first (same as
     * {@link VidarRuntime#attachVision} tearing down the previous OpMode's portals).
     *
     * @return generation after this attach (1 after first successful Auto INIT)
     */
    public synchronized int attach() {
        if (attached) {
            releaseInternal();
        }
        attached = true;
        generation++;
        return generation;
    }

    /**
     * Close the current session.
     *
     * @return {@code true} if this call performed the release (caller must close portals)
     */
    public synchronized boolean detach() {
        if (!attached) {
            return false;
        }
        releaseInternal();
        return true;
    }

    public synchronized boolean isAttached() {
        return attached;
    }

    public synchronized int generation() {
        return generation;
    }

    public synchronized int releaseCount() {
        return releaseCount;
    }

    /** RC process teardown — not OpMode {@code stop()}. */
    public synchronized void reset() {
        if (attached) {
            releaseInternal();
        }
        generation = 0;
    }

    private void releaseInternal() {
        attached = false;
        releaseCount++;
    }
}
