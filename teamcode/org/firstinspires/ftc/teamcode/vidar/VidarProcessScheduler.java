package org.firstinspires.ftc.teamcode.vidar;

/**
 * Per-camera tic-toc slots shared across processors on the same frame timestamp.
 */
public final class VidarProcessScheduler {

    public enum Slot {
        /** Game elements + alliance plates (shared contour pass). */
        ELEMENT,
        /** AprilTag scout/decode on odd frames. */
        TAG_SCOUT,
        TAG_DECODE
    }

    private Slot activeSlot = Slot.ELEMENT;

    private int frameIndex;
    private long activeCaptureNanos = Long.MIN_VALUE;

    public VidarProcessScheduler() {
        this(0);
    }

    /** {@code phaseOffsetFrames} — use {@code cameraIndex % 2} for out-of-phase multi-camera tic-toc. */
    public VidarProcessScheduler(int phaseOffsetFrames) {
        frameIndex = Math.max(0, phaseOffsetFrames);
    }

    public Slot beginFrame(long captureTimeNanos) {
        if (captureTimeNanos == activeCaptureNanos) {
            return activeSlot;
        }
        activeCaptureNanos = captureTimeNanos;
        if (frameIndex % 2 == 0) {
            activeSlot = Slot.ELEMENT;
        } else {
            activeSlot = Slot.TAG_SCOUT;
        }
        frameIndex++;
        return activeSlot;
    }

    public void setOddSlot(Slot slot) {
        if (activeSlot != Slot.ELEMENT) {
            activeSlot = slot;
        }
    }

    public Slot activeSlot() {
        return activeSlot;
    }
}
