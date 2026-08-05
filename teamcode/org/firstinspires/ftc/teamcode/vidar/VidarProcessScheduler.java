package org.firstinspires.ftc.teamcode.vidar;

/**
 * Per-camera tic-toc slots shared across processors on the same frame timestamp.
 */
public final class VidarProcessScheduler {

    public enum Slot {
        BALL,
        PLATE_SCOUT,
        TAG_DECODE
    }

    private int frameIndex;
    private long activeCaptureNanos = Long.MIN_VALUE;
    private Slot activeSlot = Slot.BALL;

    public Slot beginFrame(long captureTimeNanos) {
        if (captureTimeNanos == activeCaptureNanos) {
            return activeSlot;
        }
        activeCaptureNanos = captureTimeNanos;
        if (frameIndex % 2 == 0) {
            activeSlot = Slot.BALL;
        } else {
            activeSlot = Slot.PLATE_SCOUT;
        }
        frameIndex++;
        return activeSlot;
    }

    public void setOddSlot(Slot slot) {
        if (activeSlot != Slot.BALL) {
            activeSlot = slot;
        }
    }

    public Slot activeSlot() {
        return activeSlot;
    }
}
