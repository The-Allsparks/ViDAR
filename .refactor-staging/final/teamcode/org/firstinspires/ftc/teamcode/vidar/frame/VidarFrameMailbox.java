package org.firstinspires.ftc.teamcode.vidar.frame;

import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarGlobalVisionWorker;
import org.opencv.core.Mat;

/**
 * Latest-frame mailbox: portal thread writes {@code pending}; worker or a synchronous drain
 * reads {@code processing} via buffer swap (one SDK copy on publish, no copy on take).
 *
 * <p>Observation {@code captureTimeNanos} is the VisionPortal {@code frameCaptureNanos} stored
 * at publish. Single- and multi-camera paths both publish here; processing is triggered by
 * {@link VidarVision#drainMailboxSync()} or {@link VidarGlobalVisionWorker}.
 */
final class VidarFrameMailbox {

    static final class Snapshot {
        final Mat frame;
        final long captureTimeNanos;
        final int generation;

        Snapshot(Mat frame, long captureTimeNanos, int generation) {
            this.frame = frame;
            this.captureTimeNanos = captureTimeNanos;
            this.generation = generation;
        }
    }

    private final VidarMetrics metrics;
    private final Object lock = new Object();

    private Mat pending;
    private Mat processing;
    /** VisionPortal {@code frameCaptureNanos} for the frame currently in {@code pending}. */
    private long pendingCaptureNanos;
    private long lastPublishedCaptureNanos = Long.MIN_VALUE;
    private int generation;
    private int lastTakenGeneration;

    VidarFrameMailbox(VidarMetrics metrics) {
        this.metrics = metrics;
    }

    void publish(Mat sdkFrame, long captureNanos) {
        if (sdkFrame == null || sdkFrame.empty()) {
            return;
        }
        synchronized (lock) {
            if (captureNanos == lastPublishedCaptureNanos) {
                return;
            }
            if (generation > lastTakenGeneration && metrics != null) {
                metrics.incrementDroppedFrames();
            }
            pending = ensureMat(pending, sdkFrame);
            sdkFrame.copyTo(pending);
            pendingCaptureNanos = captureNanos;
            lastPublishedCaptureNanos = captureNanos;
            generation++;
        }
    }

    Snapshot tryTake(int lastProcessedGeneration) {
        synchronized (lock) {
            if (generation <= lastProcessedGeneration || pending == null || pending.empty()) {
                return null;
            }
            Mat tmp = processing;
            processing = pending;
            pending = tmp;
            lastTakenGeneration = generation;
            return new Snapshot(processing, pendingCaptureNanos, generation);
        }
    }

    void release() {
        synchronized (lock) {
            if (pending != null) {
                pending.release();
                pending = null;
            }
            if (processing != null) {
                processing.release();
                processing = null;
            }
        }
    }

    private static Mat ensureMat(Mat mat, Mat source) {
        if (mat == null || mat.empty()
                || mat.cols() != source.cols()
                || mat.rows() != source.rows()
                || mat.type() != source.type()) {
            if (mat != null) {
                mat.release();
            }
            return new Mat(source.rows(), source.cols(), source.type());
        }
        return mat;
    }
}

