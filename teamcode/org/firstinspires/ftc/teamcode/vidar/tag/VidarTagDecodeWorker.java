package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameMailbox;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

/**
 * Background AprilTag decode — one instance per {@link org.firstinspires.ftc.teamcode.vidar.VidarSession}.
 */
public final class VidarTagDecodeWorker extends Thread {

    private static final class DecodeRequest {
        final VidarAdaptiveTagProcessor processor;
        final Rect localRegion;
        final int decimation;
        final long captureTimeNanos;
        final VidarTagScoutObservation scout;

        DecodeRequest(
                VidarAdaptiveTagProcessor processor,
                Rect localRegion,
                int decimation,
                long captureTimeNanos,
                VidarTagScoutObservation scout) {
            this.processor = processor;
            this.localRegion = localRegion;
            this.decimation = decimation;
            this.captureTimeNanos = captureTimeNanos;
            this.scout = scout;
        }
    }

    static final class Job {
        final VidarAdaptiveTagProcessor processor;
        final Mat crop;
        final Rect decodeRegion;
        final int decimation;
        final long captureTimeNanos;
        final VidarTagScoutObservation scout;

        Job(
                VidarAdaptiveTagProcessor processor,
                Mat crop,
                Rect decodeRegion,
                int decimation,
                long captureTimeNanos,
                VidarTagScoutObservation scout) {
            this.processor = processor;
            this.crop = crop;
            this.decodeRegion = decodeRegion;
            this.decimation = decimation;
            this.captureTimeNanos = captureTimeNanos;
            this.scout = scout;
        }
    }

    private final Object lock = new Object();
    private volatile boolean running = true;
    private volatile boolean started;

    private Mat pendingCrop;
    private Mat processingCrop;
    private DecodeRequest pendingRequest;

    public VidarTagDecodeWorker() {
        super("VidarTagDecodeWorker");
        setPriority(Thread.NORM_PRIORITY - 2);
    }

    public void ensureStarted() {
        if (!VidarConfig.ASYNC_TAG_DECODE_ENABLED || !VidarTagConfig.ENABLED) {
            return;
        }
        if (!started) {
            started = true;
            start();
        }
    }

    public void submit(
            VidarAdaptiveTagProcessor processor,
            Mat frame,
            Rect decodeRegion,
            int decimation,
            long captureTimeNanos,
            VidarTagScoutObservation scout,
            VidarMetrics metrics) {
        if (!VidarConfig.ASYNC_TAG_DECODE_ENABLED || processor == null
                || frame == null || frame.empty() || decodeRegion == null) {
            return;
        }
        int w = decodeRegion.width;
        int h = decodeRegion.height;
        if (w <= 0 || h <= 0) {
            return;
        }
        ensureStarted();
        publish(processor, frame, decodeRegion, w, h, decimation, captureTimeNanos, scout, metrics);
    }

    public void shutdownAndJoin() {
        running = false;
        interrupt();
        synchronized (lock) {
            lock.notifyAll();
        }
        if (started) {
            try {
                join(750);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
        releaseBuffers();
        started = false;
    }

    private void publish(
            VidarAdaptiveTagProcessor processor,
            Mat frame,
            Rect decodeRegion,
            int w,
            int h,
            int decimation,
            long captureTimeNanos,
            VidarTagScoutObservation scout,
            VidarMetrics metrics) {
        synchronized (lock) {
            if (pendingRequest != null && metrics != null) {
                metrics.incrementDroppedDecodeJobs();
            }

            pendingCrop = ensureCropMat(pendingCrop, h, w, frame.type());
            Mat roi = new Mat(frame, decodeRegion);
            try {
                roi.copyTo(pendingCrop);
            } finally {
                roi.release();
            }

            pendingRequest = new DecodeRequest(
                    processor,
                    new Rect(0, 0, w, h),
                    decimation,
                    captureTimeNanos,
                    scout);
            lock.notify();
        }
    }

    private Job tryTake() {
        synchronized (lock) {
            while (pendingRequest == null && running) {
                try {
                    lock.wait(50);
                } catch (InterruptedException ex) {
                    if (!running) {
                        return null;
                    }
                    Thread.currentThread().interrupt();
                }
            }
            if (!running || pendingRequest == null) {
                return null;
            }

            Mat tmp = processingCrop;
            processingCrop = pendingCrop;
            pendingCrop = tmp;

            DecodeRequest request = pendingRequest;
            pendingRequest = null;

            return new Job(
                    request.processor,
                    processingCrop,
                    request.localRegion,
                    request.decimation,
                    request.captureTimeNanos,
                    request.scout);
        }
    }

    private void releaseBuffers() {
        synchronized (lock) {
            pendingRequest = null;
            if (pendingCrop != null) {
                pendingCrop.release();
                pendingCrop = null;
            }
            if (processingCrop != null) {
                processingCrop.release();
                processingCrop = null;
            }
        }
    }

    private static Mat ensureCropMat(Mat mat, int rows, int cols, int type) {
        rows = Math.min(rows, VidarConfig.PORTAL_RESOLUTION.getHeight());
        cols = Math.min(cols, VidarConfig.PORTAL_RESOLUTION.getWidth());
        if (mat == null || mat.empty()
                || mat.rows() != rows
                || mat.cols() != cols
                || mat.type() != type) {
            if (mat != null) {
                mat.release();
            }
            return new Mat(rows, cols, type);
        }
        return mat;
    }

    @Override
    public void run() {
        while (running) {
            Job job = tryTake();
            if (job == null) {
                continue;
            }
            job.processor.executeDecodeJob(
                    job.crop,
                    job.decodeRegion,
                    job.decimation,
                    job.captureTimeNanos,
                    job.scout);
        }
    }
}
