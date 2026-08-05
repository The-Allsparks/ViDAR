package org.firstinspires.ftc.teamcode.vidar;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.function.Supplier;

/**
 * Adaptive AprilTag path: scout on plate frames; official FTC processor decode on schedule.
 * Scout observations guide scheduling but never alter absolute field pose.
 */
public class VidarAdaptiveTagProcessor implements VisionProcessor {

    private final VidarProcessScheduler scheduler;
    private final VidarTagCropDecoder cropDecoder = new VidarTagCropDecoder();
    private final Supplier<Pose2D> odomSupplier;
    private final String cameraName;
    private final VidarCameraProfile profile;
    private final VidarMetrics metrics;

    private VidarTagScoutResult lastScout;
    private VidarTagObservation latestTag;
    private VidarTagScoutObservation latestScoutObservation;
    private long lastDecodeNanos;
    private Rect lastDecodeRegion;
    private int lastDecodePixels;
    private int currentDecimation = VidarTagConfig.SCOUT_DECIMATION;

    public VidarAdaptiveTagProcessor(
            VidarProcessScheduler scheduler,
            VidarCameraProfile profile,
            String cameraName,
            Supplier<Pose2D> odomSupplier,
            Supplier<Pose2D> fieldPosePriorSupplier) {
        this(scheduler, profile, cameraName, odomSupplier, fieldPosePriorSupplier, null);
    }

    public VidarAdaptiveTagProcessor(
            VidarProcessScheduler scheduler,
            VidarCameraProfile profile,
            String cameraName,
            Supplier<Pose2D> odomSupplier,
            Supplier<Pose2D> fieldPosePriorSupplier,
            VidarMetrics metrics) {
        this.scheduler = scheduler;
        this.profile = profile;
        this.cameraName = cameraName;
        this.odomSupplier = odomSupplier;
        this.metrics = metrics;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cropDecoder.init(width, height, calibration);
        lastDecodeNanos = 0;
        latestTag = null;
        latestScoutObservation = null;
        lastScout = null;
        lastDecodePixels = 0;
        currentDecimation = VidarTagConfig.SCOUT_DECIMATION;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long t0 = System.nanoTime();
        if (!VidarTagConfig.ENABLED || frame == null || frame.empty()) {
            recordTagTime(t0);
            return latestTag;
        }

        VidarProcessScheduler.Slot slot = scheduler.beginFrame(captureTimeNanos);
        if (slot == VidarProcessScheduler.Slot.BALL) {
            recordTagTime(t0);
            return latestTag;
        }

        lastScout = VidarTagScout.run(frame);
        if (lastScout != null) {
            latestScoutObservation = VidarTagScoutObservation.fromScoutResult(
                    lastScout, profile, frame.cols(), captureTimeNanos, cameraName);
            if (VidarTagCropPlanner.worthDecode(
                    lastScout.widthPx, VidarTagConfig.SCOUT_WIDTH, frame.cols())) {
                currentDecimation = VidarTagConfig.DECODE_DECIMATION_AFTER_SCOUT;
            }
        }

        boolean forceDecode = VidarTagGate.consumeDriverRequest();
        boolean worthDecode = lastScout != null
                && VidarTagCropPlanner.worthDecode(
                        lastScout.widthPx, VidarTagConfig.SCOUT_WIDTH, frame.cols());
        boolean canDecode = lastScout != null
                && (worthDecode || forceDecode)
                && (forceDecode || VidarTagGate.shouldSample(lastScout, frame.cols()))
                && VidarDecodeArbiter.tryAcquire(captureTimeNanos, cameraName);

        if (canDecode) {
            scheduler.setOddSlot(VidarProcessScheduler.Slot.TAG_DECODE);
            tryDecode(frame, captureTimeNanos);
            recordTagTime(t0);
            return latestTag;
        }

        scheduler.setOddSlot(VidarProcessScheduler.Slot.PLATE_SCOUT);
        currentDecimation = VidarTagConfig.SCOUT_DECIMATION;
        recordTagTime(t0);
        return latestTag;
    }

    private void recordTagTime(long t0) {
        if (metrics != null) {
            metrics.recordProcessorTime("tag", (System.nanoTime() - t0) / 1_000_000.0);
        }
    }

    private void tryDecode(Mat frame, long captureTimeNanos) {
        if (lastScout == null) {
            return;
        }

        int decimation = VidarTagCropPlanner.chooseDecimation(
                lastScout.widthPx,
                VidarTagConfig.SCOUT_WIDTH,
                frame.cols());
        decimation = Math.max(decimation, currentDecimation);
        lastDecodeRegion = VidarFrameRegions.tagDecodeCrop(
                profile, frame.cols(), frame.rows(), lastScout.band);

        VidarTagCropDecoder.DecodeResult decoded = cropDecoder.decode(
                frame,
                lastDecodeRegion,
                decimation,
                captureTimeNanos,
                lastScout);

        lastDecodeNanos = captureTimeNanos;

        if (decoded != null) {
            lastDecodePixels = decoded.decodePixels;
            Pose2D odomAtCapture = odomSupplier != null ? odomSupplier.get() : null;
            latestTag = new VidarTagObservation(
                    decoded.tagId,
                    decoded.fieldPose,
                    odomAtCapture,
                    captureTimeNanos,
                    decoded.centerX,
                    decoded.centerY,
                    lastScout.band,
                    decoded.decimationUsed,
                    decoded.decodePixels);
        }
    }

    public VidarTagScoutResult getLastScout() {
        return lastScout;
    }

    public VidarTagObservation getLatestTag() {
        return latestTag;
    }

    public VidarTagScoutObservation getLatestScoutObservation() {
        return latestScoutObservation;
    }

    /** @deprecated Scout landmarks no longer localize — use {@link #getLatestScoutObservation()}. */
    @Deprecated
    public VidarScoutLandmarkObservation getLatestScoutLandmark() {
        return null;
    }

    public Rect getLastDecodeRegion() {
        return lastDecodeRegion;
    }

    public int getLastDecodePixels() {
        return lastDecodePixels;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext) {
        if (lastDecodeRegion == null) {
            return;
        }

        Paint band = new Paint();
        band.setColor(Color.argb(110, 180, 140, 255));
        band.setStyle(Paint.Style.STROKE);
        band.setStrokeWidth(2f * scaleCanvasDensity);
        float l = (float) (lastDecodeRegion.x * scaleBmpPxToCanvasPx);
        float t = (float) (lastDecodeRegion.y * scaleBmpPxToCanvasPx);
        float r = (float) ((lastDecodeRegion.x + lastDecodeRegion.width) * scaleBmpPxToCanvasPx);
        float b = (float) ((lastDecodeRegion.y + lastDecodeRegion.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(l, t, r, b, band);

        if (latestTag != null) {
            Paint text = new Paint();
            text.setColor(Color.WHITE);
            text.setTextSize(12f * scaleCanvasDensity);
            text.setAntiAlias(true);
            canvas.drawText(
                    "tag " + latestTag.tagId + " @" + latestTag.decodePixels + "px",
                    l + 4f,
                    t + 14f * scaleCanvasDensity,
                    text);
        }
    }
}
