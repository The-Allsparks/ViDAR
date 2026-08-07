package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.teamcode.vidar.VidarTagScoutResult;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameMailbox;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarProcessScheduler;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

/**
 * Adaptive AprilTag path: scout on tag frames; official FTC processor decode on schedule.
 * ViDAR does not consume odometry — tag observations carry field pose at capture time only.
 */
public class VidarAdaptiveTagProcessor implements VisionProcessor {

    private final VidarProcessScheduler scheduler;
    private final VidarTagCropDecoder cropDecoder = new VidarTagCropDecoder();
    private final VidarTagScoutRunner tagScout = new VidarTagScoutRunner();
    private final VidarSeasonConfig season;
    private final String cameraName;
    private final VidarCameraProfile profile;
    private final VidarMetrics metrics;
    private final VidarResourceBudget resourceBudget;

    private volatile VidarTagScoutResult lastScout;
    private volatile VidarTagObservation latestTag;
    private volatile VidarTagScoutObservation latestScoutObservation;
    private volatile Rect lastDecodeRegion;
    private volatile int lastDecodePixels;
    private int currentDecimation = VidarTagConfig.SCOUT_DECIMATION;
    private VidarFrameMailbox frameMailbox;

    public VidarAdaptiveTagProcessor(
            VidarProcessScheduler scheduler,
            VidarCameraProfile profile,
            String cameraName) {
        this(scheduler, profile, cameraName, null, null, null);
    }

    public VidarAdaptiveTagProcessor(
            VidarProcessScheduler scheduler,
            VidarCameraProfile profile,
            String cameraName,
            VidarMetrics metrics,
            VidarSeasonConfig season,
            VidarResourceBudget resourceBudget) {
        this.scheduler = scheduler;
        this.profile = profile;
        this.cameraName = cameraName;
        this.metrics = metrics;
        this.season = season;
        this.resourceBudget = resourceBudget;
    }

    public void setFrameMailbox(VidarFrameMailbox mailbox) {
        this.frameMailbox = mailbox;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cropDecoder.init(width, height, calibration, season, profile);
        latestTag = null;
        latestScoutObservation = null;
        lastScout = null;
        lastDecodePixels = 0;
        lastDecodeRegion = null;
        currentDecimation = VidarTagConfig.SCOUT_DECIMATION;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frameMailbox != null) {
            return latestTag;
        }
        processOwnedFrame(frame, captureTimeNanos);
        return latestTag;
    }

    public void processOwnedFrame(Mat frame, long captureTimeNanos) {
        VidarProcessScheduler.Slot slot = scheduler.beginFrame(captureTimeNanos);
        if (slot == VidarProcessScheduler.Slot.ELEMENT) {
            if (metrics != null) {
                metrics.incrementSkippedSlots();
            }
            return;
        }
        processTagPass(frame, captureTimeNanos);
    }

    public void processTagPass(Mat frame, long captureTimeNanos) {
        long t0 = System.nanoTime();
        if (!VidarTagConfig.ENABLED || frame == null || frame.empty()) {
            recordTagTime(t0);
            return;
        }

        if (metrics != null) {
            metrics.recordFrameAge((System.nanoTime() - captureTimeNanos) / 1_000_000.0);
        }

        lastScout = tagScout.run(frame, profile);
        if (lastScout != null) {
            latestScoutObservation = VidarTagScoutObservation.fromScoutResult(
                    lastScout, profile, frame.cols(), captureTimeNanos, cameraName);
            if (VidarTagCropPlanner.worthDecode(
                    lastScout.widthPx, VidarTagConfig.SCOUT_WIDTH, frame.cols())) {
                currentDecimation = VidarTagConfig.DECODE_DECIMATION_AFTER_SCOUT;
            }
        }

        boolean reduceTag = resourceBudget != null && resourceBudget.shouldReduceTagFrequency();
        boolean forceDecode = VidarTagGate.consumeDriverRequest();
        VidarTagScoutResult scout = lastScout;
        boolean worthDecode = scout != null
                && VidarTagCropPlanner.worthDecode(
                        scout.widthPx, VidarTagConfig.SCOUT_WIDTH, frame.cols());
        boolean canDecode = !reduceTag
                && scout != null
                && (worthDecode || forceDecode)
                && (forceDecode || VidarTagGate.shouldSample(scout, frame.cols()))
                && VidarDecodeArbiter.tryAcquire(captureTimeNanos, cameraName);

        if (canDecode) {
            scheduler.setOddSlot(VidarProcessScheduler.Slot.TAG_DECODE);
            scheduleDecode(frame, captureTimeNanos, scout);
            recordTagTime(t0);
            return;
        }

        scheduler.setOddSlot(VidarProcessScheduler.Slot.TAG_SCOUT);
        currentDecimation = VidarTagConfig.SCOUT_DECIMATION;
        recordTagTime(t0);
    }

    private void recordTagTime(long t0) {
        if (metrics != null) {
            metrics.recordProcessorTime("tag", (System.nanoTime() - t0) / 1_000_000.0);
        }
    }

    private void scheduleDecode(Mat frame, long captureTimeNanos, VidarTagScoutResult scout) {
        if (scout == null) {
            return;
        }

        int decimation = VidarTagCropPlanner.chooseDecimation(
                scout.widthPx, VidarTagConfig.SCOUT_WIDTH, frame.cols());
        decimation = Math.max(decimation, currentDecimation);
        Rect decodeRegion = VidarFrameRegions.tagDecodeCrop(
                profile, frame.cols(), frame.rows(), scout.band);
        lastDecodeRegion = decodeRegion;

        if (VidarConfig.ASYNC_TAG_DECODE_ENABLED) {
            VidarTagDecodeWorker.submit(
                    this, frame, decodeRegion, decimation, captureTimeNanos, scout, metrics);
        } else {
            tryDecode(frame, captureTimeNanos, decimation, decodeRegion, scout);
        }
    }

    void executeDecodeJob(
            Mat cropCopy,
            Rect decodeRegion,
            int decimation,
            long captureTimeNanos,
            VidarTagScoutResult scout) {
        long t0 = System.nanoTime();
        try {
            VidarTagCropDecoder.DecodeResult decoded = cropDecoder.decode(
                    cropCopy, decodeRegion, decimation, captureTimeNanos, scout);
            applyDecodeResult(decoded, captureTimeNanos, scout);
        } catch (RuntimeException ex) {
            if (metrics != null) {
                metrics.setLastError(ex.getMessage());
            }
            throw ex;
        } finally {
            if (metrics != null) {
                metrics.recordProcessorTime("tagDecode", (System.nanoTime() - t0) / 1_000_000.0);
            }
        }
    }

    private void tryDecode(
            Mat frame,
            long captureTimeNanos,
            int decimation,
            Rect decodeRegion,
            VidarTagScoutResult scout) {
        long t0 = System.nanoTime();
        VidarTagCropDecoder.DecodeResult decoded = cropDecoder.decode(
                frame, decodeRegion, decimation, captureTimeNanos, scout);
        applyDecodeResult(decoded, captureTimeNanos, scout);
        if (metrics != null) {
            metrics.recordProcessorTime("tagDecode", (System.nanoTime() - t0) / 1_000_000.0);
        }
    }

    private void applyDecodeResult(
            VidarTagCropDecoder.DecodeResult decoded,
            long captureTimeNanos,
            VidarTagScoutResult scout) {
        if (decoded == null || scout == null) {
            return;
        }
        lastDecodePixels = decoded.decodePixels;
        latestTag = new VidarTagObservation(
                decoded.tagId,
                decoded.fieldPose,
                captureTimeNanos,
                cameraName,
                decoded.centerX,
                decoded.centerY,
                scout.band,
                decoded.decimationUsed,
                decoded.decodePixels);
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
        Rect region = lastDecodeRegion;
        if (region == null) {
            return;
        }

        Paint band = new Paint();
        band.setColor(Color.argb(110, 180, 140, 255));
        band.setStyle(Paint.Style.STROKE);
        band.setStrokeWidth(2f * scaleCanvasDensity);
        float l = (float) (region.x * scaleBmpPxToCanvasPx);
        float t = (float) (region.y * scaleBmpPxToCanvasPx);
        float r = (float) ((region.x + region.width) * scaleBmpPxToCanvasPx);
        float b = (float) ((region.y + region.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(l, t, r, b, band);

        VidarTagObservation tag = latestTag;
        if (tag != null) {
            Paint text = new Paint();
            text.setColor(Color.WHITE);
            text.setTextSize(12f * scaleCanvasDensity);
            text.setAntiAlias(true);
            canvas.drawText(
                    "tag " + tag.tagId + " @" + tag.decodePixels + "px",
                    l + 4f,
                    t + 14f * scaleCanvasDensity,
                    text);
        }
    }
}
