package org.firstinspires.ftc.teamcode.vidar.detect;



import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarGeometry;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameMailbox;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFramePipeline;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementRejectionStats;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarProcessScheduler;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader;

import org.firstinspires.ftc.teamcode.vidar.config.VidarElementSpec;

import org.firstinspires.ftc.teamcode.vidar.config.VidarPlateSpec;

import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;



import android.graphics.Canvas;

import android.graphics.Color;

import android.graphics.Paint;



import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Mat;

import org.opencv.core.Point;

import org.opencv.core.RotatedRect;

import org.opencv.core.Size;

import org.opencv.imgproc.Imgproc;



import java.util.ArrayList;

import java.util.HashMap;

import java.util.List;

import java.util.Map;

import java.util.concurrent.atomic.AtomicReference;



/**

 * Unified color-contour processor for all season {@code elements[]} and {@code plates[]}.

 * One scaled ROI and one HSV conversion per frame; each target gets its own mask pass.

 */

public class VidarContourProcessor implements VisionProcessor {



    private static final class ScoredObservation {

        final VidarElementObservation observation;

        final double score;



        ScoredObservation(VidarElementObservation observation, double score) {

            this.observation = observation;

            this.score = score;

        }

    }



    private final VidarCameraProfile profile;

    private final String cameraName;

    private final VidarProcessScheduler scheduler;

    private final VidarMetrics metrics;

    private final VidarSeasonConfig season;

    private final List<VidarContourTarget> targets;

    private final VidarResourceBudget resourceBudget;

    private final Map<String, VidarElementSpec> seasonElementsById = new HashMap<>();

    private final VidarContourWorkspacePool workspacePool = new VidarContourWorkspacePool();

    private final List<ScoredObservation> frameCandidates = new ArrayList<>();



    private final Map<String, VidarElementObservation> bestGameById = new HashMap<>();

    private volatile VidarElementObservation bestGame;

    private volatile VidarPlateObservation bestPlate;

    private VidarPlateObservation bestRed;

    private VidarPlateObservation bestBlue;

    private VidarElementObservation bestGameDraw;

    private RotatedRect bestPlateDraw;

    private final AtomicReference<VidarRankedElementFrame> rankedFrameRef =

            new AtomicReference<>(VidarRankedElementFrame.empty("", VidarConfig.DEFAULT_MAX_RANKED_ELEMENTS));

    private volatile int maxRankedElements = VidarConfig.DEFAULT_MAX_RANKED_ELEMENTS;



    private Mat reusableScaledRgba;

    private Mat reusableHsv;

    private Mat reusableRgb;

    private Mat reusableMask;

    private Mat reusableWrapMask;

    private Mat kernelEllipse;

    private Mat kernelRect;

    private Mat reusableGray;

    private Mat reusableCircles;

    private Mat reusableHierarchy;

    private VidarElementObservation[] rankedScratch;

    private double[] rankedScoreScratch;

    private long lastElementCaptureNanos;

    private final VidarElementRejectionStats rejectionStats = new VidarElementRejectionStats();

    private VidarFrameMailbox frameMailbox;

    private Runnable mailboxDrainCallback;



    private Paint gameDrawStroke;

    private Paint plateDrawStroke;



    public void setMaxRankedElements(int max) {

        maxRankedElements = Math.max(1, Math.min(max, VidarConfig.MAX_RANKED_ELEMENTS_CAP));

    }



    public int maxRankedElements() {

        return maxRankedElements;

    }



    public void setFrameMailbox(VidarFrameMailbox mailbox) {

        this.frameMailbox = mailbox;

    }

    public void setMailboxDrainCallback(Runnable callback) {
        this.mailboxDrainCallback = callback;
    }



    public VidarContourProcessor(

            VidarCameraProfile profile,

            String cameraName,

            VidarProcessScheduler scheduler) {

        this(profile, cameraName, scheduler, null, null, null);

    }



    public VidarContourProcessor(

            VidarCameraProfile profile,

            String cameraName,

            VidarProcessScheduler scheduler,

            VidarSeasonConfig season) {

        this(profile, cameraName, scheduler, null, season, null);

    }



    public VidarContourProcessor(

            VidarCameraProfile profile,

            String cameraName,

            VidarProcessScheduler scheduler,

            VidarMetrics metrics,

            VidarSeasonConfig season) {

        this(profile, cameraName, scheduler, metrics, season, null);

    }



    public VidarContourProcessor(

            VidarCameraProfile profile,

            String cameraName,

            VidarProcessScheduler scheduler,

            VidarMetrics metrics,

            VidarSeasonConfig season,

            VidarResourceBudget resourceBudget) {

        this.profile = profile;

        this.cameraName = cameraName;

        this.scheduler = scheduler;

        this.metrics = metrics;

        this.season = season != null ? season : VidarConfigLoader.defaultSeason();

        this.resourceBudget = resourceBudget;

        this.targets = VidarContourTarget.fromSeason(this.season);

        for (VidarElementSpec spec : this.season.elements) {

            seasonElementsById.put(spec.id, spec);

        }

        this.rankedFrameRef.set(VidarRankedElementFrame.empty(cameraName, maxRankedElements));

    }



    @Override

    public void init(int width, int height, CameraCalibration calibration) {

        clear();

        releaseReusable();

        ensureMorphKernels();

    }



    private void clear() {

        bestGameById.clear();

        bestGame = null;

        bestPlate = null;

        bestRed = null;

        bestBlue = null;

        bestGameDraw = null;

        bestPlateDraw = null;

        rankedFrameRef.set(VidarRankedElementFrame.empty(cameraName, maxRankedElements));

    }



    private void ensureMorphKernels() {

        if (kernelEllipse == null) {

            kernelEllipse = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));

        }

        if (kernelRect == null) {

            kernelRect = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

        }

    }



    private void releaseReusable() {

        if (reusableScaledRgba != null) reusableScaledRgba.release();

        if (reusableHsv != null) reusableHsv.release();

        if (reusableRgb != null) reusableRgb.release();

        if (reusableMask != null) reusableMask.release();

        if (reusableWrapMask != null) reusableWrapMask.release();

        if (kernelEllipse != null) kernelEllipse.release();

        if (kernelRect != null) kernelRect.release();

        if (reusableGray != null) reusableGray.release();

        if (reusableCircles != null) reusableCircles.release();

        if (reusableHierarchy != null) reusableHierarchy.release();

        reusableScaledRgba = reusableHsv = reusableRgb = reusableMask = reusableWrapMask = null;

        kernelEllipse = kernelRect = null;

        reusableGray = reusableCircles = reusableHierarchy = null;

    }



    @Override

    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (frameMailbox != null) {

            frameMailbox.publish(frame, captureTimeNanos);

            if (mailboxDrainCallback != null) {

                mailboxDrainCallback.run();

            }

            return bestGame;

        }

        processOwnedFrame(frame, captureTimeNanos);

        return bestGame;

    }



    /** Worker or legacy fallback — runs tic-toc slot selection then the matching pass. */

    public void processOwnedFrame(Mat frame, long captureTimeNanos) {

        VidarProcessScheduler.Slot slot = scheduler.beginFrame(captureTimeNanos);

        if (slot == VidarProcessScheduler.Slot.ELEMENT) {

            processElementPass(frame, captureTimeNanos);

        } else if (metrics != null) {

            metrics.incrementSkippedSlots();

        }

    }



    /** Global worker — slot already chosen; run element pass only. */

    public void processElementPass(Mat frame, long captureTimeNanos) {

        long t0 = System.nanoTime();

        if (metrics != null) {

            double ageMs = (System.nanoTime() - captureTimeNanos) / 1_000_000.0;

            metrics.recordFrameAge(ageMs);

            if (captureTimeNanos <= lastElementCaptureNanos) {

                metrics.incrementStaleFrames();

            }

            metrics.incrementProcessedElementFrames();

        }

        lastElementCaptureNanos = captureTimeNanos;



        clear();

        rejectionStats.reset();

        if (frame == null || frame.empty()) {

            recordTime(t0);

            return;

        }



        if (reusableScaledRgba == null) {

            reusableScaledRgba = new Mat();

        }

        if (reusableHsv == null) {

            reusableHsv = new Mat();

        }

        if (reusableHierarchy == null) {

            reusableHierarchy = new Mat();

        }

        ensureMorphKernels();



        double roiScale = resourceBudget == null

                ? VidarConfig.PROCESS_ROI_SCALE

                : resourceBudget.processingRoiScale();

        boolean disablePlates = resourceBudget != null && resourceBudget.shouldDisablePlates();

        boolean disableLocalHough = resourceBudget != null && resourceBudget.shouldDisableLocalHough();

        boolean prepareGray = !disableLocalHough && anyTargetNeedsLocalHough();



        VidarFramePipeline.ScaledRoi scaled = VidarFramePipeline.detectionScaledInto(

                frame, profile, roiScale, reusableScaledRgba);

        if (scaled == null) {

            recordTime(t0);

            return;

        }



        frameCandidates.clear();

        VidarContourWorkspace workspace = workspacePool.borrow();

        try {

            if (reusableRgb == null) {

                reusableRgb = new Mat();

            }

            Imgproc.cvtColor(scaled.image, reusableRgb, Imgproc.COLOR_RGBA2RGB);

            Imgproc.cvtColor(reusableRgb, reusableHsv, Imgproc.COLOR_RGB2HSV);



            if (reusableMask == null) {

                reusableMask = new Mat();

            }

            if (reusableWrapMask == null) {

                reusableWrapMask = new Mat();

            }



            if (prepareGray) {

                if (reusableGray == null) {

                    reusableGray = new Mat();

                }

                if (reusableCircles == null) {

                    reusableCircles = new Mat();

                }

                Imgproc.cvtColor(scaled.image, reusableGray, Imgproc.COLOR_RGBA2GRAY);

                Imgproc.GaussianBlur(reusableGray, reusableGray, new Size(3, 3), 0);

            }



            for (VidarContourTarget target : targets) {

                if (disablePlates && target.kind == VidarContourTarget.Kind.PLATE) {

                    continue;

                }



                Mat kernel = target.morphEllipseKernel ? kernelEllipse : kernelRect;

                VidarContourDetect.buildMask(reusableHsv, reusableMask, target, reusableWrapMask);

                VidarContourDetect.applyMorphology(reusableMask, kernel, target);



                if (target.kind == VidarContourTarget.Kind.PLATE) {

                    processPlateTarget(

                            target, scaled, frame.cols(), frame.rows(), captureTimeNanos, workspace);

                } else {

                    processGameTarget(

                            target, scaled, frame.cols(), frame.rows(),

                            captureTimeNanos, disableLocalHough, prepareGray, workspace);

                }

            }



            bestPlate = pickStronger(bestRed, bestBlue);

            finalizeRankedFrame(frameCandidates, captureTimeNanos);

            bestGame = pickBestGame();

        } finally {

            workspacePool.release(workspace);

            scaled.release();

        }



        recordTime(t0);

    }



    public VidarProcessScheduler scheduler() {

        return scheduler;

    }



    private boolean anyTargetNeedsLocalHough() {

        for (VidarContourTarget target : targets) {

            if (target.kind != VidarContourTarget.Kind.PLATE

                    && target.detector == VidarElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH) {

                return true;

            }

        }

        return false;

    }



    private void finalizeRankedFrame(List<ScoredObservation> candidates, long captureTimeNanos) {

        int cap = maxRankedElements;

        if (rankedScratch == null || rankedScratch.length != cap) {

            rankedScratch = new VidarElementObservation[cap];

            rankedScoreScratch = new double[cap];

        }

        int count = selectTopK(candidates, cap, rankedScratch, rankedScoreScratch);

        int overflow = Math.max(0, candidates.size() - cap);

        VidarElementObservation[] ranked = new VidarElementObservation[count];

        System.arraycopy(rankedScratch, 0, ranked, 0, count);

        rankedFrameRef.set(new VidarRankedElementFrame(

                ranked, count, overflow, captureTimeNanos, cameraName, cap));

        if (metrics != null) {

            metrics.recordElementOverflow(overflow);

        }

        if (count > 0) {

            bestGameDraw = ranked[0];

        }

    }



    private static int selectTopK(

            List<ScoredObservation> candidates,

            int k,

            VidarElementObservation[] out,

            double[] scores) {

        int count = 0;

        for (ScoredObservation candidate : candidates) {

            if (count < k) {

                out[count] = candidate.observation;

                scores[count] = candidate.score;

                count++;

                bubbleUp(out, scores, count - 1);

            } else if (candidate.score <= scores[k - 1]) {

                continue;

            } else {

                out[k - 1] = candidate.observation;

                scores[k - 1] = candidate.score;

                bubbleUp(out, scores, k - 1);

            }

        }

        return count;

    }



    private static void bubbleUp(

            VidarElementObservation[] out,

            double[] scores,

            int index) {

        while (index > 0 && scores[index] > scores[index - 1]) {

            double scoreTmp = scores[index];

            scores[index] = scores[index - 1];

            scores[index - 1] = scoreTmp;

            VidarElementObservation obsTmp = out[index];

            out[index] = out[index - 1];

            out[index - 1] = obsTmp;

            index--;

        }

    }



    private void processGameTarget(

            VidarContourTarget target,

            VidarFramePipeline.ScaledRoi scaled,

            int frameW,

            int frameH,

            long captureTimeNanos,

            boolean disableLocalHough,

            boolean grayReady,

            VidarContourWorkspace workspace) {

        switch (target.shape) {

            case RECT:

                processRectGameTarget(

                        target, scaled, frameW, frameH, captureTimeNanos, workspace);

                break;

            case BLOB:

            case CIRCLE:

            default:

                processCircleGameTarget(

                        target, scaled, frameW, frameH, captureTimeNanos,

                        disableLocalHough, grayReady, workspace);

                break;

        }

    }



    private void processCircleGameTarget(

            VidarContourTarget target,

            VidarFramePipeline.ScaledRoi scaled,

            int frameW,

            int frameH,

            long captureTimeNanos,

            boolean disableLocalHough,

            boolean grayReady,

            VidarContourWorkspace workspace) {

        List<VidarContourDetect.CircleHit> hits = VidarContourDetect.findCircleHits(

                scaled.image, reusableMask, reusableHierarchy, target, scaled,

                frameW, frameH, profile, workspace);

        if (!disableLocalHough

                && grayReady

                && target.detector == VidarElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH

                && !hits.isEmpty()) {

            hits = VidarContourDetect.applyLocalHough(

                    scaled, hits, target, reusableGray, reusableCircles);

        }



        VidarElementSpec elementSpec = seasonElement(target.id);

        VidarElementObservation bestForTarget = null;

        double bestScore = -1;

        for (VidarContourDetect.CircleHit hit : hits) {

            VidarElementObservation obs = VidarGeometry.fuseElementObservation(

                    hit.cx, hit.cy, hit.radius, hit.area, hit.aspectRatio, hit.circularity,

                    hit.fillRatio, hit.interiorScore, target.detector, profile, cameraName,

                    captureTimeNanos, hit.touchesBoundary, false, hit.circleFitQuality,

                    elementSpec, season);

            if (obs.confidence < season.minElementConfidence) {

                continue;

            }

            double localCy = (hit.cy - scaled.sourceCrop.y) / scaled.scale;

            double floorWeight = 0.25 + 0.75 * (localCy / scaled.image.rows());

            double score = obs.confidence * obs.radiusPx * obs.radiusPx * floorWeight * floorWeight;

            frameCandidates.add(new ScoredObservation(obs, score));

            if (score > bestScore) {

                bestScore = score;

                bestForTarget = obs;

            }

        }

        if (bestForTarget != null) {

            bestGameById.put(target.id, bestForTarget);

        }

    }



    private void processRectGameTarget(

            VidarContourTarget target,

            VidarFramePipeline.ScaledRoi scaled,

            int frameW,

            int frameH,

            long captureTimeNanos,

            VidarContourWorkspace workspace) {

        List<VidarContourDetect.RectHit> hits = VidarContourDetect.findRectHits(

                scaled.image, reusableMask, reusableHierarchy, target, scaled,

                frameW, frameH, profile, workspace);



        VidarElementSpec elementSpec = seasonElement(target.id);

        VidarElementObservation bestForTarget = null;

        double bestScore = -1;

        for (VidarContourDetect.RectHit hit : hits) {

            double absCx = scaled.toFullX(hit.box.center.x);

            double absCy = scaled.toFullY(hit.box.center.y);

            double fullWidthPx = Math.max(hit.box.size.width, hit.box.size.height) * scaled.scale;

            double fullHeightPx = Math.min(hit.box.size.width, hit.box.size.height) * scaled.scale;

            double radiusPx = Math.max(fullWidthPx, fullHeightPx) * 0.5;



            VidarElementObservation obs = VidarGeometry.fuseElementObservation(

                    absCx, absCy, radiusPx, hit.contourArea, hit.aspect, hit.rectangularity,

                    hit.rectangularity, 0.5, target.detector, profile, cameraName,

                    captureTimeNanos, hit.touchesBoundary, false, hit.rectangularity,

                    elementSpec, season);

            if (obs.confidence < season.minElementConfidence) {

                continue;

            }

            double score = obs.confidence * hit.contourArea;

            frameCandidates.add(new ScoredObservation(obs, score));

            if (score > bestScore) {

                bestScore = score;

                bestForTarget = obs;

            }

        }

        if (bestForTarget != null) {

            bestGameById.put(target.id, bestForTarget);

        }

    }



    private void processPlateTarget(

            VidarContourTarget target,

            VidarFramePipeline.ScaledRoi scaled,

            int frameW,

            int frameH,

            long captureTimeNanos,

            VidarContourWorkspace workspace) {

        List<VidarContourDetect.RectHit> hits = VidarContourDetect.findRectHits(

                scaled.image, reusableMask, reusableHierarchy, target, scaled,

                frameW, frameH, profile, workspace);



        VidarPlateSpec plateSpec = season.plateSpec(target.alliance);

        VidarPlateObservation bestForAlliance = null;

        double bestScore = -1;



        for (VidarContourDetect.RectHit hit : hits) {

            double absCx = scaled.toFullX(hit.box.center.x);

            double absCy = scaled.toFullY(hit.box.center.y);

            double fullWidthPx = Math.max(hit.box.size.width, hit.box.size.height) * scaled.scale;

            double fullHeightPx = Math.min(hit.box.size.width, hit.box.size.height) * scaled.scale;



            double rotationPenalty = Math.abs(hit.box.angle % 90) / 45.0;

            double partialPenalty = hit.rectangularity < 0.65 ? 0.7 : 1.0;

            double viewingPenalty = 1.0 - Math.min(0.5, rotationPenalty * 0.25);



            double dWidth = VidarGeometry.distanceFromWidth(

                    profile.plateWidth, profile.focalLengthPx, fullWidthPx);

            double cyForFloor = (absCy - scaled.sourceCrop.y) / scaled.scale;

            boolean nearHorizon = absCy <= profile.horizonRowPx + 8;

            double horizonConf = profile.horizonRowPx > 0
                    ? Math.max(0.3, 1.0 - profile.horizonRowPx / 120.0) : 0.5;

            double dFloor = VidarGeometry.distanceFromFloor(cyForFloor, profile);

            double dGround = VidarGeometry.distanceFromGroundPlane(absCx, absCy, profile, 0.0);

            VidarRangeResult rangeResult = VidarGeometry.fusePlateRange(

                    absCx, absCy, cyForFloor,

                    dWidth, fullWidthPx, hit.rectangularity, hit.whiteRatio,

                    partialPenalty < 1.0, hit.touchesBoundary, rotationPenalty,

                    nearHorizon, horizonConf, profile, season.maxRangeMismatchRatio);

            double range = rangeResult.isValid() ? rangeResult.distance : Double.NaN;

            double confidence = VidarGeometry.composePlateConfidence(

                    hit.whiteRatio, hit.contourArea, hit.rectangularity, hit.aspect,

                    rangeResult, viewingPenalty, partialPenalty, plateSpec);

            if (confidence < season.minPlateConfidence) {

                continue;

            }



            double[] robotPoint = VidarGeometry.floorPointInRobot(absCx, absCy, range, profile);

            double score = confidence * hit.contourArea;

            if (score > bestScore) {

                bestScore = score;

                bestForAlliance = new VidarPlateObservation(

                        target.alliance,

                        absCx,

                        absCy,

                        fullWidthPx,

                        fullHeightPx,

                        hit.box.angle,

                        hit.aspect,

                        hit.whiteRatio,

                        range,

                        rangeResult.uncertainty,

                        dWidth,

                        dFloor,

                        dGround,

                        rangeResult,

                        viewingPenalty,

                        partialPenalty,

                        confidence,

                        robotPoint[0],

                        robotPoint[1],

                        cameraName,

                        captureTimeNanos);

                bestPlateDraw = hit.box;

            }

        }



        if (bestForAlliance == null) {

            return;

        }

        if (target.alliance == VidarAlliance.RED) {

            bestRed = bestForAlliance;

        } else {

            bestBlue = bestForAlliance;

        }

    }



    private VidarElementSpec seasonElement(String id) {

        VidarElementSpec spec = seasonElementsById.get(id);

        return spec != null ? spec : season.primaryElement();

    }



    private VidarElementObservation pickBestGame() {

        VidarElementObservation best = rankedFrameRef.get().best();

        if (best != null) {

            return best;

        }

        double bestScore = -1;

        for (VidarElementObservation obs : bestGameById.values()) {

            double score = obs.confidence * obs.areaPx;

            if (score > bestScore) {

                bestScore = score;

                best = obs;

            }

        }

        return best;

    }



    private static VidarPlateObservation pickStronger(VidarPlateObservation a, VidarPlateObservation b) {

        if (a == null) return b;

        if (b == null) return a;

        return a.confidence >= b.confidence ? a : b;

    }



    private void recordTime(long t0) {

        if (metrics != null) {

            metrics.recordProcessorTime("element", (System.nanoTime() - t0) / 1_000_000.0);

        }

    }



    public VidarElementObservation getBestElement() {

        return bestGame;

    }



    public VidarRankedElementFrame getRankedElements() {

        return rankedFrameRef.get();

    }



    public synchronized VidarElementObservation getGameElement(String id) {

        return bestGameById.get(id);

    }



    public synchronized Map<String, VidarElementObservation> getGameElements() {

        return new HashMap<>(bestGameById);

    }



    public VidarPlateObservation getBestPlate() {

        return bestPlate;

    }



    public VidarPlateObservation getBestRed() {

        return bestRed;

    }



    public VidarPlateObservation getBestBlue() {

        return bestBlue;

    }



    public VidarElementRejectionStats getRejectionStats() {

        return rejectionStats;

    }



    @Override

    public void onDrawFrame(

            Canvas canvas,

            int onscreenWidth,

            int onscreenHeight,

            float scaleBmpPxToCanvasPx,

            float scaleCanvasDensity,

            Object userContext) {

        if (bestGameDraw != null) {

            if (gameDrawStroke == null) {

                gameDrawStroke = new Paint();

                gameDrawStroke.setColor(Color.rgb(220, 220, 240));

                gameDrawStroke.setStyle(Paint.Style.STROKE);

                gameDrawStroke.setAntiAlias(true);

            }

            gameDrawStroke.setStrokeWidth(3f * scaleCanvasDensity);

            float cx = (float) (bestGameDraw.cx * scaleBmpPxToCanvasPx);

            float cy = (float) (bestGameDraw.cy * scaleBmpPxToCanvasPx);

            float r = (float) (bestGameDraw.radiusPx * scaleBmpPxToCanvasPx);

            canvas.drawCircle(cx, cy, r, gameDrawStroke);

        }



        if (bestPlate != null) {

            if (plateDrawStroke == null) {

                plateDrawStroke = new Paint();

                plateDrawStroke.setStyle(Paint.Style.STROKE);

                plateDrawStroke.setAntiAlias(true);

            }

            plateDrawStroke.setStrokeWidth(3f * scaleCanvasDensity);

            plateDrawStroke.setColor(bestPlate.alliance == VidarAlliance.RED

                    ? Color.rgb(255, 90, 90)

                    : Color.rgb(90, 140, 255));

            RotatedRect draw = bestPlate.toRotatedRect();

            Point[] pts = new Point[4];

            draw.points(pts);

            for (int i = 0; i < 4; i++) {

                float x1 = (float) (pts[i].x * scaleBmpPxToCanvasPx);

                float y1 = (float) (pts[i].y * scaleBmpPxToCanvasPx);

                float x2 = (float) (pts[(i + 1) % 4].x * scaleBmpPxToCanvasPx);

                float y2 = (float) (pts[(i + 1) % 4].y * scaleBmpPxToCanvasPx);

                canvas.drawLine(x1, y1, x2, y2, plateDrawStroke);

            }

        }

    }

}


