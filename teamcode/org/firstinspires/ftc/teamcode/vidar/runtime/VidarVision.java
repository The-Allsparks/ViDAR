package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.detect.VidarContourProcessor;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameMailbox;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarMotionCorrection;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementRejectionStats;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarCameraScheduler;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarProcessScheduler;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.TagDecodeBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarAdaptiveTagProcessor;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagConfig;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagDecodeWorker;
import org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;

/**
 * One camera: tic-toc element / plate / tag at 640×480 with shared scheduler per portal.
 */
public class VidarVision {

    private final VidarProcessScheduler scheduler;
    private final VidarContourProcessor contourProcessor;
    private final VidarAdaptiveTagProcessor tagProcessor;
    private final VidarCameraScheduler cameraScheduler;
    private final VidarMetrics metrics;
    private final org.firstinspires.ftc.vision.VisionPortal portal;
    private final VidarCameraProfile profile;
    private final String cameraName;
    private final VidarSeasonConfig season;
    private final VidarFrameMailbox frameMailbox;
    private final boolean asyncWorkerEnabled;
    private volatile boolean excludedFromRotation;
    private boolean failed;

    private VidarElementObservation bestElement;
    private VidarPlateObservation bestPlate;

    private int lastProcessedGeneration;

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            CameraPipelineConfig config) {
        this(
                hardwareMap,
                config.cameraName,
                config.profile,
                config.odomSupplier,
                config.portalLabel,
                config.fieldPosePriorSupplier,
                config.season,
                config.resourceBudget,
                config.robotCameraCount,
                config.cameraIndex,
                config.decodeBudget,
                config.decodeWorker);
    }

    public VidarVision(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        this(hardwareMap, VidarConfig.CAMERA_NAME, VidarConfig.cameraProfile(), null, VidarConfig.CAMERA_NAME, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName) {
        this(hardwareMap, cameraName, VidarConfig.cameraProfile(), null, cameraName, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile) {
        this(hardwareMap, cameraName, profile, null, cameraName, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier) {
        this(hardwareMap, cameraName, profile, odomSupplier, cameraName, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel) {
        this(hardwareMap, cameraName, profile, odomSupplier, portalLabel, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier) {
        this(hardwareMap, cameraName, profile, odomSupplier, portalLabel, fieldPosePriorSupplier, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier,
            VidarSeasonConfig season) {
        this(hardwareMap, cameraName, profile, odomSupplier, portalLabel, fieldPosePriorSupplier, season, null, 1, 0);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier,
            VidarSeasonConfig season,
            VidarResourceBudget resourceBudget,
            int robotCameraCount) {
        this(hardwareMap, cameraName, profile, odomSupplier, portalLabel, fieldPosePriorSupplier,
                season, resourceBudget, robotCameraCount, 0);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier,
            VidarSeasonConfig season,
            VidarResourceBudget resourceBudget,
            int robotCameraCount,
            int cameraIndex) {
        this(hardwareMap, cameraName, profile, odomSupplier, portalLabel, fieldPosePriorSupplier,
                season, resourceBudget, robotCameraCount, cameraIndex, null, null);
    }

    public VidarVision(
            com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier,
            VidarSeasonConfig season,
            VidarResourceBudget resourceBudget,
            int robotCameraCount,
            int cameraIndex,
            TagDecodeBudget decodeBudget,
            VidarTagDecodeWorker decodeWorker) {
        this.profile = profile;
        this.cameraName = portalLabel;
        this.season = season != null ? season : VidarConfigLoader.defaultSeason();
        metrics = new VidarMetrics(portalLabel);

        asyncWorkerEnabled = VidarConfig.useGlobalVisionWorker(robotCameraCount);
        scheduler = new VidarProcessScheduler(asyncWorkerEnabled ? (cameraIndex % 2) : 0);
        cameraScheduler = new VidarCameraScheduler();

        contourProcessor = new VidarContourProcessor(
                profile, portalLabel, scheduler, metrics, this.season, resourceBudget);
        tagProcessor = new VidarAdaptiveTagProcessor(
                scheduler, profile, portalLabel, metrics, this.season, resourceBudget,
                decodeBudget, decodeWorker);

        frameMailbox = new VidarFrameMailbox(metrics);
        contourProcessor.setFrameMailbox(frameMailbox);
        tagProcessor.setFrameMailbox(frameMailbox);
        if (asyncWorkerEnabled) {
            contourProcessor.setMailboxDrainCallback(null);
        } else {
            contourProcessor.setMailboxDrainCallback(this::drainMailboxSync);
        }

        org.firstinspires.ftc.vision.VisionPortal.Builder builder =
                new org.firstinspires.ftc.vision.VisionPortal.Builder()
                        .addProcessor(contourProcessor)
                        .setCameraResolution(VidarConfig.portalCameraResolution())
                        .setStreamFormat(VidarConfig.portalStreamFormat(robotCameraCount))
                        .enableLiveView(VidarConfig.LIVE_VIEW_ENABLED)
                        .setCamera(hardwareMap.get(
                                org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                                cameraName));

        if (VidarTagConfig.ENABLED) {
            builder.addProcessor(tagProcessor);
        }

        portal = builder.build();
        VidarTagGate.setCameraBearingDeg(profile.bearingDeg);
        metrics.setHealth(VidarMetrics.CameraHealth.STREAMING);
    }

    public void update() {
        try {
            bestElement = contourProcessor.getBestElement();
            bestPlate = contourProcessor.getBestPlate();
            metrics.setPortalFps(portal.getFps());
            if (!failed) {
                metrics.setHealth(VidarMetrics.CameraHealth.HEALTHY);
            }
        } catch (RuntimeException ex) {
            failed = true;
            metrics.setHealth(VidarMetrics.CameraHealth.FAILED);
            metrics.setLastError(ex.getMessage());
        }
    }

    /** Drop this camera from worker rotation and disable processors (e.g. hang/error). */
    public void setExcludedFromRotation(boolean excluded) {
        excludedFromRotation = excluded;
        if (excluded) {
            setIdle(true);
            metrics.setHealth(VidarMetrics.CameraHealth.FAILED);
        } else {
            setIdle(false);
        }
    }

    public boolean isExcludedFromRotation() {
        return excludedFromRotation;
    }

    public void setMaxRankedElements(int max) {
        contourProcessor.setMaxRankedElements(max);
    }

    /** Apply camera processing state (PRIMARY, SECONDARY, IDLE, DEEP_IDLE). Default is PRIMARY. */
    public void setCameraState(VidarCameraScheduler.State state) {
        if (state == null) {
            state = VidarCameraScheduler.State.PRIMARY;
        }
        cameraScheduler.apply(portal, contourProcessor, tagProcessor, state, metrics);
    }

    /** Enable or disable vision processing on this camera while keeping the stream alive. */
    public void setIdle(boolean idle) {
        setCameraState(idle ? VidarCameraScheduler.State.IDLE : VidarCameraScheduler.State.PRIMARY);
    }

    /**
     * Opt-in direction-based PRIMARY/SECONDARY only — never sets IDLE.
     * Prefer {@link #setCameraState(VidarCameraScheduler.State)} for explicit control.
     */
    public void applyDirectionTier(double travelHeadingDeg, double speedInPerSec) {
        VidarCameraScheduler.State state =
                cameraScheduler.tierForCamera(profile.bearingDeg, travelHeadingDeg, speedInPerSec);
        cameraScheduler.apply(portal, contourProcessor, tagProcessor, state, metrics);
    }

    public VidarElementObservation getGameElement(String id) {
        return contourProcessor.getGameElement(id);
    }

    public java.util.Map<String, VidarElementObservation> getGameElements() {
        return contourProcessor.getGameElements();
    }

    public VidarElementObservation getBestElement() {
        return bestElement;
    }

    public VidarRankedElementFrame getRankedElements() {
        return contourProcessor.getRankedElements();
    }

    public VidarPlateObservation getBestPlate() {
        return bestPlate;
    }

    public VidarTagScoutObservation getLastTagScout() {
        return tagProcessor.getLastScout();
    }

    public VidarTagObservation getLatestTag() {
        return tagProcessor.getLatestTag();
    }

    public VidarTagScoutObservation getLatestScoutObservation() {
        return tagProcessor.getLatestScoutObservation();
    }

    public org.firstinspires.ftc.robotcore.external.navigation.Pose2D getBackdatedFieldPose(
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D odomAtCapture,
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D odomNow) {
        return VidarMotionCorrection.tagFieldNow(getLatestTag(), odomAtCapture, odomNow);
    }

    public VidarSeasonConfig getSeasonConfig() {
        return season;
    }

    public VidarCameraProfile getProfile() {
        return profile;
    }

    public String getCameraName() {
        return cameraName;
    }

    public org.firstinspires.ftc.vision.VisionPortal getPortal() {
        return portal;
    }

    public VidarCameraScheduler.State directionState() {
        return cameraScheduler.currentState();
    }

    public VidarMetrics metrics() {
        return metrics;
    }

    public VidarElementRejectionStats elementRejectionStats() {
        return contourProcessor.getRejectionStats();
    }

    public boolean isFailed() {
        return failed;
    }

    public boolean hasAsyncWorker() {
        return asyncWorkerEnabled;
    }

    public VidarFrameMailbox frameMailbox() {
        return frameMailbox;
    }

    public boolean isWorkerProcessingAllowed() {
        if (excludedFromRotation || failed) {
            return false;
        }
        VidarCameraScheduler.State state = cameraScheduler.currentState();
        return state != VidarCameraScheduler.State.IDLE
                && state != VidarCameraScheduler.State.DEEP_IDLE;
    }

    public void processSnapshot(VidarFrameMailbox.Snapshot snap) {
        if (snap == null || snap.frame == null || snap.frame.empty()) {
            return;
        }
        VidarCameraScheduler.State state = cameraScheduler.currentState();
        if (state == VidarCameraScheduler.State.IDLE
                || state == VidarCameraScheduler.State.DEEP_IDLE) {
            return;
        }

        VidarProcessScheduler.Slot slot = scheduler.beginFrame(snap.captureTimeNanos);
        if (slot == VidarProcessScheduler.Slot.ELEMENT) {
            contourProcessor.processElementPass(snap.frame, snap.captureTimeNanos);
        } else if (state == VidarCameraScheduler.State.PRIMARY) {
            tagProcessor.processTagPass(snap.frame, snap.captureTimeNanos);
        } else if (metrics != null) {
            metrics.incrementSkippedSlots();
        }
    }

    /** Synchronous mailbox drain for single-camera setups (same snapshot path as the global worker). */
    void drainMailboxSync() {
        if (frameMailbox == null || !isWorkerProcessingAllowed()) {
            return;
        }
        VidarFrameMailbox.Snapshot snap = frameMailbox.tryTake(lastProcessedGeneration);
        if (snap != null) {
            lastProcessedGeneration = snap.generation;
            processSnapshot(snap);
        }
    }

    public float portalFps() {
        return portal.getFps();
    }

    public void close() {
        if (frameMailbox != null) {
            frameMailbox.release();
        }
        portal.close();
    }
}
