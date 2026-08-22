package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarCalibrationDiagnostics;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarTransformRegistry;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetricsLogger;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntimeConfig;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarCameraScheduler;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.TagDecodeBudget;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Multi-camera fusion engine — polls cameras from {@link org.firstinspires.ftc.teamcode.vidar.runtime.VidarVisionAttachment}
 * and fuses observations. Owned by {@link org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntime}; not constructed by teams.
 */
public final class VidarFusionEngine implements VidarVisionFusion {

    private final VidarVision[] cameras;
    private final int cameraCount;
    private final VidarRobotConfig robotConfig;
    private final VidarSeasonConfig season;
    private final Supplier<VidarAlliance> ourAlliance;
    private final Supplier<Pose2D> odomSupplier;
    private final VidarLocalizationFusion localization = new VidarLocalizationFusion();
    private final VidarRuntimeConfig runtimeConfig = new VidarRuntimeConfig();
    private final VidarTemporalFilter temporalFilter = new VidarTemporalFilter(runtimeConfig);
    private final VidarMetricsLogger metricsLogger = new VidarMetricsLogger();
    private final VidarResourceBudget resourceBudget;
    private final VidarOdomHistory odomHistory = new VidarOdomHistory();
    private final VidarTransformRegistry transformRegistry;
    private final VidarCalibrationDiagnostics calibrationDiagnostics =
            new VidarCalibrationDiagnostics();
    private final TagDecodeBudget tagDecodeBudget;

    public static VidarFusionEngine create(
            VidarVision[] cameras,
            int cameraCount,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> ourAlliance,
            TagDecodeBudget tagDecodeBudget,
            VidarResourceBudget resourceBudget) {
        return new VidarFusionEngine(
                cameras,
                cameraCount,
                robot,
                season,
                odomSupplier,
                ourAlliance,
                tagDecodeBudget,
                resourceBudget);
    }

    private Pose2D lastOdomSample;
    private long lastOdomNanos;
    private double travelHeadingDeg;
    private double speedInPerSec;

    private VidarElementObservation bestElement;
    private VidarRankedElementFrame fusedRankedElements =
            VidarRankedElementFrame.empty("fused", VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
    private VidarPlateObservation bestPlate;
    private VidarPlateObservation bestFoe;
    private VidarPlateObservation bestAlly;
    private VidarTagObservation latestTag;
    private VidarTagScoutObservation latestScoutObservation;
    private VidarTagScoutObservation lastTagScout;
    private Pose2D fusedFieldPose;
    private Pose2D odomAtLastFusedFieldPose;
    private volatile VidarObservationFrame latestFrame = VidarObservationFrame.empty();

    private VidarFusionEngine(
            VidarVision[] cameras,
            int cameraCount,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> ourAlliance,
            TagDecodeBudget tagDecodeBudget,
            VidarResourceBudget resourceBudget) {
        this.cameras = cameras;
        this.cameraCount = cameraCount;
        this.season = season != null ? season : VidarConfigLoader.defaultSeason();
        VidarRobotConfig activeRobot = robot != null ? robot : VidarConfigLoader.defaultRobot();
        this.robotConfig = activeRobot;
        this.transformRegistry = new VidarTransformRegistry(activeRobot);
        this.calibrationDiagnostics.updateFromRegistry(
                transformRegistry, activeRobot.activeCameraIndex);
        this.ourAlliance = ourAlliance == null ? () -> activeRobot.defaultAlliance : ourAlliance;
        this.odomSupplier = odomSupplier;
        this.tagDecodeBudget = tagDecodeBudget != null ? tagDecodeBudget : new TagDecodeBudget();
        this.resourceBudget = resourceBudget != null ? resourceBudget : new VidarResourceBudget();
        this.tagDecodeBudget.reset();
    }

    public void setFieldPosePrior(Pose2D prior) {
        localization.setFieldPosePrior(prior);
    }

    /**
     * Poll all cameras, fuse observations, and return an immutable snapshot for this cycle.
     * Also available via {@link #getLatestFrame()} if you prefer a two-call pattern.
     */
    public VidarObservationFrame update() {
        long loopStart = System.nanoTime();
        long updateTimeNanos = System.nanoTime();

        VidarRankedElementFrame[] rankedByCamera = new VidarRankedElementFrame[cameraCount];
        VidarTagObservation[] tagsByCamera = new VidarTagObservation[cameraCount];

        int connected = 0;
        for (VidarVision camera : cameras) {
            if (camera == null) {
                continue;
            }
            connected++;
            camera.metrics().beginCycle();
            try {
                camera.update();
                camera.metrics().recordLoopCpu((System.nanoTime() - loopStart) / 1_000_000.0);
            } catch (RuntimeException ex) {
                camera.metrics().setHealth(VidarMetrics.CameraHealth.FAILED);
                camera.metrics().setLastError(ex.getMessage());
            }
        }

        resourceBudget.update(collectMetrics(), connected);
        applyResourceBudgetActions();
        metricsLogger.recordCycle(collectMetrics());

        bestElement = null;
        fusedRankedElements = MultiCameraFusion.fuseRankedElements(
                cameras, temporalFilter, runtimeConfig.fusionMaxRankedElements());
        if (fusedRankedElements.best() != null) {
            bestElement = fusedRankedElements.best();
        }
        bestPlate = null;
        bestFoe = null;
        bestAlly = null;
        latestTag = null;
        latestScoutObservation = null;
        lastTagScout = null;

        double bestPlateScore = -1;
        double bestFoeScore = -1;
        double bestAllyScore = -1;
        long newestTagTime = Long.MIN_VALUE;
        int bestDecodePixels = -1;
        double bestScoutScore = -1;

        for (int i = 0; i < cameraCount; i++) {
            VidarVision camera = cameras[i];
            if (!MultiCameraFusion.isUsableCamera(camera)) {
                continue;
            }

            rankedByCamera[i] = camera.getRankedElements();
            tagsByCamera[i] = camera.getLatestTag();

            VidarPlateObservation plate = camera.getBestPlate();
            if (plate != null) {
                plate = temporalFilter.filterPlate(plate);
                if (plate != null) {
                    double score = plateScore(plate);
                    VidarAlliance ours = ourAlliance.get();
                    if (score > bestPlateScore) {
                        bestPlateScore = score;
                        bestPlate = plate;
                    }
                    if (plate.isFoe(ours) && score > bestFoeScore) {
                        bestFoeScore = score;
                        bestFoe = plate;
                    }
                    if (plate.isAlly(ours) && score > bestAllyScore) {
                        bestAllyScore = score;
                        bestAlly = plate;
                    }
                }
            }

            VidarTagObservation tag = camera.getLatestTag();
            if (tag != null) {
                if (tag.decodePixels > bestDecodePixels
                        || (tag.decodePixels == bestDecodePixels && tag.captureTimeNanos > newestTagTime)) {
                    bestDecodePixels = tag.decodePixels;
                    newestTagTime = tag.captureTimeNanos;
                    latestTag = tag;
                }
            }

            VidarTagScoutObservation scout = camera.getLatestScoutObservation();
            if (scout != null) {
                double score = scout.scoutConfidence * scout.apparentWidthPx;
                if (score > bestScoutScore) {
                    bestScoutScore = score;
                    latestScoutObservation = scout;
                }
            }

            VidarTagScoutObservation scoutResult = camera.getLastTagScout();
            if (scoutResult != null && (lastTagScout == null
                    || scoutResult.apparentWidthPx > lastTagScout.apparentWidthPx)) {
                lastTagScout = scoutResult;
            }
        }

        latestFrame = new VidarObservationFrame(
                updateTimeNanos,
                fusedRankedElements,
                bestElement,
                bestPlate,
                bestFoe,
                bestAlly,
                latestTag,
                latestScoutObservation,
                lastTagScout,
                rankedByCamera,
                tagsByCamera);
        calibrationDiagnostics.updateFromRegistry(
                transformRegistry, robotConfig.activeCameraIndex);
        if (bestElement != null && bestElement.captureTimeNanos > 0) {
            calibrationDiagnostics.recordObservationAge(
                    (updateTimeNanos - bestElement.captureTimeNanos) / 1_000_000.0);
        }
        refreshFusedFieldPose();
        return latestFrame;
    }

    private void refreshFusedFieldPose() {
        if (odomSupplier == null) {
            fusedFieldPose = localization.lastFusedFieldPose();
            return;
        }
        Pose2D odomNow = odomSupplier.get();
        if (latestTag != null && latestTag.captureTimeNanos > 0) {
            Pose2D odomAtCapture = odomHistory.at(latestTag.captureTimeNanos);
            if (odomAtCapture == null) {
                // Cannot latency-compensate — keep prior fused; do not stamp odom-at-fuse.
                fusedFieldPose = localization.lastFusedFieldPose();
                return;
            }
            VidarLocalizationFusion.Result result = localization.fusedFieldPoseNow(
                    latestTag, latestScoutObservation, odomAtCapture, odomNow);
            fusedFieldPose = result.pose;
            // Only stamp odom when a gate-accepted correction lands — otherwise fuse→setPose
            // re-propagation collapses to (now - now) on the next worker tick.
            if (result.acceptedNewCorrection) {
                odomAtLastFusedFieldPose = odomNow;
            }
            return;
        }
        fusedFieldPose = localization.lastFusedFieldPose();
    }

    /**
     * Field pose for motion-corrected world tracks — extrapolates tag fusion with odom between
     * decode cycles. When odom is configured but no tag anchor exists yet, uses odom directly
     * (Pinpoint / Pedro field-relative pose).
     */
    public Pose2D getFieldPoseForMotionTracking() {
        if (odomSupplier == null) {
            if (fusedFieldPose != null) {
                return fusedFieldPose;
            }
            return localization.fieldPosePrior();
        }
        Pose2D odomNow = odomSupplier.get();
        if (odomNow == null) {
            return fusedFieldPose;
        }
        if (latestTag != null && latestTag.fieldPoseAtCapture != null) {
            Pose2D odomAtCapture = odomHistory.at(latestTag.captureTimeNanos);
            if (odomAtCapture == null) {
                odomAtCapture = odomNow;
            }
            Pose2D backdated = VidarMotionCorrection.tagFieldNow(latestTag, odomAtCapture, odomNow);
            if (backdated != null) {
                return backdated;
            }
        }
        Pose2D anchor = localization.lastFusedFieldPose();
        if (anchor != null && odomAtLastFusedFieldPose != null) {
            return VidarMotionCorrection.robotFieldPoseNow(anchor, odomAtLastFusedFieldPose, odomNow);
        }
        return odomNow;
    }

    /** Last snapshot from {@link #update()} — safe to read multiple times per cycle. */
    public VidarObservationFrame getLatestFrame() {
        return latestFrame;
    }

    private void applyResourceBudgetActions() {
        if (!resourceBudget.shouldIdleRearCameras()) {
            return;
        }
        for (VidarVision camera : cameras) {
            if (camera == null || camera.isExcludedFromRotation()) {
                continue;
            }
            if (Math.abs(camera.getProfile().bearingDeg) > 90.0) {
                camera.setIdle(true);
            }
        }
    }

    private VidarMetrics[] collectMetrics() {
        VidarMetrics[] out = new VidarMetrics[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            out[i] = cameras[i] == null ? null : cameras[i].metrics();
        }
        return out;
    }

    private void sampleOdomMotion() {
        if (odomSupplier == null) {
            speedInPerSec = 0;
            return;
        }
        Pose2D now = odomSupplier.get();
        recordOdom(now);
    }

    /** Ring buffer for latency compensation — record each loop before {@link #update()}. */
    public VidarOdomHistory odomHistory() {
        return odomHistory;
    }

    /** Append an odometry sample (uses {@link System#nanoTime()}). */
    public void recordOdom(Pose2D odom) {
        odomHistory.record(odom);
        sampleOdomMotionFromPose(odom);
    }

    /**
     * {@link #update()} then batch backdate all observations to robot-now.
     * When an odom supplier is configured, records odom before vision and uses a fresh sample for {@code odomNow}.
     */
    public VidarCorrectedFrame updateCorrected() {
        Pose2D odomNow = null;
        if (odomSupplier != null) {
            odomNow = odomSupplier.get();
            recordOdom(odomNow);
        }
        VidarObservationFrame frame = update();
        if (odomSupplier != null) {
            odomNow = odomSupplier.get();
            odomHistory.record(odomNow);
        }
        return frame.toRobotNow(odomHistory, odomNow);
    }

    private void sampleOdomMotionFromPose(Pose2D now) {
        if (now == null) {
            speedInPerSec = 0;
            return;
        }
        long t = System.nanoTime();
        if (lastOdomSample != null && lastOdomNanos > 0) {
            double dt = (t - lastOdomNanos) / 1e9;
            if (dt > 0.02) {
                double dx = now.getX(DistanceUnit.INCH) - lastOdomSample.getX(DistanceUnit.INCH);
                double dy = now.getY(DistanceUnit.INCH) - lastOdomSample.getY(DistanceUnit.INCH);
                speedInPerSec = Math.hypot(dx, dy) / dt;
                if (speedInPerSec > 1.0) {
                    travelHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
                }
            }
        }
        lastOdomSample = now;
        lastOdomNanos = t;
    }

    private double elementScore(VidarElementObservation element) {
        if (element.confidence < season.minElementConfidence) {
            return -1;
        }
        return MultiCameraFusion.elementRankScore(element);
    }

    private static double plateScore(VidarPlateObservation plate) {
        double rangeWeight = Double.isNaN(plate.range) ? 0.5 : 1.0 / Math.max(8.0, plate.range);
        return plate.confidence * plate.whiteRatio * rangeWeight;
    }

    public VidarSeasonConfig getSeasonConfig() {
        return season;
    }

    public VidarRobotConfig getRobotConfig() {
        return robotConfig;
    }

    public VidarCalibrationDiagnostics calibrationDiagnostics() {
        return calibrationDiagnostics;
    }

    /** Active linear unit for config distances and observation fields (robot overrides season). */
    public VidarDistanceUnit distanceUnit() {
        return VidarDistanceUnit.effective(robotConfig, season);
    }

    public int getCameraCount() {
        return cameraCount;
    }

    public VidarVision camera(int index) {
        if (index < 0 || index >= cameraCount) {
            return null;
        }
        return cameras[index];
    }

    /** Set processing state for one camera (PRIMARY, SECONDARY, IDLE, DEEP_IDLE). */
    public void setCameraState(int index, VidarCameraScheduler.State state) {
        VidarVision camera = camera(index);
        if (camera != null) {
            camera.setCameraState(state);
        }
    }

    /** Idle one camera (processors off, stream stays on). Resumes PRIMARY when {@code idle} is false. */
    public void setCameraIdle(int index, boolean idle) {
        VidarVision camera = camera(index);
        if (camera != null) {
            camera.setIdle(idle);
        }
    }

    /** Exclude a hung/errored camera from worker rotation and disable its processors. */
    public void setCameraFailed(int index, boolean failed) {
        VidarVision camera = camera(index);
        if (camera != null) {
            camera.setExcludedFromRotation(failed);
        }
    }

    /** Per-camera ranked-element cap (e.g. front=8, side=2). */
    public void setCameraMaxRankedElements(int index, int max) {
        VidarVision camera = camera(index);
        if (camera != null) {
            camera.setMaxRankedElements(max);
        }
    }

    public VidarRuntimeConfig runtimeConfig() {
        return runtimeConfig;
    }

    public VidarMetricsLogger metricsLogger() {
        return metricsLogger;
    }

    /**
     * Optional tag fusion helper — ViDAR does not consume odometry during vision processing.
     * Team supplies {@code odomNow} when fusing.
     */
    public VidarLocalizationFusion localizationFusion() {
        return localization;
    }

    /**
     * Opt-in direction-based PRIMARY/SECONDARY for all cameras — never auto-idles.
     * Requires {@link VidarConfig#DIRECTION_SCHEDULER_ENABLED} and a motion sample source.
     */
    public void applyDirectionTier(double travelHeadingDeg, double speedInPerSec) {
        for (VidarVision camera : cameras) {
            if (camera == null || camera.isFailed()) {
                continue;
            }
            camera.applyDirectionTier(travelHeadingDeg, speedInPerSec);
        }
    }

    public VidarElementObservation getBestElement() {
        return bestElement;
    }

    /**
     * Fused 360° element ranks 0 … fusion cap−1 in robot frame.
     * {@link VidarRankedElementFrame#overflowCount()} is the overflow bucket across all cameras.
     */
    public VidarRankedElementFrame getRankedElements() {
        return fusedRankedElements;
    }

    /** Fused rank {@code 0} (best) … {@code 4}, or null. */
    public VidarElementObservation getRankedElement(int rank) {
        return fusedRankedElements.at(rank);
    }

    /** Best fused observation for a specific season element id (e.g. {@code artifact_purple}). */
    public VidarElementObservation getGameElement(String elementId) {
        VidarElementObservation best = null;
        double bestScore = -1;
        for (VidarVision camera : cameras) {
            if (camera == null || camera.isFailed()) {
                continue;
            }
            VidarElementObservation obs = camera.getGameElement(elementId);
            if (obs == null) {
                continue;
            }
            obs = temporalFilter.filterElement(obs);
            if (obs == null) {
                continue;
            }
            double score = elementScore(obs);
            if (score > bestScore) {
                bestScore = score;
                best = obs;
            }
        }
        return best;
    }

    /** Best per-season-type element observations merged across cameras. */
    public Map<String, VidarElementObservation> getGameElements() {
        Map<String, VidarElementObservation> merged = new HashMap<>();
        for (VidarVision camera : cameras) {
            if (camera == null || camera.isFailed()) {
                continue;
            }
            for (Map.Entry<String, VidarElementObservation> entry : camera.getGameElements().entrySet()) {
                VidarElementObservation obs = temporalFilter.filterElement(entry.getValue());
                if (obs == null) {
                    continue;
                }
                VidarElementObservation existing = merged.get(entry.getKey());
                if (existing == null || elementScore(obs) > elementScore(existing)) {
                    merged.put(entry.getKey(), obs);
                }
            }
        }
        return merged;
    }

    public VidarPlateObservation getBestPlate() {
        return bestPlate;
    }

    public VidarPlateObservation getBestFoe() {
        return bestFoe;
    }

    public VidarPlateObservation getBestAlly() {
        return bestAlly;
    }

    public VidarTagScoutObservation getLastTagScout() {
        return lastTagScout;
    }

    public VidarTagObservation getLatestTag() {
        return latestTag;
    }

    public VidarTagScoutObservation getLatestScoutObservation() {
        return latestScoutObservation;
    }

    /** Latest localization fusion result (tag + team odom). Never uses external Pedro supplier. */
    public Pose2D getFusedFieldPose() {
        return fusedFieldPose;
    }

    /** {@link System#nanoTime()} when the last gate-accepted tag correction was applied (0 = none). */
    public long lastTagCorrectionNanos() {
        return localization.lastCorrectionNanos();
    }

    /**
     * Gated tag fix re-propagated to the current odom sample — safe for {@code follower.setPose}.
     *
     * <p>Unlike {@link #getFieldPoseForMotionTracking()}, this never uses an ungated {@code latestTag}
     * and never falls back to raw odom. Returns {@code null} until a pose gate has accepted a fix.
     */
    public Pose2D getGatedTagCorrectedFieldPoseNow() {
        Pose2D anchor = localization.lastFusedFieldPose();
        if (anchor == null) {
            return null;
        }
        if (odomSupplier == null) {
            return anchor;
        }
        Pose2D odomNow = odomSupplier.get();
        if (odomNow == null || odomAtLastFusedFieldPose == null) {
            return anchor;
        }
        return VidarMotionCorrection.robotFieldPoseNow(anchor, odomAtLastFusedFieldPose, odomNow);
    }

    /**
     * Backdate the best tag to {@code odomNow} using odometry at tag capture time.
     */
    public Pose2D getBackdatedFieldPose(Pose2D odomAtCapture, Pose2D odomNow) {
        VidarTagObservation tag = latestTag;
        if (tag == null) {
            return null;
        }
        return VidarMotionCorrection.tagFieldNow(tag, odomAtCapture, odomNow);
    }

    public double travelSpeedInPerSec() {
        return speedInPerSec;
    }

    public VidarResourceBudget resourceBudget() {
        return resourceBudget;
    }

    /** Clear fusion scratch and temporal filter between match periods. */
    public void resetMatchState() {
        temporalFilter.resetMatchState();
        localization.resetMatchState();
        bestElement = null;
        fusedRankedElements = VidarRankedElementFrame.empty("fused", VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
        bestPlate = null;
        bestFoe = null;
        bestAlly = null;
        latestTag = null;
        latestScoutObservation = null;
        lastTagScout = null;
        fusedFieldPose = null;
        odomAtLastFusedFieldPose = null;
        latestFrame = VidarObservationFrame.empty();
        lastOdomSample = null;
        lastOdomNanos = 0;
        speedInPerSec = 0;
    }
}
