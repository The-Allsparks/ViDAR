package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.api.VidarDiagnostics;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarSpatialSnapshot;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarFusionEngine;
import org.firstinspires.ftc.teamcode.vidar.model.VidarOffensiveLaneAnalysis;
import org.firstinspires.ftc.teamcode.vidar.runtime.RuntimeBootstrap;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarLatencyWindow;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarObservationWorker;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.VidarTeamConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Pedro-style facade for ViDAR spatial knowledge — one {@link #update()} per robot loop.
 *
 * <p>Three spatial groups — {@link #elements()}, {@link #allies()}, {@link #foes()} — plus pose.
 * ViDAR never commands motors.
 *
 * <p>Perception (camera poll, fusion, world model) runs continuously in a background worker.
 * {@link #update()} pins the latest published snapshot so all getters are stable for one loop
 * iteration — it does <em>not</em> drive perception on the robot thread.
 *
 * <p>Motion-corrected track memory requires an odom supplier at {@link #create} and
 * {@link #setMotionTrackingEnabled(boolean)} true (default from {@link VidarConfig}).
 */
public final class VidarSpatial {

    private final VidarRuntime runtime;
    private VidarDiagnostics diagnostics = VidarDiagnostics.empty();
    private VidarSpatialSnapshot snapshot = VidarSpatialSnapshot.empty();

    private VidarSpatial(VidarRuntime runtime) {
        this.runtime = runtime;
        update();
    }

    public static VidarSpatial create(HardwareMap hardwareMap) {
        return create(hardwareMap, null, null);
    }

    public static VidarSpatial create(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        try {
            return createInternal(
                    hardwareMap,
                    VidarTeamConfig.loadRobot(hardwareMap),
                    VidarTeamConfig.loadSeason(hardwareMap),
                    odomSupplier,
                    allianceSupplier,
                    VidarDiagnostics.ConfigSource.TEAM_ASSETS);
        } catch (IOException e) {
            throw new VidarConfigException(
                    "Missing ViDAR config assets. Copy season.json and robot.json to "
                            + "TeamCode/src/main/assets/vidar/ or call createWithBundledDefaults().",
                    e);
        }
    }

    public static VidarSpatial createWithBundledDefaults(HardwareMap hardwareMap) {
        return createWithBundledDefaults(hardwareMap, null, null);
    }

    public static VidarSpatial createWithBundledDefaults(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return createInternal(
                hardwareMap,
                VidarTeamConfig.defaultRobot(),
                VidarTeamConfig.defaultSeason(),
                odomSupplier,
                allianceSupplier,
                VidarDiagnostics.ConfigSource.BUNDLED_DEFAULTS);
    }

    public static VidarSpatial create(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return createInternal(
                hardwareMap,
                robot,
                season,
                odomSupplier,
                allianceSupplier,
                VidarDiagnostics.ConfigSource.TEAM_ASSETS);
    }

    private static VidarSpatial createInternal(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier,
            VidarDiagnostics.ConfigSource configSource) {
        RuntimeBootstrap bootstrap = new RuntimeBootstrap(odomSupplier, allianceSupplier, configSource);
        VidarRuntime runtime = VidarRuntime.getOrCreate(bootstrap);
        runtime.attachVision(hardwareMap, robot, season);
        return new VidarSpatial(runtime);
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> supplier) {
        runtime.setFieldPoseSupplier(supplier);
    }

    public void setFieldPosePrior(Pose2D prior) {
        runtime.setFieldPosePrior(prior);
    }

    public void setMotionTrackingEnabled(boolean enabled) {
        runtime.setMotionTrackingEnabled(enabled);
    }

    public boolean isMotionTrackingEnabled() {
        return runtime.world().isMotionTrackingEnabled();
    }

    public boolean isOdomConfigured() {
        return runtime.fieldPoseContext().odomSupplier() != null;
    }

    public boolean isMotionTrackingActive() {
        return runtime.world().isMotionTrackingActive();
    }

    /**
     * Pin the latest background snapshot and refresh diagnostics for this robot-loop iteration.
     * Call once per {@code while (opModeIsActive())} loop before reading {@link #elements()} etc.
     */
    public void update() {
        snapshot = runtime.pinSnapshot();
        refreshDiagnostics();
    }

    public VidarCorrectedFrame updateCorrected() {
        VidarCorrectedFrame corrected = runtime.updateCorrected();
        update();
        return corrected;
    }

    /** Same pinned snapshot as {@link #elements()} — updated each {@link #update()}. */
    public VidarSpatialSnapshot snapshot() {
        return snapshot;
    }

    public VidarObservationFrame lastFrame() {
        return runtime.lastFrame();
    }

    public List<VidarSpatialPoint> elements() {
        return snapshot.elements;
    }

    public List<VidarSpatialPoint> allies() {
        return snapshot.allies;
    }

    public List<VidarSpatialPoint> foes() {
        return snapshot.foes;
    }

    public VidarSpatialPoint bestElement() {
        return snapshot.elements.isEmpty() ? null : snapshot.elements.get(0);
    }

    public VidarSpatialPoint nearestElement() {
        if (runtime.world().isMotionTrackingActive()) {
            return VidarSpatialPoint.fromTrack(runtime.world().nearestElement());
        }
        return bestElement();
    }

    public VidarSpatialPoint bestFoe() {
        for (VidarSpatialPoint p : snapshot.foes) {
            if (p.source == VidarSpatialPoint.Source.LIVE) {
                return p;
            }
        }
        return snapshot.foes.isEmpty() ? null : snapshot.foes.get(0);
    }

    public VidarSpatialPoint nearestFoe() {
        if (runtime.world().isMotionTrackingActive()) {
            return VidarSpatialPoint.fromTrack(runtime.world().nearestFoe());
        }
        return bestFoe();
    }

    public VidarSpatialPoint bestAlly() {
        for (VidarSpatialPoint p : snapshot.allies) {
            if (p.source == VidarSpatialPoint.Source.LIVE) {
                return p;
            }
        }
        return snapshot.allies.isEmpty() ? null : snapshot.allies.get(0);
    }

    public boolean intakeBlocked() {
        return snapshot.intakeBlocked;
    }

    public int trackCount() {
        return snapshot.trackCount;
    }

    public Pose2D fieldPose() {
        return snapshot.fieldPose;
    }

    /**
     * Last gate-accepted tag fused pose (freeze at fuse-time). Never uses
     * {@link #setFieldPoseSupplier} — safe as a Pedro novelty / telemetry anchor when Pedro is
     * also wired as the continuous field supplier. Pinned with {@link #update()}.
     */
    public Pose2D fusedFieldPose() {
        return snapshot.fusedFieldPose;
    }

    /**
     * {@link System#nanoTime()} of the last gate-accepted tag correction (0 = none). Use with
     * {@link org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroCorrectionTracker} so
     * novelty is event-based, not pose-epsilon based. Pinned with {@link #update()}.
     */
    public long lastTagCorrectionNanos() {
        return snapshot.lastTagCorrectionNanos;
    }

    /**
     * Gated tag-fused field pose re-propagated to the odom sample at the last publish.
     *
     * <p>Unlike {@link #fieldPose()}, this advances the last <em>gate-accepted</em> fix from
     * odom-at-fuse to odom-at-publish. It bypasses {@link #setFieldPoseSupplier} so Pedro can
     * still supply continuous pose for world tracks while corrections stay ViDAR-derived. Use for
     * {@code follower.setPose} — see
     * {@link org.firstinspires.ftc.teamcode.vidar.integration.VidarPedroCorrectionTracker}.
     *
     * <p>Pinned with {@link #update()} so a mid-loop worker tick cannot change the inject pose.
     * Returns {@code null} until a tag fix has passed localization gates.
     */
    public Pose2D tagCorrectedFieldPoseNow() {
        return snapshot.tagCorrectedFieldPoseNow;
    }

    public Pose2D robotPose() {
        Supplier<Pose2D> odom = runtime.fieldPoseContext().odomSupplier();
        return odom != null ? odom.get() : null;
    }

    public VidarOffensiveLaneAnalysis offensiveLaneAnalysis() {
        return VidarOffensiveLaneAnalysis.fromFoes(foes());
    }

    public VidarOffensiveLane recommendOffensiveLane() {
        return offensiveLaneAnalysis().recommended;
    }

    public VidarDistanceUnit distanceUnit() {
        VidarFusionEngine fusion = runtime.fusionEngine();
        return fusion != null ? fusion.distanceUnit() : VidarDistanceUnit.IN;
    }

    public int cameraCount() {
        return runtime.cameraCount();
    }

    public VidarDiagnostics diagnostics() {
        return diagnostics;
    }

    public VidarRuntime runtime() {
        return runtime;
    }

    /** Detach FTC cameras — runtime persists for the next OpMode. */
    public void close() {
        runtime.detachVision();
    }

    private void refreshDiagnostics() {
        List<String> warnings = new ArrayList<>();
        int connected = 0;
        int count = runtime.cameraCount();
        org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics.CameraHealth[] health =
                new org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics.CameraHealth[count];
        for (int i = 0; i < count; i++) {
            org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision cam = runtime.camera(i);
            if (cam == null) {
                warnings.add("Camera " + i + " failed to initialize — check Driver Station webcam name.");
                health[i] = org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics.CameraHealth.FAILED;
                continue;
            }
            connected++;
            health[i] = cam.metrics().health();
            if (cam.isFailed()) {
                String err = cam.metrics().lastError();
                warnings.add("Camera " + i + " (" + cam.getProfile().name + ") failed"
                        + (err == null || err.isEmpty() ? "" : ": " + err));
            }
        }
        if (runtime.configSource() == VidarDiagnostics.ConfigSource.BUNDLED_DEFAULTS) {
            warnings.add(0, "Using bundled default season/robot JSON — deploy team assets for match tuning.");
        }
        VidarObservationWorker worker = runtime.observationWorker();
        String workerError = worker.lastErrorMessage();
        int workerConsecutive = worker.consecutiveFailureCount();
        int workerTotal = worker.totalFailureCount();
        if (workerTotal > 0) {
            warnings.add("Observation worker failures=" + workerTotal
                    + " consecutive=" + workerConsecutive
                    + (workerError.isEmpty() ? "" : ": " + workerError));
        }
        VidarLatencyWindow tickLatency = runtime.observationTickLatency();
        diagnostics = new VidarDiagnostics(
                runtime.configSource(), count, connected, warnings, health,
                workerError, workerConsecutive, workerTotal,
                tickLatency.p50Ms(), tickLatency.p95Ms(), tickLatency.maxMs(),
                tickLatency.sampleCount());
    }
}
