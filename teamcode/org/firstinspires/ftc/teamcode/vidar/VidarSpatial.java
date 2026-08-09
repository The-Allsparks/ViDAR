package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.api.VidarDiagnostics;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarSpatialSnapshot;
import org.firstinspires.ftc.teamcode.vidar.fusion.FieldPoseContext;
import org.firstinspires.ftc.teamcode.vidar.model.VidarOffensiveLaneAnalysis;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Pedro-style facade for ViDAR spatial knowledge — one object, one {@link #update()} per loop.
 *
 * <p>Three spatial groups — {@link #elements()}, {@link #allies()}, {@link #foes()} — plus pose.
 * ViDAR never commands motors.
 *
 * <p>Motion-corrected track memory requires an odom supplier at {@link #create} and
 * {@link #setMotionTrackingEnabled(boolean)} true (default from {@link VidarConfig}).
 * Without odom, queries return live camera detections only.
 */
public final class VidarSpatial {

    private final VidarSession session;
    private VidarDiagnostics diagnostics = VidarDiagnostics.empty();
    private VidarSpatialSnapshot snapshot = VidarSpatialSnapshot.empty();

    private VidarSpatial(VidarSession session) {
        this.session = session;
        refreshDiagnostics();
    }

    public static VidarSpatial create(HardwareMap hardwareMap) {
        return create(hardwareMap, null, null);
    }

    /**
     * Load team {@code assets/vidar/season.json} and {@code robot.json}.
     *
     * @throws VidarConfigException if assets are missing — use {@link #createWithBundledDefaults}
     *         for bundled fallbacks during bring-up.
     */
    public static VidarSpatial create(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return new VidarSpatial(VidarSession.create(hardwareMap, odomSupplier, allianceSupplier));
    }

    /** Explicit bundled defaults when team assets are not deployed yet. */
    public static VidarSpatial createWithBundledDefaults(HardwareMap hardwareMap) {
        return createWithBundledDefaults(hardwareMap, null, null);
    }

    public static VidarSpatial createWithBundledDefaults(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return new VidarSpatial(
                VidarSession.createWithBundledDefaults(hardwareMap, odomSupplier, allianceSupplier));
    }

    public static VidarSpatial create(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        return new VidarSpatial(
                VidarSession.create(hardwareMap, robot, season, odomSupplier, allianceSupplier));
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> supplier) {
        session.setFieldPoseSupplier(supplier);
    }

    public void setFieldPosePrior(Pose2D prior) {
        session.setFieldPosePrior(prior);
    }

    /** Enable/disable motion-corrected world tracks (no-op without odom supplier). */
    public void setMotionTrackingEnabled(boolean enabled) {
        session.setMotionTrackingEnabled(enabled);
    }

    public boolean isMotionTrackingEnabled() {
        return session.world().isMotionTrackingEnabled();
    }

    public boolean isOdomConfigured() {
        return session.fieldPoseContext().odomSupplier() != null;
    }

    /** True when world-model motion correction and track memory are active. */
    public boolean isMotionTrackingActive() {
        return session.world().isMotionTrackingActive();
    }

    public void update() {
        session.update();
        snapshot = VidarSpatialSnapshot.build(
                session.vision(), session.world(), session.fieldPoseContext()::fieldPoseForSnapshot);
        refreshDiagnostics();
    }

    public VidarCorrectedFrame updateCorrected() {
        if (session.fieldPoseContext().odomSupplier() != null) {
            session.vision().recordOdom(session.fieldPoseContext().odomSupplier().get());
        }
        VidarCorrectedFrame corrected = session.vision().updateCorrected();
        session.world().update(session.vision(), System.nanoTime());
        snapshot = VidarSpatialSnapshot.build(
                session.vision(), session.world(), session.fieldPoseContext()::fieldPoseForSnapshot);
        refreshDiagnostics();
        return corrected;
    }

    /** Immutable spatial groups from the last {@link #update()} — stable within one loop. */
    public VidarSpatialSnapshot snapshot() {
        return snapshot;
    }

    /** Latest observation frame from the last vision update. */
    public VidarObservationFrame lastFrame() {
        return session.vision().getLatestFrame();
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
        if (session.world().isMotionTrackingActive()) {
            return VidarSpatialPoint.fromTrack(session.world().nearestElement());
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
        if (session.world().isMotionTrackingActive()) {
            return VidarSpatialPoint.fromTrack(session.world().nearestFoe());
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

    public Pose2D robotPose() {
        Supplier<Pose2D> odom = session.fieldPoseContext().odomSupplier();
        return odom != null ? odom.get() : null;
    }

    /** Foe counts in left / center / right forward lanes from {@link #foes()}. */
    public VidarOffensiveLaneAnalysis offensiveLaneAnalysis() {
        return VidarOffensiveLaneAnalysis.fromFoes(foes());
    }

    /** Lane with the fewest foes in the forward cone; tie-break center, then left. */
    public VidarOffensiveLane recommendOffensiveLane() {
        return offensiveLaneAnalysis().recommended;
    }

    public VidarDistanceUnit distanceUnit() {
        return session.vision().distanceUnit();
    }

    public int cameraCount() {
        return session.vision().getCameraCount();
    }

    public VidarDiagnostics diagnostics() {
        return diagnostics;
    }

    /**
     * @deprecated Prefer {@link #diagnostics()} and {@link #lastFrame()} for student-facing access.
     */
    @Deprecated
    public VidarMultiVision vision() {
        return session.vision();
    }

    public VidarSession session() {
        return session;
    }

    public VidarWorldModel worldModel() {
        return session.world();
    }

    public void close() {
        session.close();
    }

    private void refreshDiagnostics() {
        List<String> warnings = new ArrayList<>();
        int connected = 0;
        org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics.CameraHealth[] health =
                new org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics.CameraHealth[cameraCount()];
        for (int i = 0; i < cameraCount(); i++) {
            org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision cam = session.vision().camera(i);
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
        if (session.configSource() == VidarDiagnostics.ConfigSource.BUNDLED_DEFAULTS) {
            warnings.add(0, "Using bundled default season/robot JSON — deploy team assets for match tuning.");
        }
        diagnostics = new VidarDiagnostics(
                session.configSource(), cameraCount(), connected, warnings, health);
    }
}
