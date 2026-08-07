package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementOccurrenceRank;
import org.firstinspires.ftc.teamcode.vidar.model.VidarOffensiveLaneAnalysis;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarRobotPose2D;
import org.firstinspires.ftc.teamcode.vidar.world.VidarSpatialTrack;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.VidarTeamConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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

    private final VidarMultiVision vision;
    private final VidarWorldModel world;
    private final Supplier<Pose2D> odomSupplier;
    private Supplier<Pose2D> fieldPoseSupplier;

    private VidarSpatial(
            VidarMultiVision vision,
            VidarWorldModel world,
            Supplier<Pose2D> odomSupplier,
            Supplier<Pose2D> fieldPoseSupplier) {
        this.vision = vision;
        this.world = world;
        this.odomSupplier = odomSupplier;
        this.fieldPoseSupplier = fieldPoseSupplier;
    }

    public static VidarSpatial create(HardwareMap hardwareMap) {
        return create(hardwareMap, null, null);
    }

    public static VidarSpatial create(
            HardwareMap hardwareMap,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        try {
            return create(
                    hardwareMap,
                    VidarTeamConfig.loadRobot(hardwareMap),
                    VidarTeamConfig.loadSeason(hardwareMap),
                    odomSupplier,
                    allianceSupplier);
        } catch (IOException e) {
            return create(hardwareMap, null, null, odomSupplier, allianceSupplier);
        }
    }

    public static VidarSpatial create(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier) {
        VidarMultiVision vision = new VidarMultiVision(
                hardwareMap, robot, season, odomSupplier, allianceSupplier);
        VidarWorldModel world = new VidarWorldModel(odomSupplier, null);
        VidarSpatial spatial = new VidarSpatial(vision, world, odomSupplier, null);
        world.setFieldPoseSupplier(spatial::fieldPoseForWorldTracks);
        return spatial;
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> supplier) {
        this.fieldPoseSupplier = supplier;
        world.setFieldPoseSupplier(this::fieldPoseForWorldTracks);
    }

    public void setFieldPosePrior(Pose2D prior) {
        vision.setFieldPosePrior(prior);
    }

    /** Enable/disable motion-corrected world tracks (no-op without odom supplier). */
    public void setMotionTrackingEnabled(boolean enabled) {
        world.setMotionTrackingEnabled(enabled);
    }

    public boolean isMotionTrackingEnabled() {
        return world.isMotionTrackingEnabled();
    }

    public boolean isOdomConfigured() {
        return odomSupplier != null;
    }

    /** True when world-model motion correction and track memory are active. */
    public boolean isMotionTrackingActive() {
        return world.isMotionTrackingActive();
    }

    public void update() {
        if (odomSupplier != null) {
            vision.recordOdom(odomSupplier.get());
        }
        vision.update();
        world.update(vision, System.nanoTime());
    }

    public VidarCorrectedFrame updateCorrected() {
        if (odomSupplier != null) {
            vision.recordOdom(odomSupplier.get());
        }
        VidarCorrectedFrame corrected = vision.updateCorrected();
        world.update(vision, System.nanoTime());
        return corrected;
    }

    /**
     * Season game elements — fused rank 0 = closest/easiest, plus remembered tracks when motion
     * tracking is active. Sorted nearest-first.
     */
    public List<VidarSpatialPoint> elements() {
        List<VidarSpatialPoint> out = new ArrayList<>();
        VidarRankedElementFrame ranked = vision.getRankedElements();
        if (ranked != null) {
            for (int i = 0; i < ranked.count(); i++) {
                VidarElementObservation obs = ranked.at(i);
                if (obs == null || obs.confidence < VidarConfig.MIN_ELEMENT_CONFIDENCE) {
                    continue;
                }
                addUnique(out, VidarSpatialPoint.fromElement(obs));
            }
        }
        if (world.isMotionTrackingActive()) {
            for (VidarSpatialTrack track : world.getTracks(VidarWorldModel.Kind.ELEMENT)) {
                addUnique(out, VidarSpatialPoint.fromTrack(track));
            }
        }
        return VidarElementOccurrenceRank.assignPerType(out);
    }

    /** Friendly alliance plates — live plus remembered tracks when motion tracking is active. */
    public List<VidarSpatialPoint> allies() {
        List<VidarSpatialPoint> out = new ArrayList<>();
        addUnique(out, bestAlly());
        if (world.isMotionTrackingActive()) {
            for (VidarSpatialTrack track : world.getTracks(VidarWorldModel.Kind.ALLY)) {
                addUnique(out, VidarSpatialPoint.fromTrack(track));
            }
        }
        sortByDistance(out);
        return out;
    }

    /** Opponent plates — live plus remembered tracks when motion tracking is active. */
    public List<VidarSpatialPoint> foes() {
        List<VidarSpatialPoint> out = new ArrayList<>();
        addUnique(out, bestFoe());
        if (world.isMotionTrackingActive()) {
            for (VidarSpatialTrack track : world.getTracks(VidarWorldModel.Kind.FOE)) {
                addUnique(out, VidarSpatialPoint.fromTrack(track));
            }
        }
        sortByDistance(out);
        return out;
    }

    public VidarSpatialPoint bestElement() {
        return VidarSpatialPoint.fromElement(vision.getBestElement());
    }

    public VidarSpatialPoint nearestElement() {
        return world.isMotionTrackingActive()
                ? VidarSpatialPoint.fromTrack(world.nearestElement())
                : bestElement();
    }

    public VidarSpatialPoint bestFoe() {
        return VidarSpatialPoint.fromPlate(vision.getBestFoe(), VidarSpatialPoint.Kind.FOE);
    }

    public VidarSpatialPoint nearestFoe() {
        if (world.isMotionTrackingActive()) {
            return VidarSpatialPoint.fromTrack(world.nearestFoe());
        }
        return bestFoe();
    }

    public VidarSpatialPoint bestAlly() {
        return VidarSpatialPoint.fromPlate(vision.getBestAlly(), VidarSpatialPoint.Kind.ALLY);
    }

    public Pose2D robotPose() {
        return odomSupplier != null ? odomSupplier.get() : null;
    }

    public boolean intakeBlocked() {
        if (world.isMotionTrackingActive()) {
            return world.intakeBlocked();
        }
        VidarSpatialPoint foe = bestFoe();
        if (foe == null || !foe.isValid()) {
            return false;
        }
        return foe.distance() <= VidarConfig.WORLD_BLOCK_RANGE_IN
                && Math.abs(foe.bearingDeg()) <= VidarConfig.WORLD_BLOCK_CONE_DEG;
    }

    /** Foe counts in left / center / right forward lanes from {@link #foes()}. */
    public VidarOffensiveLaneAnalysis offensiveLaneAnalysis() {
        return VidarOffensiveLaneAnalysis.fromFoes(foes());
    }

    /** Lane with the fewest foes in the forward cone; tie-break center, then left. */
    public VidarOffensiveLane recommendOffensiveLane() {
        return offensiveLaneAnalysis().recommended;
    }

    public int trackCount() {
        return world.isMotionTrackingActive() ? world.trackCount() : 0;
    }

    public Pose2D fieldPose() {
        if (fieldPoseSupplier != null) {
            Pose2D external = fieldPoseSupplier.get();
            if (external != null) {
                return external;
            }
        }
        return vision.getFusedFieldPose();
    }

    public VidarDistanceUnit distanceUnit() {
        return vision.distanceUnit();
    }

    public int cameraCount() {
        return vision.getCameraCount();
    }

    public VidarMultiVision vision() {
        return vision;
    }

    public VidarWorldModel worldModel() {
        return world;
    }

    public void close() {
        vision.close();
    }

    /** Team override first, then odom-extrapolated field pose for world tracks. */
    Pose2D fieldPoseForWorldTracks() {
        if (fieldPoseSupplier != null) {
            Pose2D external = fieldPoseSupplier.get();
            if (external != null) {
                return external;
            }
        }
        return vision.getFieldPoseForMotionTracking();
    }

    private static void addUnique(List<VidarSpatialPoint> list, VidarSpatialPoint candidate) {
        if (candidate == null || !candidate.isValid()) {
            return;
        }
        if (candidate.trackId >= 0) {
            for (int i = 0; i < list.size(); i++) {
                if (list.get(i).trackId == candidate.trackId) {
                    if (candidate.source == VidarSpatialPoint.Source.LIVE) {
                        list.set(i, candidate);
                    }
                    return;
                }
            }
        }
        for (int i = 0; i < list.size(); i++) {
            VidarSpatialPoint existing = list.get(i);
            if (existing.kind != candidate.kind) {
                continue;
            }
            if (VidarRobotPose2D.withinRadius(
                    existing.robotX, existing.robotY,
                    candidate.robotX, candidate.robotY,
                    VidarConfig.WORLD_TRACK_GATE_RADIUS_IN)) {
                if (candidate.source == VidarSpatialPoint.Source.LIVE
                        && existing.source == VidarSpatialPoint.Source.REMEMBERED) {
                    list.set(i, candidate);
                }
                return;
            }
        }
        list.add(candidate);
    }

    private static void sortByDistance(List<VidarSpatialPoint> points) {
        Collections.sort(points, Comparator.comparingDouble(VidarSpatialPoint::distance));
    }
}
