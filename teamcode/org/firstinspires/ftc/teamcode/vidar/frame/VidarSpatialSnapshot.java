package org.firstinspires.ftc.teamcode.vidar.frame;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarMultiVision;
import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarSpatialPoint;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarRobotPose2D;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementOccurrenceRank;
import org.firstinspires.ftc.teamcode.vidar.world.VidarSpatialTrack;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

/**
 * Immutable spatial query result for one {@link org.firstinspires.ftc.teamcode.vidar.VidarSpatial#update()}
 * cycle. Repeated getter calls within the same loop return identical lists.
 */
public final class VidarSpatialSnapshot {

    private static final VidarSpatialSnapshot EMPTY = new VidarSpatialSnapshot(
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            false,
            null,
            0);

    public final List<VidarSpatialPoint> elements;
    public final List<VidarSpatialPoint> allies;
    public final List<VidarSpatialPoint> foes;
    public final boolean intakeBlocked;
    public final Pose2D fieldPose;
    public final int trackCount;

    public VidarSpatialSnapshot(
            List<VidarSpatialPoint> elements,
            List<VidarSpatialPoint> allies,
            List<VidarSpatialPoint> foes,
            boolean intakeBlocked,
            Pose2D fieldPose,
            int trackCount) {
        this.elements = Collections.unmodifiableList(new ArrayList<>(elements));
        this.allies = Collections.unmodifiableList(new ArrayList<>(allies));
        this.foes = Collections.unmodifiableList(new ArrayList<>(foes));
        this.intakeBlocked = intakeBlocked;
        this.fieldPose = fieldPose;
        this.trackCount = trackCount;
    }

    public static VidarSpatialSnapshot empty() {
        return EMPTY;
    }

    public static VidarSpatialSnapshot build(
            VidarMultiVision vision,
            VidarWorldModel world,
            Supplier<Pose2D> fieldPoseSupplier) {
        if (vision == null) {
            return empty();
        }

        List<VidarSpatialPoint> elementsOut = new ArrayList<>();
        VidarRankedElementFrame ranked = vision.getRankedElements();
        if (ranked != null) {
            for (int i = 0; i < ranked.count(); i++) {
                VidarElementObservation obs = ranked.at(i);
                if (obs == null || obs.confidence < VidarConfig.MIN_ELEMENT_CONFIDENCE) {
                    continue;
                }
                addUnique(elementsOut, VidarSpatialPoint.fromElement(obs));
            }
        }
        if (world != null && world.isMotionTrackingActive()) {
            for (VidarSpatialTrack track : world.getTracks(VidarWorldModel.Kind.ELEMENT)) {
                addUnique(elementsOut, VidarSpatialPoint.fromTrack(track));
            }
        }
        elementsOut = VidarElementOccurrenceRank.assignPerType(elementsOut);

        List<VidarSpatialPoint> alliesOut = new ArrayList<>();
        addUnique(alliesOut, VidarSpatialPoint.fromPlate(vision.getBestAlly(), VidarSpatialPoint.Kind.ALLY));
        if (world != null && world.isMotionTrackingActive()) {
            for (VidarSpatialTrack track : world.getTracks(VidarWorldModel.Kind.ALLY)) {
                addUnique(alliesOut, VidarSpatialPoint.fromTrack(track));
            }
        }
        sortByDistance(alliesOut);

        List<VidarSpatialPoint> foesOut = new ArrayList<>();
        addUnique(foesOut, VidarSpatialPoint.fromPlate(vision.getBestFoe(), VidarSpatialPoint.Kind.FOE));
        if (world != null && world.isMotionTrackingActive()) {
            for (VidarSpatialTrack track : world.getTracks(VidarWorldModel.Kind.FOE)) {
                addUnique(foesOut, VidarSpatialPoint.fromTrack(track));
            }
        }
        sortByDistance(foesOut);

        boolean blocked = computeIntakeBlocked(world, foesOut);
        Pose2D fieldPose = resolveFieldPose(vision, fieldPoseSupplier);
        int tracks = world != null && world.isMotionTrackingActive() ? world.trackCount() : 0;
        return new VidarSpatialSnapshot(elementsOut, alliesOut, foesOut, blocked, fieldPose, tracks);
    }

    private static boolean computeIntakeBlocked(VidarWorldModel world, List<VidarSpatialPoint> foes) {
        if (world != null && world.isMotionTrackingActive()) {
            return world.intakeBlocked();
        }
        for (VidarSpatialPoint foe : foes) {
            if (foe == null || !foe.isValid()) {
                continue;
            }
            if (foe.distance() <= VidarConfig.WORLD_BLOCK_RANGE_IN
                    && Math.abs(foe.bearingDeg()) <= VidarConfig.WORLD_BLOCK_CONE_DEG) {
                return true;
            }
        }
        return false;
    }

    private static Pose2D resolveFieldPose(
            VidarMultiVision vision,
            Supplier<Pose2D> fieldPoseSupplier) {
        if (fieldPoseSupplier != null) {
            Pose2D external = fieldPoseSupplier.get();
            if (external != null) {
                return external;
            }
        }
        return vision.getFusedFieldPose();
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
