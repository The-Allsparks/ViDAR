package org.firstinspires.ftc.teamcode.vidar.world;

import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarMultiVision;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementOccurrenceRank;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Short-term spatial memory with predict/gate/associate tracking.
 *
 * <p>When {@link #isMotionTrackingActive()} is false, {@link #update} is a no-op — live vision only.
 */
public class VidarWorldModel {

    public enum Kind {
        ELEMENT,
        ALLY,
        FOE
    }

    private final List<VidarSpatialTrack> tracks = new ArrayList<>();
    private final Supplier<Pose2D> odomSupplier;
    private Supplier<Pose2D> fieldPoseSupplier;
    private boolean motionTrackingEnabled = VidarConfig.WORLD_MOTION_TRACKING_ENABLED;
    private int nextTrackId = 1;

    public VidarWorldModel() {
        this(null, null);
    }

    public VidarWorldModel(Supplier<Pose2D> odomSupplier, Supplier<Pose2D> fieldPoseSupplier) {
        this.odomSupplier = odomSupplier;
        this.fieldPoseSupplier = fieldPoseSupplier;
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> fieldPoseSupplier) {
        this.fieldPoseSupplier = fieldPoseSupplier;
    }

    public void setMotionTrackingEnabled(boolean enabled) {
        this.motionTrackingEnabled = enabled;
        if (!isMotionTrackingActive()) {
            tracks.clear();
            nextTrackId = 1;
        }
    }

    public boolean isMotionTrackingEnabled() {
        return motionTrackingEnabled;
    }

    public boolean isMotionTrackingActive() {
        return motionTrackingEnabled && odomSupplier != null;
    }

    public void update(VidarMultiVision vision, long nowNanos) {
        if (!isMotionTrackingActive() || vision == null) {
            return;
        }

        Pose2D fieldPose = fieldPoseSupplier == null ? null : fieldPoseSupplier.get();
        List<VidarTrackDetection> detections = collectDetections(vision);
        List<VidarSpatialTrack> current = new ArrayList<>(tracks);
        int[] nextId = { nextTrackId };
        List<VidarSpatialTrack> associated = VidarTrackAssociator.associate(
                current, detections, fieldPose, nowNanos, nextId);
        tracks.clear();
        tracks.addAll(associated);
        nextTrackId = nextId[0];
    }

    private List<VidarTrackDetection> collectDetections(VidarMultiVision vision) {
        List<VidarTrackDetection> out = new ArrayList<>();
        List<VidarTrackDetection> elementDets = new ArrayList<>();

        VidarRankedElementFrame ranked = vision.getRankedElements();
        if (ranked != null) {
            for (int i = 0; i < ranked.count(); i++) {
                VidarElementObservation obs = ranked.at(i);
                if (obs == null || obs.confidence < VidarConfig.MIN_ELEMENT_CONFIDENCE) {
                    continue;
                }
                VidarTrackDetection det = VidarTrackDetection.fromElement(obs);
                if (det != null) {
                    elementDets.add(det);
                }
            }
        }
        out.addAll(VidarElementOccurrenceRank.assignDetectionRanks(elementDets));

        VidarPlateObservation foe = vision.getBestFoe();
        if (foe != null && foe.confidence >= VidarConfig.MIN_PLATE_CONFIDENCE) {
            VidarTrackDetection det = VidarTrackDetection.fromPlate(foe, Kind.FOE);
            if (det != null) {
                out.add(det);
            }
        }

        VidarPlateObservation ally = vision.getBestAlly();
        if (ally != null && ally.confidence >= VidarConfig.MIN_PLATE_CONFIDENCE) {
            VidarTrackDetection det = VidarTrackDetection.fromPlate(ally, Kind.ALLY);
            if (det != null) {
                out.add(det);
            }
        }
        return out;
    }

    public List<VidarSpatialTrack> getTracks() {
        return new ArrayList<>(tracks);
    }

    public List<VidarSpatialTrack> getTracks(Kind kind) {
        List<VidarSpatialTrack> out = new ArrayList<>();
        for (VidarSpatialTrack track : tracks) {
            if (track.kind == kind) {
                out.add(track);
            }
        }
        return out;
    }

    public VidarSpatialTrack nearestElement() {
        return nearest(Kind.ELEMENT);
    }

    public VidarSpatialTrack nearestFoe() {
        return nearest(Kind.FOE);
    }

    private VidarSpatialTrack nearest(Kind kind) {
        VidarSpatialTrack best = null;
        double bestDist = Double.MAX_VALUE;
        for (VidarSpatialTrack track : tracks) {
            if (track.kind != kind) {
                continue;
            }
            double dist = track.distance();
            if (dist < bestDist) {
                bestDist = dist;
                best = track;
            }
        }
        return best;
    }

    public boolean intakeBlocked() {
        for (VidarSpatialTrack track : tracks) {
            if (track.kind != Kind.FOE) {
                continue;
            }
            if (track.distance() > VidarConfig.WORLD_BLOCK_RANGE_IN) {
                continue;
            }
            if (Math.abs(track.bearingDeg()) <= VidarConfig.WORLD_BLOCK_CONE_DEG) {
                return true;
            }
        }
        return false;
    }

    public int trackCount() {
        return tracks.size();
    }
}
