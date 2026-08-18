package org.firstinspires.ftc.teamcode.vidar.world;

import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarVisionFusion;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.model.VidarElementOccurrenceRank;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

/**
 * Short-term spatial memory with predict/gate/associate tracking.
 *
 * <p>When {@link #isMotionTrackingActive()} is false, {@link #update} is a no-op — live vision only.
 *
 * <p>Association clocks on observation {@code captureTimeNanos}, not observation-worker ticks.
 * Repeating the same fused detections is a miss; {@code update(null, now)} coasts so detach ages
 * tracks. Consumers should treat high {@link VidarSpatialTrack#missCount} or old
 * {@link VidarSpatialTrack#lastSeenNanos} as stale.
 */
public class VidarWorldModel {

    public enum Kind {
        ELEMENT,
        ALLY,
        FOE
    }

    private final List<VidarSpatialTrack> tracks = new ArrayList<>();
    private Supplier<Pose2D> odomSupplier;
    private Supplier<Pose2D> fieldPoseSupplier;
    private boolean motionTrackingEnabled = VidarConfig.WORLD_MOTION_TRACKING_ENABLED;
    private int nextTrackId = 1;
    /** Max captureTimeNanos from the last fused frame that was treated as new. */
    private long lastObservationCaptureNanos;
    /** Wall-clock of the last associate/coast so 1 ms ticks do not count as miss frames. */
    private long lastAssociateNanos;

    public VidarWorldModel() {
        this(null, null);
    }

    public VidarWorldModel(Supplier<Pose2D> odomSupplier, Supplier<Pose2D> fieldPoseSupplier) {
        this.odomSupplier = odomSupplier;
        this.fieldPoseSupplier = fieldPoseSupplier;
    }

    /** Rebind odom when a new OpMode recreates {@code VidarSpatial} against a live runtime. */
    public void setOdomSupplier(Supplier<Pose2D> odomSupplier) {
        this.odomSupplier = odomSupplier;
        if (!isMotionTrackingActive()) {
            clearTracks();
        }
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> fieldPoseSupplier) {
        this.fieldPoseSupplier = fieldPoseSupplier;
    }

    public void setMotionTrackingEnabled(boolean enabled) {
        this.motionTrackingEnabled = enabled;
        if (!isMotionTrackingActive()) {
            clearTracks();
        }
    }

    public boolean isMotionTrackingEnabled() {
        return motionTrackingEnabled;
    }

    public boolean isMotionTrackingActive() {
        return motionTrackingEnabled && odomSupplier != null;
    }

    public void update(VidarVisionFusion vision, long nowNanos) {
        if (!isMotionTrackingActive()) {
            return;
        }

        List<VidarTrackDetection> detections = vision == null
                ? Collections.emptyList()
                : collectDetections(vision);
        updateFromDetections(detections, maxCaptureNanos(detections), nowNanos);
    }

    /**
     * Associate, coast, or prune using an explicit detection list and observation capture time.
     * Package-visible so java-pure tests can drive the world model without a fusion engine.
     */
    void updateFromDetections(
            List<VidarTrackDetection> detections, long observationCaptureNanos, long nowNanos) {
        List<VidarTrackDetection> toAssociate =
                detections == null ? Collections.emptyList() : detections;

        boolean staleFrame = VidarConfig.WORLD_ASSOCIATE_ON_NEW_FRAME_ONLY
                && observationCaptureNanos != 0
                && observationCaptureNanos == lastObservationCaptureNanos;
        if (staleFrame) {
            toAssociate = Collections.emptyList();
        }

        boolean newFrame = observationCaptureNanos != 0
                && observationCaptureNanos != lastObservationCaptureNanos;
        if (toAssociate.isEmpty() && shouldSkipIdleCoast(nowNanos)) {
            return;
        }

        if (newFrame) {
            lastObservationCaptureNanos = observationCaptureNanos;
        }
        lastAssociateNanos = nowNanos;

        Pose2D fieldPose = fieldPoseSupplier == null ? null : fieldPoseSupplier.get();
        List<VidarSpatialTrack> current = new ArrayList<>(tracks);
        int[] nextId = { nextTrackId };
        List<VidarSpatialTrack> associated = VidarTrackAssociator.associate(
                current, toAssociate, fieldPose, nowNanos, nextId);
        tracks.clear();
        tracks.addAll(associated);
        nextTrackId = nextId[0];
    }

    private boolean shouldSkipIdleCoast(long nowNanos) {
        if (lastAssociateNanos <= 0) {
            return false;
        }
        return VidarSpatialTrack.dtSeconds(lastAssociateNanos, nowNanos)
                < VidarConfig.WORLD_TRACK_MIN_DT_SEC;
    }

    private static long maxCaptureNanos(List<VidarTrackDetection> detections) {
        long max = 0L;
        if (detections == null) {
            return max;
        }
        for (VidarTrackDetection detection : detections) {
            if (detection != null && detection.captureTimeNanos > max) {
                max = detection.captureTimeNanos;
            }
        }
        return max;
    }

    private List<VidarTrackDetection> collectDetections(VidarVisionFusion vision) {
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

    /** Clear short-term tracks between match periods. */
    public void resetMatchState() {
        clearTracks();
    }

    private void clearTracks() {
        tracks.clear();
        nextTrackId = 1;
        lastObservationCaptureNanos = 0L;
        lastAssociateNanos = 0L;
    }
}
