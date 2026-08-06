package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Predict → gate → assign association for {@link VidarSpatialTrack} lists.
 */
final class VidarTrackAssociator {

    private VidarTrackAssociator() {}

    static final class Match {
        final int trackIndex;
        final int detectionIndex;
        final double distance;

        Match(int trackIndex, int detectionIndex, double distance) {
            this.trackIndex = trackIndex;
            this.detectionIndex = detectionIndex;
            this.distance = distance;
        }
    }

    static List<VidarSpatialTrack> associate(
            List<VidarSpatialTrack> tracks,
            List<VidarTrackDetection> detections,
            Pose2D fieldPose,
            long nowNanos,
            int[] nextTrackIdHolder) {
        if (tracks.isEmpty() && detections.isEmpty()) {
            return tracks;
        }

        List<VidarSpatialTrack> predicted = new ArrayList<>(tracks.size());
        for (VidarSpatialTrack track : tracks) {
            predicted.add(track.predict(nowNanos, fieldPose));
        }

        if (detections.isEmpty()) {
            List<VidarSpatialTrack> coasted = new ArrayList<>();
            for (VidarSpatialTrack track : predicted) {
                coasted.add(track.coast(nowNanos, fieldPose));
            }
            return prune(coasted, nowNanos);
        }

        List<Match> candidates = new ArrayList<>();
        for (int ti = 0; ti < predicted.size(); ti++) {
            VidarSpatialTrack track = predicted.get(ti);
            for (int di = 0; di < detections.size(); di++) {
                VidarTrackDetection det = detections.get(di);
                if (!kindsMatch(track, det)) {
                    continue;
                }
                if (!elementIdsCompatible(track.elementId, det.elementId)) {
                    continue;
                }
                double dist = track.gateDistanceTo(det);
                if (dist <= track.gateRadius()) {
                    candidates.add(new Match(ti, di, dist));
                }
            }
        }

        Collections.sort(candidates, Comparator.comparingDouble(m -> m.distance));

        boolean[] trackUsed = new boolean[predicted.size()];
        boolean[] detUsed = new boolean[detections.size()];
        List<VidarSpatialTrack> updated = new ArrayList<>();

        for (Match match : candidates) {
            if (trackUsed[match.trackIndex] || detUsed[match.detectionIndex]) {
                continue;
            }
            VidarSpatialTrack predictedTrack = predicted.get(match.trackIndex);
            VidarSpatialTrack sourceTrack = tracks.get(match.trackIndex);
            VidarTrackDetection det = detections.get(match.detectionIndex);
            double dtSec = VidarSpatialTrack.dtSeconds(sourceTrack.lastUpdateNanos, nowNanos);
            updated.add(predictedTrack.updateFromDetection(det, nowNanos, fieldPose, dtSec));
            trackUsed[match.trackIndex] = true;
            detUsed[match.detectionIndex] = true;
        }

        for (int ti = 0; ti < predicted.size(); ti++) {
            if (trackUsed[ti]) {
                continue;
            }
            updated.add(predicted.get(ti).coast(nowNanos, fieldPose));
        }

        for (int di = 0; di < detections.size(); di++) {
            if (detUsed[di]) {
                continue;
            }
            VidarTrackDetection det = detections.get(di);
            if (det.confidence < minConfidence(det.kind)) {
                continue;
            }
            updated.add(VidarSpatialTrack.birth(nextTrackIdHolder[0]++, det, nowNanos, fieldPose));
        }

        return prune(updated, nowNanos);
    }

    private static boolean kindsMatch(VidarSpatialTrack track, VidarTrackDetection det) {
        return track.kind == det.kind;
    }

    private static boolean elementIdsCompatible(String trackId, String detId) {
        if (trackId.isEmpty() || detId.isEmpty()) {
            return true;
        }
        return trackId.equals(detId);
    }

    private static double minConfidence(VidarWorldModel.Kind kind) {
        switch (kind) {
            case FOE:
            case ALLY:
                return VidarConfig.MIN_PLATE_CONFIDENCE;
            case ELEMENT:
            default:
                return VidarConfig.MIN_ELEMENT_CONFIDENCE;
        }
    }

    private static List<VidarSpatialTrack> prune(List<VidarSpatialTrack> tracks, long nowNanos) {
        List<VidarSpatialTrack> out = new ArrayList<>();
        for (VidarSpatialTrack track : tracks) {
            if (track.missCount > VidarConfig.WORLD_TRACK_MAX_MISS_FRAMES) {
                continue;
            }
            double ageSec = VidarSpatialTrack.dtSeconds(track.lastSeenNanos, nowNanos);
            if (track.missCount > 0 && ageSec > ttlSeconds(track.kind)) {
                continue;
            }
            if (track.confidence < 0.05) {
                continue;
            }
            out.add(track);
        }
        return out;
    }

    private static double ttlSeconds(VidarWorldModel.Kind kind) {
        switch (kind) {
            case ELEMENT:
                return VidarConfig.WORLD_ELEMENT_TTL_SEC;
            case FOE:
                return VidarConfig.WORLD_FOE_TTL_SEC;
            case ALLY:
            default:
                return VidarConfig.WORLD_ALLY_TTL_SEC;
        }
    }
}
