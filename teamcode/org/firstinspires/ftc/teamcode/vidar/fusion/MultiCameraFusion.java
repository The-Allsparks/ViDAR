package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarTemporalFilter;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarRobotPose2D;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Pure multi-camera element rank fusion — extracted from {@link VidarFusionEngine}.
 */
public final class MultiCameraFusion {

    private MultiCameraFusion() {}

    public static VidarRankedElementFrame fuseRankedElements(
            VidarVision[] cameras,
            VidarTemporalFilter temporalFilter,
            int fusionCap) {
        List<ScoredElement> candidates = new ArrayList<>();
        for (VidarVision camera : cameras) {
            if (camera == null || camera.isFailed() || camera.isExcludedFromRotation()) {
                continue;
            }
            VidarRankedElementFrame frame = camera.getRankedElements();
            for (int i = 0; i < frame.count(); i++) {
                VidarElementObservation obs = frame.at(i);
                if (obs == null) {
                    continue;
                }
                obs = temporalFilter.filterElement(obs);
                if (obs == null) {
                    continue;
                }
                candidates.add(new ScoredElement(obs, elementRankScore(obs)));
            }
        }

        candidates.sort(Comparator.comparingDouble((ScoredElement s) -> s.score).reversed());
        List<VidarElementObservation> deduped = new ArrayList<>();
        int overflow = 0;
        for (ScoredElement candidate : candidates) {
            if (isDuplicateRobot(candidate.observation, deduped)) {
                continue;
            }
            if (deduped.size() < fusionCap) {
                deduped.add(candidate.observation);
            } else {
                overflow++;
            }
        }

        VidarElementObservation[] ranked = new VidarElementObservation[fusionCap];
        for (int i = 0; i < deduped.size(); i++) {
            ranked[i] = deduped.get(i);
        }
        long newestCapture = 0;
        for (VidarElementObservation obs : deduped) {
            newestCapture = Math.max(newestCapture, obs.captureTimeNanos);
        }
        return new VidarRankedElementFrame(ranked, deduped.size(), overflow, newestCapture, "fused", fusionCap);
    }

    public static double elementRankScore(VidarElementObservation obs) {
        double rangeWeight = Double.isNaN(obs.range) ? 0.5 : 1.0 / Math.max(6.0, obs.range);
        return obs.confidence * obs.areaPx * rangeWeight;
    }

    private static boolean isDuplicateRobot(VidarElementObservation obs, List<VidarElementObservation> kept) {
        for (VidarElementObservation other : kept) {
            if (VidarRobotPose2D.withinRadius(
                    obs.robotX, obs.robotY, other.robotX, other.robotY,
                    VidarConfig.WORLD_MERGE_RADIUS_IN)) {
                return true;
            }
        }
        return false;
    }

    private static final class ScoredElement {
        final VidarElementObservation observation;
        final double score;

        ScoredElement(VidarElementObservation observation, double score) {
            this.observation = observation;
            this.score = score;
        }
    }
}
