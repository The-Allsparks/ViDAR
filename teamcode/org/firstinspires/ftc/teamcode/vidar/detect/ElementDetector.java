package org.firstinspires.ftc.teamcode.vidar.detect;

import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarGeometry;
import org.firstinspires.ftc.teamcode.vidar.config.VidarElementSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFramePipeline;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Pure element detection from a scaled ROI — Mat in, scored observations out.
 */
public final class ElementDetector {

    public static final class ScoredObservation {
        public final VidarElementObservation observation;
        public final double score;

        public ScoredObservation(VidarElementObservation observation, double score) {
            this.observation = observation;
            this.score = score;
        }
    }

    public static final class Result {
        public final List<ScoredObservation> frameCandidates;
        public final Map<String, VidarElementObservation> bestById;

        public Result(List<ScoredObservation> frameCandidates, Map<String, VidarElementObservation> bestById) {
            this.frameCandidates = frameCandidates;
            this.bestById = bestById;
        }
    }

    private ElementDetector() {}

    public static Result detectGameTarget(
            VidarContourTarget target,
            VidarFramePipeline.ScaledRoi scaled,
            int frameW,
            int frameH,
            long captureTimeNanos,
            boolean disableLocalHough,
            boolean grayReady,
            VidarContourWorkspace workspace,
            Mat reusableMask,
            Mat reusableHierarchy,
            Mat reusableGray,
            Mat reusableCircles,
            VidarCameraProfile profile,
            String cameraName,
            VidarSeasonConfig season,
            Function<String, VidarElementSpec> elementLookup) {
        List<ScoredObservation> candidates = new ArrayList<>();
        Map<String, VidarElementObservation> bestById = new HashMap<>();
        switch (target.shape) {
            case RECT:
                detectRect(target, scaled, frameW, frameH, captureTimeNanos, workspace,
                        reusableMask, reusableHierarchy, profile, cameraName, season,
                        elementLookup, candidates, bestById);
                break;
            case BLOB:
            case CIRCLE:
            default:
                detectCircle(target, scaled, frameW, frameH, captureTimeNanos, disableLocalHough,
                        grayReady, workspace, reusableMask, reusableHierarchy, reusableGray,
                        reusableCircles, profile, cameraName, season, elementLookup,
                        candidates, bestById);
                break;
        }
        return new Result(candidates, bestById);
    }

    private static void detectCircle(
            VidarContourTarget target,
            VidarFramePipeline.ScaledRoi scaled,
            int frameW,
            int frameH,
            long captureTimeNanos,
            boolean disableLocalHough,
            boolean grayReady,
            VidarContourWorkspace workspace,
            Mat reusableMask,
            Mat reusableHierarchy,
            Mat reusableGray,
            Mat reusableCircles,
            VidarCameraProfile profile,
            String cameraName,
            VidarSeasonConfig season,
            Function<String, VidarElementSpec> elementLookup,
            List<ScoredObservation> frameCandidates,
            Map<String, VidarElementObservation> bestById) {
        List<VidarContourDetect.CircleHit> hits = VidarContourDetect.findCircleHits(
                scaled.image, reusableMask, reusableHierarchy, target, scaled,
                frameW, frameH, profile, workspace);
        if (!disableLocalHough
                && grayReady
                && target.detector == VidarElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH
                && !hits.isEmpty()) {
            hits = VidarContourDetect.applyLocalHough(
                    scaled, hits, target, reusableGray, reusableCircles);
        }

        VidarElementSpec elementSpec = elementLookup.apply(target.id);
        VidarElementObservation bestForTarget = null;
        double bestScore = -1;
        for (VidarContourDetect.CircleHit hit : hits) {
            VidarElementObservation obs = VidarGeometry.fuseElementObservation(
                    hit.cx, hit.cy, hit.radius, hit.area, hit.aspectRatio, hit.circularity,
                    hit.fillRatio, hit.interiorScore, target.detector, profile, cameraName,
                    captureTimeNanos, hit.touchesBoundary, false, hit.circleFitQuality,
                    elementSpec, season);
            if (obs.confidence < season.minElementConfidence) {
                continue;
            }
            double localCy = (hit.cy - scaled.sourceCrop.y) / scaled.scale;
            double floorWeight = 0.25 + 0.75 * (localCy / scaled.image.rows());
            double score = obs.confidence * obs.radiusPx * obs.radiusPx * floorWeight * floorWeight;
            frameCandidates.add(new ScoredObservation(obs, score));
            if (score > bestScore) {
                bestScore = score;
                bestForTarget = obs;
            }
        }
        if (bestForTarget != null) {
            bestById.put(target.id, bestForTarget);
        }
    }

    private static void detectRect(
            VidarContourTarget target,
            VidarFramePipeline.ScaledRoi scaled,
            int frameW,
            int frameH,
            long captureTimeNanos,
            VidarContourWorkspace workspace,
            Mat reusableMask,
            Mat reusableHierarchy,
            VidarCameraProfile profile,
            String cameraName,
            VidarSeasonConfig season,
            Function<String, VidarElementSpec> elementLookup,
            List<ScoredObservation> frameCandidates,
            Map<String, VidarElementObservation> bestById) {
        List<VidarContourDetect.RectHit> hits = VidarContourDetect.findRectHits(
                scaled.image, reusableMask, reusableHierarchy, target, scaled,
                frameW, frameH, profile, workspace);

        VidarElementSpec elementSpec = elementLookup.apply(target.id);
        VidarElementObservation bestForTarget = null;
        double bestScore = -1;
        for (VidarContourDetect.RectHit hit : hits) {
            double absCx = scaled.toFullX(hit.box.center.x);
            double absCy = scaled.toFullY(hit.box.center.y);
            double fullWidthPx = Math.max(hit.box.size.width, hit.box.size.height) * scaled.scale;
            double fullHeightPx = Math.min(hit.box.size.width, hit.box.size.height) * scaled.scale;
            double radiusPx = Math.max(fullWidthPx, fullHeightPx) * 0.5;

            VidarElementObservation obs = VidarGeometry.fuseElementObservation(
                    absCx, absCy, radiusPx, hit.contourArea, hit.aspect, hit.rectangularity,
                    hit.rectangularity, 0.5, target.detector, profile, cameraName,
                    captureTimeNanos, hit.touchesBoundary, false, hit.rectangularity,
                    elementSpec, season);
            if (obs.confidence < season.minElementConfidence) {
                continue;
            }
            double score = obs.confidence * hit.contourArea;
            frameCandidates.add(new ScoredObservation(obs, score));
            if (score > bestScore) {
                bestScore = score;
                bestForTarget = obs;
            }
        }
        if (bestForTarget != null) {
            bestById.put(target.id, bestForTarget);
        }
    }
}
