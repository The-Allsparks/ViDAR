package org.firstinspires.ftc.teamcode.vidar.detect;

import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.VidarGeometry;
import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.config.VidarPlateSpec;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFramePipeline;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Pure plate detection from a scaled ROI — Mat in, best plate observation out.
 */
public final class PlateDetector {

    public static final class Result {
        public final VidarPlateObservation observation;
        public final RotatedRect drawBox;

        public Result(VidarPlateObservation observation, RotatedRect drawBox) {
            this.observation = observation;
            this.drawBox = drawBox;
        }
    }

    private PlateDetector() {}

    public static Result detectPlate(
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
            VidarSeasonConfig season) {
        List<VidarContourDetect.RectHit> hits = VidarContourDetect.findRectHits(
                scaled.image, reusableMask, reusableHierarchy, target, scaled,
                frameW, frameH, profile, workspace);

        VidarPlateSpec plateSpec = season.plateSpec(target.alliance);
        VidarPlateObservation bestForAlliance = null;
        RotatedRect bestDraw = null;
        double bestScore = -1;

        for (VidarContourDetect.RectHit hit : hits) {
            double absCx = scaled.toFullX(hit.box.center.x);
            double absCy = scaled.toFullY(hit.box.center.y);
            double fullWidthPx = Math.max(hit.box.size.width, hit.box.size.height) * scaled.scale;
            double fullHeightPx = Math.min(hit.box.size.width, hit.box.size.height) * scaled.scale;

            double rotationPenalty = Math.abs(hit.box.angle % 90) / 45.0;
            double partialPenalty = hit.rectangularity < 0.65 ? 0.7 : 1.0;
            double viewingPenalty = 1.0 - Math.min(0.5, rotationPenalty * 0.25);

            double dWidth = VidarGeometry.distanceFromWidth(
                    profile.plateWidth, profile.focalLengthPx, fullWidthPx);
            double cyForFloor = (absCy - scaled.sourceCrop.y) / scaled.scale;
            boolean nearHorizon = absCy <= profile.horizonRowPx + 8;
            double horizonConf = profile.horizonRowPx > 0
                    ? Math.max(0.3, 1.0 - profile.horizonRowPx / 120.0) : 0.5;
            double dFloor = VidarGeometry.distanceFromFloor(cyForFloor, profile);
            double dGround = VidarGeometry.distanceFromGroundPlane(absCx, absCy, profile, 0.0);
            VidarRangeResult rangeResult = VidarGeometry.fusePlateRange(
                    absCx, absCy, cyForFloor,
                    dWidth, fullWidthPx, hit.rectangularity, hit.whiteRatio,
                    partialPenalty < 1.0, hit.touchesBoundary, rotationPenalty,
                    nearHorizon, horizonConf, profile, season.maxRangeMismatchRatio);
            double range = rangeResult.isValid() ? rangeResult.distance : Double.NaN;
            double confidence = VidarGeometry.composePlateConfidence(
                    hit.whiteRatio, hit.contourArea, hit.rectangularity, hit.aspect,
                    rangeResult, viewingPenalty, partialPenalty, plateSpec);
            if (confidence < season.minPlateConfidence) {
                continue;
            }

            double[] robotPoint = VidarGeometry.floorPointInRobot(absCx, absCy, range, profile);
            double score = confidence * hit.contourArea;
            if (score > bestScore) {
                bestScore = score;
                bestForAlliance = new VidarPlateObservation(
                        target.alliance,
                        absCx,
                        absCy,
                        fullWidthPx,
                        fullHeightPx,
                        hit.box.angle,
                        hit.aspect,
                        hit.whiteRatio,
                        range,
                        rangeResult.uncertainty,
                        dWidth,
                        dFloor,
                        dGround,
                        rangeResult,
                        viewingPenalty,
                        partialPenalty,
                        confidence,
                        robotPoint[0],
                        robotPoint[1],
                        cameraName,
                        captureTimeNanos);
                bestDraw = hit.box;
            }
        }

        if (bestForAlliance == null) {
            return null;
        }
        return new Result(bestForAlliance, bestDraw);
    }
}
