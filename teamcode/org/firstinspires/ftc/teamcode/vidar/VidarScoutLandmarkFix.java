package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.function.Supplier;

/**
 * @deprecated Replaced by {@link VidarTagScoutObservation} — scout quads must not localize.
 * Retained for reference only.
 */
@Deprecated
public final class VidarScoutLandmarkFix {

    private static final class TagCandidate {
        final int id;
        final double fieldXIn;
        final double fieldYIn;
        final double bearingErrDeg;

        TagCandidate(int id, double fieldXIn, double fieldYIn, double bearingErrDeg) {
            this.id = id;
            this.fieldXIn = fieldXIn;
            this.fieldYIn = fieldYIn;
            this.bearingErrDeg = bearingErrDeg;
        }
    }

    private VidarScoutLandmarkFix() {}

    public static VidarScoutLandmarkObservation compute(
            VidarTagScoutResult scout,
            VidarCameraProfile profile,
            int frameCols,
            Supplier<Pose2D> fieldPosePrior,
            Supplier<Pose2D> odomSupplier,
            long captureTimeNanos,
            String cameraName) {
        if (!VidarTagConfig.SCOUT_LANDMARK_ENABLED || scout == null || frameCols <= 0) {
            return null;
        }
        if (VidarTagCropPlanner.worthDecode(
                scout.widthPx, VidarTagConfig.SCOUT_WIDTH, frameCols)) {
            return null;
        }

        Pose2D prior = fieldPosePrior == null ? null : fieldPosePrior.get();
        if (prior == null) {
            return null;
        }

        double observedBearingDeg = observedBearingDeg(scout.cx, frameCols, profile.bearingDeg);
        TagCandidate best = bestTagAssociation(prior, observedBearingDeg);
        if (best == null) {
            return null;
        }

        double fullTagWidthPx = scout.widthPx * ((double) frameCols / VidarTagConfig.SCOUT_WIDTH);
        double rangeIn = (profile.focalLengthPx * VidarTagConfig.TAG_SIZE_IN) / Math.max(8.0, fullTagWidthPx);
        if (rangeIn < VidarTagConfig.SCOUT_LANDMARK_MIN_RANGE_IN
                || rangeIn > VidarTagConfig.SCOUT_LANDMARK_MAX_RANGE_IN) {
            return null;
        }

        double angleToTagDeg = Math.toDegrees(Math.atan2(
                best.fieldYIn - prior.getY(DistanceUnit.INCH),
                best.fieldXIn - prior.getX(DistanceUnit.INCH)));
        double headingDeg = normalizeDeg(angleToTagDeg - observedBearingDeg - profile.bearingDeg);

        double robotX = best.fieldXIn - rangeIn * Math.cos(Math.toRadians(angleToTagDeg));
        double robotY = best.fieldYIn - rangeIn * Math.sin(Math.toRadians(angleToTagDeg));

        double margin = best.bearingErrDeg;
        double confidence = Math.max(0.05, Math.min(0.55,
                0.55 - margin / 40.0 - rangeIn / 200.0));

        Pose2D odomAtCapture = odomSupplier == null ? null : odomSupplier.get();
        Pose2D fieldPose = new Pose2D(
                DistanceUnit.INCH, robotX, robotY, AngleUnit.DEGREES, headingDeg);

        return new VidarScoutLandmarkObservation(
                best.id,
                fieldPose,
                odomAtCapture,
                captureTimeNanos,
                cameraName,
                confidence,
                rangeIn,
                observedBearingDeg);
    }

    private static TagCandidate bestTagAssociation(Pose2D prior, double observedBearingDeg) {
        AprilTagLibrary library;
        try {
            library = AprilTagGameDatabase.getCurrentGameTagLibrary();
        } catch (RuntimeException ex) {
            return null;
        }
        if (library == null) {
            return null;
        }

        double rx = prior.getX(DistanceUnit.INCH);
        double ry = prior.getY(DistanceUnit.INCH);
        double rh = prior.getHeading(AngleUnit.DEGREES);

        TagCandidate best = null;
        TagCandidate second = null;

        for (AprilTagMetadata meta : library.getTags()) {
            double tx = DistanceUnit.METER.toInches(meta.fieldX);
            double ty = DistanceUnit.METER.toInches(meta.fieldY);
            double fieldBearing = Math.toDegrees(Math.atan2(ty - ry, tx - rx));
            double robotBearing = normalizeDeg(fieldBearing - rh);
            double err = Math.abs(normalizeDeg(robotBearing - observedBearingDeg));
            if (err > VidarTagConfig.SCOUT_LANDMARK_MAX_BEARING_ERR_DEG) {
                continue;
            }
            TagCandidate cand = new TagCandidate(meta.id, tx, ty, err);
            if (best == null || err < best.bearingErrDeg) {
                second = best;
                best = cand;
            } else if (second == null || err < second.bearingErrDeg) {
                second = cand;
            }
        }

        if (best == null) {
            return null;
        }
        double margin = second == null ? 90.0 : second.bearingErrDeg - best.bearingErrDeg;
        if (margin < VidarTagConfig.SCOUT_LANDMARK_MIN_MARGIN_DEG) {
            return null;
        }
        return best;
    }

    private static double observedBearingDeg(double cx, int frameCols, double cameraBearingDeg) {
        double centerErr = cx - frameCols / 2.0;
        double halfFovDeg = VidarTagConfig.HORIZONTAL_FOV_DEG / 2.0;
        double bearingErr = centerErr / Math.max(1, frameCols) * VidarTagConfig.HORIZONTAL_FOV_DEG;
        return normalizeDeg(cameraBearingDeg + bearingErr);
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
