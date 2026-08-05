package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Fuses decoded AprilTag observations only — scout observations never alter absolute pose.
 */
public final class VidarLocalizationFusion {

    private Pose2D fieldPosePrior;
    private Pose2D lastFusedFieldPose;
    private long lastCorrectionNanos;

    public void setFieldPosePrior(Pose2D prior) {
        fieldPosePrior = prior;
    }

    public Pose2D fieldPosePrior() {
        return fieldPosePrior;
    }

    public Pose2D lastFusedFieldPose() {
        return lastFusedFieldPose;
    }

    /**
     * Apply decoded tag correction with pose gates. Scout observations are ignored for pose fusion.
     */
    public Pose2D fusedFieldPoseNow(
            VidarTagObservation decoded,
            VidarTagScoutObservation scout,
            Pose2D odomNow) {
        if (decoded != null && decoded.fieldPoseAtCapture != null) {
            Pose2D candidate = VidarPoseBackdate.fieldPoseNow(
                    decoded, odomNow, DistanceUnit.INCH, AngleUnit.DEGREES);
            if (passesPoseGates(decoded, candidate, odomNow)) {
                lastFusedFieldPose = applyCorrectionLimit(candidate);
                lastCorrectionNanos = System.nanoTime();
            }
            return lastFusedFieldPose;
        }
        return lastFusedFieldPose;
    }

    private boolean passesPoseGates(VidarTagObservation tag, Pose2D candidate, Pose2D odomNow) {
        if (tag == null || candidate == null) {
            return false;
        }

        long ageMs = (System.nanoTime() - tag.captureTimeNanos) / 1_000_000L;
        if (ageMs > VidarTagConfig.MAX_OBSERVATION_AGE_MS) {
            return false;
        }

        if (tag.tagId < 0) {
            return false;
        }

        if (VidarTagConfig.MIN_DECISION_MARGIN > 0 && tag.decodePixels < VidarTagConfig.MIN_DECISION_MARGIN) {
            return false;
        }

        long sinceLastMs = (System.nanoTime() - lastCorrectionNanos) / 1_000_000L;
        if (lastCorrectionNanos > 0 && sinceLastMs < VidarTagConfig.CORRECTION_COOLDOWN_MS) {
            return false;
        }

        if (lastFusedFieldPose != null) {
            double dx = candidate.getX(DistanceUnit.INCH) - lastFusedFieldPose.getX(DistanceUnit.INCH);
            double dy = candidate.getY(DistanceUnit.INCH) - lastFusedFieldPose.getY(DistanceUnit.INCH);
            double trans = Math.hypot(dx, dy);
            if (trans > VidarTagConfig.MAX_TRANSLATION_RESIDUAL_IN) {
                return false;
            }
            double dHeading = Math.abs(normalizeDeg(
                    candidate.getHeading(AngleUnit.DEGREES) - lastFusedFieldPose.getHeading(AngleUnit.DEGREES)));
            if (dHeading > VidarTagConfig.MAX_HEADING_RESIDUAL_DEG) {
                return false;
            }
        }

        return true;
    }

    private Pose2D applyCorrectionLimit(Pose2D candidate) {
        if (lastFusedFieldPose == null) {
            return candidate;
        }
        double dx = candidate.getX(DistanceUnit.INCH) - lastFusedFieldPose.getX(DistanceUnit.INCH);
        double dy = candidate.getY(DistanceUnit.INCH) - lastFusedFieldPose.getY(DistanceUnit.INCH);
        double trans = Math.hypot(dx, dy);
        if (trans <= VidarTagConfig.MAX_CORRECTION_MAGNITUDE_IN) {
            return candidate;
        }
        double scale = VidarTagConfig.MAX_CORRECTION_MAGNITUDE_IN / trans;
        double newX = lastFusedFieldPose.getX(DistanceUnit.INCH) + dx * scale;
        double newY = lastFusedFieldPose.getY(DistanceUnit.INCH) + dy * scale;
        double dHeading = normalizeDeg(
                candidate.getHeading(AngleUnit.DEGREES) - lastFusedFieldPose.getHeading(AngleUnit.DEGREES));
        if (Math.abs(dHeading) > VidarTagConfig.MAX_HEADING_RESIDUAL_DEG) {
            dHeading = Math.signum(dHeading) * VidarTagConfig.MAX_HEADING_RESIDUAL_DEG;
        }
        return new Pose2D(
                DistanceUnit.INCH, newX, newY,
                AngleUnit.DEGREES,
                lastFusedFieldPose.getHeading(AngleUnit.DEGREES) + dHeading);
    }

    /** Scout-only observations must never change fused pose — explicit test hook. */
    public boolean wouldScoutAlterPose(VidarTagScoutObservation scout) {
        return false;
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
