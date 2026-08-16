package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagObservation;
import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class LocalizationFusionTest {

    @Test
    void scoutNeverAltersPose() {
        VidarLocalizationFusion fusion = new VidarLocalizationFusion();
        fusion.setFieldPosePrior(new Pose2D(DistanceUnit.INCH, 10, 10, AngleUnit.DEGREES, 0));
        VidarTagScoutObservation scout = new VidarTagScoutObservation(
                45, 40, 0.9, "cam", VidarFrameRegions.HorizontalBand.MIDDLE, 100, 100, 0);
        assertFalse(fusion.wouldScoutAlterPose(scout));
        Pose2D before = fusion.lastFusedFieldPose();
        VidarLocalizationFusion.Result result = fusion.fusedFieldPoseNow(null, scout, null, null);
        assertFalse(result.acceptedNewCorrection);
        assertEquals(before, fusion.lastFusedFieldPose());
    }

    @Test
    void decodedTagCanUpdatePose() {
        VidarLocalizationFusion fusion = new VidarLocalizationFusion();
        Pose2D fieldAtCapture = new Pose2D(DistanceUnit.INCH, 20, 30, AngleUnit.DEGREES, 90);
        VidarTagObservation tag = newTag(fieldAtCapture, System.nanoTime());
        Pose2D odom = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        VidarLocalizationFusion.Result result = fusion.fusedFieldPoseNow(tag, null, odom, odom);
        assertTrue(result.acceptedNewCorrection);
        assertNotNull(result.pose);
        assertEquals(20.0, result.pose.getX(DistanceUnit.INCH), 0.01);
        assertEquals(30.0, result.pose.getY(DistanceUnit.INCH), 0.01);
        assertTrue(result.correctionNanos > 0);
        assertEquals(result.correctionNanos, fusion.lastCorrectionNanos());
    }

    @Test
    void rejectedRepeatDoesNotAdvanceCorrectionNanos() {
        VidarLocalizationFusion fusion = new VidarLocalizationFusion();
        Pose2D fieldAtCapture = new Pose2D(DistanceUnit.INCH, 20, 30, AngleUnit.DEGREES, 90);
        Pose2D odom = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        VidarLocalizationFusion.Result first =
                fusion.fusedFieldPoseNow(newTag(fieldAtCapture, System.nanoTime()), null, odom, odom);
        assertTrue(first.acceptedNewCorrection);
        long nanos = fusion.lastCorrectionNanos();

        // Same decode within cooldown — must not accept or re-stamp.
        VidarLocalizationFusion.Result second =
                fusion.fusedFieldPoseNow(newTag(fieldAtCapture, System.nanoTime()), null, odom, odom);
        assertFalse(second.acceptedNewCorrection);
        assertEquals(nanos, fusion.lastCorrectionNanos());
        assertEquals(first.pose.getX(DistanceUnit.INCH), second.pose.getX(DistanceUnit.INCH), 1e-9);
    }

    @Test
    void gatedPropagationUsesOdomStampAtAcceptNotLaterTicks() {
        Pose2D anchor = new Pose2D(DistanceUnit.INCH, 20, 0, AngleUnit.DEGREES, 0);
        Pose2D odomAtFuse = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D odomLater = new Pose2D(DistanceUnit.INCH, 3, 1, AngleUnit.DEGREES, 10);
        // Simulate correct stamp behavior: keep odomAtFuse fixed across later ticks.
        Pose2D corrected = VidarMotionCorrection.robotFieldPoseNow(anchor, odomAtFuse, odomLater);
        assertEquals(23.0, corrected.getX(DistanceUnit.INCH), 1e-9);
        assertEquals(1.0, corrected.getY(DistanceUnit.INCH), 1e-9);
        assertEquals(10.0, corrected.getHeading(AngleUnit.DEGREES), 1e-9);
        // Wrong stamp (odomAtFuse overwritten to later) collapses to frozen fuse pose:
        Pose2D collapsed = VidarMotionCorrection.robotFieldPoseNow(anchor, odomLater, odomLater);
        assertEquals(20.0, collapsed.getX(DistanceUnit.INCH), 1e-9);
    }

    private static VidarTagObservation newTag(Pose2D fieldAtCapture, long captureNanos) {
        return new VidarTagObservation(
                1,
                fieldAtCapture,
                captureNanos,
                "cam",
                320,
                240,
                VidarFrameRegions.HorizontalBand.MIDDLE,
                2,
                500);
    }
}
