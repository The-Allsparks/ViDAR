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
        fusion.fusedFieldPoseNow(null, scout, null, null);
        assertEquals(before, fusion.lastFusedFieldPose());
    }

    @Test
    void decodedTagCanUpdatePose() {
        VidarLocalizationFusion fusion = new VidarLocalizationFusion();
        Pose2D fieldAtCapture = new Pose2D(DistanceUnit.INCH, 20, 30, AngleUnit.DEGREES, 90);
        VidarTagObservation tag = new VidarTagObservation(
                1,
                fieldAtCapture,
                System.nanoTime(),
                "cam",
                320,
                240,
                VidarFrameRegions.HorizontalBand.MIDDLE,
                2,
                500);
        Pose2D odom = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D fused = fusion.fusedFieldPoseNow(tag, null, odom, odom);
        assertNotNull(fused);
        assertEquals(20.0, fused.getX(DistanceUnit.INCH), 0.01);
        assertEquals(30.0, fused.getY(DistanceUnit.INCH), 0.01);
    }
}
