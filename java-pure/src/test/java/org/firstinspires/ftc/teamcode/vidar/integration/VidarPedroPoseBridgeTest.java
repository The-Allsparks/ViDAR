package org.firstinspires.ftc.teamcode.vidar.integration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class VidarPedroPoseBridgeTest {

    @Test
    void roundTripPreservesInchesAndRadians() {
        Pose2D original = new Pose2D(DistanceUnit.INCH, 12.5, -4.0, AngleUnit.DEGREES, 90.0);
        VidarPedroPose pedro = VidarPedroPoseBridge.fromPose2D(original);
        assertNotNull(pedro);
        assertEquals(12.5, pedro.x, 1e-9);
        assertEquals(-4.0, pedro.y, 1e-9);
        assertEquals(Math.toRadians(90.0), pedro.headingRad, 1e-9);

        Pose2D back = VidarPedroPoseBridge.toPose2D(pedro);
        assertEquals(12.5, back.getX(DistanceUnit.INCH), 1e-9);
        assertEquals(-4.0, back.getY(DistanceUnit.INCH), 1e-9);
        assertEquals(90.0, back.getHeading(AngleUnit.DEGREES), 1e-6);
    }

    @Test
    void nullSafe() {
        assertNull(VidarPedroPoseBridge.fromPose2D(null));
        assertNull(VidarPedroPoseBridge.toPose2D((VidarPedroPose) null));
        assertNull(VidarPedroPoseBridge.asPose2DSupplier(null));
    }

    @Test
    void componentSupplierBuildsPose2D() {
        Pose2D pose = VidarPedroPoseBridge.asPose2DSupplier(
                () -> 1.0, () -> 2.0, () -> Math.PI / 2.0).get();
        assertEquals(1.0, pose.getX(DistanceUnit.INCH), 1e-9);
        assertEquals(2.0, pose.getY(DistanceUnit.INCH), 1e-9);
        assertEquals(90.0, pose.getHeading(AngleUnit.DEGREES), 1e-6);
    }
}
