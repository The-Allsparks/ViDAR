package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraRoiConfig;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Vitelli axes (roll +X, pitch +Y, yaw +Z) with intrinsic yaw → pitch → roll:
 * {@code Rz(bearing+yaw) * Ry(pitch_rh) * Rx(roll) * opticalToRobotBase},
 * with config {@code pitchDeg} aviation-signed (negative = look down).
 */
class MountRotationConventionTest {

    private static final double EPS = 1e-6;

    private static VidarCameraProfile profile(
            double bearing, double yaw, double pitch, double roll) {
        return new VidarCameraProfile(
                "test",
                bearing,
                12,
                340,
                340,
                320,
                240,
                70,
                55,
                new double[] {95},
                new double[] {12},
                0,
                0,
                9,
                yaw,
                pitch,
                roll,
                12.0,
                VidarCameraRoiConfig.DEFAULT);
    }

    private static VidarVec3 opticalAxisInRobot(
            VidarCameraProfile p, double ox, double oy, double oz) {
        VidarTransformRegistry.CameraTransforms ct =
                VidarTransformRegistry.buildForProfile(p);
        return ct.robotTCamera.transformDirection(new VidarVec3(ox, oy, oz));
    }

    private static void assertVec(VidarVec3 v, double x, double y, double z) {
        assertEquals(x, v.x, EPS, "x");
        assertEquals(y, v.y, EPS, "y");
        assertEquals(z, v.z, EPS, "z");
    }

    @Test
    void opticalToRobotBaseMapsOpenCvAxes() {
        VidarRotation3D r = VidarRotation3D.opticalToRobotBase();
        assertVec(r.rotate(new VidarVec3(0, 0, 1)), 1, 0, 0);
        assertVec(r.rotate(new VidarVec3(1, 0, 0)), 0, -1, 0);
        assertVec(r.rotate(new VidarVec3(0, 1, 0)), 0, 0, -1);
    }

    @Test
    void identityMountFacesForwardLevel() {
        VidarCameraProfile p = profile(0, 0, 0, 0);
        assertVec(opticalAxisInRobot(p, 0, 0, 1), 1, 0, 0);
        assertVec(opticalAxisInRobot(p, 1, 0, 0), 0, -1, 0);
        assertVec(opticalAxisInRobot(p, 0, 1, 0), 0, 0, -1);
    }

    @Test
    void negativePitchNodsOpticalForwardDown() {
        VidarCameraProfile p = profile(0, 0, -30, 0);
        double s30 = Math.sin(Math.toRadians(30));
        double c30 = Math.cos(Math.toRadians(30));
        // pitchDeg -30 → RH pitch +30 → optical forward (+X) toward −Z
        assertVec(opticalAxisInRobot(p, 0, 0, 1), c30, 0, -s30);
        assertVec(opticalAxisInRobot(p, 1, 0, 0), 0, -1, 0);
        assertVec(opticalAxisInRobot(p, 0, 1, 0), -s30, 0, -c30);
    }

    @Test
    void rollBanksAboutOpticalForward() {
        VidarCameraProfile p = profile(0, 0, 0, 30);
        double s30 = Math.sin(Math.toRadians(30));
        double c30 = Math.cos(Math.toRadians(30));
        assertVec(opticalAxisInRobot(p, 0, 0, 1), 1, 0, 0);
        assertVec(opticalAxisInRobot(p, 1, 0, 0), 0, -c30, -s30);
        assertVec(opticalAxisInRobot(p, 0, 1, 0), 0, s30, -c30);
    }

    @Test
    void yawAndBearingPanAboutVertical() {
        VidarCameraProfile byYaw = profile(0, 30, 0, 0);
        VidarCameraProfile byBearing = profile(30, 0, 0, 0);
        double s30 = Math.sin(Math.toRadians(30));
        double c30 = Math.cos(Math.toRadians(30));
        assertVec(opticalAxisInRobot(byYaw, 0, 0, 1), c30, s30, 0);
        assertVec(opticalAxisInRobot(byBearing, 0, 0, 1), c30, s30, 0);
    }

    @Test
    void rollAndYawAreIndependentAxes() {
        VidarVec3 afterRoll = opticalAxisInRobot(profile(0, 0, 0, 30), 0, 0, 1);
        VidarVec3 afterYaw = opticalAxisInRobot(profile(0, 30, 0, 0), 0, 0, 1);
        assertEquals(1.0, afterRoll.x, EPS);
        assertTrue(Math.abs(afterYaw.y) > 0.4);
        assertNotEquals(afterRoll.y, afterYaw.y, 0.1);
    }

    @Test
    void bearing90FacesRobotPositiveY() {
        VidarCameraProfile p = profile(90, 0, 0, 0);
        assertVec(opticalAxisInRobot(p, 0, 0, 1), 0, 1, 0);
        assertVec(opticalAxisInRobot(p, 1, 0, 0), 1, 0, 0);
        assertVec(opticalAxisInRobot(p, 0, 1, 0), 0, 0, -1);
    }

    @Test
    void bearing90WithPitchDownTiltsIntoFloor() {
        VidarCameraProfile p = profile(90, 0, -30, 0);
        double s30 = Math.sin(Math.toRadians(30));
        double c30 = Math.cos(Math.toRadians(30));
        // Yaw 90 then pitch: forward was +Y, pitch about Y moves it toward −Z
        assertVec(opticalAxisInRobot(p, 0, 0, 1), 0, c30, -s30);
    }

    @Test
    void fromRollPitchYawIsVitelliXyz() {
        VidarRotation3D r = VidarRotation3D.fromRollPitchYawDeg(10, -20, 30);
        VidarRotation3D expected = VidarRotation3D.rotateZ(Math.toRadians(30))
                .times(VidarRotation3D.rotateY(Math.toRadians(-20)))
                .times(VidarRotation3D.rotateX(Math.toRadians(10)));
        for (int i = 0; i < 9; i++) {
            assertEquals(expected.m[i], r.m[i], EPS, "m[" + i + "]");
        }
    }

    @Test
    void defaultConfigPitchDepressesOpticalAxis() {
        VidarCameraProfile p = profile(0, 0, -12, 0);
        VidarVec3 forward = opticalAxisInRobot(p, 0, 0, 1);
        assertTrue(forward.z < -0.15, "pitchDeg -12 must depress optical forward");
        assertEquals(0.0, forward.y, EPS);
    }
}
