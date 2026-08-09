package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class MotionTransformTest {

    @Test
    void forwardMotion() {
        VidarMotionTransform t = VidarMotionTransform.fromOdomDelta(0, 0, 0, 12, 0, 0);
        double[] p = t.transformPoint(24, 0);
        assertEquals(12, p[0], 0.01);
        assertEquals(0, p[1], 0.01);
    }

    @Test
    void rotationInPlace() {
        VidarMotionTransform t = VidarMotionTransform.fromOdomDelta(0, 0, 0, 0, 0, 90);
        double[] p = t.transformPoint(10, 0);
        assertEquals(0, p[0], 0.1);
        assertEquals(-10, p[1], 0.1);
    }

    @Test
    void angleWraparound() {
        VidarMotionTransform t = VidarMotionTransform.fromOdomDelta(0, 0, 170, 0, 0, -170);
        assertEquals(20, Math.abs(t.deltaHeadingDeg), 1.0);
    }
}
