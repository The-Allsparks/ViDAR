package org.firstinspires.ftc.teamcode.vidar.frame;

import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRoiRect;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class FrameRegionsTest {

    @Test
    void lowerFractionDefaultElement() {
        VidarRoiRect roi = VidarRoiRect.lowerFraction(640, 480, 0.65);
        assertEquals((int) (480 * 0.35), roi.y);
        assertEquals((int) (480 * 0.65), roi.height);
    }

    @Test
    void roiLocalToFull() {
        VidarRoiRect roi = new VidarRoiRect(0, 240, 640, 240, true);
        assertEquals(100, roi.toFullX(100));
        assertEquals(290, roi.toFullY(50));
    }

    @Test
    void invalidRoiClamped() {
        VidarRoiRect roi = new VidarRoiRect(-10, -5, 700, 500, true).clamped(640, 480);
        assertTrue(roi.x >= 0);
        assertTrue(roi.y >= 0);
        assertTrue(roi.x + roi.width <= 640);
    }
}
