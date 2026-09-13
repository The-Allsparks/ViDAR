package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.junit.jupiter.api.Test;
import org.opencv.core.Rect;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CaptureResolutionTest {

    @Test
    void portalsRequest1280x720() {
        assertEquals(1280, VidarTagConfig.CAPTURE_RESOLUTION.getWidth());
        assertEquals(720, VidarTagConfig.CAPTURE_RESOLUTION.getHeight());
        assertEquals(1280, VidarConfig.PORTAL_RESOLUTION.getWidth());
        assertEquals(720, VidarConfig.PORTAL_RESOLUTION.getHeight());
        assertEquals(1280, VidarConfig.portalCameraResolution().getWidth());
        assertEquals(720, VidarConfig.portalCameraResolution().getHeight());
        assertEquals("1280x720", VidarConfig.portalCaptureLabel());
        assertTrue(VidarConfig.PORTAL_RESOLUTION.getWidth() < 2000,
                "Native 8MP (e.g. 3264x2448) must not be selected");
    }

    @Test
    void streamFormatIsMjpegNeverYuy2() {
        assertEquals(VisionPortal.StreamFormat.MJPEG, VidarConfig.portalStreamFormat(1));
        assertEquals(VisionPortal.StreamFormat.MJPEG, VidarConfig.portalStreamFormat(4));
    }

    @Test
    void tagDecodeCropIsSmallerThanFull720pFrame() {
        int fullW = VidarTagConfig.CAPTURE_RESOLUTION.getWidth();
        int fullH = VidarTagConfig.CAPTURE_RESOLUTION.getHeight();
        Rect crop = VidarFrameRegions.tagDecodeCrop(
                fullW, fullH, VidarFrameRegions.HorizontalBand.MIDDLE);
        assertTrue(crop.width < fullW, "AprilTag must not run on the full 720p width");
        assertTrue(crop.height < fullH, "AprilTag must not run on the full 720p height");
        assertTrue(crop.width * crop.height < fullW * fullH);
        assertEquals(fullW / 2, crop.width);
    }

    @Test
    void defaultFovIs105AndFxScalesFrom640() {
        assertEquals(105.0, VidarTagConfig.HORIZONTAL_FOV_DEG, 1e-9);
        assertEquals(492.0, VidarTagConfig.LENS_FX, 1e-9);
        assertEquals(246.0 * (1280.0 / 640.0), VidarTagConfig.LENS_FX, 1e-9);
    }
}
