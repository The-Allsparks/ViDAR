package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeEstimate;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RangeFusionTest {

    @Test
    void sizeOnlyDistance() {
        double d = distanceFromSize(5.0, 340, 20);
        assertTrue(d > 40 && d < 45);
    }

    @Test
    void weightedFusionAgreement() {
        VidarRangeEstimate size = VidarRangeFusion.buildSizeEstimate(36, 20, 0.9, false, false);
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(38, 60, 0.8, false);
        VidarRangeResult result = VidarRangeFusion.fuseRangeWeighted(size, floor);
        assertTrue(result.isValid());
        assertTrue(result.distance > 34 && result.distance < 40);
        assertTrue(result.confidence > 0);
    }

    @Test
    void invalidFloorRejected() {
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(Double.NaN, 60, 0.8, true);
        assertFalse(floor.isValid());
        assertTrue("near_horizon".equals(floor.rejectionReason)
                || "invalid_lut".equals(floor.rejectionReason));
    }

    @Test
    void estimatorDisagreementLowersConfidence() {
        VidarRangeEstimate size = VidarRangeFusion.buildSizeEstimate(24, 20, 0.9, false, false);
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(48, 60, 0.8, false);
        VidarRangeResult result = VidarRangeFusion.fuseRangeWeighted(size, floor);
        assertTrue(result.confidence < 0.8);
    }

    @Test
    void threeWayFusionWhenGeometryAgrees() {
        VidarRangeEstimate size = VidarRangeFusion.buildSizeEstimate(24, 14, 0.9, false, false);
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(25, 200, 0.8, false);
        VidarRangeEstimate ground = VidarRangeFusion.buildGroundPlaneEstimate(24.5, 200, 0.8, false);
        VidarRangeResult result = VidarRangeFusion.fuseRangeWeighted(size, floor, ground);
        assertTrue(result.isValid());
        assertEquals(3, result.sourceCount);
        assertEquals(24.5, result.distance, 1e-9);
        assertTrue(result.confidence > 0.7);
        assertEquals(VidarRangeEstimate.Source.GROUND_PLANE, result.source0.source);
    }

    @Test
    void geometryDisagreementKeepsGroundDistance() {
        VidarRangeEstimate size = VidarRangeFusion.buildSizeEstimate(23, 20, 0.9, false, false);
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(23, 60, 0.8, false);
        VidarRangeEstimate ground = VidarRangeFusion.buildGroundPlaneEstimate(42, 200, 0.8, false);
        VidarRangeResult result = VidarRangeFusion.fuseRangeWeighted(size, floor, ground);
        assertTrue(result.isValid());
        assertEquals(42.0, result.distance, 1e-9);
        assertTrue(result.confidence < 0.5);
        assertTrue(result.distance > 35);
    }

    @Test
    void geometryOnlyUsesGroundPlane() {
        VidarRangeEstimate ground = VidarRangeFusion.buildGroundPlaneEstimate(36, 200, 0.8, false);
        VidarRangeResult result = VidarRangeFusion.fuseRangeWeighted(ground);
        assertTrue(result.isValid());
        assertEquals(36.0, result.distance, 1e-9);
        assertEquals(VidarRangeEstimate.Source.GROUND_PLANE, result.source0.source);
    }

    @Test
    void heuristicsOnlyStillWeighted() {
        VidarRangeEstimate size = VidarRangeFusion.buildSizeEstimate(36, 20, 0.9, false, false);
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(38, 60, 0.8, false);
        VidarRangeResult result = VidarRangeFusion.fuseRangeWeighted(size, floor);
        assertTrue(result.isValid());
        assertTrue(result.distance > 34 && result.distance < 40);
    }

    @Test
    void plateWidthEstimateValid() {
        double d = distanceFromWidth(12.0, 340, 80);
        assertTrue(d > 48 && d < 52);
        VidarRangeEstimate est = VidarRangeFusion.buildPlateWidthEstimate(
                d, 80, 0.8, 0.3, false, false, 0.1);
        assertTrue(est.isValid());
    }

    private static double distanceFromSize(double diameter, double focalPx, double radiusPx) {
        if (radiusPx <= 0 || focalPx <= 0 || diameter <= 0) {
            return Double.NaN;
        }
        return (diameter * focalPx) / (2.0 * radiusPx);
    }

    private static double distanceFromWidth(double physicalWidth, double focalPx, double pixelWidth) {
        if (pixelWidth <= 0 || focalPx <= 0 || physicalWidth <= 0) {
            return Double.NaN;
        }
        return (physicalWidth * focalPx) / pixelWidth;
    }
}
