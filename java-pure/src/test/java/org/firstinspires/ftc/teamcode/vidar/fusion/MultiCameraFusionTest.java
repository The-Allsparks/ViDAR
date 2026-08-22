package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MultiCameraFusionTest {

    @Test
    void elementRankScorePrefersCloserObservations() {
        VidarElementObservation near = obs(0.9, 100, 12);
        VidarElementObservation far = obs(0.9, 100, 48);
        assertTrue(MultiCameraFusion.elementRankScore(near) > MultiCameraFusion.elementRankScore(far));
    }

    @Test
    void isUsableCameraRejectsNullFailedAndExcluded() {
        assertFalse(MultiCameraFusion.isUsableCamera(null));

        VidarVision ok = cameraWith(obs(0.9, 100, 12, 5, 5));
        assertTrue(MultiCameraFusion.isUsableCamera(ok));

        VidarVision failed = cameraWith(obs(0.9, 100, 12, 5, 5));
        failed.setFailed(true);
        assertFalse(MultiCameraFusion.isUsableCamera(failed));

        VidarVision excluded = cameraWith(obs(0.9, 100, 12, 5, 5));
        excluded.setExcludedFromRotation(true);
        assertFalse(MultiCameraFusion.isUsableCamera(excluded));
    }

    @Test
    void fuseRankedElementsDedupesNearbyDetections() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        VidarElementObservation a = obs(0.95, 100, 12, 10, 10);
        VidarElementObservation b = obs(0.9, 100, 14, 10.5, 10.2);

        VidarVision cam0 = cameraWith(a);
        VidarVision cam1 = cameraWith(b);
        VidarVision[] cameras = {cam0, cam1};

        VidarRankedElementFrame fused = MultiCameraFusion.fuseRankedElements(
                cameras, filter, VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
        assertEquals(1, fused.count());
        assertEquals(0, fused.overflowCount());
    }

    @Test
    void fuseRankedElementsSkipsFailedCameras() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        VidarVision healthy = cameraWith(obs(0.9, 100, 12, 20, 0));
        VidarVision failed = cameraWith(obs(0.99, 200, 8, 0, 20));
        failed.setFailed(true);

        VidarRankedElementFrame fused = MultiCameraFusion.fuseRankedElements(
                new VidarVision[] {healthy, failed}, filter, VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
        assertEquals(1, fused.count());
        assertEquals(20.0, fused.at(0).robotX, 1e-9);
    }

    @Test
    void fuseRankedElementsSkipsExcludedCameras() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        VidarVision healthy = cameraWith(obs(0.9, 100, 12, 20, 0));
        VidarVision excluded = cameraWith(obs(0.99, 200, 8, 0, 20));
        excluded.setExcludedFromRotation(true);

        VidarRankedElementFrame fused = MultiCameraFusion.fuseRankedElements(
                new VidarVision[] {healthy, excluded}, filter, VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
        assertEquals(1, fused.count());
        assertEquals(20.0, fused.at(0).robotX, 1e-9);
    }

    private static VidarVision cameraWith(VidarElementObservation observation) {
        VidarElementObservation[] ranked = new VidarElementObservation[8];
        ranked[0] = observation;
        return new VidarVision(new VidarRankedElementFrame(ranked, 1, 0, 1, "cam", 8));
    }

    private static VidarElementObservation obs(
            double confidence, double areaPx, double range, double robotX, double robotY) {
        return new VidarElementObservation(
                "cam",
                1L,
                0, 0,
                10, 10,
                5, 5, 5,
                areaPx, 1, 1, 1,
                0.5,
                VidarElementDetectorType.COLOR_BLOB,
                confidence,
                range, 0.1,
                range, range,
                null,
                robotX, robotY);
    }

    private static VidarElementObservation obs(double confidence, double areaPx, double range) {
        return obs(confidence, areaPx, range, 0, 0);
    }
}
