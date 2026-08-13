package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarRankedElementFrame;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MultiCameraFusionTest {

    @Test
    void elementRankScorePrefersCloserObservations() {
        VidarElementObservation near = obs(0.9, 100, 12);
        VidarElementObservation far = obs(0.9, 100, 48);
        assertTrue(MultiCameraFusion.elementRankScore(near) > MultiCameraFusion.elementRankScore(far));
    }

    @Test
    void fuseRankedElementsDedupesNearbyDetections() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        VidarElementObservation a = obs(0.95, 100, 12, 10, 10);
        VidarElementObservation b = obs(0.9, 100, 14, 10.5, 10.2);

        VidarElementObservation[] rankedA = new VidarElementObservation[8];
        rankedA[0] = a;
        VidarElementObservation[] rankedB = new VidarElementObservation[8];
        rankedB[0] = b;
        VidarVision cam0 = new VidarVision(new VidarRankedElementFrame(rankedA, 1, 0, 1, "cam0", 8));
        VidarVision cam1 = new VidarVision(new VidarRankedElementFrame(rankedB, 1, 0, 2, "cam1", 8));
        VidarVision[] cameras = {cam0, cam1};

        VidarRankedElementFrame fused = MultiCameraFusion.fuseRankedElements(
                cameras, filter, VidarConfig.FUSION_MAX_RANKED_ELEMENTS);
        assertEquals(1, fused.count());
        assertEquals(0, fused.overflowCount());
    }

    private static VidarElementObservation obs(
            double confidence, double areaPx, double range, double robotX, double robotY) {
        return new VidarElementObservation(
                "cam",
                1L,
                0, 0,
                10, 10,
                robotX, robotY, 5,
                areaPx, 1, 1, 1,
                0.5,
                VidarElementDetectorType.COLOR_BLOB,
                confidence,
                range, 0.1,
                range, range,
                null,
                0, range);
    }

    private static VidarElementObservation obs(double confidence, double areaPx, double range) {
        return obs(confidence, areaPx, range, 0, 0);
    }
}
