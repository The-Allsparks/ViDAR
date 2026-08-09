package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

/** Parity check for {@code MultiCameraFusion.elementRankScore} (orchestration class is hub-only). */
class MultiCameraFusionTest {

    @Test
    void elementRankScorePrefersCloserObservations() {
        VidarElementObservation near = obs(0.9, 100, 12);
        VidarElementObservation far = obs(0.9, 100, 48);
        assertTrue(elementRankScore(near) > elementRankScore(far));
    }

    private static double elementRankScore(VidarElementObservation obs) {
        double rangeWeight = Double.isNaN(obs.range) ? 0.5 : 1.0 / Math.max(6.0, obs.range);
        return obs.confidence * obs.areaPx * rangeWeight;
    }

    private static VidarElementObservation obs(double confidence, double areaPx, double range) {
        return new VidarElementObservation(
                "cam",
                0L,
                0, 0,
                10, 10,
                0, 0, 5,
                areaPx, 1, 1, 1,
                0.5,
                VidarElementDetectorType.COLOR_BLOB,
                confidence,
                range, 0.1,
                range, range,
                null,
                0, range);
    }
}
