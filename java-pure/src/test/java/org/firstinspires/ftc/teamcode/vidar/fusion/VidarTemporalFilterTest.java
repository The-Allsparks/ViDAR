package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VidarTemporalFilterTest {

    @Test
    void weakObservationRejectedWithoutPendingState() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        assertNull(filter.filterElement(obs(0.4, 10, 10, 1)));
    }

    @Test
    void weakObservationConfirmsAfterStrongPrime() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        filter.filterElement(obs(0.95, 10, 10, 1));
        VidarElementObservation weak = obs(0.4, 10.1, 10, 2);
        assertEquals(weak, filter.filterElement(weak));
    }

    @Test
    void strongObservationPassesImmediately() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        VidarElementObservation strong = obs(0.95, 10, 10, 1);
        assertEquals(strong, filter.filterElement(strong));
    }

    @Test
    void resetMatchStateClearsPending() {
        VidarTemporalFilter filter = new VidarTemporalFilter();
        filter.filterElement(obs(0.4, 10, 10, 1));
        filter.resetMatchState();
        assertNull(filter.filterElement(obs(0.4, 10.2, 10.1, 2)));
    }

    private static VidarElementObservation obs(double confidence, double x, double y, long captureNanos) {
        return new VidarElementObservation(
                "cam",
                captureNanos,
                0, 0,
                10, 10,
                x, y, 5,
                100, 1, 1, 1,
                0.5,
                VidarElementDetectorType.COLOR_BLOB,
                confidence,
                12, 0.1,
                12, 12,
                null,
                0, 12);
    }
}
