package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeEstimate;
import org.firstinspires.ftc.teamcode.vidar.model.VidarRangeResult;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Generous ceiling for JVM range-fusion throughput.
 *
 * <p>GitHub-hosted runners are not a Control Hub. This is not a millisecond
 * budget — it only fails if fusion accidentally becomes catastrophically slow.
 */
class RangeFusionBudgetTest {

    private static final int ITERATIONS = 50_000;
    private static final long MAX_MS = 5_000;

    @Test
    void fiftyThousandThreeWayFusionsStayUnderFiveSeconds() {
        VidarRangeEstimate size = VidarRangeFusion.buildSizeEstimate(24, 14, 0.9, false, false);
        VidarRangeEstimate floor = VidarRangeFusion.buildFloorEstimate(25, 200, 0.8, false);
        VidarRangeEstimate ground = VidarRangeFusion.buildGroundPlaneEstimate(24.5, 200, 0.8, false);

        long start = System.nanoTime();
        VidarRangeResult last = null;
        for (int i = 0; i < ITERATIONS; i++) {
            last = VidarRangeFusion.fuseRangeWeighted(size, floor, ground);
        }
        long elapsedMs = (System.nanoTime() - start) / 1_000_000L;

        assertTrue(last != null && last.isValid(), "fusion must remain valid");
        assertTrue(
                elapsedMs < MAX_MS,
                "range fusion took " + elapsedMs + " ms for " + ITERATIONS
                        + " calls; expected < " + MAX_MS + " ms (algorithmic regression)");
    }
}
