package org.firstinspires.ftc.teamcode.vidar.world;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Generous ceiling for world-model association. Not a Control Hub measurement.
 */
class WorldModelBudgetTest {

    private static final int TRACKS = 24;
    private static final int UPDATES = 400;
    private static final long MAX_MS = 5_000;

    @Test
    void repeatedAssociationStaysUnderFiveSeconds() {
        VidarWorldModel world = new VidarWorldModel(() -> null, null);
        long capture = 1_000_000_000L;
        world.updateFromDetections(detections(capture, TRACKS), capture, capture);

        long start = System.nanoTime();
        long now = capture;
        for (int i = 1; i <= UPDATES; i++) {
            now = capture + i * 20_000_000L;
            world.updateFromDetections(detections(now, TRACKS), now, now);
        }
        long elapsedMs = (System.nanoTime() - start) / 1_000_000L;

        assertTrue(world.trackCount() > 0, "tracks should persist across updates");
        assertTrue(
                elapsedMs < MAX_MS,
                "world association took " + elapsedMs + " ms for " + UPDATES
                        + " updates of " + TRACKS + " detections; expected < " + MAX_MS + " ms");
    }

    private static List<VidarTrackDetection> detections(long captureNanos, int count) {
        List<VidarTrackDetection> list = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            double x = 12.0 + i * 3.0;
            list.add(new VidarTrackDetection(
                    VidarWorldModel.Kind.ELEMENT,
                    "artifact_purple",
                    i,
                    x,
                    0.0,
                    x,
                    0.9,
                    "Webcam 1",
                    captureNanos));
        }
        return list;
    }
}
