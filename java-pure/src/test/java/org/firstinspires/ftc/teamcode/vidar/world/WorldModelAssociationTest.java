package org.firstinspires.ftc.teamcode.vidar.world;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class WorldModelAssociationTest {

    private static final long CAPTURE = 1000L;
    private static final long MS = 1_000_000L;

    @Test
    void birthsTrackFromDetection() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);

        assertEquals(1, world.trackCount());
        VidarSpatialTrack track = world.getTracks().get(0);
        assertEquals(0, track.missCount);
        assertEquals(CAPTURE, track.lastSeenNanos);
        assertEquals(VidarWorldModel.Kind.ELEMENT, track.kind);
    }

    @Test
    void replayingSameCaptureIsAMissNotAHit() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);
        long lastSeen = world.getTracks().get(0).lastSeenNanos;

        long later = CAPTURE + nanos(0.2);
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, later);

        assertEquals(1, world.trackCount());
        VidarSpatialTrack track = world.getTracks().get(0);
        assertTrue(track.missCount > 0, "stale capture must coast, not reset TTL");
        assertEquals(lastSeen, track.lastSeenNanos);
    }

    @Test
    void sameCapturePastTtlPrunesTrack() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);

        long pastTtl = CAPTURE + nanos(VidarConfig.WORLD_ELEMENT_TTL_SEC + 0.1);
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, pastTtl);

        assertEquals(0, world.trackCount());
    }

    @Test
    void emptyDetectionsPastTtlPruneTrack() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);

        long pastTtl = CAPTURE + nanos(VidarConfig.WORLD_ELEMENT_TTL_SEC + 0.1);
        world.updateFromDetections(Collections.emptyList(), 0L, pastTtl);

        assertEquals(0, world.trackCount());
    }

    @Test
    void updateNullCoastsAndPrunesAfterTtl() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);
        assertEquals(1, world.trackCount());

        world.update(null, CAPTURE + nanos(0.2));
        assertEquals(1, world.trackCount());
        assertTrue(world.getTracks().get(0).missCount > 0);

        world.update(null, CAPTURE + nanos(VidarConfig.WORLD_ELEMENT_TTL_SEC + 0.1));
        assertEquals(0, world.trackCount());
    }

    @Test
    void oneMillisecondStaleReplaysDoNotCountAsMissFrames() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);
        long lastSeen = world.getTracks().get(0).lastSeenNanos;

        for (int i = 1; i <= 20; i++) {
            world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE + i * MS);
        }

        assertEquals(1, world.trackCount());
        VidarSpatialTrack track = world.getTracks().get(0);
        assertEquals(lastSeen, track.lastSeenNanos);
        assertTrue(track.missCount <= 1, "1 ms worker ticks must not explode missCount");
    }

    @Test
    void intakeBlockedClearsWhenFoeTrackPruned() {
        VidarWorldModel world = model();
        VidarTrackDetection foe = new VidarTrackDetection(
                VidarWorldModel.Kind.FOE, "", -1, 20.0, 0.0, 20.0, 0.9, "Webcam 1", CAPTURE);
        world.updateFromDetections(List.of(foe), CAPTURE, CAPTURE);
        assertTrue(world.intakeBlocked());

        long pastTtl = CAPTURE + nanos(VidarConfig.WORLD_FOE_TTL_SEC + 0.1);
        world.updateFromDetections(Collections.emptyList(), 0L, pastTtl);

        assertEquals(0, world.trackCount());
        assertFalse(world.intakeBlocked());
    }

    @Test
    void associatorPrunesAfterMaxMissFrames() {
        VidarWorldModel world = model();
        world.updateFromDetections(List.of(element(CAPTURE)), CAPTURE, CAPTURE);

        long now = CAPTURE + nanos(VidarConfig.WORLD_TRACK_MIN_DT_SEC);
        for (int i = 0; i <= VidarConfig.WORLD_TRACK_MAX_MISS_FRAMES; i++) {
            now += nanos(VidarConfig.WORLD_TRACK_MIN_DT_SEC);
            world.updateFromDetections(Collections.emptyList(), 0L, now);
        }

        assertEquals(0, world.trackCount());
    }

    private static VidarWorldModel model() {
        return new VidarWorldModel(() -> null, null);
    }

    private static VidarTrackDetection element(long captureNanos) {
        return new VidarTrackDetection(
                VidarWorldModel.Kind.ELEMENT,
                "artifact_purple",
                0,
                12.0,
                0.0,
                12.0,
                0.9,
                "Webcam 1",
                captureNanos);
    }

    private static long nanos(double seconds) {
        return (long) (seconds * 1_000_000_000L);
    }
}
