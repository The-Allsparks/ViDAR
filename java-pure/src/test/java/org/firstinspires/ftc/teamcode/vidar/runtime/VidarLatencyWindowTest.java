package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VidarLatencyWindowTest {

    @Test
    void emptyWindowReportsZeros() {
        VidarLatencyWindow window = new VidarLatencyWindow(8);
        assertEquals(0, window.sampleCount());
        assertEquals(0.0, window.p50Ms(), 1e-9);
        assertEquals(0.0, window.p95Ms(), 1e-9);
        assertEquals(0.0, window.maxMs(), 1e-9);
    }

    @Test
    void rejectsNonPositiveCapacity() {
        assertThrows(IllegalArgumentException.class, () -> new VidarLatencyWindow(0));
    }

    @Test
    void percentilesMatchSortedSeries() {
        VidarLatencyWindow window = new VidarLatencyWindow(16);
        // 1..20 ms as nanoseconds (only last 16 kept once capacity fills).
        for (int ms = 1; ms <= 20; ms++) {
            window.record(ms * 1_000_000L);
        }
        assertEquals(16, window.sampleCount());
        // Window holds 5..20 ms. p50 → ceil(0.5*16)-1 = 7 → 12 ms; p95 → ceil(0.95*16)-1 = 15 → 20 ms.
        assertEquals(12.0, window.p50Ms(), 1e-9);
        assertEquals(20.0, window.p95Ms(), 1e-9);
        assertEquals(20.0, window.maxMs(), 1e-9);
    }

    @Test
    void clampsNegativeSamples() {
        VidarLatencyWindow window = new VidarLatencyWindow(4);
        window.record(-5_000_000L);
        window.record(2_000_000L);
        assertEquals(0.0, window.percentileMs(0.0), 1e-9);
        assertEquals(2.0, window.maxMs(), 1e-9);
    }

    @Test
    void clearResetsWindow() {
        VidarLatencyWindow window = new VidarLatencyWindow(4);
        window.record(3_000_000L);
        window.clear();
        assertEquals(0, window.sampleCount());
        assertEquals(0.0, window.maxMs(), 1e-9);
    }

    @Test
    void recordDoesNotAllocateBeyondConstruction() {
        VidarLatencyWindow window = new VidarLatencyWindow(64);
        Runtime runtime = Runtime.getRuntime();
        runtime.gc();
        long before = usedHeap(runtime);
        for (int i = 0; i < 10_000; i++) {
            window.record(i);
        }
        long after = usedHeap(runtime);
        // Generous: recording must not create thousands of objects. Allow a few MB of noise.
        assertTrue(after - before < 2_000_000L,
                "record() heap delta too large: " + (after - before) + " bytes");
        assertEquals(64, window.sampleCount());
    }

    private static long usedHeap(Runtime runtime) {
        return runtime.totalMemory() - runtime.freeMemory();
    }
}
