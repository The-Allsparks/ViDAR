package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.junit.jupiter.api.Test;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ObservationWorkerTest {

    @Test
    void recordsFailureThenResetsConsecutiveOnSuccess() {
        VidarObservationWorker worker = new VidarObservationWorker(() -> { });
        worker.onTickFailure(new RuntimeException("boom"));

        assertEquals(1, worker.consecutiveFailureCount());
        assertEquals(1, worker.totalFailureCount());
        assertTrue(worker.lastErrorMessage().contains("boom"));
        assertTrue(worker.lastErrorMessage().length() <= VidarObservationWorker.MAX_ERROR_CHARS);

        worker.onTickSuccess();
        assertEquals(0, worker.consecutiveFailureCount());
        assertEquals(1, worker.totalFailureCount());
        assertTrue(worker.lastErrorMessage().contains("boom"));
    }

    @Test
    void boundsLastErrorMessage() {
        VidarObservationWorker worker = new VidarObservationWorker(() -> { });
        worker.onTickFailure(new RuntimeException("x".repeat(500)));
        assertEquals(VidarObservationWorker.MAX_ERROR_CHARS, worker.lastErrorMessage().length());
    }

    @Test
    void threadStaysAliveAfterThrowThenSuccess() throws InterruptedException {
        AtomicInteger ticks = new AtomicInteger();
        CountDownLatch recovered = new CountDownLatch(1);
        VidarObservationWorker worker = new VidarObservationWorker(() -> {
            int n = ticks.getAndIncrement();
            if (n == 0) {
                throw new RuntimeException("tick failed");
            }
            recovered.countDown();
        });
        worker.start();
        try {
            assertTrue(recovered.await(5, TimeUnit.SECONDS), "worker should run a successful tick after the throw");
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(1);
            while (System.nanoTime() < deadline && worker.consecutiveFailureCount() != 0) {
                Thread.sleep(1);
            }
            assertTrue(worker.isAlive(), "worker thread must stay alive after a tick exception");
            assertEquals(1, worker.totalFailureCount());
            assertEquals(0, worker.consecutiveFailureCount());
            assertTrue(worker.lastErrorMessage().contains("tick failed"));
        } finally {
            worker.shutdownAndJoin();
        }
    }
}
