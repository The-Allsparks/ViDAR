package org.firstinspires.ftc.teamcode.vidar.runtime;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Persistent background worker — advances fusion and world model independently of the robot loop.
 *
 * <p>Tick {@link RuntimeException}s are recorded and the worker stays alive so perception does not
 * take down the RC process. Students see last error and counts via {@link org.firstinspires.ftc.teamcode.vidar.api.VidarDiagnostics}.
 */
public final class VidarObservationWorker extends Thread {

    static final int MAX_ERROR_CHARS = 200;

    private final Runnable observationTick;
    private volatile boolean running = true;

    private final AtomicReference<String> lastErrorMessage = new AtomicReference<>("");
    private final AtomicInteger consecutiveFailureCount = new AtomicInteger(0);
    private final AtomicInteger totalFailureCount = new AtomicInteger(0);

    public VidarObservationWorker(Runnable observationTick) {
        super("VidarObservationWorker");
        setPriority(Thread.NORM_PRIORITY - 1);
        this.observationTick = observationTick;
    }

    public String lastErrorMessage() {
        return lastErrorMessage.get();
    }

    public int consecutiveFailureCount() {
        return consecutiveFailureCount.get();
    }

    public int totalFailureCount() {
        return totalFailureCount.get();
    }

    public void shutdownAndJoin() {
        running = false;
        interrupt();
        try {
            join(750);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void run() {
        while (running) {
            try {
                observationTick.run();
                onTickSuccess();
            } catch (RuntimeException ex) {
                onTickFailure(ex);
            }
            if (!running) {
                break;
            }
            sleepQuiet(1);
        }
    }

    /** Package-visible so tests can record without racing the 1 ms loop. */
    void onTickSuccess() {
        consecutiveFailureCount.set(0);
    }

    /** Package-visible so tests can record without racing the 1 ms loop. */
    void onTickFailure(RuntimeException ex) {
        lastErrorMessage.set(boundMessage(ex));
        consecutiveFailureCount.incrementAndGet();
        totalFailureCount.incrementAndGet();
    }

    static String boundMessage(Throwable ex) {
        String name = ex.getClass().getSimpleName();
        String detail = ex.getMessage();
        String msg = (detail == null || detail.isEmpty()) ? name : name + ": " + detail;
        if (msg.length() <= MAX_ERROR_CHARS) {
            return msg;
        }
        return msg.substring(0, MAX_ERROR_CHARS);
    }

    private static void sleepQuiet(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
