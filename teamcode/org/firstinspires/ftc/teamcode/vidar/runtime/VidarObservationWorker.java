package org.firstinspires.ftc.teamcode.vidar.runtime;

/**
 * Persistent background worker — advances fusion and world model independently of the robot loop.
 */
public final class VidarObservationWorker extends Thread {

    private final Runnable observationTick;
    private volatile boolean running = true;

    public VidarObservationWorker(Runnable observationTick) {
        super("VidarObservationWorker");
        setPriority(Thread.NORM_PRIORITY - 1);
        this.observationTick = observationTick;
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
            } catch (RuntimeException ignored) {
                // Keep worker alive — perception must not depend on robot loop.
            }
            if (!running) {
                break;
            }
            sleepQuiet(1);
        }
    }

    private static void sleepQuiet(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
