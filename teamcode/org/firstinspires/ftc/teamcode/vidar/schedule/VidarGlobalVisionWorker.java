package org.firstinspires.ftc.teamcode.vidar.schedule;

import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameMailbox;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarVision;
/**
 * Round-robin global worker — one camera per step, skip when no new mailbox generation.
 * Tic-toc slots are out-of-phase via {@link VidarProcessScheduler#VidarProcessScheduler(int)}.
 */
public final class VidarGlobalVisionWorker extends Thread {

    private final VidarWorkerCameraSlot[] slots;
    private volatile boolean running = true;
    private int roundRobinIndex;

    public VidarGlobalVisionWorker(VidarVision[] cameras) {
        super("VidarGlobalVisionWorker");
        setPriority(Thread.NORM_PRIORITY - 1);
        int count = 0;
        for (VidarVision camera : cameras) {
            if (camera != null && camera.hasAsyncWorker()) {
                count++;
            }
        }
        slots = new VidarWorkerCameraSlot[count];
        int j = 0;
        for (VidarVision camera : cameras) {
            if (camera != null && camera.hasAsyncWorker()) {
                slots[j++] = new VidarWorkerCameraSlot(camera);
            }
        }
    }

    public void shutdownAndJoin() {
        running = false;
        interrupt();
        try {
            join(500);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void run() {
        while (running) {
            if (slots.length == 0) {
                sleepQuiet(5);
                continue;
            }
            if (!processNext()) {
                sleepQuiet(1);
            }
        }
    }

    private boolean processNext() {
        for (int attempt = 0; attempt < slots.length; attempt++) {
            int index = (roundRobinIndex + attempt) % slots.length;
            VidarWorkerCameraSlot slot = slots[index];
            if (!slot.camera.isWorkerProcessingAllowed()) {
                continue;
            }
            VidarFrameMailbox.Snapshot snap = slot.camera.frameMailbox().tryTake(slot.lastProcessedGeneration);
            if (snap == null) {
                continue;
            }

            slot.camera.processSnapshot(snap);
            slot.lastProcessedGeneration = snap.generation;
            roundRobinIndex = (index + 1) % slots.length;
            return true;
        }
        return false;
    }

    private static void sleepQuiet(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private static final class VidarWorkerCameraSlot {
        final VidarVision camera;
        int lastProcessedGeneration;

        VidarWorkerCameraSlot(VidarVision camera) {
            this.camera = camera;
        }
    }
}
