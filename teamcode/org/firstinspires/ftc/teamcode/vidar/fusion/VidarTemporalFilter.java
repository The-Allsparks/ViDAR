package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.teamcode.vidar.VidarPlateObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarElementObservation;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntimeConfig;
import java.util.HashMap;
import java.util.Map;

/**
 * Temporal confirmation filter before observations enter the world model.
 * Tunable at runtime via {@link VidarRuntimeConfig}.
 */
public final class VidarTemporalFilter {

    private static final class Pending {
        int frames;
        double lastX;
        double lastY;
        long lastCaptureNanos;
        boolean hasPosition;
        VidarElementObservation element;
        VidarPlateObservation plate;
    }

    private final Map<String, Pending> elementPending = new HashMap<>();
    private final Map<String, Pending> platePending = new HashMap<>();
    private volatile VidarRuntimeConfig runtimeConfig;

    public VidarTemporalFilter() {}

    public VidarTemporalFilter(VidarRuntimeConfig runtimeConfig) {
        this.runtimeConfig = runtimeConfig;
    }

    public void setRuntimeConfig(VidarRuntimeConfig runtimeConfig) {
        this.runtimeConfig = runtimeConfig;
    }

    public VidarElementObservation filterElement(VidarElementObservation obs) {
        if (obs == null) {
            return null;
        }
        if (obs.confidence >= strongConfidence()) {
            primePending(elementPending, obs.cameraName, obs.robotX, obs.robotY,
                    obs.captureTimeNanos, obs, null);
            return obs;
        }

        Pending p = elementPending.get(obs.cameraName);
        if (p == null || obs.captureTimeNanos <= p.lastCaptureNanos) {
            return null;
        }

        if (p.hasPosition) {
            double jump = Math.hypot(obs.robotX - p.lastX, obs.robotY - p.lastY);
            if (jump > maxJump()) {
                elementPending.remove(obs.cameraName);
                return null;
            }
        }

        p.frames++;
        p.lastX = obs.robotX;
        p.lastY = obs.robotY;
        p.hasPosition = true;
        p.lastCaptureNanos = obs.captureTimeNanos;
        p.element = obs;

        if (p.frames >= confirmFrames()) {
            return obs;
        }
        return null;
    }

    public VidarPlateObservation filterPlate(VidarPlateObservation obs) {
        if (obs == null) {
            return null;
        }
        if (obs.confidence >= strongConfidence()) {
            primePending(platePending, obs.cameraName, obs.robotX, obs.robotY,
                    obs.captureTimeNanos, null, obs);
            return obs;
        }

        Pending p = platePending.get(obs.cameraName);
        if (p == null || obs.captureTimeNanos <= p.lastCaptureNanos) {
            return null;
        }

        if (p.hasPosition) {
            double jump = Math.hypot(obs.robotX - p.lastX, obs.robotY - p.lastY);
            if (jump > maxJump()) {
                platePending.remove(obs.cameraName);
                return null;
            }
        }

        p.frames++;
        p.lastX = obs.robotX;
        p.lastY = obs.robotY;
        p.hasPosition = true;
        p.lastCaptureNanos = obs.captureTimeNanos;
        p.plate = obs;

        if (p.frames >= confirmFrames()) {
            return obs;
        }
        return null;
    }

    private void primePending(Map<String, Pending> map, String camera,
                              double x, double y, long captureNanos,
                              VidarElementObservation element, VidarPlateObservation plate) {
        Pending p = map.computeIfAbsent(camera, k -> new Pending());
        p.frames = Math.max(1, confirmFrames());
        p.lastX = x;
        p.lastY = y;
        p.hasPosition = true;
        p.lastCaptureNanos = captureNanos;
        p.element = element;
        p.plate = plate;
    }

    private int confirmFrames() {
        VidarRuntimeConfig cfg = runtimeConfig;
        return cfg == null ? VidarConfig.TEMPORAL_CONFIRM_FRAMES : cfg.temporalConfirmFrames();
    }

    private double strongConfidence() {
        VidarRuntimeConfig cfg = runtimeConfig;
        return cfg == null ? VidarConfig.TEMPORAL_STRONG_CONFIDENCE : cfg.temporalStrongConfidence();
    }

    private double maxJump() {
        VidarRuntimeConfig cfg = runtimeConfig;
        return cfg == null ? VidarConfig.TEMPORAL_MAX_JUMP : cfg.temporalMaxJump();
    }

    /** Clear pending confirmation state between match periods. */
    public void resetMatchState() {
        elementPending.clear();
        platePending.clear();
    }
}
