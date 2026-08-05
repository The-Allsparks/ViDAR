package org.firstinspires.ftc.teamcode.vidar;

import java.util.HashMap;
import java.util.Map;

/**
 * Temporal confirmation filter before observations enter the world model.
 * Uses capture timestamps; rejects stale frames and impossible jumps.
 */
public final class VidarTemporalFilter {

    private static final class Pending {
        int frames;
        double lastX;
        double lastY;
        long lastCaptureNanos;
        String cameraName;
        VidarBallObservation ball;
        VidarPlateObservation plate;
    }

    private final Map<String, Pending> ballPending = new HashMap<>();
    private final Map<String, Pending> platePending = new HashMap<>();

    public VidarBallObservation filterBall(VidarBallObservation obs) {
        if (obs == null) {
            return null;
        }
        if (obs.confidence >= VidarConfig.TEMPORAL_STRONG_CONFIDENCE) {
            updatePending(ballPending, obs.cameraName, obs.robotXIn, obs.robotYIn,
                    obs.captureTimeNanos, obs, null);
            return obs;
        }

        Pending p = ballPending.get(obs.cameraName);
        if (p == null || obs.captureTimeNanos <= p.lastCaptureNanos) {
            return null;
        }

        if (p.lastX != 0 || p.lastY != 0) {
            double jump = Math.hypot(obs.robotXIn - p.lastX, obs.robotYIn - p.lastY);
            if (jump > VidarConfig.TEMPORAL_MAX_JUMP_IN) {
                ballPending.remove(obs.cameraName);
                return null;
            }
        }

        p.frames++;
        p.lastX = obs.robotXIn;
        p.lastY = obs.robotYIn;
        p.lastCaptureNanos = obs.captureTimeNanos;
        p.ball = obs;

        if (p.frames >= VidarConfig.TEMPORAL_CONFIRM_FRAMES) {
            return obs;
        }
        return null;
    }

    public VidarPlateObservation filterPlate(VidarPlateObservation obs) {
        if (obs == null) {
            return null;
        }
        if (obs.confidence >= VidarConfig.TEMPORAL_STRONG_CONFIDENCE) {
            updatePending(platePending, obs.cameraName, obs.robotXIn, obs.robotYIn,
                    obs.captureTimeNanos, null, obs);
            return obs;
        }

        Pending p = platePending.get(obs.cameraName);
        if (p == null || obs.captureTimeNanos <= p.lastCaptureNanos) {
            return null;
        }

        if (p.lastX != 0 || p.lastY != 0) {
            double jump = Math.hypot(obs.robotXIn - p.lastX, obs.robotYIn - p.lastY);
            if (jump > VidarConfig.TEMPORAL_MAX_JUMP_IN) {
                platePending.remove(obs.cameraName);
                return null;
            }
        }

        p.frames++;
        p.lastX = obs.robotXIn;
        p.lastY = obs.robotYIn;
        p.lastCaptureNanos = obs.captureTimeNanos;
        p.plate = obs;

        if (p.frames >= VidarConfig.TEMPORAL_CONFIRM_FRAMES) {
            return obs;
        }
        return null;
    }

    private static void updatePending(Map<String, Pending> map, String camera,
                                      double x, double y, long captureNanos,
                                      VidarBallObservation ball, VidarPlateObservation plate) {
        Pending p = map.computeIfAbsent(camera, k -> new Pending());
        p.frames = VidarConfig.TEMPORAL_CONFIRM_FRAMES;
        p.lastX = x;
        p.lastY = y;
        p.lastCaptureNanos = captureNanos;
        p.cameraName = camera;
        p.ball = ball;
        p.plate = plate;
    }
}
