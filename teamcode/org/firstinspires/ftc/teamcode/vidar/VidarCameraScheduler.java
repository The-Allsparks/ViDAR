package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Camera scheduling with processor disabling before stream shutdown.
 * States: PRIMARY, SECONDARY, IDLE, DEEP_IDLE (stream stop after delay).
 */
public final class VidarCameraScheduler {

    public enum State {
        /** Full processing: ball + plate + tag schedule. */
        PRIMARY,
        /** Lightweight ball only; expensive processors disabled. */
        SECONDARY,
        /** Streaming continues; all vision processors disabled. */
        IDLE,
        /** Stream may stop after {@link VidarConfig#DEEP_IDLE_DELAY_MS}. */
        DEEP_IDLE
    }

    private State state = State.PRIMARY;
    private State pendingState;
    private long pendingSinceMs;
    private long idleSinceMs;
    private long stateEnteredMs = System.currentTimeMillis();
    private boolean streaming = true;
    private boolean ballEnabled = true;
    private boolean plateEnabled = true;
    private boolean tagEnabled = true;
    private boolean lightweightBall = false;
    private long lastStreamRestartMs;

    public State tierForCamera(double cameraBearingDeg, double travelHeadingDeg, double speedInPerSec) {
        if (!VidarConfig.DIRECTION_SCHEDULER_ENABLED || speedInPerSec < VidarConfig.DIRECTION_MIN_SPEED_IN_PER_SEC) {
            return State.PRIMARY;
        }
        double align = angleDiff(cameraBearingDeg, travelHeadingDeg);
        if (align <= VidarConfig.DIRECTION_PRIMARY_CONE_DEG) {
            return State.PRIMARY;
        }
        if (align <= VidarConfig.DIRECTION_SECONDARY_CONE_DEG) {
            return State.SECONDARY;
        }
        return State.IDLE;
    }

    public void apply(VisionPortal portal,
                      VidarBallProcessor ball,
                      VidarPlateProcessor plate,
                      VidarAdaptiveTagProcessor tag,
                      State requested,
                      VidarMetrics metrics) {
        if (portal == null) {
            return;
        }

        long now = System.currentTimeMillis();
        State target = debounceState(requested, now);
        if (target != state) {
            state = target;
            stateEnteredMs = now;
            if (metrics != null) {
                metrics.setCameraState(state);
            }
        }

        configureProcessorsForState(target);

        portal.setProcessorEnabled(ball, ballEnabled);
        portal.setProcessorEnabled(plate, plateEnabled);
        if (tag != null) {
            portal.setProcessorEnabled(tag, tagEnabled);
        }

        if (target == State.DEEP_IDLE) {
            if (idleSinceMs == 0) {
                idleSinceMs = now;
            }
            if (streaming && now - idleSinceMs >= VidarConfig.DEEP_IDLE_DELAY_MS) {
                long t0 = System.nanoTime();
                portal.stopStreaming();
                streaming = false;
                lastStreamRestartMs = (System.nanoTime() - t0) / 1_000_000L;
                if (metrics != null) {
                    metrics.recordStreamTransition(lastStreamRestartMs);
                }
            }
        } else {
            idleSinceMs = 0;
            if (!streaming) {
                long t0 = System.nanoTime();
                portal.resumeStreaming();
                streaming = true;
                lastStreamRestartMs = (System.nanoTime() - t0) / 1_000_000L;
                if (metrics != null) {
                    metrics.recordStreamTransition(lastStreamRestartMs);
                }
            }
        }

        if (metrics != null) {
            int active = (ballEnabled ? 1 : 0) + (plateEnabled ? 1 : 0) + (tagEnabled ? 1 : 0);
            metrics.setActiveProcessors(active);
            metrics.setHealth(streaming ? VidarMetrics.CameraHealth.PROCESSING
                    : VidarMetrics.CameraHealth.STREAMING);
        }
    }

    private State debounceState(State requested, long nowMs) {
        if (requested == state) {
            pendingState = null;
            return state;
        }
        if (pendingState != requested) {
            pendingState = requested;
            pendingSinceMs = nowMs;
            return state;
        }
        if (nowMs - pendingSinceMs >= VidarConfig.STATE_TRANSITION_DEBOUNCE_MS) {
            pendingState = null;
            return requested;
        }
        return state;
    }

    private void configureProcessorsForState(State s) {
        switch (s) {
            case PRIMARY:
                ballEnabled = true;
                plateEnabled = true;
                tagEnabled = true;
                lightweightBall = false;
                break;
            case SECONDARY:
                ballEnabled = true;
                plateEnabled = false;
                tagEnabled = false;
                lightweightBall = true;
                break;
            case IDLE:
                ballEnabled = false;
                plateEnabled = false;
                tagEnabled = false;
                lightweightBall = false;
                break;
            case DEEP_IDLE:
                ballEnabled = false;
                plateEnabled = false;
                tagEnabled = false;
                lightweightBall = false;
                break;
        }
    }

    /** @deprecated Use {@link #apply} with {@link State}. */
    @Deprecated
    public void apply(VisionPortal portal,
                      VidarBallProcessor ball,
                      VidarPlateProcessor plate,
                      VidarAdaptiveTagProcessor tag,
                      Tier tier) {
        State state = tier == Tier.PRIMARY ? State.PRIMARY
                : tier == Tier.SECONDARY ? State.SECONDARY : State.DEEP_IDLE;
        apply(portal, ball, plate, tag, state, null);
    }

    /** @deprecated Use {@link State}. */
    @Deprecated
    public enum Tier {
        PRIMARY,
        SECONDARY,
        IDLE
    }

    public State currentState() {
        return state;
    }

    /** @deprecated Use {@link #currentState()}. */
    @Deprecated
    public Tier currentTier() {
        switch (state) {
            case PRIMARY: return Tier.PRIMARY;
            case SECONDARY: return Tier.SECONDARY;
            default: return Tier.IDLE;
        }
    }

    public boolean isStreaming() {
        return streaming;
    }

    public boolean isLightweightBall() {
        return lightweightBall;
    }

    public long timeInStateMs() {
        return System.currentTimeMillis() - stateEnteredMs;
    }

    public long lastStreamRestartDurationMs() {
        return lastStreamRestartMs;
    }

    private static double angleDiff(double a, double b) {
        double d = a - b;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return Math.abs(d);
    }
}
