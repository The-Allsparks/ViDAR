package org.firstinspires.ftc.teamcode.vidar.api;

import org.firstinspires.ftc.teamcode.vidar.runtime.VidarMetrics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Init-time and runtime diagnostic snapshot for student OpModes.
 *
 * <p>Populated by {@link org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntime} and exposed via
 * {@link org.firstinspires.ftc.teamcode.vidar.VidarSpatial#diagnostics()}.
 */
public final class VidarDiagnostics {

    public enum ConfigSource {
        TEAM_ASSETS,
        BUNDLED_DEFAULTS
    }

    public final ConfigSource configSource;
    public final int cameraCount;
    public final int connectedCameras;
    public final List<String> warnings;
    public final VidarMetrics.CameraHealth[] cameraHealth;
    public final String observationWorkerLastError;
    public final int observationWorkerConsecutiveFailures;
    public final int observationWorkerTotalFailures;

    public VidarDiagnostics(
            ConfigSource configSource,
            int cameraCount,
            int connectedCameras,
            List<String> warnings,
            VidarMetrics.CameraHealth[] cameraHealth) {
        this(configSource, cameraCount, connectedCameras, warnings, cameraHealth, "", 0, 0);
    }

    public VidarDiagnostics(
            ConfigSource configSource,
            int cameraCount,
            int connectedCameras,
            List<String> warnings,
            VidarMetrics.CameraHealth[] cameraHealth,
            String observationWorkerLastError,
            int observationWorkerConsecutiveFailures,
            int observationWorkerTotalFailures) {
        this.configSource = configSource;
        this.cameraCount = cameraCount;
        this.connectedCameras = connectedCameras;
        this.warnings = warnings == null
                ? Collections.emptyList()
                : Collections.unmodifiableList(new ArrayList<>(warnings));
        this.cameraHealth = cameraHealth == null ? new VidarMetrics.CameraHealth[0] : cameraHealth;
        this.observationWorkerLastError =
                observationWorkerLastError == null ? "" : observationWorkerLastError;
        this.observationWorkerConsecutiveFailures = observationWorkerConsecutiveFailures;
        this.observationWorkerTotalFailures = observationWorkerTotalFailures;
    }

    public static VidarDiagnostics empty() {
        return new VidarDiagnostics(ConfigSource.BUNDLED_DEFAULTS, 0, 0, List.of(), new VidarMetrics.CameraHealth[0]);
    }

    public boolean hasWarnings() {
        return !warnings.isEmpty();
    }

    public String formatSummary() {
        StringBuilder sb = new StringBuilder();
        sb.append("config=").append(configSource.name());
        sb.append(" cameras=").append(connectedCameras).append('/').append(cameraCount);
        if (!warnings.isEmpty()) {
            sb.append(" warnings=").append(warnings.size());
        }
        if (observationWorkerTotalFailures > 0) {
            sb.append(" workerFailures=").append(observationWorkerTotalFailures);
        }
        return sb.toString();
    }
}
