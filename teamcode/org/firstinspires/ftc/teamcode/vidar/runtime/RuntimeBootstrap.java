package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.api.VidarDiagnostics;

import java.util.function.Supplier;

/**
 * Process-scoped runtime configuration — odom/alliance suppliers and config source.
 */
public final class RuntimeBootstrap {

    public final Supplier<Pose2D> odomSupplier;
    public final Supplier<VidarAlliance> allianceSupplier;
    public final VidarDiagnostics.ConfigSource configSource;

    public RuntimeBootstrap(
            Supplier<Pose2D> odomSupplier,
            Supplier<VidarAlliance> allianceSupplier,
            VidarDiagnostics.ConfigSource configSource) {
        this.odomSupplier = odomSupplier;
        this.allianceSupplier = allianceSupplier;
        this.configSource = configSource != null
                ? configSource
                : VidarDiagnostics.ConfigSource.TEAM_ASSETS;
    }
}
