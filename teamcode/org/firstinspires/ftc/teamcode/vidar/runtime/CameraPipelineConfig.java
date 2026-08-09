package org.firstinspires.ftc.teamcode.vidar.runtime;

import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.TagDecodeBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagDecodeWorker;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;

/**
 * Single constructor input for {@link VidarVision} — replaces the eight overload chain.
 */
public final class CameraPipelineConfig {

    public final String cameraName;
    public final VidarCameraProfile profile;
    public final Supplier<Pose2D> odomSupplier;
    public final String portalLabel;
    public final Supplier<Pose2D> fieldPosePriorSupplier;
    public final VidarSeasonConfig season;
    public final VidarResourceBudget resourceBudget;
    public final int robotCameraCount;
    public final int cameraIndex;
    public final TagDecodeBudget decodeBudget;
    public final VidarTagDecodeWorker decodeWorker;

    public CameraPipelineConfig(
            String cameraName,
            VidarCameraProfile profile,
            Supplier<Pose2D> odomSupplier,
            String portalLabel,
            Supplier<Pose2D> fieldPosePriorSupplier,
            VidarSeasonConfig season,
            VidarResourceBudget resourceBudget,
            int robotCameraCount,
            int cameraIndex,
            TagDecodeBudget decodeBudget,
            VidarTagDecodeWorker decodeWorker) {
        this.cameraName = cameraName;
        this.profile = profile;
        this.odomSupplier = odomSupplier;
        this.portalLabel = portalLabel == null || portalLabel.isEmpty() ? cameraName : portalLabel;
        this.fieldPosePriorSupplier = fieldPosePriorSupplier;
        this.season = season;
        this.resourceBudget = resourceBudget;
        this.robotCameraCount = robotCameraCount;
        this.cameraIndex = cameraIndex;
        this.decodeBudget = decodeBudget;
        this.decodeWorker = decodeWorker;
    }

    public static CameraPipelineConfig singleCamera(String cameraName) {
        return new CameraPipelineConfig(
                cameraName,
                org.firstinspires.ftc.teamcode.vidar.VidarConfig.cameraProfile(),
                null,
                cameraName,
                null,
                null,
                null,
                1,
                0,
                null,
                null);
    }
}
