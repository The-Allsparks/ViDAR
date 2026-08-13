package org.firstinspires.ftc.teamcode.vidar.runtime;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarGlobalVisionWorker;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.TagDecodeBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagDecodeWorker;

import java.util.function.Supplier;

/**
 * Ephemeral FTC camera bundle — VisionPortals, processors, mailboxes, and global worker.
 * Created on {@code attachVision()} and destroyed on {@code detachVision()}.
 */
public final class VidarVisionAttachment {

    private final VidarVision[] cameras;
    private final int cameraCount;
    private final VidarRobotConfig robotConfig;
    private final VidarSeasonConfig season;
    private VidarGlobalVisionWorker globalWorker;

    private VidarVisionAttachment(
            VidarVision[] cameras,
            int cameraCount,
            VidarRobotConfig robotConfig,
            VidarSeasonConfig season,
            VidarGlobalVisionWorker globalWorker) {
        this.cameras = cameras;
        this.cameraCount = cameraCount;
        this.robotConfig = robotConfig;
        this.season = season;
        this.globalWorker = globalWorker;
    }

    public static VidarVisionAttachment attach(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season,
            Supplier<Pose2D> odomSupplier,
            TagDecodeBudget tagDecodeBudget,
            VidarTagDecodeWorker tagDecodeWorker,
            VidarResourceBudget resourceBudget) {
        VidarSeasonConfig activeSeason = season != null ? season : VidarConfigLoader.defaultSeason();
        VidarRobotConfig activeRobot = robot != null ? robot : VidarConfigLoader.defaultRobot();
        int count = activeRobot.activeCameraCount();
        VidarVision[] cameras = new VidarVision[count];
        VidarGlobalVisionWorker worker = null;

        try {
            for (int i = 0; i < count; i++) {
                try {
                    VidarCameraMount mount = activeRobot.cameraMount(i);
                    cameras[i] = new VidarVision(
                            hardwareMap,
                            mount.webcamName,
                            mount.profile,
                            odomSupplier,
                            mount.webcamName,
                            null,
                            activeSeason,
                            resourceBudget,
                            count,
                            i,
                            tagDecodeBudget,
                            tagDecodeWorker);
                } catch (RuntimeException ex) {
                    cameras[i] = null;
                }
            }
            if (VidarConfig.useGlobalVisionWorker(count)) {
                worker = new VidarGlobalVisionWorker(cameras);
                worker.start();
            }
            return new VidarVisionAttachment(cameras, count, activeRobot, activeSeason, worker);
        } catch (RuntimeException ex) {
            if (worker != null) {
                worker.shutdownAndJoin();
            }
            for (VidarVision camera : cameras) {
                if (camera != null) {
                    camera.close();
                }
            }
            throw ex;
        }
    }

    public VidarVision[] cameras() {
        return cameras;
    }

    public int cameraCount() {
        return cameraCount;
    }

    public VidarRobotConfig robotConfig() {
        return robotConfig;
    }

    public VidarSeasonConfig season() {
        return season;
    }

    public VidarGlobalVisionWorker globalVisionWorker() {
        return globalWorker;
    }

    public VidarVision camera(int index) {
        if (index < 0 || index >= cameraCount) {
            return null;
        }
        return cameras[index];
    }

    public void close() {
        if (globalWorker != null) {
            globalWorker.shutdownAndJoin();
            globalWorker = null;
        }
        for (VidarVision camera : cameras) {
            if (camera != null) {
                camera.close();
            }
        }
    }
}
