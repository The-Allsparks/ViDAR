package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;

import java.io.IOException;

/**
 * Team-owned ViDAR configuration — mirror of how Pedro Pathing constants live in team code.
 *
 * <p>Copy templates from the ViDAR repo into your Android project:
 * <ul>
 *   <li>{@code config/seasons/&lt;year&gt;-&lt;game&gt;.json} (e.g. {@code 2025-decode.json}, {@code 2026-biobuzz.json}) → {@code TeamCode/src/main/assets/vidar/season.json}</li>
 *   <li>{@code config/robots/example-robot.json} → {@code TeamCode/src/main/assets/vidar/robot.json}</li>
 * </ul>
 *
 * <p>Edit {@code robot.json} for your camera count, webcam names (Driver Station config),
 * mount offsets ({@code mountX}, {@code mountY}), and per-camera floor LUT calibration.
 * Edit {@code season.json} when FTC releases a new game (element color/size, plate HSV).
 */
public final class VidarTeamConfig {

    private static final String SEASON_ASSET = "vidar/season.json";
    private static final String ROBOT_ASSET = "vidar/robot.json";

    private VidarTeamConfig() {}

    public static VidarSeasonConfig loadSeason(HardwareMap hardwareMap) throws IOException {
        String json = VidarConfigLoader.readAsset(hardwareMap.appContext, SEASON_ASSET);
        return VidarConfigLoader.loadSeason(json);
    }

    public static VidarRobotConfig loadRobot(HardwareMap hardwareMap) throws IOException {
        String json = VidarConfigLoader.readAsset(hardwareMap.appContext, ROBOT_ASSET);
        return VidarConfigLoader.loadRobot(json);
    }

    /** Fallback when assets are not yet deployed (uses legacy {@code VidarConfig} constants). */
    public static VidarSeasonConfig defaultSeason() {
        return VidarConfigLoader.defaultSeason();
    }

    public static VidarRobotConfig defaultRobot() {
        return VidarConfigLoader.defaultRobot();
    }
}
