package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;

/**
 * Runtime alliance for friend/foe plate logic.
 *
 * <p>Typical use:
 * <ol>
 *   <li>Mount a color sensor on your own ROBOT SIGN (colored background, not the white digits).</li>
 *   <li>During INIT, call {@link #pollInit(Gamepad)} — gamepad overrides sensor if a button is held.</li>
 *   <li>Each loop, call {@link #pollRuntime(Gamepad)} for optional in-match toggle.</li>
 * </ol>
 *
 * <p>Init gamepad: {@link VidarConfig#ALLIANCE_GAMEPAD_RED_BUTTON}} = RED,
 * {@link VidarConfig#ALLIANCE_GAMEPAD_BLUE_BUTTON} = BLUE.
 * Runtime toggle: {@link VidarConfig#ALLIANCE_GAMEPAD_TOGGLE_BUTTON} (if enabled).
 */
public class VidarAllianceSelector {

    public enum Source {
        CONFIG_DEFAULT,
        COLOR_SENSOR,
        GAMEPAD_INIT,
        GAMEPAD_TOGGLE
    }

    private VidarAlliance alliance;
    private Source source = Source.CONFIG_DEFAULT;
    private NormalizedColorSensor colorSensor;
    private boolean lastToggle;

    public VidarAllianceSelector(HardwareMap hardwareMap) {
        this(hardwareMap, VidarConfigLoader.defaultRobot());
    }

    public VidarAllianceSelector(HardwareMap hardwareMap, VidarRobotConfig robot) {
        VidarRobotConfig active = robot != null ? robot : VidarConfigLoader.defaultRobot();
        alliance = active.defaultAlliance;
        source = Source.CONFIG_DEFAULT;
        tryInitColorSensor(hardwareMap, active);
    }

    private void tryInitColorSensor(HardwareMap hardwareMap) {
        tryInitColorSensor(hardwareMap, VidarConfigLoader.defaultRobot());
    }

    private void tryInitColorSensor(HardwareMap hardwareMap, VidarRobotConfig robot) {
        if (!robot.allianceUseColorSensor) {
            return;
        }
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, robot.allianceColorSensor);
        } catch (Exception ignored) {
            colorSensor = null;
        }
    }

    /**
     * Call during INIT (before {@code waitForStart}) until alliance is known.
     * Gamepad selection wins over color sensor when a selection button is held.
     */
    public void pollInit(Gamepad gamepad) {
        VidarAlliance fromPad = readInitGamepad(gamepad);
        if (fromPad != VidarAlliance.UNKNOWN) {
            setAlliance(fromPad, Source.GAMEPAD_INIT);
            return;
        }

        if (VidarConfig.ALLIANCE_USE_COLOR_SENSOR && colorSensor != null) {
            VidarAlliance fromSensor = readColorSensor();
            if (fromSensor != VidarAlliance.UNKNOWN) {
                setAlliance(fromSensor, Source.COLOR_SENSOR);
            }
        }
    }

    /**
     * Optional in-match toggle (e.g. if init was wrong). Disabled when
     * {@link VidarConfig#ALLIANCE_ALLOW_RUNTIME_TOGGLE} is false.
     */
    public void pollRuntime(Gamepad gamepad) {
        if (!VidarConfig.ALLIANCE_ALLOW_RUNTIME_TOGGLE || gamepad == null) {
            return;
        }

        boolean toggle = readToggleButton(gamepad);
        if (toggle && !lastToggle) {
            VidarAlliance next = alliance == VidarAlliance.RED
                    ? VidarAlliance.BLUE
                    : VidarAlliance.RED;
            setAlliance(next, Source.GAMEPAD_TOGGLE);
        }
        lastToggle = toggle;
    }

    public VidarAlliance get() {
        return alliance;
    }

    public Source getSource() {
        return source;
    }

    public boolean isKnown() {
        return alliance != VidarAlliance.UNKNOWN;
    }

    public boolean hasColorSensor() {
        return colorSensor != null;
    }

    public void setAlliance(VidarAlliance value, Source newSource) {
        if (value == null || value == VidarAlliance.UNKNOWN) {
            return;
        }
        alliance = value;
        source = newSource;
    }

    /** Re-sample own plate color (low rate OK — alliance does not change mid-match). */
    public void refreshFromColorSensor() {
        if (colorSensor == null) {
            return;
        }
        VidarAlliance fromSensor = readColorSensor();
        if (fromSensor != VidarAlliance.UNKNOWN) {
            setAlliance(fromSensor, Source.COLOR_SENSOR);
        }
    }

    private VidarAlliance readInitGamepad(Gamepad gamepad) {
        if (gamepad == null) {
            return VidarAlliance.UNKNOWN;
        }
        if (gamepadButton(gamepad, VidarConfig.ALLIANCE_GAMEPAD_RED_BUTTON)) {
            return VidarAlliance.RED;
        }
        if (gamepadButton(gamepad, VidarConfig.ALLIANCE_GAMEPAD_BLUE_BUTTON)) {
            return VidarAlliance.BLUE;
        }
        return VidarAlliance.UNKNOWN;
    }

    private boolean readToggleButton(Gamepad gamepad) {
        return gamepadButton(gamepad, VidarConfig.ALLIANCE_GAMEPAD_TOGGLE_BUTTON);
    }

    private static boolean gamepadButton(Gamepad gamepad, String name) {
        if (name == null || name.isEmpty()) {
            return false;
        }
        switch (name.toLowerCase()) {
            case "a":
                return gamepad.a;
            case "b":
                return gamepad.b;
            case "x":
                return gamepad.x;
            case "y":
                return gamepad.y;
            case "left_bumper":
                return gamepad.left_bumper;
            case "right_bumper":
                return gamepad.right_bumper;
            case "back":
                return gamepad.back;
            case "start":
                return gamepad.start;
            case "dpad_up":
                return gamepad.dpad_up;
            case "dpad_down":
                return gamepad.dpad_down;
            case "dpad_left":
                return gamepad.dpad_left;
            case "dpad_right":
                return gamepad.dpad_right;
            default:
                return false;
        }
    }

    /**
     * Point sensor at solid red/blue ROBOT SIGN background (R402), not white digits.
     */
    public VidarAlliance readColorSensor() {
        if (colorSensor == null) {
            return VidarAlliance.UNKNOWN;
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;
        float alpha = colors.alpha;
        if (alpha < VidarConfig.ALLIANCE_COLOR_MIN_ALPHA) {
            return VidarAlliance.UNKNOWN;
        }

        float max = Math.max(r, Math.max(g, b));
        if (max < VidarConfig.ALLIANCE_COLOR_MIN_BRIGHTNESS) {
            return VidarAlliance.UNKNOWN;
        }

        float redScore = r - Math.max(g, b);
        float blueScore = b - Math.max(r, g);

        if (redScore >= VidarConfig.ALLIANCE_COLOR_MIN_MARGIN
                && r >= VidarConfig.ALLIANCE_COLOR_MIN_DOMINANT) {
            return VidarAlliance.RED;
        }
        if (blueScore >= VidarConfig.ALLIANCE_COLOR_MIN_MARGIN
                && b >= VidarConfig.ALLIANCE_COLOR_MIN_DOMINANT) {
            return VidarAlliance.BLUE;
        }
        return VidarAlliance.UNKNOWN;
    }

    public String formatStatus() {
        return String.format("%s via %s", alliance.name(), source.name());
    }
}
