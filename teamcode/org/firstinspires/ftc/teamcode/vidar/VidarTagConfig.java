package org.firstinspires.ftc.teamcode.vidar;



import android.util.Size;



/**

 * Adaptive AprilTag sampling — scout on plate frames, decode at most once per

 * {@link #DECODE_INTERVAL_MS} when {@link VidarTagCropPlanner#worthDecode} passes.

 */

public final class VidarTagConfig {



    private VidarTagConfig() {}



    public static final boolean ENABLED = true;



    /** All cameras capture at 640×480 (480p). */

    public static final Size CAPTURE_RESOLUTION = new Size(640, 480);



    /** Minimum time between full decode passes (global across cameras). */

    public static final long DECODE_INTERVAL_MS = 1000;



    /** Scout downscale width for top-half search (lower = faster). */

    public static final int SCOUT_WIDTH = 320;



    /** Approximate horizontal FOV for bearing from scout cx (C920-class). */

    public static final double HORIZONTAL_FOV_DEG = 70.0;



    /** Normalized cx below this → left 50% decode band. */

    public static final double BAND_LEFT_MAX = 0.33;



    /** Normalized cx above this → right 50% decode band. */

    public static final double BAND_RIGHT_MIN = 0.67;



    /** Ignore scout quads smaller than this in scout pixels. */

    public static final double SCOUT_MIN_WIDTH_PX = 8;



    /** Only decode when scout width maps to at least this many full-frame px. */

    public static final double DECODE_MIN_TAG_WIDTH_PX = 28;



    /** Pose gate: expected tag bearing must be within this many degrees (0 = disabled). */

    public static final double POSE_GATE_DEG = 30;



    /** FTC 36h11 black square size (4 in). */

    public static final double TAG_SIZE_IN = 4.0;



    /** Lens intrinsics @ 640×480 (C920 defaults — tune or calibrate on your webcam). */

    public static final double LENS_FX = 622.001;

    public static final double LENS_FY = 622.001;

    public static final double LENS_CX = 319.803;

    public static final double LENS_CY = 241.251;



    /** Filter to this tag id, or -1 for any. */

    public static final int DESIRED_TAG_ID = -1;



    /** AprilTag decimation inside crop decode (1 = full crop pixels). */

    public static final int DECIMATION_MIN = 1;



    /** AprilTag decimation when tag is large in frame. */

    public static final int DECIMATION_MAX = 3;



    // --- Scout observations (non-localizing — decode scheduling only) ---

    // --- Localization pose gates ---
    public static final double MAX_TRANSLATION_RESIDUAL_IN = 18.0;
    public static final double MAX_HEADING_RESIDUAL_DEG = 25.0;
    public static final long MAX_OBSERVATION_AGE_MS = 500;
    public static final double MAX_TAG_DISTANCE_IN = 120.0;
    public static final int MIN_DECISION_MARGIN = 0;
    public static final double MAX_CORRECTION_MAGNITUDE_IN = 12.0;
    public static final long CORRECTION_COOLDOWN_MS = 750;

    /** High decimation while scouting; reduced after probable tag. */
    public static final int SCOUT_DECIMATION = 3;
    public static final int DECODE_DECIMATION_AFTER_SCOUT = 1;

    /** Preferred camera index for AprilTag (-1 = any). */
    public static final int PREFERRED_TAG_CAMERA_INDEX = 0;

    /** Global decode budget per interval. */
    public static final int MAX_DECODES_PER_INTERVAL = 1;
}

