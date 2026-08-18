package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraMount;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagConfig;
import android.util.Size;

/**
 * ViDAR tuning constants — change these in one place when calibrating on the field.
 *
 * <b>Preferred:</b> teams load JSON via {@link org.firstinspires.ftc.teamcode.vidar.config.VidarConfigLoader}
 * and pass configs to {@link org.firstinspires.ftc.teamcode.vidar.runtime.VidarRuntime} via
 * {@link org.firstinspires.ftc.teamcode.vidar.VidarSpatial}. Effective tuning is merged in
 * {@link org.firstinspires.ftc.teamcode.vidar.config.VidarSettings}. See
 * {@link org.firstinspires.ftc.teamcode.VidarTeamConfig}.
 *
 * <p>Season tuning (elements, plates, fusion thresholds, AprilTags) lives in bundled
 * {@code default-season.json} / team {@code season.json}. Robot layout lives in
 * {@code default-robot.json} / team {@code robot.json}. Constants below are
 * <b>hardware-only fallbacks</b> (camera names, USB count, world-model radii not yet in JSON)
 * and backward-compatible defaults when assets are missing.
 *
 * Robot config (Driver Station → Configure Robot) must include a USB webcam named
 * {@link #CAMERA_NAME} on the Control Hub.
 */
public final class VidarConfig {

    private VidarConfig() {}

    /** Must match webcam names in Driver Station robot configuration. */
    public static final String[] CAMERA_NAMES = {
            "Webcam 1",
            "Webcam 2",
            "Webcam 3",
            "Webcam 4"
    };

    /** Active cameras: 1–4. Names and profiles use indices 0 … count−1. */
    public static final int CAMERA_COUNT = 1;

    /** Fallback when gamepad and color sensor do not resolve alliance. */
    public static final VidarAlliance DEFAULT_ALLIANCE = VidarAlliance.RED;

    // --- Alliance selection (friend/foe plates) ---
    /** Configure REV Color Sensor V3 on your own ROBOT SIGN background. */
    public static final String ALLIANCE_COLOR_SENSOR = "alliance_color";
    public static final boolean ALLIANCE_USE_COLOR_SENSOR = true;
    public static final boolean ALLIANCE_ALLOW_RUNTIME_TOGGLE = true;
    /** Gamepad button names — see {@link VidarAllianceSelector}. */
    public static final String ALLIANCE_GAMEPAD_RED_BUTTON = "y";
    public static final String ALLIANCE_GAMEPAD_BLUE_BUTTON = "b";
    public static final String ALLIANCE_GAMEPAD_TOGGLE_BUTTON = "back";
    public static final float ALLIANCE_COLOR_MIN_ALPHA = 0.1f;
    public static final float ALLIANCE_COLOR_MIN_BRIGHTNESS = 0.18f;
    public static final float ALLIANCE_COLOR_MIN_DOMINANT = 0.28f;
    public static final float ALLIANCE_COLOR_MIN_MARGIN = 0.08f;

    /** Legacy single-camera name — same as {@link #CAMERA_NAMES}[0]. */
    public static final String CAMERA_NAME = CAMERA_NAMES[0];

    /** All USB webcams stream at 640×480 for CPU/USB budget. */
    public static final Size PORTAL_RESOLUTION = new Size(640, 480);

    /** Downscale bottom-half ROI before element/plate OpenCV (0.5 → half size). */
    public static final double PROCESS_ROI_SCALE = 0.5;

    /** Lower ROI scale applied when {@link VidarResourceBudget} requests reduced resolution. */
    public static final double DEGRADED_PROCESS_ROI_SCALE = 0.35;

    /** Intermediate ROI scale under moderate frame-age pressure. */
    public static final double MEDIUM_PROCESS_ROI_SCALE = 0.42;

    /** Default ranked detections per camera when not overridden. */
    public static final int DEFAULT_MAX_RANKED_ELEMENTS = 5;

    /** Hard cap for per-camera or fusion ranked lists. */
    public static final int MAX_RANKED_ELEMENTS_CAP = 32;

    /** Max ranked elements retained after multi-camera fusion. */
    public static final int FUSION_MAX_RANKED_ELEMENTS = 16;

    /** Disable RC LiveView during matches to save CPU. */
    public static final boolean LIVE_VIEW_ENABLED = false;

    /** Use MJPEG when more than one camera is active (USB hub bandwidth). */
    public static final boolean MJPEG_MULTI_CAMERA = true;

    /** Serialize heavy OpenCV across cameras on a background worker (round-robin). */
    public static final boolean GLOBAL_VISION_WORKER_ENABLED = true;

    /** Minimum active cameras before the global worker replaces synchronous processing. */
    public static final int GLOBAL_VISION_WORKER_MIN_CAMERAS = 1;

    /** When true, resource budget may auto-idle rear-facing cameras under extreme CPU load. */
    public static final boolean RESOURCE_BUDGET_AUTO_IDLE_REAR = false;

    /** Run full AprilTag decode on a background thread so scan round-robin is not blocked. */
    public static final boolean ASYNC_TAG_DECODE_ENABLED = true;

    public static boolean useGlobalVisionWorker(int activeCameraCount) {
        return GLOBAL_VISION_WORKER_ENABLED
                && activeCameraCount >= GLOBAL_VISION_WORKER_MIN_CAMERAS;
    }

    // --- Direction-based camera tiers (opt-in only; never auto-idle) ---
    /** When true, {@link VidarVision#applyDirectionTier} may adjust PRIMARY/SECONDARY by travel heading. */
    public static final boolean DIRECTION_SCHEDULER_ENABLED = false;

    public static final double DIRECTION_MIN_SPEED_IN_PER_SEC = 4.0;

    public static final double DIRECTION_PRIMARY_CONE_DEG = 50.0;

    public static final double DIRECTION_SECONDARY_CONE_DEG = 120.0;

    /** Lower resolution = faster processing. 320x240 is a good teaching default. */
    public static final Size CAMERA_RESOLUTION = new Size(320, 240);

    /**
     * Vertical crop on the camera frame before detection (pixels at {@link #CAMERA_RESOLUTION} height).
     * Sim uses 640×480 — divide these values by 2 when copying from sim tuning.
     */
    public static final int VERTICAL_CROP_OFFSET = 120;
    public static final int VERTICAL_CROP_HEIGHT = 120;

    /** Process scale factor applied after crop (0.5 → half width/height). */
    public static final double DOWNSCALE_RATIO = 0.5;

    /** Ignore blobs smaller than this (pixels). */
    public static final double MIN_BLOB_AREA = 25;

    /** Minimum Hough circle area (px²) in the process crop. */
    public static final double MIN_ELEMENT_AREA_PX = 45;

    // --- Element detection pipeline (legacy defaults when JSON not loaded) ---
    public static final VidarElementDetectorType DEFAULT_ELEMENT_DETECTOR =
            VidarElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH;

    /** Default element HSV range (calibrate on field). */
    public static final int DEFAULT_ELEMENT_HSV_H_MIN = 0;
    public static final int DEFAULT_ELEMENT_HSV_H_MAX = 179;
    public static final int DEFAULT_ELEMENT_HSV_S_MIN = 0;
    public static final int DEFAULT_ELEMENT_HSV_S_MAX = 85;
    public static final int DEFAULT_ELEMENT_HSV_V_MIN = 55;
    public static final int DEFAULT_ELEMENT_HSV_V_MAX = 255;

    public static final double DEFAULT_ELEMENT_MIN_AREA_PX = 45;
    public static final double DEFAULT_ELEMENT_MAX_AREA_PX = 12000;
    public static final double DEFAULT_ELEMENT_MIN_WIDTH_PX = 8;
    public static final double DEFAULT_ELEMENT_MAX_WIDTH_PX = 80;
    public static final double DEFAULT_ELEMENT_MIN_HEIGHT_PX = 8;
    public static final double DEFAULT_ELEMENT_MAX_HEIGHT_PX = 80;
    public static final double DEFAULT_ELEMENT_MAX_ASPECT_RATIO = 2.0;
    public static final double DEFAULT_ELEMENT_MIN_CIRCULARITY = 0.55;
    public static final double DEFAULT_ELEMENT_MIN_FILL_RATIO = 0.55;
    public static final double DEFAULT_ELEMENT_MIN_INTERIOR_SCORE = 0.12;
    public static final int DEFAULT_ELEMENT_INTERIOR_BRIGHT = 90;
    public static final int DEFAULT_ELEMENT_INTERIOR_SPREAD = 60;
    public static final int DEFAULT_ELEMENT_HOLE_DARK_MAX = 45;

    public static final int DEFAULT_ELEMENT_MORPH_ERODE_PASSES = 0;
    public static final int DEFAULT_ELEMENT_MORPH_DILATE_PASSES = 0;
    public static final int DEFAULT_ELEMENT_MORPH_OPEN_PASSES = 1;
    public static final int DEFAULT_ELEMENT_MORPH_CLOSE_PASSES = 2;

    // --- Camera scheduling ---
    public static final long DEEP_IDLE_DELAY_MS = 2500;
    public static final long STATE_TRANSITION_DEBOUNCE_MS = 500;

    // --- Temporal tracking ---
    /** Single-frame confirmation — observations publish on first qualifying frame. */
    public static final int TEMPORAL_CONFIRM_FRAMES = 1;
    public static final double TEMPORAL_STRONG_CONFIDENCE = 0.85;
    public static final double TEMPORAL_MAX_JUMP = 24.0;

    // --- Resource degradation ---
    public static final boolean RESOURCE_BUDGET_ENABLED = true;
    public static final double DEGRADATION_LOOP_BUDGET_MS = 45.0;

    // --- Default element geometry (calibrate on field) ---
    /** Primary ranging diameter in the active distance unit. */
    public static final double DEFAULT_ELEMENT_DIAMETER = 5.0;

    /** Reject fused observations below this confidence (0–1). */
    public static final double MIN_ELEMENT_CONFIDENCE = 0.35;

    /** Size vs floor range disagreement above this ratio lowers confidence sharply. */
    public static final double MAX_RANGE_MISMATCH_RATIO = 0.28;

    /** Which side camera profile to use when {@link #CAMERA_COUNT} is 1 (0=front … 3=left). */
    public static final int ACTIVE_CAMERA_INDEX = 0;

    public static int activeCameraCount() {
        return Math.max(1, Math.min(4, CAMERA_COUNT));
    }

    public static VidarCameraMount cameraMount(int index) {
        int i = Math.max(0, Math.min(3, index));
        return new VidarCameraMount(CAMERA_NAMES[i], VidarCameraProfile.forIndex(i));
    }

    public static VidarCameraProfile cameraProfile() {
        return VidarCameraProfile.forIndex(ACTIVE_CAMERA_INDEX);
    }

    // --- Hough circle (matches sim/vidar-tuning.json) ---
    public static final double HOUGH_DP = 1.2;
    public static final double HOUGH_MIN_DIST = 24;
    public static final double HOUGH_PARAM1 = 80;
    public static final double HOUGH_PARAM2 = 11;
    public static final double HOUGH_MIN_VOTES_SCALE = 0.45;
    public static final int HOUGH_MIN_RADIUS = 8;
    public static final int HOUGH_MAX_RADIUS = 36;
    public static final double HOUGH_MIN_INTERIOR = 0.14;
    public static final int HOUGH_INTERIOR_BRIGHT = 90;
    public static final int HOUGH_INTERIOR_SPREAD = 60;

    /** Ignore blobs larger than this (pixels). Lower if glare fills the frame. */
    public static final double MAX_BLOB_AREA = 20000;

    /**
     * Search the full camera frame (not a center crop).
     */
    public static final boolean USE_CENTER_ROI = false;

    // --- Alliance plate detection (rotated rect + white digits) ---
    public static Size portalCameraResolution() {
        if (VidarTagConfig.ENABLED) {
            return VidarTagConfig.CAPTURE_RESOLUTION;
        }
        return PORTAL_RESOLUTION;
    }

    public static org.firstinspires.ftc.vision.VisionPortal.StreamFormat portalStreamFormat() {
        return portalStreamFormat(activeCameraCount());
    }

    /** Uses robot JSON camera count when building multi-camera portals. */
    public static org.firstinspires.ftc.vision.VisionPortal.StreamFormat portalStreamFormat(int activeCameraCount) {
        if (MJPEG_MULTI_CAMERA && activeCameraCount > 1) {
            return org.firstinspires.ftc.vision.VisionPortal.StreamFormat.MJPEG;
        }
        return org.firstinspires.ftc.vision.VisionPortal.StreamFormat.YUY2;
    }

    /** Bottom-half ROI for plate locators at the portal resolution. */
    public static org.firstinspires.ftc.vision.opencv.ImageRegion elementPlateRoi() {
        Size s = portalCameraResolution();
        int half = s.getHeight() / 2;
        return org.firstinspires.ftc.vision.opencv.ImageRegion.asImageCoordinates(
                0, half, s.getWidth(), s.getHeight());
    }

    // --- Alliance plate detection (rotated rect + white digits) ---
    public static final double PLATE_MIN_AREA_PX = 120;
    public static final double PLATE_MAX_AREA_PX = 12000;
    public static final double PLATE_MIN_ASPECT = 1.15;
    public static final double PLATE_MAX_ASPECT = 4.5;
    public static final double PLATE_MIN_RECTANGULARITY = 0.45;
    public static final double PLATE_MIN_WHITE_RATIO = 0.12;
    public static final int PLATE_WHITE_SAMPLE_GRID = 5;
    public static final int PLATE_WHITE_BRIGHT_MIN = 175;
    public static final int PLATE_WHITE_SPREAD_MAX = 55;
    public static final int PLATE_S_MIN = 80;
    public static final int PLATE_V_MIN = 60;
    public static final int PLATE_RED_H_MIN = 0;
    public static final int PLATE_RED_H_MAX = 12;
    public static final int PLATE_RED_WRAP_H_MIN = 168;
    public static final int PLATE_BLUE_H_MIN = 95;
    public static final int PLATE_BLUE_H_MAX = 135;
    public static final double MIN_PLATE_CONFIDENCE = 0.35;

    // --- Short-term world model (VidarWorldModel) ---
    /** When false, skip motion correction and track memory (live vision only). */
    public static final boolean WORLD_MOTION_TRACKING_ENABLED = true;
    /**
     * Associate/coast on new observation capture times, not 1 ms worker ticks.
     * Set false to restore last-blob re-association every tick (rollback).
     */
    public static final boolean WORLD_ASSOCIATE_ON_NEW_FRAME_ONLY = true;
    public static final double WORLD_ELEMENT_TTL_SEC = 2.5;
    public static final double WORLD_FOE_TTL_SEC = 3.0;
    public static final double WORLD_ALLY_TTL_SEC = 4.0;
    /** @deprecated use {@link #WORLD_TRACK_GATE_RADIUS_IN} for association */
    public static final double WORLD_MERGE_RADIUS_IN = 8.0;
    public static final double WORLD_BLOCK_RANGE_IN = 36.0;
    public static final double WORLD_BLOCK_CONE_DEG = 35.0;

    /** Association gate — detection must fall within this of predicted robot position (inches). */
    public static final double WORLD_TRACK_GATE_RADIUS_IN = 12.0;
    public static final double WORLD_TRACK_GATE_RADIUS_FOE_IN = 18.0;
    /** Position filter on match: new = alpha * det + (1-alpha) * pred. */
    public static final double WORLD_TRACK_POS_ALPHA = 0.7;
    public static final double WORLD_TRACK_VEL_ALPHA = 0.35;
    public static final double WORLD_TRACK_MIN_DT_SEC = 0.05;
    public static final double WORLD_TRACK_MAX_DT_SEC = 0.5;
    public static final double WORLD_TRACK_STATIC_SPEED_IN_PER_SEC = 2.0;
    public static final int WORLD_TRACK_STATIC_FRAMES = 5;
    public static final double WORLD_TRACK_MOVING_SPEED_IN_PER_SEC = 4.0;
    public static final int WORLD_TRACK_MAX_MISS_FRAMES = 8;

    // --- Offensive lane helper (VidarOffensiveLaneAnalysis) ---
    /** Max robot-frame range for foe lane counts (+X forward). */
    public static final double OFFENSIVE_LANE_MAX_RANGE_IN = 48.0;
    /** Half-width of forward cone; split into three equal lanes. */
    public static final double OFFENSIVE_LANE_CONE_HALF_DEG = 35.0;

    // --- Element density map (VidarElementDensityMap) ---
    /** Grid cell size in robot frame (+X forward, +Y left). */
    public static final double DENSITY_CELL_SIZE_IN = 6.0;
    public static final double DENSITY_FORWARD_MAX_IN = 72.0;
    public static final double DENSITY_LATERAL_MAX_IN = 48.0;
    /** Gaussian splat sigma when accumulating element confidence into the grid. */
    public static final double DENSITY_SPLAT_SIGMA_IN = 8.0;

    /** Pixel size of cropped+downscaled detection frame on the Control Hub. */
    public static Size processingResolution() {
        int cropH = Math.min(VERTICAL_CROP_HEIGHT, CAMERA_RESOLUTION.getHeight() - VERTICAL_CROP_OFFSET);
        int w = Math.max(80, (int) Math.round(CAMERA_RESOLUTION.getWidth() * DOWNSCALE_RATIO));
        int h = Math.max(60, (int) Math.round(cropH * DOWNSCALE_RATIO));
        return new Size(w, h);
    }

    /** Full camera frame at {@link #DOWNSCALE_RATIO} — crop ROI is applied on this image. */
    public static Size downscaledCameraResolution() {
        int w = Math.max(80, (int) Math.round(CAMERA_RESOLUTION.getWidth() * DOWNSCALE_RATIO));
        int h = Math.max(60, (int) Math.round(CAMERA_RESOLUTION.getHeight() * DOWNSCALE_RATIO));
        return new Size(w, h);
    }

    /** Vertical crop band in downscaled camera pixel coordinates. */
    public static org.firstinspires.ftc.vision.opencv.ImageRegion processingCropRoi() {
        double r = DOWNSCALE_RATIO;
        int off = VERTICAL_CROP_OFFSET;
        int capW = CAMERA_RESOLUTION.getWidth();
        int capH = CAMERA_RESOLUTION.getHeight();
        int cropH = Math.min(VERTICAL_CROP_HEIGHT, capH - off);
        return org.firstinspires.ftc.vision.opencv.ImageRegion.asImageCoordinates(
                0,
                (int) Math.round(off * r),
                (int) Math.round(capW * r),
                (int) Math.round((off + cropH) * r));
    }

    public static int processingFrameWidth() {
        return processingResolution().getWidth();
    }

    /**
     * OpenCV crop rect for the processing band within a downscaled portal frame.
     * Prefer this over parsing {@link org.firstinspires.ftc.vision.opencv.ImageRegion} directly.
     */
    public static int[] processingCropRect(int frameCols, int frameRows) {
        double r = DOWNSCALE_RATIO;
        int off = VERTICAL_CROP_OFFSET;
        int capH = CAMERA_RESOLUTION.getHeight();
        int cropH = Math.min(VERTICAL_CROP_HEIGHT, capH - off);
        int left = 0;
        int top = (int) Math.round(off * r);
        int right = Math.min(frameCols, (int) Math.round(CAMERA_RESOLUTION.getWidth() * r));
        int bottom = Math.min(frameRows, (int) Math.round((off + cropH) * r));
        return new int[] {left, top, Math.max(1, right - left), Math.max(1, bottom - top)};
    }
}
