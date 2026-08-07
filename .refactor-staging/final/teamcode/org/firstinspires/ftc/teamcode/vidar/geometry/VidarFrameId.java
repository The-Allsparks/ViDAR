package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * Named coordinate frames used by ViDAR.
 *
 * <p>Transforms use {@code destination_T_source} notation: a point expressed in {@code source}
 * is mapped into {@code destination}.
 */
public enum VidarFrameId {
    /** FTC field frame: origin at field center, +X right, +Y forward, +Z up (inches). */
    FIELD,
    /** ViDAR robot frame: +X forward, +Y left, +Z up from floor (active distance unit). */
    ROBOT,
    /**
     * Camera body frame aligned with the optical axis convention used by ViDAR intrinsics:
     * +X image-right, +Y image-down, +Z optical-forward.
     */
    CAMERA_OPTICAL;

    /** Frame id for a configured side camera ({@code front}, {@code right}, …). */
    public static String cameraBody(String cameraName) {
        if (cameraName == null || cameraName.isEmpty()) {
            return "camera";
        }
        return "camera_" + cameraName;
    }
}
