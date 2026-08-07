package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.VidarGeometry;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarFrameRegions;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarRoiRect;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;

/**
 * Authoritative static transforms derived from {@link VidarCameraProfile} mount configuration.
 *
 * <p>Caches {@code robot_T_cameraOptical} and inverse per profile at init — no per-frame allocation.
 */
public final class VidarTransformRegistry {

    public static final class CameraTransforms {
        public final String cameraName;
        public final VidarCameraIntrinsics intrinsics;
        /** robot_T_cameraOptical */
        public final VidarTransform3D robotTCamera;
        /** cameraOptical_T_robot (derived inverse) */
        public final VidarTransform3D cameraTRobot;

        CameraTransforms(
                String cameraName,
                VidarCameraIntrinsics intrinsics,
                VidarTransform3D robotTCamera) {
            this.cameraName = cameraName;
            this.intrinsics = intrinsics;
            this.robotTCamera = robotTCamera;
            this.cameraTRobot = robotTCamera.inverse();
        }
    }

    private final CameraTransforms[] cameras;

    public VidarTransformRegistry(VidarRobotConfig robot) {
        this(robot, 640, 480);
    }

    public VidarTransformRegistry(VidarRobotConfig robot, int frameWidth, int frameHeight) {
        int count = robot == null ? 0 : robot.activeCameraCount();
        cameras = new CameraTransforms[count];
        if (robot == null) {
            return;
        }
        for (int i = 0; i < count; i++) {
            VidarCameraProfile profile = robot.cameraMount(i).profile;
            cameras[i] = buildForProfile(profile, frameWidth, frameHeight);
        }
    }

    public CameraTransforms forIndex(int index) {
        if (index < 0 || index >= cameras.length) {
            return null;
        }
        return cameras[index];
    }

    public CameraTransforms forName(String cameraName) {
        if (cameraName == null) {
            return null;
        }
        for (CameraTransforms ct : cameras) {
            if (ct != null && cameraName.equals(ct.cameraName)) {
                return ct;
            }
        }
        return null;
    }

    public int cameraCount() {
        return cameras.length;
    }

    public static CameraTransforms buildForProfile(VidarCameraProfile profile) {
        return buildForProfile(profile, 640, 480);
    }

    /**
     * Builds {@code robot_T_cameraOptical} matching legacy {@link org.firstinspires.ftc.teamcode.vidar.VidarGeometry#rayDirectionRobotFrame}.
     *
     * <p>Rotation chain on optical axes: {@code Rz(bearing+yaw) * Rx(pitch) * Rz(roll) * R_optical_base}.
     */
    public static CameraTransforms buildForProfile(
            VidarCameraProfile profile, int frameWidth, int frameHeight) {
        if (profile == null) {
            return null;
        }
        VidarRotation3D mountRot = VidarRotation3D.rotateZ(
                        Math.toRadians(profile.bearingDeg + profile.mountYawDeg))
                .times(VidarRotation3D.rotateX(Math.toRadians(profile.mountPitchDeg)))
                .times(VidarRotation3D.rotateZ(Math.toRadians(profile.mountRollDeg)));
        VidarRotation3D rot = mountRot.times(VidarRotation3D.opticalToRobotBase());
        VidarVec3 trans = new VidarVec3(profile.mountX, profile.mountY, profile.mountZ);
        VidarTransform3D robotTCamera = new VidarTransform3D(
                VidarFrameId.ROBOT,
                VidarFrameId.CAMERA_OPTICAL,
                "robot",
                VidarFrameId.cameraBody(profile.name),
                rot,
                trans);
        VidarCameraIntrinsics intrinsics = VidarCameraIntrinsics.fromProfile(
                profile, frameWidth, frameHeight);
        return new CameraTransforms(profile.name, intrinsics, robotTCamera);
    }

    /**
     * Unit ray in robot frame from a full-frame sensor pixel, using cached mount transform.
     */
    public VidarVec3 rayDirectionRobotFrame(
            double sensorPixelX, double sensorPixelY, CameraTransforms transforms) {
        if (transforms == null || !transforms.intrinsics.isValid()) {
            return new VidarVec3(Double.NaN, Double.NaN, Double.NaN);
        }
        VidarVec3 rayCamera = transforms.intrinsics.pixelToRay(sensorPixelX, sensorPixelY);
        return transforms.robotTCamera.transformDirection(rayCamera);
    }

    /**
     * Processed pixel → sensor pixel → ray in robot frame (accounts for crop/scale).
     */
    public VidarVec3 rayDirectionRobotFrameProcessed(
            double processedX,
            double processedY,
            CameraTransforms transforms,
            VidarImageTransform imageTransform) {
        if (imageTransform == null || transforms == null) {
            return new VidarVec3(Double.NaN, Double.NaN, Double.NaN);
        }
        double sensorX = imageTransform.toSensorX(processedX);
        double sensorY = imageTransform.toSensorY(processedY);
        return rayDirectionRobotFrame(sensorX, sensorY, transforms);
    }

    /** Build image transform for element detection ROI on a full capture frame. */
    public static VidarImageTransform elementImageTransform(
            VidarCameraProfile profile,
            int fullWidth,
            int fullHeight,
            double processScale) {
        VidarRoiRect roi = VidarFrameRegions.elementRoi(
                profile, fullWidth, fullHeight);
        if (!roi.enabled || processScale <= 0) {
            return VidarImageTransform.identity(fullWidth, fullHeight);
        }
        VidarRoiRect clamped = roi.clamped(fullWidth, fullHeight);
        int procW = Math.max(32, (int) Math.round(clamped.width * processScale));
        int procH = Math.max(24, (int) Math.round(clamped.height * processScale));
        return VidarImageTransform.fromCropAndScale(
                clamped.x, clamped.y, clamped.width, clamped.height, procW, procH);
    }
}
