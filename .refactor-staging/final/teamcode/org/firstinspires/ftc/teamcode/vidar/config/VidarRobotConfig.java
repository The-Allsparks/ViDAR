package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit;
import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraMount;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraRoiConfig;

/**
 * Per-robot hardware layout: camera names, mount positions, and field calibration.
 * Teams maintain a JSON robot file (like Pedro Pathing constants) and load it at init.
 */
public final class VidarRobotConfig {

    public final String robotName;
    public final int activeCameraIndex;
    public final VidarCameraMount[] cameras;
    public final VidarRobotDimensions dimensions;
    public final VidarAlliance defaultAlliance;
    public final String allianceColorSensor;
    public final boolean allianceUseColorSensor;
    public final boolean allianceAllowRuntimeToggle;

    /** When non-null, overrides season {@code distanceUnit} for this robot. */
    public final org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit distanceUnitOverride;

    public VidarRobotConfig(
            String robotName,
            int activeCameraIndex,
            VidarCameraMount[] cameras,
            VidarRobotDimensions dimensions,
            VidarAlliance defaultAlliance,
            String allianceColorSensor,
            boolean allianceUseColorSensor,
            boolean allianceAllowRuntimeToggle) {
        this(robotName, activeCameraIndex, cameras, dimensions, defaultAlliance,
                allianceColorSensor, allianceUseColorSensor, allianceAllowRuntimeToggle, null);
    }

    public VidarRobotConfig(
            String robotName,
            int activeCameraIndex,
            VidarCameraMount[] cameras,
            VidarRobotDimensions dimensions,
            VidarAlliance defaultAlliance,
            String allianceColorSensor,
            boolean allianceUseColorSensor,
            boolean allianceAllowRuntimeToggle,
            org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit distanceUnitOverride) {
        this.robotName = robotName;
        this.activeCameraIndex = activeCameraIndex;
        this.cameras = cameras == null ? new VidarCameraMount[0] : cameras;
        this.dimensions = dimensions;
        this.defaultAlliance = defaultAlliance == null ? VidarAlliance.RED : defaultAlliance;
        this.allianceColorSensor = allianceColorSensor;
        this.allianceUseColorSensor = allianceUseColorSensor;
        this.allianceAllowRuntimeToggle = allianceAllowRuntimeToggle;
        this.distanceUnitOverride = distanceUnitOverride;
    }

    public int activeCameraCount() {
        return Math.max(1, Math.min(4, cameras.length));
    }

    public VidarCameraMount cameraMount(int index) {
        if (index < 0 || index >= cameras.length) {
            throw new IndexOutOfBoundsException("Camera index " + index);
        }
        return cameras[index];
    }

    public VidarCameraProfile activeCameraProfile() {
        int i = Math.max(0, Math.min(cameras.length - 1, activeCameraIndex));
        return cameras[i].profile;
    }

    /** ROI defaults shared when JSON omits per-camera roi block. */
    public static final class CameraProfileSpec {

        public final String name;
        public final double bearingDeg;
        public final int horizonRowPx;
        public final double focalLengthPx;
        public final double focalLengthYPx;
        public final double principalPointX;
        public final double principalPointY;
        public final double horizontalFovDeg;
        public final double verticalFovDeg;
        public final double[] floorCyPx;
        public final double[] floorDist;
        public final double mountX;
        public final double mountY;
        public final double mountZ;
        public final double mountYawDeg;
        public final double mountPitchDeg;
        public final double mountRollDeg;
        public final double plateWidth;
        public final VidarCameraRoiConfig roiConfig;
        public final int calibrationWidth;
        public final int calibrationHeight;
        public final org.firstinspires.ftc.teamcode.vidar.geometry.VidarCameraIntrinsics.DistortionModel distortionModel;
        public final double[] distortionCoeffs;
        public final String calibrationVersion;
        public final String calibrationDate;

        public CameraProfileSpec(
                String name,
                double bearingDeg,
                int horizonRowPx,
                double focalLengthPx,
                double focalLengthYPx,
                double principalPointX,
                double principalPointY,
                double horizontalFovDeg,
                double verticalFovDeg,
                double[] floorCyPx,
                double[] floorDist,
                double mountX,
                double mountY,
                double mountZ,
                double mountYawDeg,
                double mountPitchDeg,
                double mountRollDeg,
                double plateWidth,
                VidarCameraRoiConfig roiConfig) {
            this(name, bearingDeg, horizonRowPx, focalLengthPx, focalLengthYPx,
                    principalPointX, principalPointY, horizontalFovDeg, verticalFovDeg,
                    floorCyPx, floorDist, mountX, mountY, mountZ,
                    mountYawDeg, mountPitchDeg, mountRollDeg, plateWidth, roiConfig,
                    0, 0,
                    org.firstinspires.ftc.teamcode.vidar.geometry.VidarCameraIntrinsics.DistortionModel.NONE,
                    null, null, null);
        }

        public CameraProfileSpec(
                String name,
                double bearingDeg,
                int horizonRowPx,
                double focalLengthPx,
                double focalLengthYPx,
                double principalPointX,
                double principalPointY,
                double horizontalFovDeg,
                double verticalFovDeg,
                double[] floorCyPx,
                double[] floorDist,
                double mountX,
                double mountY,
                double mountZ,
                double mountYawDeg,
                double mountPitchDeg,
                double mountRollDeg,
                double plateWidth,
                VidarCameraRoiConfig roiConfig,
                int calibrationWidth,
                int calibrationHeight,
                org.firstinspires.ftc.teamcode.vidar.geometry.VidarCameraIntrinsics.DistortionModel distortionModel,
                double[] distortionCoeffs,
                String calibrationVersion,
                String calibrationDate) {
            this.name = name;
            this.bearingDeg = bearingDeg;
            this.horizonRowPx = horizonRowPx;
            this.focalLengthPx = focalLengthPx;
            this.focalLengthYPx = focalLengthYPx;
            this.principalPointX = principalPointX;
            this.principalPointY = principalPointY;
            this.horizontalFovDeg = horizontalFovDeg;
            this.verticalFovDeg = verticalFovDeg;
            this.floorCyPx = floorCyPx;
            this.floorDist = floorDist;
            this.mountX = mountX;
            this.mountY = mountY;
            this.mountZ = mountZ;
            this.mountYawDeg = mountYawDeg;
            this.mountPitchDeg = mountPitchDeg;
            this.mountRollDeg = mountRollDeg;
            this.plateWidth = plateWidth;
            this.roiConfig = roiConfig == null ? VidarCameraRoiConfig.DEFAULT : roiConfig;
            this.calibrationWidth = calibrationWidth;
            this.calibrationHeight = calibrationHeight;
            this.distortionModel = distortionModel == null
                    ? org.firstinspires.ftc.teamcode.vidar.geometry.VidarCameraIntrinsics.DistortionModel.NONE
                    : distortionModel;
            this.distortionCoeffs = distortionCoeffs;
            this.calibrationVersion = calibrationVersion;
            this.calibrationDate = calibrationDate;
        }

        public VidarCameraProfile toProfile() {
            return new VidarCameraProfile(
                    name, bearingDeg, horizonRowPx,
                    focalLengthPx, focalLengthYPx,
                    principalPointX, principalPointY,
                    horizontalFovDeg, verticalFovDeg,
                    floorCyPx, floorDist,
                    mountX, mountY, mountZ,
                    mountYawDeg, mountPitchDeg, mountRollDeg,
                    plateWidth, roiConfig,
                    calibrationWidth, calibrationHeight,
                    distortionModel, distortionCoeffs,
                    calibrationVersion, calibrationDate);
        }
    }
}
