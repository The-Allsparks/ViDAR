package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * Ray intersection with a horizontal ground plane (robot Z = 0).
 *
 * <p>Assumes the target contacts the floor. Rejects rays parallel to the plane or pointing away
 * from it. Preserves uncertainty by scaling slant-range uncertainty to horizontal distance.
 */
public final class VidarGroundPlane {

    public static final class Intersection {
        public final double robotX;
        public final double robotY;
        public final double slantRange;
        public final double horizontalUncertainty;
        public final boolean valid;
        public final String rejectionReason;

        Intersection(
                double robotX,
                double robotY,
                double slantRange,
                double horizontalUncertainty,
                boolean valid,
                String rejectionReason) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.slantRange = slantRange;
            this.horizontalUncertainty = horizontalUncertainty;
            this.valid = valid;
            this.rejectionReason = rejectionReason;
        }

        public static Intersection rejected(String reason) {
            return new Intersection(
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN, false, reason);
        }
    }

    private VidarGroundPlane() {}

    /**
     * Intersect a ray from camera origin through {@code rayDirectionRobot} with z = 0 plane.
     *
     * @param cameraOriginRobot lens position in robot frame
     * @param rayDirectionRobot unit direction in robot frame
     */
    public static Intersection intersect(
            VidarVec3 cameraOriginRobot,
            VidarVec3 rayDirectionRobot,
            double slantRangeUncertainty) {
        if (cameraOriginRobot == null || rayDirectionRobot == null
                || !cameraOriginRobot.isFinite() || !rayDirectionRobot.isFinite()) {
            return Intersection.rejected("invalid_ray");
        }
        double dz = rayDirectionRobot.z;
        if (Math.abs(dz) < 1e-6) {
            return Intersection.rejected("parallel_to_plane");
        }
        if (cameraOriginRobot.z <= 0 && dz >= 0) {
            return Intersection.rejected("pointing_away_from_floor");
        }
        double t = -cameraOriginRobot.z / dz;
        if (t <= 0) {
            return Intersection.rejected("behind_camera");
        }
        VidarVec3 hit = cameraOriginRobot.plus(rayDirectionRobot.scaled(t));
        double horizontal = Math.hypot(hit.x, hit.y);
        double horizUnc = Double.isNaN(slantRangeUncertainty)
                ? Double.NaN
                : slantRangeUncertainty * (horizontal / Math.max(t, 1e-6));
        return new Intersection(hit.x, hit.y, t, horizUnc, true, null);
    }

    /**
     * Floor contact from slant range along a pixel ray (production path using {@code robot_T_camera}).
     */
    public static Intersection floorPointFromSlantRange(
            VidarTransformRegistry.CameraTransforms transforms,
            double sensorPixelX,
            double sensorPixelY,
            double slantRange,
            double slantRangeUncertainty) {
        if (transforms == null || Double.isNaN(slantRange) || slantRange <= 0) {
            return Intersection.rejected("invalid_range");
        }
        VidarVec3 origin = new VidarVec3(
                transforms.robotTCamera.translation.x,
                transforms.robotTCamera.translation.y,
                transforms.robotTCamera.translation.z);
        VidarVec3 dir = transforms.robotTCamera.transformDirection(
                transforms.intrinsics.pixelToRay(sensorPixelX, sensorPixelY));
        Intersection planeHit = intersect(origin, dir, slantRangeUncertainty);
        if (!planeHit.valid) {
            return planeHit;
        }
        // Scale along ray to measured slant range instead of plane intersection distance.
        VidarVec3 scaled = origin.plus(dir.scaled(slantRange));
        double horizUnc = Double.isNaN(slantRangeUncertainty)
                ? Double.NaN
                : slantRangeUncertainty;
        return new Intersection(scaled.x, scaled.y, slantRange, horizUnc, true, null);
    }
}
