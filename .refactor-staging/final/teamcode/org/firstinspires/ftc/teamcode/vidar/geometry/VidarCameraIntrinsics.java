package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;

import java.util.ArrayList;
import java.util.List;

/**
 * Pinhole camera intrinsics with optional distortion metadata.
 *
 * <p>Operational model: zero-distortion pinhole. Non-zero distortion coefficients are stored for
 * future offline refinement but are not applied in the hot path unless the model is {@code none}.
 */
public final class VidarCameraIntrinsics {

    public enum DistortionModel {
        NONE,
        /** Brown-Conrady (k1, k2, p1, p2, k3) — planned runtime support. */
        BROWN_CONRADY,
        /** Unsupported on-robot — fail clearly if used. */
        FISHEYE;

        public static DistortionModel fromJson(String value) {
            if (value == null || value.isEmpty() || "none".equalsIgnoreCase(value)) {
                return NONE;
            }
            if ("brown_conrady".equalsIgnoreCase(value) || "plumb_bob".equalsIgnoreCase(value)) {
                return BROWN_CONRADY;
            }
            if ("fisheye".equalsIgnoreCase(value) || "kannala_brandt".equalsIgnoreCase(value)) {
                return FISHEYE;
            }
            return NONE;
        }
    }

    public final double fx;
    public final double fy;
    public final double cx;
    public final double cy;
    public final int imageWidth;
    public final int imageHeight;
    public final DistortionModel distortionModel;
    public final double[] distortionCoeffs;
    public final String calibrationVersion;
    public final String calibrationDate;

    public VidarCameraIntrinsics(
            double fx,
            double fy,
            double cx,
            double cy,
            int imageWidth,
            int imageHeight) {
        this(fx, fy, cx, cy, imageWidth, imageHeight,
                DistortionModel.NONE, null, null, null);
    }

    public VidarCameraIntrinsics(
            double fx,
            double fy,
            double cx,
            double cy,
            int imageWidth,
            int imageHeight,
            DistortionModel distortionModel,
            double[] distortionCoeffs,
            String calibrationVersion,
            String calibrationDate) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.distortionModel = distortionModel == null ? DistortionModel.NONE : distortionModel;
        this.distortionCoeffs = distortionCoeffs == null ? new double[0] : distortionCoeffs.clone();
        this.calibrationVersion = calibrationVersion;
        this.calibrationDate = calibrationDate;
    }

    public static VidarCameraIntrinsics fromProfile(VidarCameraProfile profile) {
        return fromProfile(profile, 640, 480);
    }

    public static VidarCameraIntrinsics fromProfile(
            VidarCameraProfile profile, int defaultWidth, int defaultHeight) {
        int w = profile.calibrationWidth > 0 ? profile.calibrationWidth : defaultWidth;
        int h = profile.calibrationHeight > 0 ? profile.calibrationHeight : defaultHeight;
        return new VidarCameraIntrinsics(
                profile.focalLengthPx,
                profile.focalLengthYPx,
                profile.principalPointX,
                profile.principalPointY,
                w,
                h,
                profile.distortionModel,
                profile.distortionCoeffs,
                profile.calibrationVersion,
                profile.calibrationDate);
    }

    /** Unit ray in camera optical frame (+X right, +Y down, +Z forward). Pixel center convention. */
    public VidarVec3 pixelToRay(double pixelX, double pixelY) {
        if (!isValid()) {
            return new VidarVec3(Double.NaN, Double.NaN, Double.NaN);
        }
        if (distortionModel == DistortionModel.FISHEYE) {
            return new VidarVec3(Double.NaN, Double.NaN, Double.NaN);
        }
        // Distortion correction not applied on-robot for BROWN_CONRADY — documented limitation.
        double u = (pixelX + 0.5 - cx) / fx;
        double v = (pixelY + 0.5 - cy) / fy;
        return new VidarVec3(u, v, 1.0).normalized();
    }

    /** Project a 3D point in camera optical frame to calibrated sensor pixels. */
    public double[] pointToPixel(VidarVec3 pointCamera) {
        if (!isValid() || pointCamera == null || !pointCamera.isFinite()) {
            return new double[] { Double.NaN, Double.NaN };
        }
        if (pointCamera.z <= 1e-9) {
            return new double[] { Double.NaN, Double.NaN };
        }
        double u = pointCamera.x / pointCamera.z;
        double v = pointCamera.y / pointCamera.z;
        return new double[] {
                u * fx + cx - 0.5,
                v * fy + cy - 0.5
        };
    }

    public List<String> validate() {
        List<String> warnings = new ArrayList<>();
        if (fx <= 0 || fy <= 0) {
            warnings.add("fx and fy must be positive");
        }
        if (imageWidth <= 0 || imageHeight <= 0) {
            warnings.add("calibration imageWidth/imageHeight must be positive");
        }
        if (cx < 0 || cx > imageWidth || cy < 0 || cy > imageHeight) {
            warnings.add("principal point outside calibration image bounds");
        }
        if (distortionModel == DistortionModel.FISHEYE) {
            warnings.add("fisheye distortion is not supported on-robot");
        }
        if (distortionModel == DistortionModel.BROWN_CONRADY
                && distortionCoeffs != null && distortionCoeffs.length > 0) {
            warnings.add("brown-conrady coeffs stored but not applied on-robot (planned)");
        }
        return warnings;
    }

    public boolean isValid() {
        return fx > 0 && fy > 0 && imageWidth > 0 && imageHeight > 0
                && Double.isFinite(cx) && Double.isFinite(cy)
                && distortionModel != DistortionModel.FISHEYE;
    }
}
