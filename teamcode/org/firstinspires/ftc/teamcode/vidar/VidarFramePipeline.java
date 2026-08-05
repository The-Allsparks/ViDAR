package org.firstinspires.ftc.teamcode.vidar;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Downscale helpers for tic-toc processing at 640×480 capture with per-camera ROIs.
 */
public final class VidarFramePipeline {

    public static final class ScaledRoi {
        public final Mat image;
        public final Rect sourceCrop;
        public final double scale;

        ScaledRoi(Mat image, Rect sourceCrop, double scale) {
            this.image = image;
            this.sourceCrop = sourceCrop;
            this.scale = scale;
        }

        public void release() {
            if (image != null) {
                image.release();
            }
        }

        public double toFullX(double localX) {
            return localX * scale + sourceCrop.x;
        }

        public double toFullY(double localY) {
            return localY * scale + sourceCrop.y;
        }

        public int fullFrameWidth() {
            return sourceCrop.x * 2 + (int) Math.round(image.cols() / scale);
        }

        public int fullFrameHeight() {
            return sourceCrop.y + sourceCrop.height + (int) Math.round(
                    (fullFrameHeightEstimate() - sourceCrop.y - sourceCrop.height));
        }

        private int fullFrameHeightEstimate() {
            return (int) Math.round(image.rows() / scale) + sourceCrop.y;
        }
    }

    private VidarFramePipeline() {}

    /** ROI downscaled for ball/plate OpenCV work. */
    public static ScaledRoi roiScaled(Mat frame, VidarRoiRect roi, double scale) {
        if (frame == null || frame.empty() || scale <= 0 || roi == null || !roi.enabled) {
            return null;
        }
        VidarRoiRect clamped = roi.clamped(frame.cols(), frame.rows());
        Rect crop = clamped.toOpenCvRect();
        if (crop.width <= 0 || crop.height <= 0) {
            return null;
        }

        Mat roiMat = new Mat(frame, crop);
        int outW = Math.max(32, (int) Math.round(crop.width * scale));
        int outH = Math.max(24, (int) Math.round(crop.height * scale));
        Mat scaled = new Mat();
        Imgproc.resize(roiMat, scaled, new Size(outW, outH), 0, 0, Imgproc.INTER_AREA);
        roiMat.release();
        return new ScaledRoi(scaled, crop, scale);
    }

    public static ScaledRoi ballScaled(Mat frame, VidarCameraProfile profile, double scale) {
        VidarRoiRect roi = VidarFrameRegions.ballRoi(profile, frame.cols(), frame.rows());
        return roiScaled(frame, roi, scale);
    }

    public static ScaledRoi plateScaled(Mat frame, VidarCameraProfile profile, double scale) {
        VidarRoiRect roi = VidarFrameRegions.plateRoi(profile, frame.cols(), frame.rows());
        return roiScaled(frame, roi, scale);
    }

    /** @deprecated Use {@link #ballScaled}. */
    @Deprecated
    public static ScaledRoi bottomHalfScaled(Mat frame, double scale) {
        if (frame == null || frame.empty()) {
            return null;
        }
        VidarRoiRect roi = VidarCameraRoiConfig.DEFAULT.ballRoi(frame.cols(), frame.rows());
        return roiScaled(frame, roi, scale);
    }
}
