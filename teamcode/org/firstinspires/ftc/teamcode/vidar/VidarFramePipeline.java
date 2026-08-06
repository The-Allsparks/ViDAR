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
        private final boolean ownsImage;

        ScaledRoi(Mat image, Rect sourceCrop, double scale) {
            this(image, sourceCrop, scale, true);
        }

        ScaledRoi(Mat image, Rect sourceCrop, double scale, boolean ownsImage) {
            this.image = image;
            this.sourceCrop = sourceCrop;
            this.scale = scale;
            this.ownsImage = ownsImage;
        }

        public void release() {
            if (ownsImage && image != null) {
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

    /** ROI downscaled for element/plate OpenCV work. */
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

    public static ScaledRoi elementScaled(Mat frame, VidarCameraProfile profile, double scale) {
        VidarRoiRect roi = VidarFrameRegions.elementRoi(profile, frame.cols(), frame.rows());
        return roiScaled(frame, roi, scale);
    }

    public static ScaledRoi plateScaled(Mat frame, VidarCameraProfile profile, double scale) {
        VidarRoiRect roi = VidarFrameRegions.plateRoi(profile, frame.cols(), frame.rows());
        return roiScaled(frame, roi, scale);
    }

    /** Shared element + plate ROI for {@link VidarContourProcessor}. */
    public static ScaledRoi detectionScaled(Mat frame, VidarCameraProfile profile, double scale) {
        VidarRoiRect roi = VidarFrameRegions.detectionRoi(profile, frame.cols(), frame.rows());
        return roiScaled(frame, roi, scale);
    }

    /**
     * Resize the detection ROI into {@code destScaled}, reusing storage when dimensions match.
     * Returns crop metadata; {@code destScaled} is owned by the caller and not released here.
     */
    public static ScaledRoi detectionScaledInto(
            Mat frame,
            VidarCameraProfile profile,
            double scale,
            Mat destScaled) {
        if (frame == null || frame.empty() || scale <= 0 || destScaled == null) {
            return null;
        }
        VidarRoiRect roi = VidarFrameRegions.detectionRoi(profile, frame.cols(), frame.rows());
        VidarRoiRect clamped = roi.clamped(frame.cols(), frame.rows());
        Rect crop = clamped.toOpenCvRect();
        if (crop.width <= 0 || crop.height <= 0) {
            return null;
        }

        int outW = Math.max(32, (int) Math.round(crop.width * scale));
        int outH = Math.max(24, (int) Math.round(crop.height * scale));
        if (destScaled.empty() || destScaled.rows() != outH || destScaled.cols() != outW) {
            if (!destScaled.empty()) {
                destScaled.release();
            }
            destScaled.create(outH, outW, org.opencv.core.CvType.CV_8UC4);
        }

        Mat roiMat = new Mat(frame, crop);
        Imgproc.resize(roiMat, destScaled, new Size(outW, outH), 0, 0, Imgproc.INTER_AREA);
        roiMat.release();
        return new ScaledRoi(destScaled, crop, scale, false);
    }
}
