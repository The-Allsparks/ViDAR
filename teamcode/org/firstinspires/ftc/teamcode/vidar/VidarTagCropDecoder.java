package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Runs official {@link AprilTagProcessor} on a cropped sub-frame only (~25% of pixels)
 * with lens intrinsics adjusted for the ROI.
 */
public final class VidarTagCropDecoder {

    public static final class DecodeResult {
        public final int tagId;
        public final double centerX;
        public final double centerY;
        public final Pose2D fieldPose;
        public final int decimationUsed;
        public final int decodePixels;

        DecodeResult(
                int tagId,
                double centerX,
                double centerY,
                Pose2D fieldPose,
                int decimationUsed,
                int decodePixels) {
            this.tagId = tagId;
            this.centerX = centerX;
            this.centerY = centerY;
            this.fieldPose = fieldPose;
            this.decimationUsed = decimationUsed;
            this.decodePixels = decodePixels;
        }
    }

    private double fx;
    private double fy;
    private double cx;
    private double cy;

    public void init(int fullWidth, int fullHeight, CameraCalibration calibration) {
        double[] intrinsics = resolveIntrinsics(fullWidth, fullHeight, calibration);
        fx = intrinsics[0];
        fy = intrinsics[1];
        cx = intrinsics[2];
        cy = intrinsics[3];
    }

    /**
     * @param scout optional match target in full-frame coordinates
     */
    public DecodeResult decode(
            Mat fullFrame,
            Rect crop,
            int decimation,
            long captureTimeNanos,
            VidarTagScoutResult scout) {
        if (fullFrame == null || fullFrame.empty() || crop == null) {
            return null;
        }

        Mat cropMat = null;
        Mat work = null;

        try {
            cropMat = new Mat(fullFrame, crop);
            work = cropMat;
            double scale = 1.0;

            if (decimation > 1) {
                scale = 1.0 / decimation;
                work = new Mat();
                int w = Math.max(32, (int) Math.round(crop.width * scale));
                int h = Math.max(24, (int) Math.round(crop.height * scale));
                Imgproc.resize(cropMat, work, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
            }

            double ifx = fx * scale;
            double ify = fy * scale;
            double icx = (cx - crop.x) * scale;
            double icy = (cy - crop.y) * scale;

            AprilTagProcessor cropTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(ifx, ify, icx, icy)
                    .build();
            cropTag.setDecimation(1);
            cropTag.processFrame(work, captureTimeNanos);

            List<AprilTagDetection> detections;
            try {
                detections = cropTag.getFreshDetections();
            } catch (RuntimeException ex) {
                detections = cropTag.getDetections();
            }

            AprilTagDetection best = pickBest(detections, scout, crop, work.cols(), work.rows());
            if (best == null) {
                return null;
            }

            double mapScaleX = (double) crop.width / Math.max(1, work.cols());
            double mapScaleY = (double) crop.height / Math.max(1, work.rows());
            double fullCenterX = best.center.x * mapScaleX + crop.x;
            double fullCenterY = best.center.y * mapScaleY + crop.y;
            Pose2D fieldPose = new Pose2D(
                    DistanceUnit.INCH,
                    best.ftcPose.x,
                    best.ftcPose.y,
                    AngleUnit.DEGREES,
                    best.ftcPose.yaw);

            return new DecodeResult(
                    best.id,
                    fullCenterX,
                    fullCenterY,
                    fieldPose,
                    decimation,
                    work.cols() * work.rows());
        } finally {
            if (work != null && work != cropMat) {
                work.release();
            }
            if (cropMat != null) {
                cropMat.release();
            }
        }
    }

    private static AprilTagDetection pickBest(
            List<AprilTagDetection> detections,
            VidarTagScoutResult scout,
            Rect crop,
            int workCols,
            int workRows) {
        if (detections == null || detections.isEmpty()) {
            return null;
        }

        double mapScaleX = (double) crop.width / Math.max(1, workCols);
        double mapScaleY = (double) crop.height / Math.max(1, workRows);

        AprilTagDetection best = null;
        double bestDist = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (VidarTagConfig.DESIRED_TAG_ID >= 0 && detection.id != VidarTagConfig.DESIRED_TAG_ID) {
                continue;
            }

            double fullX = detection.center.x * mapScaleX + crop.x;
            double fullY = detection.center.y * mapScaleY + crop.y;

            if (scout != null) {
                double dist = Math.hypot(fullX - scout.cx, fullY - scout.cy);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = detection;
                }
            } else if (best == null) {
                best = detection;
            }
        }
        return best;
    }

    private static double[] resolveIntrinsics(
            int width,
            int height,
            CameraCalibration calibration) {
        double sx = width / 640.0;
        double sy = height / 480.0;
        return new double[] {
                VidarTagConfig.LENS_FX * sx,
                VidarTagConfig.LENS_FY * sy,
                VidarTagConfig.LENS_CX * sx,
                VidarTagConfig.LENS_CY * sy,
        };
    }
}
