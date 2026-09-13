package org.firstinspires.ftc.teamcode.vidar.tag;

import org.firstinspires.ftc.teamcode.vidar.model.VidarTagScoutObservation;
import org.firstinspires.ftc.teamcode.vidar.runtime.VidarCameraProfile;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Runs official {@link AprilTagProcessor} on a cropped sub-frame only (~25% of pixels)
 * with lens intrinsics adjusted for the ROI.
 *
 * <p>FTC SDK 12.0: {@link AprilTagDetection} is the shared base. Tag {@code id} and pixel
 * {@code center} live on {@link AprilTagSingleDetection}. BIOBUZZ HIVE openings arrive as
 * {@link AprilTagClusterDetection}; {@code ftcPose} is camera-relative to the cluster origin
 * (CELL opening), not one tag center. Do not call {@code .id} or {@code .center} on the base type.
 */
public final class VidarTagCropDecoder {

    /** Sentinel tag id when the hit is a named cluster (SDK samples use -1). */
    public static final int CLUSTER_TAG_ID = -1;

    public static final class DecodeResult {
        public final int tagId;
        /** Cluster metadata name, or {@code null} for a single-tag hit. */
        public final String clusterName;
        public final boolean cluster;
        public final double centerX;
        public final double centerY;
        /**
         * Camera-relative pose copied from {@code ftcPose} (x/y/yaw). Not SDK
         * {@code robotPose} and not {@code field_T_robot} — that mapping is a follow-up.
         */
        public final Pose2D fieldPose;
        public final int decimationUsed;
        public final int decodePixels;

        DecodeResult(
                int tagId,
                String clusterName,
                boolean cluster,
                double centerX,
                double centerY,
                Pose2D fieldPose,
                int decimationUsed,
                int decodePixels) {
            this.tagId = tagId;
            this.clusterName = clusterName;
            this.cluster = cluster;
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
    private AprilTagLibrary tagLibrary;

    private AprilTagProcessor cropTagProcessor;
    private double lastIfx = Double.NaN;
    private double lastIfy = Double.NaN;
    private double lastIcx = Double.NaN;
    private double lastIcy = Double.NaN;

    private Mat reusableWork;

    public void init(int width, int height, CameraCalibration calibration) {
        init(width, height, calibration, null, null);
    }

    public void init(
            int fullWidth,
            int fullHeight,
            CameraCalibration calibration,
            VidarSeasonConfig season) {
        init(fullWidth, fullHeight, calibration, season, null);
    }

    public void init(
            int fullWidth,
            int fullHeight,
            CameraCalibration calibration,
            VidarSeasonConfig season,
            VidarCameraProfile profile) {
        double[] intrinsics = resolveIntrinsics(fullWidth, fullHeight, calibration, profile);
        fx = intrinsics[0];
        fy = intrinsics[1];
        cx = intrinsics[2];
        cy = intrinsics[3];
        tagLibrary = season == null ? null : season.aprilTagLibrary();
        cropTagProcessor = null;
        lastIfx = lastIfy = lastIcx = lastIcy = Double.NaN;
    }

    /**
     * @param scout optional match target in full-frame coordinates
     */
    public DecodeResult decode(
            Mat fullFrame,
            Rect crop,
            int decimation,
            long captureTimeNanos,
            VidarTagScoutObservation scout) {
        if (fullFrame == null || fullFrame.empty() || crop == null) {
            return null;
        }

        Mat cropMat = null;
        try {
            cropMat = new Mat(fullFrame, crop);
            Mat work = cropMat;
            double scale = 1.0;

            if (decimation > 1) {
                scale = 1.0 / decimation;
                int w = Math.max(32, (int) Math.round(crop.width * scale));
                int h = Math.max(24, (int) Math.round(crop.height * scale));
                if (reusableWork == null) {
                    reusableWork = new Mat();
                }
                if (reusableWork.empty() || reusableWork.rows() != h || reusableWork.cols() != w) {
                    reusableWork.create(h, w, cropMat.type());
                }
                Imgproc.resize(cropMat, reusableWork, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
                work = reusableWork;
            }

            double ifx = fx * scale;
            double ify = fy * scale;
            double icx = (cx - crop.x) * scale;
            double icy = (cy - crop.y) * scale;

            AprilTagProcessor cropTag = ensureProcessor(ifx, ify, icx, icy);
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
            return toDecodeResult(best, crop, work.cols(), work.rows(), decimation);
        } finally {
            if (cropMat != null) {
                cropMat.release();
            }
        }
    }

    private AprilTagProcessor ensureProcessor(double ifx, double ify, double icx, double icy) {
        if (cropTagProcessor == null
                || ifx != lastIfx
                || ify != lastIfy
                || icx != lastIcx
                || icy != lastIcy) {
            AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(ifx, ify, icx, icy);
            if (tagLibrary != null) {
                builder.setTagLibrary(tagLibrary);
            }
            cropTagProcessor = builder.build();
            lastIfx = ifx;
            lastIfy = ify;
            lastIcx = icx;
            lastIcy = icy;
        }
        return cropTagProcessor;
    }

    /**
     * Prefer a cluster hit (CELL opening) over a single tag. Single-tag {@code id} is identity only.
     */
    static AprilTagDetection pickBest(
            List<AprilTagDetection> detections,
            VidarTagScoutObservation scout,
            Rect crop,
            int workCols,
            int workRows) {
        if (detections == null || detections.isEmpty()) {
            return null;
        }

        double mapScaleX = (double) crop.width / Math.max(1, workCols);
        double mapScaleY = (double) crop.height / Math.max(1, workRows);

        AprilTagDetection bestCluster = null;
        AprilTagDetection bestSingle = null;
        double bestDist = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (VidarTagConfig.DESIRED_TAG_ID >= 0 && single.id != VidarTagConfig.DESIRED_TAG_ID) {
                    continue;
                }
                if (single.center == null) {
                    continue;
                }

                double fullX = single.center.x * mapScaleX + crop.x;
                double fullY = single.center.y * mapScaleY + crop.y;

                if (scout != null) {
                    double dist = Math.hypot(fullX - scout.cx, fullY - scout.cy);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestSingle = single;
                    }
                } else if (bestSingle == null) {
                    bestSingle = single;
                }
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                if (bestCluster == null) {
                    bestCluster = clusterDet;
                }
            }
        }
        return bestCluster != null ? bestCluster : bestSingle;
    }

    static DecodeResult toDecodeResult(
            AprilTagDetection best,
            Rect crop,
            int workCols,
            int workRows,
            int decimation) {
        if (best == null) {
            return null;
        }

        Pose2D cameraRelative = poseFromFtc(best.ftcPose);
        int decodePixels = Math.max(1, workCols) * Math.max(1, workRows);

        if (best instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) best;
            double mapScaleX = (double) crop.width / Math.max(1, workCols);
            double mapScaleY = (double) crop.height / Math.max(1, workRows);
            Point center = single.center;
            double fullCenterX = center == null ? Double.NaN : center.x * mapScaleX + crop.x;
            double fullCenterY = center == null ? Double.NaN : center.y * mapScaleY + crop.y;
            return new DecodeResult(
                    single.id,
                    null,
                    false,
                    fullCenterX,
                    fullCenterY,
                    cameraRelative,
                    decimation,
                    decodePixels);
        }

        if (!(best instanceof AprilTagClusterDetection)) {
            return null;
        }
        AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) best;
        String clusterName = clusterDet.metadata == null ? null : clusterDet.metadata.name;
        return new DecodeResult(
                CLUSTER_TAG_ID,
                clusterName,
                true,
                Double.NaN,
                Double.NaN,
                cameraRelative,
                decimation,
                decodePixels);
    }

    /** Camera-relative {@code ftcPose} only. Do not read {@code detection.robotPose}. */
    private static Pose2D poseFromFtc(AprilTagPoseFtc ftcPose) {
        if (ftcPose == null) {
            return null;
        }
        return new Pose2D(
                DistanceUnit.INCH,
                ftcPose.x,
                ftcPose.y,
                AngleUnit.DEGREES,
                ftcPose.yaw);
    }

    private static double[] resolveIntrinsics(
            int width,
            int height,
            CameraCalibration calibration,
            VidarCameraProfile profile) {
        int calW = (profile != null && profile.calibrationWidth > 0)
                ? profile.calibrationWidth
                : VidarTagConfig.CAPTURE_RESOLUTION.getWidth();
        int calH = (profile != null && profile.calibrationHeight > 0)
                ? profile.calibrationHeight
                : VidarTagConfig.CAPTURE_RESOLUTION.getHeight();
        double sx = width / (double) Math.max(1, calW);
        double sy = height / (double) Math.max(1, calH);
        return new double[] {
                VidarTagConfig.lensFx(profile, sx),
                VidarTagConfig.lensFy(profile, sy),
                VidarTagConfig.lensCx(profile, sx),
                VidarTagConfig.lensCy(profile, sy),
        };
    }
}
