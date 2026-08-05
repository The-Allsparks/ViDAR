package org.firstinspires.ftc.teamcode.vidar;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Primary ball detector: color segmentation + geometric filtering.
 * Optional local Hough validation; legacy full-ROI Hough when configured.
 */
public class VidarBallProcessor implements VisionProcessor {

    private static class Candidate {
        final double cx;
        final double cy;
        final double radius;
        final double area;
        final double aspectRatio;
        final double circularity;
        final double fillRatio;
        final double interiorScore;
        final double circleFitQuality;
        final boolean touchesBoundary;
        final int houghVotes;

        Candidate(double cx, double cy, double radius, double area,
                  double aspectRatio, double circularity, double fillRatio,
                  double interiorScore, double circleFitQuality,
                  boolean touchesBoundary, int houghVotes) {
            this.cx = cx;
            this.cy = cy;
            this.radius = radius;
            this.area = area;
            this.aspectRatio = aspectRatio;
            this.circularity = circularity;
            this.fillRatio = fillRatio;
            this.interiorScore = interiorScore;
            this.circleFitQuality = circleFitQuality;
            this.touchesBoundary = touchesBoundary;
            this.houghVotes = houghVotes;
        }
    }

    private final VidarCameraProfile profile;
    private final String cameraName;
    private final VidarProcessScheduler scheduler;
    private final VidarBallDetectorType detectorType;
    private final VidarMetrics metrics;

    private VidarBallObservation bestBall;
    private Candidate bestDraw;
    private final VidarBallRejectionStats rejectionStats = new VidarBallRejectionStats();

    private Mat reusableGray;
    private Mat reusableMask;
    private Mat reusableKernel;
    private Mat reusableCircles;

    public VidarBallProcessor(
            VidarCameraProfile profile,
            String cameraName,
            VidarProcessScheduler scheduler) {
        this(profile, cameraName, scheduler, VidarConfig.BALL_DETECTOR_TYPE, null);
    }

    public VidarBallProcessor(
            VidarCameraProfile profile,
            String cameraName,
            VidarProcessScheduler scheduler,
            VidarBallDetectorType detectorType,
            VidarMetrics metrics) {
        this.profile = profile;
        this.cameraName = cameraName;
        this.scheduler = scheduler;
        this.detectorType = detectorType == null ? VidarConfig.BALL_DETECTOR_TYPE : detectorType;
        this.metrics = metrics;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        bestBall = null;
        bestDraw = null;
        releaseReusable();
    }

    private void releaseReusable() {
        if (reusableGray != null) reusableGray.release();
        if (reusableMask != null) reusableMask.release();
        if (reusableKernel != null) reusableKernel.release();
        if (reusableCircles != null) reusableCircles.release();
        reusableGray = reusableMask = reusableKernel = reusableCircles = null;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long t0 = System.nanoTime();
        VidarProcessScheduler.Slot slot = scheduler.beginFrame(captureTimeNanos);
        if (slot != VidarProcessScheduler.Slot.BALL) {
            return bestBall;
        }

        bestBall = null;
        bestDraw = null;
        rejectionStats.reset();

        if (frame == null || frame.empty()) {
            recordProcessorTime(t0);
            return null;
        }

        if (detectorType == VidarBallDetectorType.LEGACY_HOUGH) {
            processLegacyHough(frame, captureTimeNanos);
            recordProcessorTime(t0);
            return bestBall;
        }

        VidarFramePipeline.ScaledRoi scaled = VidarFramePipeline.ballScaled(
                frame, profile, VidarConfig.PROCESS_ROI_SCALE);
        if (scaled == null) {
            recordProcessorTime(t0);
            return null;
        }

        try {
            List<Candidate> candidates = detectColorBlobCandidates(scaled, frame.cols(), frame.rows());
            if (detectorType == VidarBallDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH) {
                candidates = applyLocalHoughValidation(scaled, candidates);
            }
            selectBestCandidate(candidates, scaled, captureTimeNanos, frame.cols(), frame.rows());
        } finally {
            scaled.release();
        }

        recordProcessorTime(t0);
        return bestBall;
    }

    private void recordProcessorTime(long t0) {
        if (metrics != null) {
            metrics.recordProcessorTime("ball", (System.nanoTime() - t0) / 1_000_000.0);
        }
    }

    private List<Candidate> detectColorBlobCandidates(
            VidarFramePipeline.ScaledRoi scaled, int frameW, int frameH) {
        List<Candidate> out = new ArrayList<>();
        Mat rgba = scaled.image;

        if (reusableMask == null) reusableMask = new Mat();
        if (reusableKernel == null) {
            reusableKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
        }

        buildPollenMask(rgba, reusableMask);
        rejectionStats.maskPixels = org.opencv.core.Core.countNonZero(reusableMask);

        applyMorphology(reusableMask, reusableKernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(reusableMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        rejectionStats.contourCount = contours.size();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < VidarConfig.BALL_MIN_AREA_PX || area > VidarConfig.BALL_MAX_AREA_PX) {
                rejectionStats.rejectedArea++;
                contour.release();
                continue;
            }

            Rect bounds = Imgproc.boundingRect(contour);
            double bw = bounds.width * scaled.scale;
            double bh = bounds.height * scaled.scale;
            if (bw < VidarConfig.BALL_MIN_WIDTH_PX || bh < VidarConfig.BALL_MIN_HEIGHT_PX
                    || bw > VidarConfig.BALL_MAX_WIDTH_PX || bh > VidarConfig.BALL_MAX_HEIGHT_PX) {
                rejectionStats.rejectedArea++;
                contour.release();
                continue;
            }

            double aspect = Math.max(bw, bh) / Math.max(1, Math.min(bw, bh));
            if (aspect > VidarConfig.BALL_MAX_ASPECT_RATIO) {
                rejectionStats.rejectedAspect++;
                contour.release();
                continue;
            }

            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double circularity = perimeter > 0 ? 4 * Math.PI * area / (perimeter * perimeter) : 0;
            if (circularity < VidarConfig.BALL_MIN_CIRCULARITY) {
                rejectionStats.rejectedCircularity++;
                contour.release();
                continue;
            }

            Point[] contourPts = contour.toArray();
            Point center = fitCircleCenter(contourPts);
            double radius = fitCircleRadius(contourPts, center);
            double fillRatio = area / Math.max(1, Math.PI * radius * radius);
            if (fillRatio < VidarConfig.BALL_MIN_FILL_RATIO) {
                rejectionStats.rejectedFillRatio++;
                contour.release();
                continue;
            }

            double interior = interiorScore(rgba, center.x, center.y, radius);
            if (interior < VidarConfig.BALL_MIN_INTERIOR_SCORE) {
                rejectionStats.rejectedInterior++;
                contour.release();
                continue;
            }

            double fullCx = scaled.toFullX(center.x);
            double fullCy = scaled.toFullY(center.y);
            double fullRadius = radius * scaled.scale;
            boolean touches = new VidarRoiRect(
                    scaled.sourceCrop.x, scaled.sourceCrop.y,
                    scaled.sourceCrop.width, scaled.sourceCrop.height)
                    .touchesBoundary(frameW, frameH, fullCx, fullCy, 3);
            if (touches) {
                rejectionStats.rejectedBoundary++;
            }

            double fitQuality = Math.min(1.0, circularity * fillRatio);
            out.add(new Candidate(fullCx, fullCy, fullRadius, area * scaled.scale * scaled.scale,
                    aspect, circularity, fillRatio, interior, fitQuality, touches, 0));
            contour.release();
        }

        return out;
    }

    private void buildPollenMask(Mat rgba, Mat mask) {
        Mat hsv = new Mat();
        try {
            Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGBA2HSV);
            Scalar low = new Scalar(
                    VidarConfig.BALL_HSV_H_MIN, VidarConfig.BALL_HSV_S_MIN, VidarConfig.BALL_HSV_V_MIN);
            Scalar high = new Scalar(
                    VidarConfig.BALL_HSV_H_MAX, VidarConfig.BALL_HSV_S_MAX, VidarConfig.BALL_HSV_V_MAX);
            Imgproc.inRange(hsv, low, high, mask);
        } finally {
            hsv.release();
        }
    }

    private void applyMorphology(Mat mask, Mat kernel) {
        if (VidarConfig.BALL_MORPH_ERODE_PASSES > 0) {
            for (int i = 0; i < VidarConfig.BALL_MORPH_ERODE_PASSES; i++) {
                Imgproc.erode(mask, mask, kernel);
            }
        }
        if (VidarConfig.BALL_MORPH_DILATE_PASSES > 0) {
            for (int i = 0; i < VidarConfig.BALL_MORPH_DILATE_PASSES; i++) {
                Imgproc.dilate(mask, mask, kernel);
            }
        }
        if (VidarConfig.BALL_MORPH_OPEN_PASSES > 0) {
            for (int i = 0; i < VidarConfig.BALL_MORPH_OPEN_PASSES; i++) {
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            }
        }
        if (VidarConfig.BALL_MORPH_CLOSE_PASSES > 0) {
            for (int i = 0; i < VidarConfig.BALL_MORPH_CLOSE_PASSES; i++) {
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
            }
        }
    }

    private List<Candidate> applyLocalHoughValidation(
            VidarFramePipeline.ScaledRoi scaled, List<Candidate> candidates) {
        if (candidates.isEmpty()) {
            return candidates;
        }
        if (reusableGray == null) reusableGray = new Mat();
        if (reusableCircles == null) reusableCircles = new Mat();

        Imgproc.cvtColor(scaled.image, reusableGray, Imgproc.COLOR_RGBA2GRAY);
        List<Candidate> validated = new ArrayList<>();

        for (Candidate c : candidates) {
            double localCx = (c.cx - scaled.sourceCrop.x) / scaled.scale;
            double localCy = (c.cy - scaled.sourceCrop.y) / scaled.scale;
            double localR = c.radius / scaled.scale;
            int pad = (int) Math.ceil(localR * 1.5);
            int x = Math.max(0, (int) localCx - pad);
            int y = Math.max(0, (int) localCy - pad);
            int w = Math.min(scaled.image.cols() - x, pad * 2);
            int h = Math.min(scaled.image.rows() - y, pad * 2);
            if (w < 8 || h < 8) {
                validated.add(c);
                continue;
            }

            Mat patch = new Mat(reusableGray, new Rect(x, y, w, h));
            Imgproc.GaussianBlur(patch, patch, new Size(3, 3), 0);
            Imgproc.HoughCircles(patch, reusableCircles, Imgproc.HOUGH_GRADIENT,
                    VidarConfig.HOUGH_DP, Math.max(8, localR),
                    VidarConfig.HOUGH_PARAM1, VidarConfig.HOUGH_PARAM2,
                    Math.max(4, (int) (localR * 0.6)), Math.max(6, (int) (localR * 1.4)));

            if (!reusableCircles.empty()) {
                validated.add(new Candidate(c.cx, c.cy, c.radius, c.area, c.aspectRatio,
                        c.circularity, c.fillRatio, c.interiorScore,
                        Math.min(1.0, c.circleFitQuality + 0.15), c.touchesBoundary, 1));
            }
            patch.release();
        }
        return validated.isEmpty() ? candidates : validated;
    }

    private void selectBestCandidate(
            List<Candidate> candidates,
            VidarFramePipeline.ScaledRoi scaled,
            long captureTimeNanos,
            int frameW,
            int frameH) {
        VidarBallObservation bestObs = null;
        Candidate bestCand = null;
        double bestScore = -1;

        for (Candidate c : candidates) {
            double processCy = (c.cy - scaled.sourceCrop.y) / scaled.scale;
            VidarBallObservation obs = VidarGeometry.fuseBallObservation(
                    c.cx, c.cy, c.radius, c.area, c.aspectRatio, c.circularity, c.fillRatio,
                    c.interiorScore,
                    detectorType, profile, cameraName, captureTimeNanos,
                    c.touchesBoundary, false, c.circleFitQuality);

            if (obs.confidence < VidarConfig.MIN_BALL_CONFIDENCE) {
                rejectionStats.rejectedConfidence++;
                continue;
            }

            double localCy = processCy;
            double frameHLocal = scaled.image.rows();
            double floorWeight = 0.25 + 0.75 * (localCy / frameHLocal);
            double score = obs.confidence * obs.radiusPx * obs.radiusPx * floorWeight * floorWeight;
            if (score > bestScore) {
                bestScore = score;
                bestObs = obs;
                bestCand = c;
            }
            rejectionStats.accepted++;
        }

        bestBall = bestObs;
        bestDraw = bestCand;
    }

    private void processLegacyHough(Mat frame, long captureTimeNanos) {
        VidarFramePipeline.ScaledRoi scaled = VidarFramePipeline.ballScaled(
                frame, profile, VidarConfig.PROCESS_ROI_SCALE);
        if (scaled == null) return;

        Mat gray = null;
        Mat circles = null;
        try {
            gray = new Mat();
            circles = new Mat();
            Imgproc.cvtColor(scaled.image, gray, Imgproc.COLOR_RGBA2GRAY);
            Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
            Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT,
                    VidarConfig.HOUGH_DP, VidarConfig.HOUGH_MIN_DIST,
                    VidarConfig.HOUGH_PARAM1, VidarConfig.HOUGH_PARAM2,
                    VidarConfig.HOUGH_MIN_RADIUS, VidarConfig.HOUGH_MAX_RADIUS);

            if (circles.empty()) return;

            double bestScore = -1;
            int count = circles.cols();
            for (int i = 0; i < count; i++) {
                double[] data = circles.get(0, i);
                if (data == null || data.length < 3) continue;

                double cx = scaled.toFullX(data[0]);
                double cy = scaled.toFullY(data[1]);
                double r = data[2] * scaled.scale;
                int votes = data.length > 3 ? (int) Math.round(data[3]) : 0;

                if (r < VidarConfig.HOUGH_MIN_RADIUS || r > VidarConfig.HOUGH_MAX_RADIUS) continue;
                double area = Math.PI * r * r;
                if (area < VidarConfig.MIN_BALL_AREA_PX) continue;

                double interior = interiorScore(scaled.image, data[0], data[1], data[2]);
                if (interior < VidarConfig.HOUGH_MIN_INTERIOR) continue;

                VidarBallObservation obs = VidarGeometry.fuseBallObservation(
                        cx, cy, r, area, 1.0, 1.0, 0.85, interior,
                        VidarBallDetectorType.LEGACY_HOUGH, profile, cameraName, captureTimeNanos,
                        false, false, Math.min(1.0, votes / 20.0));
                if (obs.confidence < VidarConfig.MIN_BALL_CONFIDENCE) continue;

                double score = obs.confidence * r * r;
                if (score > bestScore) {
                    bestScore = score;
                    bestBall = obs;
                    bestDraw = new Candidate(cx, cy, r, area, 1, 1, 0.85, interior, 1, false, votes);
                }
            }
        } finally {
            scaled.release();
            if (gray != null) gray.release();
            if (circles != null) circles.release();
        }
    }

    private static Point fitCircleCenter(Point[] pts) {
        double sx = 0, sy = 0;
        for (Point p : pts) {
            sx += p.x;
            sy += p.y;
        }
        return new Point(sx / pts.length, sy / pts.length);
    }

    private static double fitCircleRadius(Point[] pts, Point center) {
        double sum = 0;
        for (Point p : pts) {
            sum += Math.hypot(p.x - center.x, p.y - center.y);
        }
        return sum / pts.length;
    }

    private static double interiorScore(Mat rgba, double cx, double cy, double radius) {
        int ri = (int) Math.floor(radius);
        int icx = (int) Math.round(cx);
        int icy = (int) Math.round(cy);
        double r2 = radius * radius;
        int inside = 0;
        int bright = 0;
        int darkHole = 0;

        for (int dy = -ri; dy <= ri; dy++) {
            for (int dx = -ri; dx <= ri; dx++) {
                if (dx * dx + dy * dy > r2) continue;
                int x = icx + dx;
                int y = icy + dy;
                if (x < 0 || y < 0 || x >= rgba.cols() || y >= rgba.rows()) continue;
                double[] px = rgba.get(y, x);
                if (px == null || px.length < 3) continue;
                inside++;
                int r = (int) px[0], g = (int) px[1], b = (int) px[2];
                int max = Math.max(r, Math.max(g, b));
                int min = Math.min(r, Math.min(g, b));
                if (max >= VidarConfig.BALL_INTERIOR_BRIGHT && max - min <= VidarConfig.BALL_INTERIOR_SPREAD) {
                    bright++;
                }
                if (max < VidarConfig.BALL_HOLE_DARK_MAX) {
                    darkHole++;
                }
            }
        }
        if (inside == 0) return 0;
        double brightRatio = (double) bright / inside;
        double holeBonus = darkHole > inside * 0.05 ? 0.15 : 0;
        return Math.min(1.0, brightRatio + holeBonus);
    }

    public VidarBallObservation getBestBall() {
        return bestBall;
    }

    public VidarBallRejectionStats getRejectionStats() {
        return rejectionStats;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (bestDraw == null) return;

        float cx = (float) (bestDraw.cx * scaleBmpPxToCanvasPx);
        float cy = (float) (bestDraw.cy * scaleBmpPxToCanvasPx);
        float r = (float) (bestDraw.radius * scaleBmpPxToCanvasPx);

        Paint stroke = new Paint();
        stroke.setColor(Color.rgb(220, 220, 240));
        stroke.setStyle(Paint.Style.STROKE);
        stroke.setStrokeWidth(3f * scaleCanvasDensity);
        stroke.setAntiAlias(true);
        canvas.drawCircle(cx, cy, r, stroke);

        if (bestBall != null && !Double.isNaN(bestBall.rangeIn)) {
            Paint text = new Paint();
            text.setColor(Color.WHITE);
            text.setTextSize(14f * scaleCanvasDensity);
            text.setAntiAlias(true);
            String label = String.format("%.0f±%.0f in %s",
                    bestBall.rangeIn,
                    Double.isNaN(bestBall.rangeUncertaintyIn) ? 0 : bestBall.rangeUncertaintyIn,
                    bestBall.detectorType.name());
            canvas.drawText(label, cx + r + 4f, cy, text);
        }
    }
}
