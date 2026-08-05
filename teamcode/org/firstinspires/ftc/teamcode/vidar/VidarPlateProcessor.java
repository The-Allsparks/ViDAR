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
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Alliance plate detector: color mask → contour → {@link Imgproc#minAreaRect} → white-digit gate.
 */
public class VidarPlateProcessor implements VisionProcessor {

    private static class Candidate {
        final VidarAlliance alliance;
        final RotatedRect box;
        final double whiteRatio;
        final double contourArea;
        final double rectangularity;
        final double aspect;

        Candidate(VidarAlliance alliance, RotatedRect box, double whiteRatio,
                  double contourArea, double rectangularity, double aspect) {
            this.alliance = alliance;
            this.box = box;
            this.whiteRatio = whiteRatio;
            this.contourArea = contourArea;
            this.rectangularity = rectangularity;
            this.aspect = aspect;
        }
    }

    private final VidarCameraProfile profile;
    private final String cameraName;
    private final VidarProcessScheduler scheduler;
    private VidarPlateObservation bestPlate;
    private VidarPlateObservation bestRed;
    private VidarPlateObservation bestBlue;
    private RotatedRect bestDraw;

    public VidarPlateProcessor(
            VidarCameraProfile profile,
            String cameraName,
            VidarProcessScheduler scheduler) {
        this.profile = profile;
        this.cameraName = cameraName;
        this.scheduler = scheduler;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        clear();
    }

    private void clear() {
        bestPlate = null;
        bestRed = null;
        bestBlue = null;
        bestDraw = null;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        VidarProcessScheduler.Slot slot = scheduler.beginFrame(captureTimeNanos);
        if (slot != VidarProcessScheduler.Slot.PLATE_SCOUT) {
            return bestPlate;
        }

        clear();
        if (frame == null || frame.empty()) {
            return null;
        }

        VidarFramePipeline.ScaledRoi scaled = VidarFramePipeline.plateScaled(
                frame, profile, VidarConfig.PROCESS_ROI_SCALE);
        if (scaled == null) {
            return null;
        }

        Mat hsv = null;
        try {
            hsv = new Mat();
            Imgproc.cvtColor(scaled.image, hsv, Imgproc.COLOR_RGBA2HSV);

            bestRed = bestForAlliance(hsv, scaled.image, scaled, VidarAlliance.RED,
                    frame.cols(), frame.rows(), captureTimeNanos);
            bestBlue = bestForAlliance(hsv, scaled.image, scaled, VidarAlliance.BLUE,
                    frame.cols(), frame.rows(), captureTimeNanos);
            bestPlate = pickStronger(bestRed, bestBlue);
            if (bestPlate != null) {
                bestDraw = bestPlate.toRotatedRect();
            }
        } finally {
            scaled.release();
            if (hsv != null) hsv.release();
        }

        return bestPlate;
    }

    private VidarPlateObservation bestForAlliance(
            Mat hsv,
            Mat rgba,
            VidarFramePipeline.ScaledRoi scaled,
            VidarAlliance alliance,
            int frameCols,
            int frameRows,
            long captureTimeNanos) {
        Mat mask = null;
        Mat kernel = null;
        try {
            mask = allianceMask(hsv, alliance);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Candidate best = null;
            double bestScore = -1;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < VidarConfig.PLATE_MIN_AREA_PX || area > VidarConfig.PLATE_MAX_AREA_PX) {
                    contour.release();
                    continue;
                }

                RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                contour.release();

                double w = box.size.width;
                double h = box.size.height;
                if (w < 4 || h < 4) {
                    continue;
                }

                double shortSide = Math.min(w, h);
                double longSide = Math.max(w, h);
                double aspect = longSide / shortSide;
                if (aspect < VidarConfig.PLATE_MIN_ASPECT || aspect > VidarConfig.PLATE_MAX_ASPECT) {
                    continue;
                }

                double rectangularity = area / (shortSide * longSide);
                if (rectangularity < VidarConfig.PLATE_MIN_RECTANGULARITY) {
                    continue;
                }

                double whiteRatio = whiteDigitRatio(rgba, box);
                if (whiteRatio < VidarConfig.PLATE_MIN_WHITE_RATIO) {
                    continue;
                }

                double score = area * whiteRatio * rectangularity;
                if (score > bestScore) {
                    bestScore = score;
                    best = new Candidate(alliance, box, whiteRatio, area, rectangularity, aspect);
                }
            }

            if (best == null) {
                return null;
            }

            double absCx = scaled.toFullX(best.box.center.x);
            double absCy = scaled.toFullY(best.box.center.y);
            double fullWidthPx = Math.max(best.box.size.width, best.box.size.height) * scaled.scale;
            double fullHeightPx = Math.min(best.box.size.width, best.box.size.height) * scaled.scale;

            boolean touchesRoi = profile.roiConfig.plateRoi(frameCols, frameRows)
                    .touchesBoundary(frameCols, frameRows, absCx, absCy, 4);
            double rotationPenalty = Math.abs(best.box.angle % 90) / 45.0;
            double partialPenalty = best.rectangularity < 0.65 ? 0.7 : 1.0;
            double viewingPenalty = 1.0 - Math.min(0.5, rotationPenalty * 0.25);

            double dWidth = VidarGeometry.distanceFromWidthInches(
                    profile.plateWidthIn, profile.focalLengthPx, fullWidthPx);
            double dFloor = VidarGeometry.distanceFromFloorInches(
                    (absCy - scaled.sourceCrop.y) / scaled.scale, profile);

            VidarRangeEstimate widthEst = VidarGeometry.buildPlateWidthEstimate(
                    dWidth, fullWidthPx, best.rectangularity, best.whiteRatio,
                    partialPenalty < 1.0, touchesRoi, rotationPenalty);
            VidarRangeEstimate floorEst = VidarGeometry.buildFloorEstimate(
                    dFloor, (absCy - scaled.sourceCrop.y) / scaled.scale,
                    0.5, false);
            VidarRangeResult rangeResult = VidarGeometry.fuseRangeWeighted(widthEst, floorEst);
            double range = rangeResult.isValid() ? rangeResult.distanceIn : Double.NaN;

            double confidence = VidarGeometry.composePlateConfidence(
                    best.whiteRatio, best.contourArea, best.rectangularity, best.aspect,
                    rangeResult, viewingPenalty, partialPenalty);
            double robotX = VidarGeometry.robotXInches(range, profile.bearingDeg, profile);
            double robotY = VidarGeometry.robotYInches(range, profile.bearingDeg, profile);

            return new VidarPlateObservation(
                    alliance,
                    absCx,
                    absCy,
                    fullWidthPx,
                    fullHeightPx,
                    best.box.angle,
                    Math.max(best.box.size.width, best.box.size.height)
                            / Math.min(best.box.size.width, best.box.size.height),
                    best.whiteRatio,
                    range,
                    rangeResult.uncertaintyIn,
                    dWidth,
                    dFloor,
                    rangeResult,
                    viewingPenalty,
                    partialPenalty,
                    confidence,
                    robotX,
                    robotY,
                    cameraName,
                    captureTimeNanos);
        } finally {
            if (mask != null) mask.release();
            if (kernel != null) kernel.release();
        }
    }

    private static Mat allianceMask(Mat hsv, VidarAlliance alliance) {
        Mat mask = new Mat();
        Scalar low;
        Scalar high;
        if (alliance == VidarAlliance.RED) {
            low = new Scalar(VidarConfig.PLATE_RED_H_MIN, VidarConfig.PLATE_S_MIN, VidarConfig.PLATE_V_MIN);
            high = new Scalar(VidarConfig.PLATE_RED_H_MAX, 255, 255);
            Imgproc.inRange(hsv, low, high, mask);
            Mat wrap = new Mat();
            Imgproc.inRange(hsv,
                    new Scalar(VidarConfig.PLATE_RED_WRAP_H_MIN, VidarConfig.PLATE_S_MIN, VidarConfig.PLATE_V_MIN),
                    new Scalar(179, 255, 255),
                    wrap);
            org.opencv.core.Core.bitwise_or(mask, wrap, mask);
            wrap.release();
        } else {
            low = new Scalar(VidarConfig.PLATE_BLUE_H_MIN, VidarConfig.PLATE_S_MIN, VidarConfig.PLATE_V_MIN);
            high = new Scalar(VidarConfig.PLATE_BLUE_H_MAX, 255, 255);
            Imgproc.inRange(hsv, low, high, mask);
        }
        return mask;
    }

    private static double whiteDigitRatio(Mat rgba, RotatedRect box) {
        Point[] corners = new Point[4];
        box.points(corners);

        int samples = 0;
        int white = 0;
        int grid = VidarConfig.PLATE_WHITE_SAMPLE_GRID;

        for (int gy = 1; gy < grid; gy++) {
            for (int gx = 1; gx < grid; gx++) {
                double u = gx / (double) grid;
                double v = gy / (double) grid;
                double x = bilinear(corners, u, v).x;
                double y = bilinear(corners, u, v).y;
                int ix = (int) Math.round(x);
                int iy = (int) Math.round(y);
                if (ix < 0 || iy < 0 || ix >= rgba.cols() || iy >= rgba.rows()) {
                    continue;
                }
                double[] px = rgba.get(iy, ix);
                if (px == null || px.length < 3) {
                    continue;
                }
                samples++;
                int r = (int) px[0];
                int g = (int) px[1];
                int b = (int) px[2];
                int max = Math.max(r, Math.max(g, b));
                int min = Math.min(r, Math.min(g, b));
                if (max >= VidarConfig.PLATE_WHITE_BRIGHT_MIN
                        && max - min <= VidarConfig.PLATE_WHITE_SPREAD_MAX) {
                    white++;
                }
            }
        }

        return samples == 0 ? 0 : (double) white / samples;
    }

    private static Point bilinear(Point[] c, double u, double v) {
        double x = (1 - u) * (1 - v) * c[0].x + u * (1 - v) * c[1].x + u * v * c[2].x + (1 - u) * v * c[3].x;
        double y = (1 - u) * (1 - v) * c[0].y + u * (1 - v) * c[1].y + u * v * c[2].y + (1 - u) * v * c[3].y;
        return new Point(x, y);
    }

    private static double plateConfidence(double whiteRatio, double area, double rangeIn) {
        double whiteScore = Math.min(1.0, whiteRatio / Math.max(0.01, VidarConfig.PLATE_MIN_WHITE_RATIO));
        double areaScore = Math.min(1.0, area / (VidarConfig.PLATE_MIN_AREA_PX * 4.0));
        double rangeScore = Double.isNaN(rangeIn) ? 0.45 : 1.0;
        return 0.5 * whiteScore + 0.35 * areaScore + 0.15 * rangeScore;
    }

    private static VidarPlateObservation pickStronger(VidarPlateObservation a, VidarPlateObservation b) {
        if (a == null) {
            return b;
        }
        if (b == null) {
            return a;
        }
        return a.confidence >= b.confidence ? a : b;
    }

    public VidarPlateObservation getBestPlate() {
        return bestPlate;
    }

    public VidarPlateObservation getBestRed() {
        return bestRed;
    }

    public VidarPlateObservation getBestBlue() {
        return bestBlue;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext) {
        if (bestDraw == null || bestPlate == null) {
            return;
        }

        Paint stroke = new Paint();
        stroke.setStyle(Paint.Style.STROKE);
        stroke.setStrokeWidth(3f * scaleCanvasDensity);
        stroke.setAntiAlias(true);
        stroke.setColor(bestPlate.alliance == VidarAlliance.RED
                ? Color.rgb(255, 90, 90)
                : Color.rgb(90, 140, 255));

        Point[] pts = new Point[4];
        bestDraw.points(pts);
        for (int i = 0; i < 4; i++) {
            float x1 = (float) (pts[i].x * scaleBmpPxToCanvasPx);
            float y1 = (float) (pts[i].y * scaleBmpPxToCanvasPx);
            float x2 = (float) (pts[(i + 1) % 4].x * scaleBmpPxToCanvasPx);
            float y2 = (float) (pts[(i + 1) % 4].y * scaleBmpPxToCanvasPx);
            canvas.drawLine(x1, y1, x2, y2, stroke);
        }

        Paint text = new Paint();
        text.setColor(Color.WHITE);
        text.setTextSize(12f * scaleCanvasDensity);
        text.setAntiAlias(true);
        String label = String.format("%s %.0f%%", bestPlate.alliance.name(), bestPlate.confidence * 100);
        canvas.drawText(label, (float) (bestPlate.cx * scaleBmpPxToCanvasPx),
                (float) (bestPlate.cy * scaleBmpPxToCanvasPx), text);
    }
}
