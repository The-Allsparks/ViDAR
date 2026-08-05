package org.firstinspires.ftc.teamcode.vidar;



import android.graphics.Canvas;

import android.graphics.Color;

import android.graphics.Paint;



import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Mat;

import org.opencv.core.Rect;

import org.opencv.core.Size;

import org.opencv.imgproc.Imgproc;



import java.util.ArrayList;

import java.util.List;



/**

 * @deprecated Use {@link VidarBallProcessor} with {@link VidarBallDetectorType#LEGACY_HOUGH}.

 * Hough-first detection replaced by color-blob pipeline.

 */

@Deprecated

public class VidarHoughBallProcessor implements VisionProcessor {



    private static class Candidate {

        final double cx;

        final double cy;

        final double radius;

        final int votes;



        Candidate(double cx, double cy, double radius, int votes) {

            this.cx = cx;

            this.cy = cy;

            this.radius = radius;

            this.votes = votes;

        }

    }



    private final VidarCameraProfile profile;

    private final String cameraName;

    private final VidarProcessScheduler scheduler;

    private VidarBallObservation bestBall;

    private Candidate bestDraw;



    public VidarHoughBallProcessor(

            VidarCameraProfile profile,

            String cameraName,

            VidarProcessScheduler scheduler) {

        this.profile = profile;

        this.cameraName = cameraName;

        this.scheduler = scheduler;

    }



    @Override

    public void init(int width, int height, CameraCalibration calibration) {

        bestBall = null;

        bestDraw = null;

    }



    @Override

    public Object processFrame(Mat frame, long captureTimeNanos) {

        VidarProcessScheduler.Slot slot = scheduler.beginFrame(captureTimeNanos);

        if (slot != VidarProcessScheduler.Slot.BALL) {

            return bestBall;

        }



        bestBall = null;

        bestDraw = null;



        if (frame == null || frame.empty()) {

            return null;

        }



        VidarFramePipeline.ScaledRoi scaled = VidarFramePipeline.bottomHalfScaled(

                frame, VidarConfig.PROCESS_ROI_SCALE);

        if (scaled == null) {

            return null;

        }



        Mat gray = null;

        Mat circles = null;



        try {

            gray = new Mat();

            Imgproc.cvtColor(scaled.image, gray, Imgproc.COLOR_RGBA2GRAY);

            Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);



            circles = new Mat();

            Imgproc.HoughCircles(

                    gray,

                    circles,

                    Imgproc.HOUGH_GRADIENT,

                    VidarConfig.HOUGH_DP,

                    VidarConfig.HOUGH_MIN_DIST,

                    VidarConfig.HOUGH_PARAM1,

                    VidarConfig.HOUGH_PARAM2,

                    VidarConfig.HOUGH_MIN_RADIUS,

                    VidarConfig.HOUGH_MAX_RADIUS);



            if (circles.empty()) {

                return null;

            }



            List<Candidate> candidates = collectCandidates(circles, scaled.image, scaled);

            if (candidates.isEmpty()) {

                return null;

            }



            double frameH = scaled.image.rows();

            VidarBallObservation bestObs = null;

            Candidate bestCand = null;

            double bestScore = -1;



            for (Candidate c : candidates) {

                double absCx = c.cx;

                double absCy = c.cy;



                VidarBallObservation obs = VidarGeometry.fuseObservation(

                        absCx, absCy, c.radius, c.votes, profile, cameraName);

                if (obs.confidence < VidarConfig.MIN_BALL_CONFIDENCE) {

                    continue;

                }



                double localCy = (absCy - scaled.sourceCrop.y) / scaled.scale;

                double floorWeight = 0.25 + 0.75 * (localCy / frameH);

                double score = obs.confidence * obs.radiusPx * obs.radiusPx

                        * floorWeight * floorWeight;

                if (score > bestScore) {

                    bestScore = score;

                    bestObs = obs;

                    bestCand = c;

                }

            }



            bestBall = bestObs;

            if (bestObs != null && bestCand != null) {

                bestDraw = bestCand;

            }

        } finally {

            scaled.release();

            if (gray != null) gray.release();

            if (circles != null) circles.release();

        }



        return bestBall;

    }



    private List<Candidate> collectCandidates(
            Mat circles, Mat rgba, VidarFramePipeline.ScaledRoi scaled) {
        List<Candidate> out = new ArrayList<>();
        int count = circles.cols();
        double mapScale = scaled.scale;

        for (int i = 0; i < count; i++) {
            double[] data = circles.get(0, i);
            if (data == null || data.length < 3) {
                continue;
            }

            double cx = scaled.toFullX(data[0]);
            double cy = scaled.toFullY(data[1]);
            double r = data[2] * mapScale;

            int votes = data.length > 3 ? (int) Math.round(data[3]) : 0;



            if (r < VidarConfig.HOUGH_MIN_RADIUS || r > VidarConfig.HOUGH_MAX_RADIUS) {

                continue;

            }



            if (votes > 0) {

                double minVotes = VidarConfig.HOUGH_PARAM2 + r * VidarConfig.HOUGH_MIN_VOTES_SCALE;

                if (votes < minVotes) {

                    continue;

                }

            }



            double area = Math.PI * r * r;

            if (area < VidarConfig.MIN_BALL_AREA_PX) {

                continue;

            }



            double interior = interiorScore(rgba, data[0], data[1], data[2]);

            if (interior < VidarConfig.HOUGH_MIN_INTERIOR) {

                continue;

            }



            out.add(new Candidate(cx, cy, r, votes));

        }

        return out;

    }



    private static double interiorScore(Mat rgba, double cx, double cy, double radius) {

        int ri = (int) Math.floor(radius);

        int icx = (int) Math.round(cx);

        int icy = (int) Math.round(cy);

        double r2 = radius * radius;



        int inside = 0;

        int bright = 0;



        for (int dy = -ri; dy <= ri; dy++) {

            for (int dx = -ri; dx <= ri; dx++) {

                if (dx * dx + dy * dy > r2) {

                    continue;

                }

                int x = icx + dx;

                int y = icy + dy;

                if (x < 0 || y < 0 || x >= rgba.cols() || y >= rgba.rows()) {

                    continue;

                }

                double[] px = rgba.get(y, x);

                if (px == null || px.length < 3) {

                    continue;

                }

                inside++;

                int r = (int) px[0];

                int g = (int) px[1];

                int b = (int) px[2];

                int max = Math.max(r, Math.max(g, b));

                int min = Math.min(r, Math.min(g, b));

                if (max >= VidarConfig.HOUGH_INTERIOR_BRIGHT

                        && max - min <= VidarConfig.HOUGH_INTERIOR_SPREAD) {

                    bright++;

                }

            }

        }



        return inside == 0 ? 0 : (double) bright / inside;

    }



    public VidarBallObservation getBestBall() {

        return bestBall;

    }



    @Override

    public void onDrawFrame(

            Canvas canvas,

            int onscreenWidth,

            int onscreenHeight,

            float scaleBmpPxToCanvasPx,

            float scaleCanvasDensity,

            Object userContext) {

        if (bestDraw == null) {

            return;

        }



        float cx = (float) (bestDraw.cx * scaleBmpPxToCanvasPx);

        float cy = (float) (bestDraw.cy * scaleBmpPxToCanvasPx);

        float r = (float) (bestDraw.radius * scaleBmpPxToCanvasPx);



        Paint stroke = new Paint();

        stroke.setColor(Color.rgb(220, 220, 240));

        stroke.setStyle(Paint.Style.STROKE);

        stroke.setStrokeWidth(3f * scaleCanvasDensity);

        stroke.setAntiAlias(true);

        canvas.drawCircle(cx, cy, r, stroke);



        Paint fill = new Paint();

        fill.setColor(Color.rgb(120, 255, 180));

        fill.setStyle(Paint.Style.FILL);

        fill.setAntiAlias(true);

        canvas.drawCircle(cx, cy, 4f * scaleCanvasDensity, fill);



        if (bestBall != null && !Double.isNaN(bestBall.rangeIn)) {

            Paint text = new Paint();

            text.setColor(Color.WHITE);

            text.setTextSize(14f * scaleCanvasDensity);

            text.setAntiAlias(true);

            String label = String.format("%.0f in (%.0f%%)", bestBall.rangeIn, bestBall.confidence * 100);

            canvas.drawText(label, cx + r + 4f, cy, text);

        }

    }

}


