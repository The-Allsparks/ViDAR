package org.firstinspires.ftc.teamcode.vidar.geometry;

/**
 * Maps processed (cropped/scaled) pixels to full calibrated sensor pixels.
 *
 * <p>ViDAR uses pixel-center convention: processed pixel {@code (u, v)} maps to sensor pixel
 * {@code ((u + 0.5) * scaleX + cropX - 0.5, …)} before {@link VidarCameraIntrinsics#pixelToRay}.
 */
public final class VidarImageTransform {

    public final int cropX;
    public final int cropY;
    public final int sourceWidth;
    public final int sourceHeight;
    public final int processedWidth;
    public final int processedHeight;
    public final double scaleX;
    public final double scaleY;

    public VidarImageTransform(
            int cropX,
            int cropY,
            int sourceWidth,
            int sourceHeight,
            int processedWidth,
            int processedHeight,
            double scaleX,
            double scaleY) {
        this.cropX = cropX;
        this.cropY = cropY;
        this.sourceWidth = sourceWidth;
        this.sourceHeight = sourceHeight;
        this.processedWidth = processedWidth;
        this.processedHeight = processedHeight;
        this.scaleX = scaleX;
        this.scaleY = scaleY;
    }

    /** Identity mapping when processing uses the full calibrated frame at 1:1 scale. */
    public static VidarImageTransform identity(int width, int height) {
        return new VidarImageTransform(0, 0, width, height, width, height, 1.0, 1.0);
    }

    /** Uniform scale from a crop rect (typical ViDAR ROI downscale). */
    public static VidarImageTransform fromCropAndScale(
            int cropX,
            int cropY,
            int sourceWidth,
            int sourceHeight,
            int processedWidth,
            int processedHeight) {
        if (processedWidth <= 0 || processedHeight <= 0) {
            return null;
        }
        double scaleX = (double) sourceWidth / processedWidth;
        double scaleY = (double) sourceHeight / processedHeight;
        return new VidarImageTransform(
                cropX, cropY, sourceWidth, sourceHeight,
                processedWidth, processedHeight, scaleX, scaleY);
    }

    public boolean isUniformScale() {
        return Math.abs(scaleX - scaleY) < 1e-6;
    }

    public boolean isValid() {
        return sourceWidth > 0 && sourceHeight > 0
                && processedWidth > 0 && processedHeight > 0
                && scaleX > 0 && scaleY > 0
                && cropX >= 0 && cropY >= 0
                && cropX + sourceWidth <= calibrationFrameWidth()
                && cropY + sourceHeight <= calibrationFrameHeight();
    }

    /** Full sensor width implied by crop + source region (requires caller context for absolute size). */
    public int calibrationFrameWidth() {
        return cropX + sourceWidth;
    }

    public int calibrationFrameHeight() {
        return cropY + sourceHeight;
    }

    public double toSensorX(double processedX) {
        return (processedX + 0.5) * scaleX + cropX - 0.5;
    }

    public double toSensorY(double processedY) {
        return (processedY + 0.5) * scaleY + cropY - 0.5;
    }

    public double toProcessedX(double sensorX) {
        return (sensorX + 0.5 - cropX) / scaleX - 0.5;
    }

    public double toProcessedY(double sensorY) {
        return (sensorY + 0.5 - cropY) / scaleY - 0.5;
    }

    /** Map principal point from calibration frame into processed coordinates. */
    public double[] mapPrincipalPoint(double cx, double cy) {
        return new double[] { toProcessedX(cx), toProcessedY(cy) };
    }
}
