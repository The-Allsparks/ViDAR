package org.firstinspires.ftc.teamcode.vidar.model;

import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedPoint;
/**
 * Robot-frame density grid of corrected element detections — peaks indicate cluster targets.
 */
public final class VidarElementDensityMap {

    public static final class Peak {
        public final double robotX;
        public final double robotY;
        public final double density;
        public final int cellCol;
        public final int cellRow;

        Peak(double robotX, double robotY, double density, int cellCol, int cellRow) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.density = density;
            this.cellCol = cellCol;
            this.cellRow = cellRow;
        }

        public double bearingDeg() {
            return Math.toDegrees(Math.atan2(robotY, robotX));
        }

        public double range() {
            return Math.hypot(robotX, robotY);
        }
    }

    private final double cellSizeIn;
    private final double minXIn;
    private final double minYIn;
    private final int cols;
    private final int rows;
    private final double[] grid;

    public VidarElementDensityMap(
            double cellSizeIn,
            double minXIn,
            double maxXIn,
            double minYIn,
            double maxYIn) {
        this.cellSizeIn = Math.max(1, cellSizeIn);
        this.minXIn = minXIn;
        this.minYIn = minYIn;
        this.cols = Math.max(1, (int) Math.ceil((maxXIn - minXIn) / this.cellSizeIn));
        this.rows = Math.max(1, (int) Math.ceil((maxYIn - minYIn) / this.cellSizeIn));
        this.grid = new double[cols * rows];
    }

    public static VidarElementDensityMap defaultRobotGrid() {
        return new VidarElementDensityMap(
                VidarConfig.DENSITY_CELL_SIZE_IN,
                0,
                VidarConfig.DENSITY_FORWARD_MAX_IN,
                -VidarConfig.DENSITY_LATERAL_MAX_IN,
                VidarConfig.DENSITY_LATERAL_MAX_IN);
    }

    public void splat(VidarCorrectedPoint point) {
        if (point == null || point.kind != VidarCorrectedPoint.Kind.ELEMENT) {
            return;
        }
        splat(point.robotX, point.robotY, point.confidence);
    }

    public void splat(double robotX, double robotY, double weight) {
        if (Double.isNaN(robotX) || Double.isNaN(robotY) || weight <= 0) {
            return;
        }
        double sigma = VidarConfig.DENSITY_SPLAT_SIGMA_IN;
        double sigma2 = sigma * sigma;
        int centerCol = (int) Math.floor((robotX - minXIn) / cellSizeIn);
        int centerRow = (int) Math.floor((robotY - minYIn) / cellSizeIn);
        int radius = Math.max(1, (int) Math.ceil(3 * sigma / cellSizeIn));
        for (int dr = -radius; dr <= radius; dr++) {
            for (int dc = -radius; dc <= radius; dc++) {
                int col = centerCol + dc;
                int row = centerRow + dr;
                if (col < 0 || col >= cols || row < 0 || row >= rows) {
                    continue;
                }
                double cx = minXIn + (col + 0.5) * cellSizeIn;
                double cy = minYIn + (row + 0.5) * cellSizeIn;
                double d2 = (cx - robotX) * (cx - robotX) + (cy - robotY) * (cy - robotY);
                grid[row * cols + col] += weight * Math.exp(-0.5 * d2 / sigma2);
            }
        }
    }

    public Peak peak() {
        int bestIndex = -1;
        double best = -1;
        for (int i = 0; i < grid.length; i++) {
            if (grid[i] > best) {
                best = grid[i];
                bestIndex = i;
            }
        }
        if (bestIndex < 0 || best <= 0) {
            return null;
        }
        int row = bestIndex / cols;
        int col = bestIndex % cols;
        double x = minXIn + (col + 0.5) * cellSizeIn;
        double y = minYIn + (row + 0.5) * cellSizeIn;
        return new Peak(x, y, best, col, row);
    }

    public double densityAt(double robotX, double robotY) {
        int col = (int) Math.floor((robotX - minXIn) / cellSizeIn);
        int row = (int) Math.floor((robotY - minYIn) / cellSizeIn);
        if (col < 0 || col >= cols || row < 0 || row >= rows) {
            return 0;
        }
        return grid[row * cols + col];
    }

    public int cols() {
        return cols;
    }

    public int rows() {
        return rows;
    }

    public double cellSizeIn() {
        return cellSizeIn;
    }

    /** Raw density grid row-major (for telemetry/debug). */
    public double[] gridCopy() {
        double[] copy = new double[grid.length];
        System.arraycopy(grid, 0, copy, 0, grid.length);
        return copy;
    }
}
