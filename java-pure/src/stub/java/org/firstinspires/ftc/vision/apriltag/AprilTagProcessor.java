package org.firstinspires.ftc.vision.apriltag;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

/** Minimal stub so crop-decoder unit tests compile without EasyOpenCV. */
public class AprilTagProcessor {

    public static class Builder {
        public Builder setLensIntrinsics(double fx, double fy, double cx, double cy) {
            return this;
        }

        public Builder setTagLibrary(AprilTagLibrary tagLibrary) {
            return this;
        }

        public AprilTagProcessor build() {
            return new AprilTagProcessor();
        }
    }

    public void setDecimation(float decimation) {
    }

    public Object processFrame(Mat input, long captureTimeNanos) {
        return null;
    }

    public List<AprilTagDetection> getDetections() {
        return new ArrayList<AprilTagDetection>();
    }

    public List<AprilTagDetection> getFreshDetections() {
        return new ArrayList<AprilTagDetection>();
    }
}
