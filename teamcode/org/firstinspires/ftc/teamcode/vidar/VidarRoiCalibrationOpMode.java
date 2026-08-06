package org.firstinspires.ftc.teamcode.vidar;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Displays per-camera ROIs, calibrated horizon, and full-frame coordinates for field tuning.
 */
@TeleOp(name = "ViDAR ROI Calibrate", group = "ViDAR")
public class VidarRoiCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        VidarVision vision = new VidarVision(hardwareMap);
        VisionPortal portal = vision.getPortal();
        VidarCameraProfile profile = vision.getProfile();
        int frameW = VidarConfig.portalCameraResolution().getWidth();
        int frameH = VidarConfig.portalCameraResolution().getHeight();

        telemetry.addLine("ViDAR ROI calibration — tune VidarCameraProfile / VidarCameraRoiConfig");
        telemetry.addData("Frame", "%d x %d", frameW, frameH);
        telemetry.addData("Element ROI", roiSummary(profile.roiConfig.elementRoi(frameW, frameH)));
        telemetry.addData("Plate ROI", roiSummary(profile.roiConfig.plateRoi(frameW, frameH)));
        telemetry.addData("Tag ROI", roiSummary(profile.roiConfig.tagRoi(frameW, frameH)));
        telemetry.addData("Horizon row (full frame)", profile.horizonRowFullFrame(frameH));
        telemetry.addData("Focal length px", profile.focalLengthPx);
        telemetry.addData("Plate width in", profile.plateWidth);
        telemetry.addData("Element detector", VidarConfig.DEFAULT_ELEMENT_DETECTOR.name());

        for (String warning : profile.validate(frameW, frameH)) {
            telemetry.addLine("WARN: " + warning);
        }

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            vision.update();
            telemetry.addData("FPS", vision.portalFps());
            telemetry.addData("Camera state", vision.directionState().name());
            telemetry.addData("Element", vision.getBestElement() != null ? "yes" : "no");
            telemetry.addData("Metrics", vision.metrics().toTelemetryMap().toString());
            telemetry.update();
        }

        vision.close();
    }

    private static String roiSummary(VidarRoiRect roi) {
        return String.format("x=%d y=%d w=%d h=%d enabled=%s",
                roi.x, roi.y, roi.width, roi.height, roi.enabled);
    }
}
