package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.firstinspires.ftc.teamcode.vidar.VidarTransformRegistry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Low-rate calibration and transform diagnostics — not updated every vision frame.
 */
public final class VidarCalibrationDiagnostics {

    private String activeProfileName = "";
    private int calibrationWidth;
    private int calibrationHeight;
    private String calibrationVersion = "";
    private double lastObservationAgeMs = Double.NaN;
    private int transformLookupFailures;
    private int invalidGroundIntersections;
    private int staleRejections;
    private double lastReprojectionErrorPx = Double.NaN;
    private double lastCrossCameraDisagreementIn = Double.NaN;
    private String lastRobotTCameraNotation = "";

    public void updateFromRegistry(VidarTransformRegistry registry, int activeIndex) {
        if (registry == null) {
            return;
        }
        VidarTransformRegistry.CameraTransforms ct = registry.forIndex(activeIndex);
        if (ct == null) {
            return;
        }
        activeProfileName = ct.cameraName;
        calibrationWidth = ct.intrinsics.imageWidth;
        calibrationHeight = ct.intrinsics.imageHeight;
        calibrationVersion = ct.intrinsics.calibrationVersion != null
                ? ct.intrinsics.calibrationVersion : "";
        lastRobotTCameraNotation = ct.robotTCamera.notationName();
    }

    public void recordObservationAge(double ageMs) {
        lastObservationAgeMs = ageMs;
    }

    public void incrementTransformLookupFailures() {
        transformLookupFailures++;
    }

    public void incrementInvalidGroundIntersections() {
        invalidGroundIntersections++;
    }

    public void incrementStaleRejections() {
        staleRejections++;
    }

    public void recordReprojectionError(double px) {
        lastReprojectionErrorPx = px;
    }

    public void recordCrossCameraDisagreement(double inches) {
        lastCrossCameraDisagreementIn = inches;
    }

    public Map<String, Object> toTelemetryMap() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("calProfile", activeProfileName);
        m.put("calRes", calibrationWidth + "x" + calibrationHeight);
        m.put("calVer", calibrationVersion);
        m.put("robot_T_cam", lastRobotTCameraNotation);
        if (Double.isFinite(lastObservationAgeMs)) {
            m.put("obsAgeMs", lastObservationAgeMs);
        }
        if (transformLookupFailures > 0) {
            m.put("tfLookupFail", transformLookupFailures);
        }
        if (invalidGroundIntersections > 0) {
            m.put("groundFail", invalidGroundIntersections);
        }
        if (staleRejections > 0) {
            m.put("staleReject", staleRejections);
        }
        if (Double.isFinite(lastReprojectionErrorPx)) {
            m.put("reprojPx", lastReprojectionErrorPx);
        }
        if (Double.isFinite(lastCrossCameraDisagreementIn)) {
            m.put("camDisagreeIn", lastCrossCameraDisagreementIn);
        }
        return m;
    }
}
