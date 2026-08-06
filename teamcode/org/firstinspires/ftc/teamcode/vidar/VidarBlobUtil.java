package org.firstinspires.ftc.teamcode.vidar;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

/**
 * Shared helpers for logging and geometry — keeps OpModes short for teaching.
 */
public final class VidarBlobUtil {

    private VidarBlobUtil() {}

    public static RotatedRect centerBox(ColorBlobLocatorProcessor.Blob blob) {
        return blob == null ? null : blob.getBoxFit();
    }

    public static double centerX(ColorBlobLocatorProcessor.Blob blob) {
        RotatedRect box = centerBox(blob);
        return box == null ? Double.NaN : box.center.x;
    }

    public static double centerY(ColorBlobLocatorProcessor.Blob blob) {
        RotatedRect box = centerBox(blob);
        return box == null ? Double.NaN : box.center.y;
    }

    public static double centerX(VidarElementObservation element) {
        return element == null ? Double.NaN : element.cx;
    }

    public static double centerY(VidarElementObservation element) {
        return element == null ? Double.NaN : element.cy;
    }

    /** Horizontal error from frame center (positive = target is to the right). */
    public static double errorFromCenter(ColorBlobLocatorProcessor.Blob blob, double frameWidth) {
        double cx = centerX(blob);
        if (Double.isNaN(cx)) {
            return 0;
        }
        return cx - (frameWidth / 2.0);
    }

    public static double errorFromCenter(VidarPlateObservation plate, double frameWidth) {
        if (plate == null) {
            return 0;
        }
        return plate.cx - (frameWidth / 2.0);
    }

    public static double errorFromCenter(VidarElementObservation element, double frameWidth) {
        double cx = centerX(element);
        if (Double.isNaN(cx)) {
            return 0;
        }
        return cx - (frameWidth / 2.0);
    }

    public static String formatPlate(VidarPlateObservation plate) {
        return formatPlate(plate, VidarConfig.DEFAULT_ALLIANCE);
    }

    public static String formatPlate(VidarPlateObservation plate, VidarAlliance ourAlliance) {
        if (plate == null) {
            return "none";
        }
        String role = plate.isFoe(ourAlliance) ? "FOE" : (plate.isAlly(ourAlliance) ? "ALLY" : plate.alliance.name());
        return String.format(
                "%s (%s) (%.0f, %.0f) white=%.0f%% conf=%.0f%%",
                plate.alliance.name(),
                role,
                plate.cx,
                plate.cy,
                plate.whiteRatio * 100,
                plate.confidence * 100);
    }

    public static String formatPlateDetail(VidarPlateObservation plate) {
        if (plate == null) {
            return "none";
        }
        return String.format(
                "width=%s floor=%s ground=%s in · fused=%s · robot (%s, %s) · cam=%s · aspect=%.2f",
                fmtIn(plate.sizeBasedRange),
                fmtIn(plate.floorBasedRange),
                fmtIn(plate.groundBasedRange),
                fmtIn(plate.range),
                fmtIn(plate.robotX),
                fmtIn(plate.robotY),
                plate.cameraName,
                plate.aspectRatio);
    }

    public static String formatWorldTrack(VidarWorldModel.Track track) {
        if (track == null) {
            return "none";
        }
        return String.format(
                "%s @ (%.0f, %.0f) in · %.0f° · conf=%.0f%% · age=%.1fs",
                track.kind.name(),
                track.robotX,
                track.robotY,
                track.bearingDeg(),
                track.confidence * 100,
                0.0);
    }

    public static String formatBlob(ColorBlobLocatorProcessor.Blob blob) {
        RotatedRect box = centerBox(blob);
        if (box == null) {
            return "none";
        }
        if (blob.getCircularity() > 0) {
            return String.format("(%.0f, %.0f) area=%.0f circ=%.2f",
                    box.center.x, box.center.y, blob.getContourArea(), blob.getCircularity());
        }
        return String.format("(%.0f, %.0f) area=%.0f",
                box.center.x, box.center.y, blob.getContourArea());
    }

    public static String formatElement(VidarElementObservation element) {
        if (element == null) {
            return "none";
        }
        if (Double.isNaN(element.range)) {
            return String.format("(%.0f, %.0f) r=%.0f conf=%.0f%%",
                    element.cx, element.cy, element.radiusPx, element.confidence * 100);
        }
        return String.format("(%.0f, %.0f) r=%.0f range=%.1f in conf=%.0f%%",
                element.cx, element.cy, element.radiusPx, element.range, element.confidence * 100);
    }

    public static String formatElementDetail(VidarElementObservation element) {
        if (element == null) {
            return "none";
        }
        String fused = element.rangeResult != null
                ? String.format(" n=%d", element.rangeResult.sourceCount) : "";
        return String.format(
                "size=%s floor=%s ground=%s in · fused=%s%s · robot (%s, %s) · votes=%d",
                fmtIn(element.dSize),
                fmtIn(element.dFloor),
                fmtIn(element.dGround),
                fmtIn(element.range),
                fused,
                fmtIn(element.robotX),
                fmtIn(element.robotY),
                element.houghVotes);
    }

    public static String formatCalibrationDiagnostics(
            java.util.Map<String, Object> diagnostics) {
        if (diagnostics == null || diagnostics.isEmpty()) {
            return "—";
        }
        StringBuilder sb = new StringBuilder();
        for (java.util.Map.Entry<String, Object> entry : diagnostics.entrySet()) {
            if (sb.length() > 0) {
                sb.append(" · ");
            }
            sb.append(entry.getKey()).append('=').append(entry.getValue());
        }
        return sb.toString();
    }

    private static String fmtIn(double v) {
        return Double.isNaN(v) ? "—" : String.format("%.1f", v);
    }

    public static String formatTag(VidarTagObservation tag) {
        if (tag == null) {
            return "none";
        }
        return String.format(
                "id=%d band=%s dec=%d %dpx age=%.2fs",
                tag.tagId,
                tag.band.name(),
                tag.decimationUsed,
                tag.decodePixels,
                VidarPoseBackdate.ageSeconds(tag));
    }

    public static String formatTagPose(VidarTagObservation tag) {
        if (tag == null || tag.fieldPoseAtCapture == null) {
            return "none";
        }
        return String.format(
                "capture (%.1f, %.1f) %.0f°",
                tag.fieldPoseAtCapture.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                tag.fieldPoseAtCapture.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                tag.fieldPoseAtCapture.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
    }

    public static String formatScoutObservation(VidarTagScoutObservation scout) {
        if (scout == null) {
            return "none (non-localizing)";
        }
        return String.format(
                "bearing=%.0f° w=%.0fpx conf=%.0f%% cam=%s (does not alter pose)",
                scout.bearingDeg,
                scout.apparentWidthPx,
                scout.scoutConfidence * 100,
                scout.cameraName);
    }

    /** Forward drive power from fused range (0 when at pickup distance). */
    public static double rangeDrivePower(VidarElementObservation element) {
        if (element == null || Double.isNaN(element.range)) {
            return 0;
        }
        if (element.range <= VidarConfig.PICKUP_STOP) {
            return 0;
        }
        if (element.range > VidarConfig.SEEK_MAX_RANGE_IN) {
            return 0;
        }
        double raw = (element.range - VidarConfig.PICKUP_STOP) * VidarConfig.RANGE_DRIVE_GAIN;
        return Math.min(VidarConfig.SEEK_DRIVE_POWER, raw);
    }
}
