package org.firstinspires.ftc.teamcode.vidar.geometry;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/**
 * Offline calibration dataset format (JSONL) for future extrinsic refinement.
 *
 * <p>Workflow (planned optimizer): Record Dataset → Extract Scans → Optimize Poses → Validate → Export.
 */
public final class VidarCalibrationDataset {

    public static final String FORMAT_VERSION = "1.0";

    public static final class Record {
        public final String cameraId;
        public final String imageRef;
        public final long timestampNanos;
        public final JSONObject odomPose;
        public final long odomTimestampNanos;
        public final int markerId;
        public final JSONArray markerCorners;
        public final JSONObject markerGeometry;
        public final JSONObject intrinsics;
        public final JSONObject robotTCamera;
        public final String softwareVersion;
        public final String configVersion;

        Record(
                String cameraId,
                String imageRef,
                long timestampNanos,
                JSONObject odomPose,
                long odomTimestampNanos,
                int markerId,
                JSONArray markerCorners,
                JSONObject markerGeometry,
                JSONObject intrinsics,
                JSONObject robotTCamera,
                String softwareVersion,
                String configVersion) {
            this.cameraId = cameraId;
            this.imageRef = imageRef;
            this.timestampNanos = timestampNanos;
            this.odomPose = odomPose;
            this.odomTimestampNanos = odomTimestampNanos;
            this.markerId = markerId;
            this.markerCorners = markerCorners;
            this.markerGeometry = markerGeometry;
            this.intrinsics = intrinsics;
            this.robotTCamera = robotTCamera;
            this.softwareVersion = softwareVersion;
            this.configVersion = configVersion;
        }

        public JSONObject toJson() throws JSONException {
            JSONObject o = new JSONObject();
            o.put("formatVersion", FORMAT_VERSION);
            o.put("cameraId", cameraId);
            o.put("imageRef", imageRef);
            o.put("timestampNanos", timestampNanos);
            o.put("odomPose", odomPose);
            o.put("odomTimestampNanos", odomTimestampNanos);
            o.put("markerId", markerId);
            o.put("markerCorners", markerCorners);
            if (markerGeometry != null) {
                o.put("markerGeometry", markerGeometry);
            }
            o.put("intrinsics", intrinsics);
            o.put("robotTCamera", robotTCamera);
            o.put("softwareVersion", softwareVersion);
            o.put("configVersion", configVersion);
            return o;
        }
    }

    private VidarCalibrationDataset() {}

    public static List<String> validateRecordJson(String line) {
        List<String> errors = new ArrayList<>();
        if (line == null || line.trim().isEmpty()) {
            errors.add("empty line");
            return errors;
        }
        try {
            JSONObject o = new JSONObject(line.trim());
            requireString(o, "formatVersion", errors);
            requireString(o, "cameraId", errors);
            if (!o.has("timestampNanos")) {
                errors.add("missing timestampNanos");
            }
            if (!o.has("intrinsics")) {
                errors.add("missing intrinsics");
            } else {
                JSONObject intr = o.getJSONObject("intrinsics");
                requirePositive(intr, "fx", errors);
                requirePositive(intr, "fy", errors);
                requirePositive(intr, "imageWidth", errors);
                requirePositive(intr, "imageHeight", errors);
            }
            if (!o.has("robotTCamera")) {
                errors.add("missing robotTCamera");
            }
            if (o.has("markerId") && o.has("markerCorners")) {
                JSONArray corners = o.getJSONArray("markerCorners");
                if (corners.length() != 4) {
                    errors.add("markerCorners must have 4 entries");
                }
            }
        } catch (JSONException ex) {
            errors.add("invalid json: " + ex.getMessage());
        }
        return errors;
    }

    private static void requireString(JSONObject o, String key, List<String> errors) {
        if (!o.has(key) || o.optString(key, "").isEmpty()) {
            errors.add("missing " + key);
        }
    }

    private static void requirePositive(JSONObject o, String key, List<String> errors)
            throws JSONException {
        if (!o.has(key) || o.getDouble(key) <= 0) {
            errors.add("invalid " + key);
        }
    }
}
