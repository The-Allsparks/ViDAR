package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.VidarElementDetectorType;
import org.firstinspires.ftc.teamcode.vidar.VidarElementShape;
import org.firstinspires.ftc.teamcode.vidar.VidarCameraMount;
import org.firstinspires.ftc.teamcode.vidar.VidarCameraProfile;
import org.firstinspires.ftc.teamcode.vidar.VidarCameraRoiConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

/**
 * Loads ViDAR season and robot JSON configs.
 *
 * <p>Teams place JSON under {@code TeamCode/src/main/assets/vidar/} and load at OpMode init:
 *
 * <pre>{@code
 * VidarSeasonConfig season = VidarConfigLoader.loadSeason(
 *         VidarConfigLoader.readAsset(hardwareMap.appContext, "vidar/season.json"));
 * VidarRobotConfig robot = VidarConfigLoader.loadRobot(
 *         VidarConfigLoader.readAsset(hardwareMap.appContext, "vidar/robot.json"));
 * VidarMultiVision vision = new VidarMultiVision(hardwareMap, robot, season, odom, alliance);
 * }</pre>
 */
public final class VidarConfigLoader {

    private VidarConfigLoader() {}

    public static String readAsset(Object appContext, String assetPath) throws IOException {
        android.content.Context context = (android.content.Context) appContext;
        try (InputStream stream = context.getAssets().open(assetPath);
             BufferedReader reader = new BufferedReader(new InputStreamReader(stream, StandardCharsets.UTF_8))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append('\n');
            }
            return sb.toString();
        }
    }

    public static VidarSeasonConfig loadSeason(String json) {
        try {
            return parseSeason(new JSONObject(json));
        } catch (JSONException ex) {
            throw new IllegalArgumentException("Invalid season JSON", ex);
        }
    }

    public static VidarRobotConfig loadRobot(String json) {
        try {
            return parseRobot(new JSONObject(json));
        } catch (JSONException ex) {
            throw new IllegalArgumentException("Invalid robot JSON", ex);
        }
    }

    /** Built-in season matching legacy {@link VidarConfig} constants. */
    public static VidarSeasonConfig defaultSeason() {
        return loadSeason(defaultSeasonJson());
    }

    /** Built-in robot layout matching legacy {@link VidarConfig} + {@link VidarCameraProfile#FOUR_SIDES}. */
    public static VidarRobotConfig defaultRobot() {
        return loadRobot(defaultRobotJson());
    }

    static VidarSeasonConfig parseSeason(JSONObject root) throws JSONException {
        JSONObject fusion = root.optJSONObject("fusion");
        VidarElementSpec[] elements = parseElements(root);
        VidarPlateSpec[] plates = parsePlates(root.getJSONArray("plates"));
        VidarFieldSpec field = parseField(root);
        double defaultTagSize = parseDefaultTagSize(root);
        VidarAprilTagSpec[] tags = parseAprilTags(root, defaultTagSize);
        return new VidarSeasonConfig(
                root.getString("seasonId"),
                root.optString("seasonName", root.getString("seasonId")),
                field,
                elements,
                plates,
                tags,
                defaultTagSize,
                parseMinElementConfidence(fusion),
                fusion != null ? fusion.optDouble("minPlateConfidence", 0.35) : 0.35,
                fusion != null ? fusion.optDouble("maxRangeMismatchRatio", 0.28) : 0.28,
                parseDistanceUnit(root));
    }

    private static VidarDistanceUnit parseDistanceUnit(JSONObject root) {
        if (!root.has("distanceUnit")) {
            return VidarDistanceUnit.IN;
        }
        return VidarDistanceUnit.fromJson(root.optString("distanceUnit", "in"));
    }

    private static VidarDistanceUnit parseOptionalDistanceUnit(JSONObject root) {
        if (!root.has("distanceUnit")) {
            return null;
        }
        return VidarDistanceUnit.fromJson(root.optString("distanceUnit", "in"));
    }

    private static double parseMinElementConfidence(JSONObject fusion) {
        if (fusion == null) {
            return 0.35;
        }
        return fusion.optDouble("minElementConfidence", 0.35);
    }

    private static VidarElementSpec[] parseElements(JSONObject root) throws JSONException {
        JSONArray array = root.getJSONArray("elements");
        return parseElements(array);
    }

    private static double readDist(JSONObject o, String primaryKey, String... legacyKeys)
            throws JSONException {
        if (o.has(primaryKey)) {
            return o.getDouble(primaryKey);
        }
        for (String key : legacyKeys) {
            if (key != null && o.has(key)) {
                return o.getDouble(key);
            }
        }
        throw new JSONException("Missing distance key: " + primaryKey);
    }

    private static double readDist(JSONObject o, String primaryKey, double defaultVal, String... legacyKeys) {
        if (o.has(primaryKey)) {
            return o.optDouble(primaryKey, defaultVal);
        }
        for (String key : legacyKeys) {
            if (key != null && o.has(key)) {
                return o.optDouble(key, defaultVal);
            }
        }
        return defaultVal;
    }

    private static VidarFieldSpec parseField(JSONObject root) throws JSONException {
        JSONObject field = root.optJSONObject("field");
        if (field == null) {
            return new VidarFieldSpec(691.2, 317.0);
        }
        return new VidarFieldSpec(
                readDist(field, "length", 691.2, "lengthDist", "lengthIn"),
                readDist(field, "width", 317.0, "widthDist", "widthIn"));
    }

    private static double parseDefaultTagSize(JSONObject root) throws JSONException {
        JSONObject block = root.optJSONObject("apriltags");
        if (block != null && block.has("defaultSize")) {
            return block.getDouble("defaultSize");
        }
        if (block != null && block.has("defaultSizeDist")) {
            return block.getDouble("defaultSizeDist");
        }
        if (block != null && block.has("defaultSizeIn")) {
            return block.getDouble("defaultSizeIn");
        }
        if (root.has("defaultTagSize")) {
            return root.getDouble("defaultTagSize");
        }
        if (root.has("defaultTagSizeDist")) {
            return root.getDouble("defaultTagSizeDist");
        }
        return root.optDouble("defaultTagSizeIn", 8.125);
    }

    private static VidarAprilTagSpec[] parseAprilTags(JSONObject root, double defaultSize)
            throws JSONException {
        JSONArray array = null;
        JSONObject block = root.optJSONObject("apriltags");
        if (block != null && block.has("tags")) {
            array = block.getJSONArray("tags");
        } else if (root.has("apriltags") && root.get("apriltags") instanceof JSONArray) {
            array = root.getJSONArray("apriltags");
        }
        if (array == null) {
            return new VidarAprilTagSpec[0];
        }
        VidarAprilTagSpec[] out = new VidarAprilTagSpec[array.length()];
        for (int i = 0; i < array.length(); i++) {
            out[i] = parseAprilTag(array.getJSONObject(i), defaultSize);
        }
        return out;
    }

    private static VidarAprilTagSpec parseAprilTag(JSONObject tag, double defaultSize)
            throws JSONException {
        JSONObject position = tag.optJSONObject("position");
        if (position == null) {
            position = tag.optJSONObject("positionIn");
        }
        JSONObject orientation = tag.optJSONObject("orientationDeg");
        double x = readTagCoord(tag, position, "x");
        double y = readTagCoord(tag, position, "y");
        double z = readTagCoord(tag, position, "z");
        return new VidarAprilTagSpec(
                tag.getInt("id"),
                tag.optString("name", "tag_" + tag.getInt("id")),
                tag.optDouble("size", tag.optDouble("sizeIn", defaultSize)),
                x, y, z,
                readTagOrientation(tag, orientation, "yaw"),
                readTagOrientation(tag, orientation, "pitch"),
                readTagOrientation(tag, orientation, "roll"),
                tag.optBoolean("localization", true));
    }

    private static double readTagCoord(JSONObject tag, JSONObject position, String axis)
            throws JSONException {
        if (position != null && position.has(axis)) {
            return position.getDouble(axis);
        }
        String flatDist = axis + "Dist";
        String flatIn = axis + "In";
        if (tag.has(flatDist)) {
            return tag.getDouble(flatDist);
        }
        if (tag.has(flatIn)) {
            return tag.getDouble(flatIn);
        }
        return Double.NaN;
    }

    private static double readTagOrientation(JSONObject tag, JSONObject orientation, String name) {
        if (orientation != null && orientation.has(name)) {
            return orientation.optDouble(name, 0);
        }
        String flatKey = name + "Deg";
        return tag.optDouble(flatKey, 0);
    }

    static VidarRobotConfig parseRobot(JSONObject root) throws JSONException {
        JSONArray camerasJson = root.getJSONArray("cameras");
        JSONObject cameraDefaults = root.optJSONObject("cameraDefaults");
        JSONObject mountDefaults = root.optJSONObject("mountDefaults");
        int requestedCount = root.optInt("cameraCount", camerasJson.length());
        int count = Math.max(1, Math.min(4, Math.min(requestedCount, camerasJson.length())));
        VidarCameraMount[] mounts = new VidarCameraMount[count];
        for (int i = 0; i < count; i++) {
            JSONObject cam = camerasJson.getJSONObject(i);
            if (cam.has("index") && cam.getInt("index") != i) {
                throw new JSONException("Camera entry index " + cam.getInt("index")
                        + " must match array position " + i + " (zero-based)");
            }
            String webcamName = cam.getString("webcamName");
            JSONObject profileJson = buildCameraProfileJson(cam, cameraDefaults, mountDefaults);
            VidarRobotConfig.CameraProfileSpec spec = parseCameraProfile(profileJson);
            mounts[i] = new VidarCameraMount(webcamName, spec.toProfile());
        }

        JSONObject alliance = root.optJSONObject("alliance");
        VidarAlliance defaultAlliance = VidarAlliance.RED;
        String colorSensor = VidarConfig.ALLIANCE_COLOR_SENSOR;
        boolean useColorSensor = VidarConfig.ALLIANCE_USE_COLOR_SENSOR;
        boolean allowToggle = VidarConfig.ALLIANCE_ALLOW_RUNTIME_TOGGLE;
        if (alliance != null) {
            defaultAlliance = parseAlliance(alliance.optString("defaultAlliance", "red"));
            colorSensor = alliance.optString("colorSensorName", colorSensor);
            useColorSensor = alliance.optBoolean("useColorSensor", useColorSensor);
            allowToggle = alliance.optBoolean("allowRuntimeToggle", allowToggle);
        }

        return new VidarRobotConfig(
                root.optString("robotName", "robot"),
                root.optInt("activeCameraIndex", 0),
                mounts,
                parseDimensions(root),
                defaultAlliance,
                colorSensor,
                useColorSensor,
                allowToggle,
                parseOptionalDistanceUnit(root));
    }

    private static VidarRobotDimensions parseDimensions(JSONObject root) throws JSONException {
        JSONObject d = root.optJSONObject("dimensions");
        if (d == null) {
            d = root.optJSONObject("dimensionsIn");
        }
        if (d == null) {
            d = root.optJSONObject("dimensions");
        }
        if (d == null) {
            return new VidarRobotDimensions(13.0, 13.0, 18.0);
        }
        return new VidarRobotDimensions(
                readDist(d, "length", 13.0, "lengthDist", "lengthIn"),
                readDist(d, "width", 13.0, "widthDist", "widthIn"),
                readDist(d, "height", 18.0, "heightDist", "heightIn"));
    }

    private static VidarElementSpec[] parseElements(JSONArray array) throws JSONException {
        VidarElementSpec[] out = new VidarElementSpec[array.length()];
        for (int i = 0; i < array.length(); i++) {
            out[i] = parseElement(array.getJSONObject(i));
        }
        return out;
    }

    private static VidarElementSpec parseElement(JSONObject b) throws JSONException {
        JSONObject hsv = b.getJSONObject("hsv");
        JSONObject filters = b.optJSONObject("filters");
        JSONObject morph = b.optJSONObject("morphology");
        JSONObject hough = b.optJSONObject("hough");
        JSONObject interior = b.optJSONObject("interior");

        double maxAspectRatio = filters != null ? filters.optDouble("maxAspectRatio", 2.0) : 2.0;
        return new VidarElementSpec(
                b.getString("id"),
                b.optString("label", b.getString("id")),
                parseShape(b.optString("shape", "circle")),
                readDist(b, "diameter", "diameterDist", "diameterIn"),
                parseDetector(b.optString("detector", "color_blob_with_local_hough")),
                parseHsv(hsv),
                filters != null ? filters.optDouble("minAreaPx", 45) : 45,
                filters != null ? filters.optDouble("maxAreaPx", 12000) : 12000,
                filters != null ? filters.optDouble("minWidthPx", 8) : 8,
                filters != null ? filters.optDouble("maxWidthPx", 80) : 80,
                filters != null ? filters.optDouble("minHeightPx", 8) : 8,
                filters != null ? filters.optDouble("maxHeightPx", 80) : 80,
                maxAspectRatio,
                filters != null ? filters.optDouble("minCircularity", 0.55) : 0.55,
                filters != null ? filters.optDouble("minRectangularity", 0) : 0,
                filters != null ? filters.optDouble("minAspect", 1.0) : 1.0,
                filters != null ? filters.optDouble("maxAspect", maxAspectRatio) : maxAspectRatio,
                filters != null ? filters.optDouble("minFillRatio", 0.55) : 0.55,
                filters != null ? filters.optDouble("minInteriorScore", 0.12) : 0.12,
                interior != null ? interior.optInt("brightMin", 90) : 90,
                interior != null ? interior.optInt("spreadMax", 60) : 60,
                interior != null ? interior.optInt("holeDarkMax", 45) : 45,
                morph != null ? morph.optInt("erodePasses", 0) : 0,
                morph != null ? morph.optInt("dilatePasses", 0) : 0,
                morph != null ? morph.optInt("openPasses", 1) : 1,
                morph != null ? morph.optInt("closePasses", 2) : 2,
                hough != null ? hough.optDouble("dp", 1.2) : 1.2,
                hough != null ? hough.optDouble("minDist", 24) : 24,
                hough != null ? hough.optDouble("param1", 80) : 80,
                hough != null ? hough.optDouble("param2", 11) : 11,
                hough != null ? hough.optInt("minRadius", 8) : 8,
                hough != null ? hough.optInt("maxRadius", 36) : 36,
                hough != null ? hough.optDouble("minInterior", 0.14) : 0.14,
                hough != null ? hough.optDouble("minAreaPx", 45) : 45);
    }

    private static VidarPlateSpec[] parsePlates(JSONArray array) throws JSONException {
        VidarPlateSpec[] out = new VidarPlateSpec[array.length()];
        for (int i = 0; i < array.length(); i++) {
            out[i] = parsePlate(array.getJSONObject(i));
        }
        return out;
    }

    private static VidarPlateSpec parsePlate(JSONObject p) throws JSONException {
        JSONObject hsv = p.getJSONObject("hsv");
        JSONObject wrap = p.optJSONObject("hsvWrap");
        JSONObject filters = p.optJSONObject("filters");
        JSONObject white = p.optJSONObject("whiteDigit");

        return new VidarPlateSpec(
                parseAlliance(p.getString("alliance")),
                parseHsv(hsv),
                wrap != null ? parseHsv(wrap) : null,
                readDist(p, "width", 12.0, "widthDist", "widthIn"),
                filters != null ? filters.optDouble("minAreaPx", 120) : 120,
                filters != null ? filters.optDouble("maxAreaPx", 12000) : 12000,
                filters != null ? filters.optDouble("minAspect", 1.15) : 1.15,
                filters != null ? filters.optDouble("maxAspect", 4.5) : 4.5,
                filters != null ? filters.optDouble("minRectangularity", 0.45) : 0.45,
                filters != null ? filters.optDouble("minWhiteRatio", 0.12) : 0.12,
                white != null ? white.optInt("sampleGrid", 5) : 5,
                white != null ? white.optInt("brightMin", 175) : 175,
                white != null ? white.optInt("spreadMax", 55) : 55);
    }

    /** Shallow-merge JSON objects (override wins). */
    private static JSONObject mergeProfile(JSONObject defaults, JSONObject override) throws JSONException {
        JSONObject merged = defaults != null ? new JSONObject(defaults.toString()) : new JSONObject();
        if (override == null) {
            return merged;
        }
        java.util.Iterator<String> keys = override.keys();
        while (keys.hasNext()) {
            String key = keys.next();
            merged.put(key, override.get(key));
        }
        return merged;
    }

    /**
     * Merges {@code cameraDefaults} + per-entry {@code camera} intrinsics with
     * {@code mountDefaults} + per-entry {@code mount} placement. Legacy {@code profile} blocks
     * are still accepted.
     */
    private static JSONObject buildCameraProfileJson(
            JSONObject cam,
            JSONObject cameraDefaults,
            JSONObject mountDefaults) throws JSONException {
        JSONObject camera = mergeProfile(cameraDefaults, cam.optJSONObject("camera"));
        JSONObject mount = mergeProfile(mountDefaults, cam.optJSONObject("mount"));
        JSONObject legacy = cam.optJSONObject("profile");
        if (legacy != null) {
            camera = mergeProfile(camera, legacy);
            mount = mergeProfile(mount, legacy);
        }

        JSONObject flat = new JSONObject();
        String name = cam.optString("name", null);
        if (name == null || name.isEmpty()) {
            name = mount.optString("name", camera.optString("name", "camera"));
        }
        flat.put("name", name);
        flat.put("bearingDeg", mount.optDouble("bearingDeg", camera.optDouble("bearingDeg", 0)));
        flat.put("horizonRowPx", camera.optInt("horizonRowPx", 12));
        flat.put("focalLengthPx", camera.optDouble("focalLengthPx", 340));
        if (camera.has("focalLengthYPx")) {
            flat.put("focalLengthYPx", camera.getDouble("focalLengthYPx"));
        }
        flat.put("principalPointX", camera.optDouble("principalPointX", 320));
        flat.put("principalPointY", camera.optDouble("principalPointY", 240));
        flat.put("calibrationWidth", camera.optInt("calibrationWidth", camera.optInt("imageWidth", 0)));
        flat.put("calibrationHeight", camera.optInt("calibrationHeight", camera.optInt("imageHeight", 0)));
        if (camera.has("distortionModel")) {
            flat.put("distortionModel", camera.getString("distortionModel"));
        }
        if (camera.has("distortionCoeffs")) {
            flat.put("distortionCoeffs", camera.getJSONArray("distortionCoeffs"));
        }
        if (camera.has("calibrationVersion")) {
            flat.put("calibrationVersion", camera.getString("calibrationVersion"));
        }
        if (camera.has("calibrationDate")) {
            flat.put("calibrationDate", camera.getString("calibrationDate"));
        }
        flat.put("horizontalFovDeg", camera.optDouble("horizontalFovDeg", 70));
        flat.put("verticalFovDeg", camera.optDouble("verticalFovDeg", 55));
        flat.put("plateWidth", camera.has("plateWidth")
                ? camera.getDouble("plateWidth")
                : camera.optDouble("plateWidthIn", 12.0));
        if (camera.has("floorLut")) {
            flat.put("floorLut", camera.getJSONArray("floorLut"));
        } else {
            throw new JSONException("Camera \"" + name + "\" missing floorLut (set cameraDefaults.floorLut)");
        }
        if (camera.has("roi")) {
            flat.put("roi", camera.getJSONObject("roi"));
        }
        flat.put("mountX", readMountPosition(mount, "x"));
        flat.put("mountY", readMountPosition(mount, "y"));
        flat.put("mountZ", readMountPosition(mount, "z"));
        flat.put("mountYawDeg", readMountOrientation(mount, "yaw"));
        flat.put("mountPitchDeg", readMountOrientation(mount, "pitch"));
        flat.put("mountRollDeg", readMountOrientation(mount, "roll"));
        return flat;
    }

    private static double readMountPosition(JSONObject mount, String axis) throws JSONException {
        JSONObject position = mount.optJSONObject("position");
        if (position == null) {
            position = mount.optJSONObject("positionIn");
        }
        String flatDist = axis + "Dist";
        String flatIn = axis + "In";
        String legacy = "mount" + Character.toUpperCase(axis.charAt(0)) + axis.substring(1) + "Dist";
        String legacyIn = "mount" + Character.toUpperCase(axis.charAt(0)) + axis.substring(1) + "In";
        if (position != null && position.has(axis)) {
            return position.getDouble(axis);
        }
        if (mount.has(flatDist)) {
            return mount.getDouble(flatDist);
        }
        if (mount.has(flatIn)) {
            return mount.getDouble(flatIn);
        }
        if (mount.has(legacy)) {
            return mount.getDouble(legacy);
        }
        if (mount.has(legacyIn)) {
            return mount.getDouble(legacyIn);
        }
        return 0;
    }

    private static double readMountOrientation(JSONObject mount, String name) throws JSONException {
        JSONObject orientation = mount.optJSONObject("orientationDeg");
        String flatKey = name + "Deg";
        String legacyKey = "mount" + Character.toUpperCase(name.charAt(0)) + name.substring(1) + "Deg";
        if (orientation != null && orientation.has(name)) {
            return orientation.getDouble(name);
        }
        if (orientation != null && "yaw".equals(name) && orientation.has("bearing")) {
            return orientation.getDouble("bearing");
        }
        return mount.optDouble(flatKey, mount.optDouble(legacyKey, 0));
    }

    private static VidarCameraRoiConfig parseRoi(JSONObject roi) throws JSONException {
        if (roi == null) {
            return VidarCameraRoiConfig.DEFAULT;
        }
        JSONObject element = roi.optJSONObject("element");
        JSONObject plate = roi.optJSONObject("plate");
        JSONObject tag = roi.optJSONObject("tag");
        return new VidarCameraRoiConfig(
                element != null
                        ? element.optDouble("lowerFraction", 0.65)
                        : roi.optDouble("elementLowerFraction", 0.65),
                plate != null
                        ? plate.optDouble("startFraction", 0.30)
                        : roi.optDouble("plateStartFraction", 0.30),
                plate != null
                        ? plate.optDouble("bandFraction", 0.40)
                        : roi.optDouble("plateBandFraction", 0.40),
                tag != null
                        ? tag.optDouble("upperFraction", 0.65)
                        : roi.optDouble("tagUpperFraction", 0.65),
                element != null
                        ? element.optBoolean("enabled", true)
                        : roi.optBoolean("elementEnabled", true),
                plate != null
                        ? plate.optBoolean("enabled", true)
                        : roi.optBoolean("plateEnabled", true),
                tag != null
                        ? tag.optBoolean("enabled", true)
                        : roi.optBoolean("tagEnabled", true));
    }

    static VidarRobotConfig.CameraProfileSpec parseCameraProfile(JSONObject p) throws JSONException {
        JSONArray lut = p.getJSONArray("floorLut");
        double[] cy = new double[lut.length()];
        double[] dist = new double[lut.length()];
        for (int i = 0; i < lut.length(); i++) {
            JSONObject pt = lut.getJSONObject(i);
            cy[i] = pt.getDouble("cy");
            dist[i] = pt.has("dist") ? pt.getDouble("dist") : pt.getDouble("distIn");
        }

        JSONObject roi = p.optJSONObject("roi");
        VidarCameraRoiConfig roiConfig = parseRoi(roi);

        int calW = p.optInt("calibrationWidth", p.optInt("imageWidth", 0));
        int calH = p.optInt("calibrationHeight", p.optInt("imageHeight", 0));
        org.firstinspires.ftc.teamcode.vidar.geometry.VidarCameraIntrinsics.DistortionModel distModel =
                org.firstinspires.ftc.teamcode.vidar.geometry.VidarCameraIntrinsics.DistortionModel
                        .fromJson(p.optString("distortionModel", "none"));
        double[] distCoeffs = parseDoubleArray(p.optJSONArray("distortionCoeffs"));

        return new VidarRobotConfig.CameraProfileSpec(
                p.getString("name"),
                p.getDouble("bearingDeg"),
                p.optInt("horizonRowPx", 12),
                p.optDouble("focalLengthPx", 340),
                p.optDouble("focalLengthYPx", p.optDouble("focalLengthPx", 340)),
                p.optDouble("principalPointX", 320),
                p.optDouble("principalPointY", 240),
                p.optDouble("horizontalFovDeg", 70),
                p.optDouble("verticalFovDeg", 55),
                cy, dist,
                p.optDouble("mountX", 0),
                p.optDouble("mountY", 0),
                p.optDouble("mountZ", 0),
                p.optDouble("mountYawDeg", 0),
                p.optDouble("mountPitchDeg", 0),
                p.optDouble("mountRollDeg", 0),
                p.optDouble("plateWidth", 12.0),
                roiConfig,
                calW, calH, distModel, distCoeffs,
                p.optString("calibrationVersion", null),
                p.optString("calibrationDate", null));
    }

    private static double[] parseDoubleArray(JSONArray array) throws JSONException {
        if (array == null) {
            return null;
        }
        double[] out = new double[array.length()];
        for (int i = 0; i < array.length(); i++) {
            out[i] = array.getDouble(i);
        }
        return out;
    }

    private static VidarHsvRange parseHsv(JSONObject hsv) throws JSONException {
        return new VidarHsvRange(
                hsv.getInt("hMin"),
                hsv.getInt("hMax"),
                hsv.getInt("sMin"),
                hsv.getInt("sMax"),
                hsv.getInt("vMin"),
                hsv.getInt("vMax"));
    }

    private static VidarElementDetectorType parseDetector(String value) {
        switch (value.toLowerCase()) {
            case "color_blob":
                return VidarElementDetectorType.COLOR_BLOB;
            case "color_blob_with_local_hough":
            default:
                return VidarElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH;
        }
    }

    private static VidarElementShape parseShape(String value) {
        if (value == null) {
            return VidarElementShape.CIRCLE;
        }
        switch (value.toLowerCase()) {
            case "rect":
            case "rectangle":
                return VidarElementShape.RECT;
            case "blob":
                return VidarElementShape.BLOB;
            case "circle":
            default:
                return VidarElementShape.CIRCLE;
        }
    }

    private static VidarAlliance parseAlliance(String value) {
        if ("blue".equalsIgnoreCase(value)) {
            return VidarAlliance.BLUE;
        }
        return VidarAlliance.RED;
    }

    /** JSON string equivalent to legacy {@link VidarConfig} element/plate tuning. */
    public static String defaultSeasonJson() {
        return "{"
                + "\"seasonId\":\"2025-decode\","
                + "\"seasonName\":\"DECODE 2025 — game elements and team plates\","
                + "\"fusion\":{"
                + "\"minElementConfidence\":" + VidarConfig.MIN_ELEMENT_CONFIDENCE + ","
                + "\"minPlateConfidence\":" + VidarConfig.MIN_PLATE_CONFIDENCE + ","
                + "\"maxRangeMismatchRatio\":" + VidarConfig.MAX_RANGE_MISMATCH_RATIO
                + "},"
                + "\"elements\":[{"
                + "\"id\":\"pollen\","
                + "\"label\":\"Pollen element\","
                + "\"diameter\":" + VidarConfig.DEFAULT_ELEMENT_DIAMETER + ","
                + "\"detector\":\"color_blob_with_local_hough\","
                + "\"hsv\":{"
                + "\"hMin\":" + VidarConfig.DEFAULT_ELEMENT_HSV_H_MIN + ","
                + "\"hMax\":" + VidarConfig.DEFAULT_ELEMENT_HSV_H_MAX + ","
                + "\"sMin\":" + VidarConfig.DEFAULT_ELEMENT_HSV_S_MIN + ","
                + "\"sMax\":" + VidarConfig.DEFAULT_ELEMENT_HSV_S_MAX + ","
                + "\"vMin\":" + VidarConfig.DEFAULT_ELEMENT_HSV_V_MIN + ","
                + "\"vMax\":" + VidarConfig.DEFAULT_ELEMENT_HSV_V_MAX
                + "},"
                + "\"filters\":{"
                + "\"minAreaPx\":" + VidarConfig.DEFAULT_ELEMENT_MIN_AREA_PX + ","
                + "\"maxAreaPx\":" + VidarConfig.DEFAULT_ELEMENT_MAX_AREA_PX + ","
                + "\"minWidthPx\":" + VidarConfig.DEFAULT_ELEMENT_MIN_WIDTH_PX + ","
                + "\"maxWidthPx\":" + VidarConfig.DEFAULT_ELEMENT_MAX_WIDTH_PX + ","
                + "\"minHeightPx\":" + VidarConfig.DEFAULT_ELEMENT_MIN_HEIGHT_PX + ","
                + "\"maxHeightPx\":" + VidarConfig.DEFAULT_ELEMENT_MAX_HEIGHT_PX + ","
                + "\"maxAspectRatio\":" + VidarConfig.DEFAULT_ELEMENT_MAX_ASPECT_RATIO + ","
                + "\"minCircularity\":" + VidarConfig.DEFAULT_ELEMENT_MIN_CIRCULARITY + ","
                + "\"minFillRatio\":" + VidarConfig.DEFAULT_ELEMENT_MIN_FILL_RATIO + ","
                + "\"minInteriorScore\":" + VidarConfig.DEFAULT_ELEMENT_MIN_INTERIOR_SCORE
                + "},"
                + "\"interior\":{"
                + "\"brightMin\":" + VidarConfig.DEFAULT_ELEMENT_INTERIOR_BRIGHT + ","
                + "\"spreadMax\":" + VidarConfig.DEFAULT_ELEMENT_INTERIOR_SPREAD + ","
                + "\"holeDarkMax\":" + VidarConfig.DEFAULT_ELEMENT_HOLE_DARK_MAX
                + "},"
                + "\"morphology\":{"
                + "\"erodePasses\":" + VidarConfig.DEFAULT_ELEMENT_MORPH_ERODE_PASSES + ","
                + "\"dilatePasses\":" + VidarConfig.DEFAULT_ELEMENT_MORPH_DILATE_PASSES + ","
                + "\"openPasses\":" + VidarConfig.DEFAULT_ELEMENT_MORPH_OPEN_PASSES + ","
                + "\"closePasses\":" + VidarConfig.DEFAULT_ELEMENT_MORPH_CLOSE_PASSES
                + "},"
                + "\"hough\":{"
                + "\"dp\":" + VidarConfig.HOUGH_DP + ","
                + "\"minDist\":" + VidarConfig.HOUGH_MIN_DIST + ","
                + "\"param1\":" + VidarConfig.HOUGH_PARAM1 + ","
                + "\"param2\":" + VidarConfig.HOUGH_PARAM2 + ","
                + "\"minRadius\":" + VidarConfig.HOUGH_MIN_RADIUS + ","
                + "\"maxRadius\":" + VidarConfig.HOUGH_MAX_RADIUS + ","
                + "\"minInterior\":" + VidarConfig.HOUGH_MIN_INTERIOR + ","
                + "\"minAreaPx\":" + VidarConfig.MIN_ELEMENT_AREA_PX
                + "}"
                + "}],"
                + "\"plates\":[{"
                + "\"alliance\":\"red\","
                + "\"width\":12.0,"
                + "\"hsv\":{"
                + "\"hMin\":" + VidarConfig.PLATE_RED_H_MIN + ","
                + "\"hMax\":" + VidarConfig.PLATE_RED_H_MAX + ","
                + "\"sMin\":" + VidarConfig.PLATE_S_MIN + ","
                + "\"sMax\":255,"
                + "\"vMin\":" + VidarConfig.PLATE_V_MIN + ","
                + "\"vMax\":255"
                + "},"
                + "\"hsvWrap\":{"
                + "\"hMin\":" + VidarConfig.PLATE_RED_WRAP_H_MIN + ","
                + "\"hMax\":179,"
                + "\"sMin\":" + VidarConfig.PLATE_S_MIN + ","
                + "\"sMax\":255,"
                + "\"vMin\":" + VidarConfig.PLATE_V_MIN + ","
                + "\"vMax\":255"
                + "},"
                + "\"filters\":{"
                + "\"minAreaPx\":" + VidarConfig.PLATE_MIN_AREA_PX + ","
                + "\"maxAreaPx\":" + VidarConfig.PLATE_MAX_AREA_PX + ","
                + "\"minAspect\":" + VidarConfig.PLATE_MIN_ASPECT + ","
                + "\"maxAspect\":" + VidarConfig.PLATE_MAX_ASPECT + ","
                + "\"minRectangularity\":" + VidarConfig.PLATE_MIN_RECTANGULARITY + ","
                + "\"minWhiteRatio\":" + VidarConfig.PLATE_MIN_WHITE_RATIO
                + "},"
                + "\"whiteDigit\":{"
                + "\"sampleGrid\":" + VidarConfig.PLATE_WHITE_SAMPLE_GRID + ","
                + "\"brightMin\":" + VidarConfig.PLATE_WHITE_BRIGHT_MIN + ","
                + "\"spreadMax\":" + VidarConfig.PLATE_WHITE_SPREAD_MAX
                + "}"
                + "},{"
                + "\"alliance\":\"blue\","
                + "\"width\":12.0,"
                + "\"hsv\":{"
                + "\"hMin\":" + VidarConfig.PLATE_BLUE_H_MIN + ","
                + "\"hMax\":" + VidarConfig.PLATE_BLUE_H_MAX + ","
                + "\"sMin\":" + VidarConfig.PLATE_S_MIN + ","
                + "\"sMax\":255,"
                + "\"vMin\":" + VidarConfig.PLATE_V_MIN + ","
                + "\"vMax\":255"
                + "},"
                + "\"filters\":{"
                + "\"minAreaPx\":" + VidarConfig.PLATE_MIN_AREA_PX + ","
                + "\"maxAreaPx\":" + VidarConfig.PLATE_MAX_AREA_PX + ","
                + "\"minAspect\":" + VidarConfig.PLATE_MIN_ASPECT + ","
                + "\"maxAspect\":" + VidarConfig.PLATE_MAX_ASPECT + ","
                + "\"minRectangularity\":" + VidarConfig.PLATE_MIN_RECTANGULARITY + ","
                + "\"minWhiteRatio\":" + VidarConfig.PLATE_MIN_WHITE_RATIO
                + "},"
                + "\"whiteDigit\":{"
                + "\"sampleGrid\":" + VidarConfig.PLATE_WHITE_SAMPLE_GRID + ","
                + "\"brightMin\":" + VidarConfig.PLATE_WHITE_BRIGHT_MIN + ","
                + "\"spreadMax\":" + VidarConfig.PLATE_WHITE_SPREAD_MAX
                + "}"
                + "}]"
                + "}";
    }

    public static String defaultRobotJson() {
        StringBuilder sb = new StringBuilder();
        sb.append("{\"robotName\":\"example-robot\",\"activeCameraIndex\":")
                .append(VidarConfig.ACTIVE_CAMERA_INDEX)
                .append(",\"cameraCount\":").append(VidarConfig.activeCameraCount())
                .append(",\"dimensions\":{\"length\":13,\"width\":13,\"height\":18}")
                .append(",\"alliance\":{")
                .append("\"defaultAlliance\":\"")
                .append(VidarConfig.DEFAULT_ALLIANCE.name().toLowerCase())
                .append("\",\"colorSensorName\":\"")
                .append(VidarConfig.ALLIANCE_COLOR_SENSOR)
                .append("\",\"useColorSensor\":")
                .append(VidarConfig.ALLIANCE_USE_COLOR_SENSOR)
                .append(",\"allowRuntimeToggle\":")
                .append(VidarConfig.ALLIANCE_ALLOW_RUNTIME_TOGGLE)
                .append("},\"cameraDefaults\":");
        appendCameraDefaultsJson(sb, VidarCameraProfile.FOUR_SIDES[0]);
        sb.append(",\"mountDefaults\":{\"yawDeg\":0,\"pitchDeg\":-12,\"rollDeg\":0},\"cameras\":[");
        int count = VidarConfig.activeCameraCount();
        for (int i = 0; i < count; i++) {
            if (i > 0) {
                sb.append(',');
            }
            appendCameraEntryJson(sb, i, VidarConfig.CAMERA_NAMES[i], VidarCameraProfile.forIndex(i));
        }
        sb.append("]}");
        return sb.toString();
    }

    private static void appendCameraDefaultsJson(StringBuilder sb, VidarCameraProfile profile) {
        sb.append("{\"horizonRowPx\":").append(profile.horizonRowPx)
                .append(",\"focalLengthPx\":").append(profile.focalLengthPx)
                .append(",\"focalLengthYPx\":").append(profile.focalLengthYPx)
                .append(",\"principalPointX\":").append(profile.principalPointX)
                .append(",\"principalPointY\":").append(profile.principalPointY);
        if (profile.calibrationWidth > 0) {
            sb.append(",\"calibrationWidth\":").append(profile.calibrationWidth);
        }
        if (profile.calibrationHeight > 0) {
            sb.append(",\"calibrationHeight\":").append(profile.calibrationHeight);
        }
        sb.append(",\"horizontalFovDeg\":").append(profile.horizontalFovDeg)
                .append(",\"verticalFovDeg\":").append(profile.verticalFovDeg)
                .append(",\"plateWidth\":").append(profile.plateWidth)
                .append(",\"floorLut\":[");
        for (int j = 0; j < profile.floorCyPx.length; j++) {
            if (j > 0) {
                sb.append(',');
            }
            sb.append("{\"cy\":").append(profile.floorCyPx[j])
                    .append(",\"dist\":").append(profile.floorDist[j]).append('}');
        }
        sb.append("],\"roi\":");
        appendRoiJson(sb, profile.roiConfig);
        sb.append('}');
    }

    private static void appendRoiJson(StringBuilder sb, VidarCameraRoiConfig roi) {
        sb.append("{\"element\":{\"lowerFraction\":").append(roi.elementLowerFraction)
                .append(",\"enabled\":").append(roi.elementEnabled).append("},")
                .append("\"plate\":{\"startFraction\":").append(roi.plateStartFraction)
                .append(",\"bandFraction\":").append(roi.plateBandFraction)
                .append(",\"enabled\":").append(roi.plateEnabled).append("},")
                .append("\"tag\":{\"upperFraction\":").append(roi.tagUpperFraction)
                .append(",\"enabled\":").append(roi.tagEnabled).append("}}");
    }

    private static void appendCameraEntryJson(
            StringBuilder sb, int index, String webcamName, VidarCameraProfile profile) {
        sb.append("{\"index\":").append(index)
                .append(",\"webcamName\":\"").append(webcamName).append("\",")
                .append("\"name\":\"").append(profile.name).append("\",")
                .append("\"mount\":{\"bearingDeg\":").append(profile.bearingDeg)
                .append(",\"x\":").append(profile.mountX)
                .append(",\"y\":").append(profile.mountY)
                .append(",\"z\":").append(profile.mountZ)
                .append(",\"pitchDeg\":").append(profile.mountPitchDeg)
                .append(",\"rollDeg\":").append(profile.mountRollDeg)
                .append("}}");
    }
}
