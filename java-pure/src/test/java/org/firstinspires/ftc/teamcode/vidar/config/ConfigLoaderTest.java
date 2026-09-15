package org.firstinspires.ftc.teamcode.vidar.config;

import org.firstinspires.ftc.teamcode.vidar.VidarDistanceUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.*;

class ConfigLoaderTest {

    private static Path repoRoot() {
        return Paths.get(System.getProperty("user.dir")).getParent();
    }

    @Test
    void loadBiobuzzSeason() throws IOException {
        String json = Files.readString(repoRoot().resolve("config/seasons/2026-biobuzz.json"));
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(json);
        assertEquals("2026-biobuzz", season.seasonId);
        assertEquals(3, season.elements.length);
        assertEquals("pollen", season.elements[0].id);
        assertEquals("nectar_red", season.elements[1].id);
        assertEquals("nectar_blue", season.elements[2].id);
        assertEquals(3.6, season.elements[1].diameter, 1e-9);
        assertEquals(3.6, season.elements[2].diameter, 1e-9);
        assertEquals(2, season.plates.length);
        assertEquals(VidarDistanceUnit.IN, season.distanceUnit);
        assertEquals(3.25, season.defaultTagSize, 1e-9);
        assertEquals(16, season.aprilTags.length);
        for (VidarAprilTagSpec tag : season.aprilTags) {
            assertFalse(tag.localization, "HIVE tags are not static landmarks: " + tag.id);
            assertEquals(3.25, tag.size, 1e-9);
            assertFalse(season.useTagForLocalization(tag.id));
        }
        assertEquals(30, season.aprilTags[0].id);
        assertEquals(45, season.aprilTags[15].id);
        assertEquals(0, season.fixtures.length);
        assertFalse(json.contains("\"fixtures\""));
        assertTrue(json.contains("\"flowerGeometry\""));
        assertTrue(json.contains("\"outerDiameter\": 1.05"));
        assertTrue(json.contains("\"wallParallelSpacing\": 3.45"));
        assertTrue(json.contains("\"topOpeningHeight\": 21.5"));
        assertTrue(json.contains("\"flower_audience\""));
    }

    @Test
    void seasonDistanceUnitMeters() {
        String json = "{"
                + "\"seasonId\":\"metric-test\","
                + "\"distanceUnit\":\"m\","
                + "\"field\":{\"length\":17.5,\"width\":8.0},"
                + "\"elements\":[{\"id\":\"ball\",\"label\":\"Ball\",\"diameter\":0.071,"
                + "\"detector\":\"color_blob\","
                + "\"hsv\":{\"hMin\":0,\"hMax\":10,\"sMin\":0,\"sMax\":255,\"vMin\":0,\"vMax\":255}}],"
                + "\"plates\":[]"
                + "}";
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(json);
        assertEquals(VidarDistanceUnit.M, season.distanceUnit);
        assertEquals(0.071, season.elements[0].diameter, 1e-9);
    }

    @Test
    void loadExampleRobot() throws IOException {
        String json = Files.readString(repoRoot().resolve("config/robots/example-robot.json"));
        VidarRobotConfig robot = VidarConfigLoader.loadRobot(json);
        assertEquals("example-robot", robot.robotName);
        assertEquals(4, robot.cameras.length);
        assertEquals("front", robot.cameras[0].profile.name);
        assertTrue(robot.cameras[0].profile.focalLengthPx > 0);
        assertEquals(492, robot.cameras[0].profile.focalLengthPx, 0.01);
        assertEquals(1280, robot.cameras[0].profile.calibrationWidth);
        assertEquals(720, robot.cameras[0].profile.calibrationHeight);
    }

    @Test
    void seasonWorldAndFusionTuningFromJson() {
        String json = "{"
                + "\"seasonId\":\"tuning-test\","
                + "\"field\":{\"length\":144,\"width\":144},"
                + "\"fusion\":{\"maxRankedElements\":12,\"defaultMaxRankedElements\":4},"
                + "\"world\":{\"mergeRadius\":6.5,\"trackGateRadius\":10.0},"
                + "\"elements\":[{\"id\":\"ball\",\"label\":\"Ball\",\"diameter\":4,"
                + "\"detector\":\"color_blob\","
                + "\"hsv\":{\"hMin\":0,\"hMax\":10,\"sMin\":0,\"sMax\":255,\"vMin\":0,\"vMax\":255}}],"
                + "\"plates\":[]"
                + "}";
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(json);
        assertEquals(12, season.fusionMaxRankedElements);
        assertEquals(4, season.defaultMaxRankedElements);
        assertEquals(6.5, season.world.mergeRadius, 1e-9);
        assertEquals(10.0, season.world.trackGateRadius, 1e-9);

        VidarSettings settings = new VidarSettings(null, season);
        assertEquals(6.5, settings.worldMergeRadiusIn, 1e-9);
        assertEquals(12, settings.fusionMaxRankedElements);
    }

    @Test
    void bundledDefaultSeasonLoads() throws IOException {
        Path path = repoRoot().resolve(
                "teamcode/org/firstinspires/ftc/teamcode/vidar/config/bundled/default-season.json");
        String json = Files.readString(path);
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(json);
        assertEquals("2025-decode", season.seasonId);
        assertEquals("pollen", season.elements[0].id);
        assertEquals(0, season.fixtures.length);
    }

    private static String minimalSeasonJson(String fixturesBlock) {
        String fixtures = fixturesBlock == null ? "" : "," + fixturesBlock;
        return "{"
                + "\"seasonId\":\"fixture-test\","
                + "\"field\":{\"length\":144,\"width\":144},"
                + "\"elements\":[{\"id\":\"ball\",\"label\":\"Ball\",\"diameter\":4,"
                + "\"detector\":\"color_blob\","
                + "\"hsv\":{\"hMin\":0,\"hMax\":10,\"sMin\":0,\"sMax\":255,\"vMin\":0,\"vMax\":255}}],"
                + "\"plates\":[]"
                + fixtures
                + "}";
    }

    @Test
    void omittedFixturesKeyLoadsEmptyArray() {
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(minimalSeasonJson(null));
        assertEquals(0, season.fixtures.length);
        assertEquals("ball", season.elements[0].id);
    }

    @Test
    void syntheticStaticFieldFixtureLoads() {
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(minimalSeasonJson(
                "\"fixtures\":[{"
                        + "\"id\":\"flower_1\","
                        + "\"label\":\"Flower 1\","
                        + "\"localization\":\"static_field\","
                        + "\"position\":{\"x\":12.5,\"y\":-64.0,\"z\":21.5},"
                        + "\"orientationDeg\":{\"yaw\":90},"
                        + "\"detectors\":[\"ordered_stack\"]"
                        + "}]"));
        assertEquals(1, season.fixtures.length);
        VidarFixtureSpec spec = season.fixtureById("flower_1");
        assertNotNull(spec);
        assertEquals("Flower 1", spec.label);
        assertEquals(VidarFixtureLocalizationMode.STATIC_FIELD, spec.localization);
        assertEquals(12.5, spec.xIn, 1e-9);
        assertEquals(-64.0, spec.yIn, 1e-9);
        assertEquals(21.5, spec.zIn, 1e-9);
        assertEquals(90.0, spec.yawDeg, 1e-9);
        assertEquals(1, spec.detectors.length);
        assertEquals("ordered_stack", spec.detectors[0]);
        assertEquals(0, spec.tagIds.length);
        assertTrue(spec.hasFieldPosition());
    }

    @Test
    void unknownFixtureLocalizationFailsLoudly() {
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> VidarConfigLoader.loadSeason(minimalSeasonJson(
                        "\"fixtures\":[{\"id\":\"bad\",\"localization\":\"nope\"}]")));
        assertTrue(ex.getMessage().contains("Unknown fixture localization"), ex.getMessage());
        assertTrue(ex.getMessage().contains("nope"), ex.getMessage());
    }

    @Test
    void blankFixtureIdFails() {
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> VidarConfigLoader.loadSeason(minimalSeasonJson(
                        "\"fixtures\":[{\"id\":\"  \",\"localization\":\"static_field\"}]")));
        assertTrue(ex.getMessage().contains("Fixture id is required"), ex.getMessage());
    }

    @Test
    void emptyFixtureLabelDefaultsToId() {
        VidarSeasonConfig season = VidarConfigLoader.loadSeason(minimalSeasonJson(
                "\"fixtures\":[{\"id\":\"flower_1\",\"label\":\"\",\"localization\":\"static_field\"}]"));
        assertEquals("flower_1", season.fixtureById("flower_1").label);
    }
}
