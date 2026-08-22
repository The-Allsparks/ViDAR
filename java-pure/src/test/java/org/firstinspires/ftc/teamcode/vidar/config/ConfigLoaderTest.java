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
        assertEquals(1, season.elements.length);
        assertEquals("pollen", season.elements[0].id);
        assertEquals(2, season.plates.length);
        assertEquals(VidarDistanceUnit.IN, season.distanceUnit);
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
        assertEquals(246, robot.cameras[0].profile.focalLengthPx, 0.01);
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
    }
}
