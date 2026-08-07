package org.firstinspires.ftc.teamcode.vidar.model;

import org.firstinspires.ftc.teamcode.vidar.VidarSpatialPoint;
import org.firstinspires.ftc.teamcode.vidar.world.VidarTrackDetection;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Assigns occurrence rank 0, 1, 2… within each season {@code elementId} by distance.
 */
public final class VidarElementOccurrenceRank {

    private VidarElementOccurrenceRank() {}

    public static List<VidarSpatialPoint> assignPerType(List<VidarSpatialPoint> points) {
        if (points == null || points.isEmpty()) {
            return points == null ? new ArrayList<VidarSpatialPoint>() : points;
        }
        List<VidarSpatialPoint> elements = new ArrayList<>();
        List<VidarSpatialPoint> other = new ArrayList<>();
        for (VidarSpatialPoint point : points) {
            if (point.kind == VidarSpatialPoint.Kind.ELEMENT) {
                elements.add(point);
            } else {
                other.add(point);
            }
        }
        List<VidarSpatialPoint> ranked = rankElementPoints(elements);
        List<VidarSpatialPoint> out = new ArrayList<>(ranked.size() + other.size());
        out.addAll(ranked);
        out.addAll(other);
        return out;
    }

    public static List<VidarTrackDetection> assignDetectionRanks(List<VidarTrackDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            return detections == null ? new ArrayList<VidarTrackDetection>() : detections;
        }
        List<VidarTrackDetection> elements = new ArrayList<>();
        List<VidarTrackDetection> other = new ArrayList<>();
        for (VidarTrackDetection det : detections) {
            if (det.kind == VidarWorldModel.Kind.ELEMENT) {
                elements.add(det);
            } else {
                other.add(det);
            }
        }
        List<VidarTrackDetection> ranked = rankDetections(elements);
        List<VidarTrackDetection> out = new ArrayList<>(ranked.size() + other.size());
        out.addAll(ranked);
        out.addAll(other);
        return out;
    }

    private static List<VidarSpatialPoint> rankElementPoints(List<VidarSpatialPoint> elements) {
        Map<String, List<VidarSpatialPoint>> grouped = groupPointsByElementId(elements);
        List<VidarSpatialPoint> out = new ArrayList<>(elements.size());
        for (List<VidarSpatialPoint> group : grouped.values()) {
            Collections.sort(group, Comparator.comparingDouble(VidarSpatialPoint::distance));
            for (int i = 0; i < group.size(); i++) {
                out.add(group.get(i).withOccurrenceRank(i));
            }
        }
        Collections.sort(out, Comparator.comparingDouble(VidarSpatialPoint::distance));
        return out;
    }

    private static List<VidarTrackDetection> rankDetections(List<VidarTrackDetection> elements) {
        Map<String, List<VidarTrackDetection>> grouped = groupDetectionsByElementId(elements);
        List<VidarTrackDetection> out = new ArrayList<>(elements.size());
        for (List<VidarTrackDetection> group : grouped.values()) {
            Collections.sort(group, Comparator.comparingDouble(VidarTrackDetection::distance));
            for (int i = 0; i < group.size(); i++) {
                out.add(group.get(i).withOccurrenceRank(i));
            }
        }
        return out;
    }

    private static Map<String, List<VidarSpatialPoint>> groupPointsByElementId(
            List<VidarSpatialPoint> elements) {
        Map<String, List<VidarSpatialPoint>> grouped = new HashMap<>();
        for (VidarSpatialPoint point : elements) {
            String key = point.elementId.isEmpty() ? "" : point.elementId;
            List<VidarSpatialPoint> bucket = grouped.get(key);
            if (bucket == null) {
                bucket = new ArrayList<>();
                grouped.put(key, bucket);
            }
            bucket.add(point);
        }
        return grouped;
    }

    private static Map<String, List<VidarTrackDetection>> groupDetectionsByElementId(
            List<VidarTrackDetection> elements) {
        Map<String, List<VidarTrackDetection>> grouped = new HashMap<>();
        for (VidarTrackDetection det : elements) {
            String key = det.elementId.isEmpty() ? "" : det.elementId;
            List<VidarTrackDetection> bucket = grouped.get(key);
            if (bucket == null) {
                bucket = new ArrayList<>();
                grouped.put(key, bucket);
            }
            bucket.add(det);
        }
        return grouped;
    }
}
