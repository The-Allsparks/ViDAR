package org.firstinspires.ftc.teamcode.vidar;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Foe density in the forward cone split into left / center / right lanes.
 *
 * <p>Teams use {@link #recommended()} to pick a lane with the fewest nearby foes.
 */
public final class VidarOffensiveLaneAnalysis {

    public final int leftCount;
    public final int centerCount;
    public final int rightCount;
    public final VidarOffensiveLane recommended;
    public final double maxRangeIn;
    public final double coneHalfDeg;

    private VidarOffensiveLaneAnalysis(
            int leftCount,
            int centerCount,
            int rightCount,
            VidarOffensiveLane recommended,
            double maxRangeIn,
            double coneHalfDeg) {
        this.leftCount = leftCount;
        this.centerCount = centerCount;
        this.rightCount = rightCount;
        this.recommended = recommended;
        this.maxRangeIn = maxRangeIn;
        this.coneHalfDeg = coneHalfDeg;
    }

    public static VidarOffensiveLaneAnalysis fromFoes(List<VidarSpatialPoint> foes) {
        return fromFoes(
                foes,
                VidarConfig.OFFENSIVE_LANE_MAX_RANGE_IN,
                VidarConfig.OFFENSIVE_LANE_CONE_HALF_DEG);
    }

    public static VidarOffensiveLaneAnalysis fromFoes(
            List<VidarSpatialPoint> foes,
            double maxRangeIn,
            double coneHalfDeg) {
        Map<VidarOffensiveLane, List<VidarSpatialPoint>> buckets = new EnumMap<>(VidarOffensiveLane.class);
        for (VidarOffensiveLane lane : VidarOffensiveLane.values()) {
            buckets.put(lane, new ArrayList<>());
        }

        if (foes != null) {
            double third = coneHalfDeg / 3.0;
            for (VidarSpatialPoint foe : foes) {
                if (foe == null || !foe.isValid()) {
                    continue;
                }
                if (foe.distance() > maxRangeIn) {
                    continue;
                }
                double bearing = foe.bearingDeg();
                if (Math.abs(bearing) > coneHalfDeg) {
                    continue;
                }
                VidarOffensiveLane lane;
                if (bearing < -third) {
                    lane = VidarOffensiveLane.LEFT;
                } else if (bearing > third) {
                    lane = VidarOffensiveLane.RIGHT;
                } else {
                    lane = VidarOffensiveLane.CENTER;
                }
                buckets.get(lane).add(foe);
            }
        }

        int left = buckets.get(VidarOffensiveLane.LEFT).size();
        int center = buckets.get(VidarOffensiveLane.CENTER).size();
        int right = buckets.get(VidarOffensiveLane.RIGHT).size();
        VidarOffensiveLane recommended = pickLane(left, center, right);
        return new VidarOffensiveLaneAnalysis(left, center, right, recommended, maxRangeIn, coneHalfDeg);
    }

    private static VidarOffensiveLane pickLane(int left, int center, int right) {
        int min = Math.min(left, Math.min(center, right));
        if (center == min) {
            return VidarOffensiveLane.CENTER;
        }
        if (left == min) {
            return VidarOffensiveLane.LEFT;
        }
        return VidarOffensiveLane.RIGHT;
    }

    public int count(VidarOffensiveLane lane) {
        switch (lane) {
            case LEFT:
                return leftCount;
            case CENTER:
                return centerCount;
            case RIGHT:
            default:
                return rightCount;
        }
    }
}
