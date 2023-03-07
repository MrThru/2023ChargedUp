package com.team1323.frc2023.field;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team254.lib.geometry.Pose2d;

public class WaypointList {
    private final List<Pose2dWithQuadrantOffsets> waypoints = new ArrayList<>();

    public void add(Pose2d waypoint) {
        waypoints.add(new Pose2dWithQuadrantOffsets(waypoint));
    }

    public void add(Pose2dWithQuadrantOffsets waypoint) {
        waypoints.add(waypoint);
    }

    public boolean hasOffsetsForQuadrant(Quadrant quadrant) {
        return waypoints.stream().anyMatch(w -> w.hasOffsetForQuadrant(quadrant));
    }

    public List<Pose2d> getWaypointsForQuadrant(Quadrant quadrant) {
        return waypoints.stream()
            .map(w -> w.getPoseForQuadrant(quadrant))
            .toList();
    }

    public static class Pose2dWithQuadrantOffsets {
        private final Pose2d pose;
        private final Map<Quadrant, Pose2d> offsetMap = new HashMap<>(Map.of(
            Quadrant.BOTTOM_LEFT, Pose2d.identity(),
            Quadrant.TOP_LEFT, Pose2d.identity(),
            Quadrant.TOP_RIGHT, Pose2d.identity(),
            Quadrant.BOTTOM_RIGHT, Pose2d.identity()
        ));

        public Pose2dWithQuadrantOffsets(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2dWithQuadrantOffsets withOffset(Quadrant quadrant, Pose2d offset) {
            offsetMap.put(quadrant, offset);
            return this;
        }

        public Pose2d getPoseForQuadrant(Quadrant quadrant) {
            return AutoZones.mirror(pose.transformBy(offsetMap.get(quadrant)), quadrant);
        }

        public boolean hasOffsetForQuadrant(Quadrant quadrant) {
            return !Pose2d.identity().equals(offsetMap.get(quadrant));
        }
    }
}
