package com.team1323.frc2023.field;

import java.util.Collections;
import java.util.Comparator;
import java.util.Map;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.vision.AprilTagTracker.AprilTag;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoringPositions {
    // All length measurements are in inches
    private static final double kAprilTagToConeLateralDisplacement = 22.0;
    private static final double kAprilTagToBarrierForwardDisplacement = 13.8;
    private static final double kScoringPoseForwardPadding = 2.0;

    private static final Map<Alliance, Map<Grid, AprilTag>> kGridToAprilTagMap = Map.of(
        Alliance.Red, Map.of(
            Grid.LEFT, AprilTag.THREE,
            Grid.CENTER, AprilTag.TWO,
            Grid.RIGHT, AprilTag.ONE
        ),
        Alliance.Blue, Map.of(
            Grid.LEFT, AprilTag.EIGHT,
            Grid.CENTER, AprilTag.SEVEN,
            Grid.RIGHT, AprilTag.SIX
        )
    );

    private static final Map<AprilTag, Grid> kAprilTagToGridMap = Map.of(
        AprilTag.THREE, Grid.LEFT,
        AprilTag.TWO, Grid.CENTER,
        AprilTag.ONE, Grid.RIGHT,
        AprilTag.EIGHT, Grid.LEFT,
        AprilTag.SEVEN, Grid.CENTER,
        AprilTag.SIX, Grid.RIGHT
    );

    public static Pose2d getScoringPose(NodeLocation nodeLocation) {
        AprilTag nodeTag = kGridToAprilTagMap.get(AllianceChooser.getAlliance()).get(nodeLocation.grid);

        double yTransform = 0.0;
        switch (nodeLocation.column) {
            case LEFT:
                yTransform = kAprilTagToConeLateralDisplacement;
                break;
            case RIGHT:
                yTransform = -kAprilTagToConeLateralDisplacement;
                break;
            case CENTER:
                break;
        }

        Pose2d tagToRobotTransform = Pose2d.fromTranslation(new Translation2d(
            -(kAprilTagToBarrierForwardDisplacement + kScoringPoseForwardPadding + Constants.kRobotHalfWidth), 
            yTransform
        ));
        Pose2d scoringPose = nodeTag.getPose2d().transformBy(tagToRobotTransform);

        return Settings.kFieldOffsets.applyOffsets(scoringPose, nodeLocation);
    }

    private static AprilTag getNearestAprilTag(Pose2d robotPose) {
        Comparator<AprilTag> nearestPositionComparator = (firstTag, secondTag) -> {
            double firstDistance = firstTag.getTranslation2d().distance(robotPose.getTranslation());
            double secondDistance = secondTag.getTranslation2d().distance(robotPose.getTranslation());

            return Double.compare(firstDistance, secondDistance);
        };
        AprilTag nearestTag = Collections.min(AllianceChooser.getCommunityAprilTags(), nearestPositionComparator);

        return nearestTag;
    }

    private static Pose2d getNearestScoringPose(Pose2d robotPose, Column column) {
        AprilTag nearestAprilTag = getNearestAprilTag(robotPose);
        // The row doesn't matter for pose calculations
        NodeLocation nodeLocation = new NodeLocation(kAprilTagToGridMap.get(nearestAprilTag), Row.TOP, column);

        return getScoringPose(nodeLocation);
    }

    public static Pose2d getLeftScoringPose(Pose2d robotPose) {
        return getNearestScoringPose(robotPose, Column.LEFT);
    }

    public static Pose2d getRightScoringPose(Pose2d robotPose) {
        return getNearestScoringPose(robotPose, Column.RIGHT);
    }

    public static Pose2d getCenterScoringPose(Pose2d robotPose) {
        return getNearestScoringPose(robotPose, Column.CENTER);
    }
}
