package com.team1323.frc2023.field;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.AprilTagTracker.AprilTag;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoringPoses {
    // All length measurements are in inches
    private static final double kAprilTagToConeLateralDisplacement = 22.0;
    private static final double kAprilTagToBarrierForwardDisplacement = 13.8;
    private static final double kScoringPoseForwardPadding = 2.0;
    private static final double kAprilTagToShelfLateralDisplacement = 24.0;
    private static final double kShelfPoseForwardPadding = 6.0;
    private static double coneLateralOffset = 0.0;

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

    public static void updateConeLateralOffset() {
        double offset = LimelightProcessor.getInstance().getRetroConeLeftRightOffset();
        SmartDashboard.putNumber("Cone Left-Right offset", offset);
        coneLateralOffset = Double.isNaN(offset) ? 0.0 : offset;
    }

    public static Pose2d getScoringPose(NodeLocation nodeLocation) {
        AprilTag nodeTag = kGridToAprilTagMap.get(AllianceChooser.getAlliance()).get(nodeLocation.grid);

        double xTransform = kScoringPoseForwardPadding;
        double yTransform = 0.0;
        switch (nodeLocation.column) {
            case LEFT:
                yTransform = -coneLateralOffset;
                yTransform += kAprilTagToConeLateralDisplacement;
                xTransform -= 2.5; // 3.5
                break;
            case RIGHT:
                yTransform = -coneLateralOffset;
                yTransform -= kAprilTagToConeLateralDisplacement;
                xTransform -= 2.5; // 3.25
                break;
            case CENTER:
                break;
        }

        Pose2d tagToRobotTransform = Pose2d.fromTranslation(new Translation2d(
            -(kAprilTagToBarrierForwardDisplacement + xTransform + Constants.kRobotHalfLength), 
            yTransform
        ));
        Pose2d scoringPose = nodeTag.getPose2d().transformBy(tagToRobotTransform);

        return Settings.kFieldOffsets.applyOffsets(scoringPose, nodeLocation);
    }

    public static Pose2d getShelfPose(boolean left) {
        double yTransform = left ? kAprilTagToShelfLateralDisplacement : -kAprilTagToShelfLateralDisplacement;
        Pose2d tagToRobotTransform = Pose2d.fromTranslation(new Translation2d(
            -(kShelfPoseForwardPadding + Constants.kRobotHalfLength),
            yTransform
        ));

        return AllianceChooser.getLoadingZoneAprilTag().getPose2d().transformBy(tagToRobotTransform);
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

    public static Pose2d getClosestScoringPosition(Pose2d robotPose) {
        List<Pose2d> getScoringPositions = Arrays.asList(ScoringPoses.getLeftScoringPose(Swerve.getInstance().getPose()), ScoringPoses.getCenterScoringPose(Swerve.getInstance().getPose()),
                ScoringPoses.getRightScoringPose(Swerve.getInstance().getPose()));
        Pose2d closestPosition = getScoringPositions.get(0);
        Translation2d robotPosition = robotPose.getTranslation();
        for(int i = 0; i < getScoringPositions.size(); i++) {
            if(closestPosition.getTranslation().distance(robotPosition) > 
                        getScoringPositions.get(i).getTranslation().distance(robotPosition)) {
                closestPosition = getScoringPositions.get(i);
            }
        }
        return closestPosition;
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
