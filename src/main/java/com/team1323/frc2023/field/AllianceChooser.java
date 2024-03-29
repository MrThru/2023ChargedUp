package com.team1323.frc2023.field;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.vision.AprilTagTracker.AprilTag;
import com.team1323.lib.math.geometry.Box2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class should serve as the one source of truth for
 * the robot's current alliance.
 */
public class AllianceChooser {
    private static Alliance alliance;
    private static final List<AprilTag> redCommunityAprilTags = Arrays.asList(AprilTag.ONE, AprilTag.TWO, AprilTag.THREE);
    private static final List<AprilTag> blueCommunityAprilTags = Arrays.asList(AprilTag.SIX, AprilTag.SEVEN, AprilTag.EIGHT);
    private static final AprilTag redLoadingZoneAprilTag = AprilTag.FIVE;
    private static final AprilTag blueLoadingZoneAprilTag = AprilTag.FOUR;
    private static final Box2d blueCommunityBoundingBox = Box2d.fromRectangleCorners(new Translation2d(40.45 + 13.8 + Constants.kRobotHalfLength, 0.0), 
            new Translation2d(40.45 + 13.8 + 136.81, 216.03));
    private static final Box2d redCommunityBoundingBox = Box2d.fromRectangleCorners(new Translation2d(610.77 - 13.8 - 136.81, 0.0), 
            new Translation2d(610.77 - 13.8 - Constants.kRobotHalfLength, 216.03));
    private static final Box2d blueLoadingZoneBoundingBox = Box2d.fromRectangleCorners(new Translation2d(636.96 - 118.25 - 62.0, 216.03), 
            new Translation2d(636.96, 216.03 + 99.07));
    private static final Box2d redLoadingZoneBoundingBox = Box2d.fromRectangleCorners(new Translation2d(14.25, 216.03), 
            new Translation2d(14.25 + 118.25 + 62.0, 216.03 + 99.07));
    private static final double blueMidConePoleX = 40.45 - 8.913;
    private static final double blueHighConePoleX = 40.45 - 25.926;
    private static final double redMidConePoleX = 610.77 + 8.913;
    private static final double redHighConePoleX = 610.77 + 25.926;
    private static Function<AprilTag, Stream<Double>> aprilTagToConePoleYs = aprilTag -> {
        double kAprilTagToConePoleLateralDisplacement = 22.0;
        double aprilTagY = aprilTag.getTranslation2d().y();

        return Stream.of(aprilTagY + kAprilTagToConePoleLateralDisplacement, aprilTagY - kAprilTagToConePoleLateralDisplacement);
    };
    private static final List<Double> blueConePoleYs = blueCommunityAprilTags.stream().flatMap(aprilTagToConePoleYs).toList();
    private static final List<Double> redConePoleYs = redCommunityAprilTags.stream().flatMap(aprilTagToConePoleYs).toList();

    public static Alliance getAlliance() {
        return alliance;
    }

    public static void update() {
        alliance = DriverStation.getAlliance();
    }

    public static List<AprilTag> getCommunityAprilTags() {
        return (alliance == Alliance.Blue) ? blueCommunityAprilTags : redCommunityAprilTags;
    }

    public static AprilTag getLoadingZoneAprilTag() {
        return (alliance == Alliance.Blue) ? blueLoadingZoneAprilTag : redLoadingZoneAprilTag;
    }

    public static Box2d getCommunityBoundingBox() {
        return (alliance == Alliance.Blue) ? blueCommunityBoundingBox : redCommunityBoundingBox;
    }

    public static Box2d getLoadingZoneBoundingBox() {
        return (alliance == Alliance.Blue) ? blueLoadingZoneBoundingBox : redLoadingZoneBoundingBox;
    }

    public static double getMidConePoleX() {
        return (alliance == Alliance.Blue) ? blueMidConePoleX : redMidConePoleX;
    }

    public static double getHighConePoleX() {
        return (alliance == Alliance.Blue) ? blueHighConePoleX : redHighConePoleX;
    }

    public static double getAverageConePoleX() {
        return (alliance == Alliance.Blue) ?
                (blueMidConePoleX + blueHighConePoleX) / 2.0 :
                (redMidConePoleX + redHighConePoleX) / 2.0;
    }

    public static List<Double> getConePoleYs() {
        return (alliance == Alliance.Blue) ? blueConePoleYs : redConePoleYs;
    }
}
