package com.team1323.frc2023.field;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.vision.AprilTagTracker.AprilTag;

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

    public static Alliance getAlliance() {
        return alliance;
    }

    public static void update() {
        alliance = DriverStation.getAlliance();
    }

    public static List<AprilTag> getCommunityAprilTags() {
        return (alliance == Alliance.Blue) ? blueCommunityAprilTags : redCommunityAprilTags;
    }
}
