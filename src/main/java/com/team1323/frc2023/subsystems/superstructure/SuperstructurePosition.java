package com.team1323.frc2023.subsystems.superstructure;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

// TODO: Update all of the robot's physical dimensions with CAD
public class SuperstructurePosition {
    /*
        * The origin of the superstructure's coordinate system is the center of
        * the robot, projected onto the ground. +x is forward (toward the front
        * the robot), and +y is upward.
        */

    // The shoulder joint's position in space when both elevators are fully retracted
    private static final Translation2d kShoulderJointRetractedPosition = new Translation2d(6.0, 12.0);
    // The length from the shoulder joint to the very end of the shoulder
    private static final double kShoulderLength = 14.0;
    // The length from the shoulder joint to the wrist joint
    private static final double kShoulderJointToWristJointLength = 12.0;
    // The length from the wrist joint to the very end of the claw
    private static final double kWristLength = 8.0;
    // The position in space of the bottom of the elevator's top bar
    private static final Translation2d kElevatorTopBarPosition = new Translation2d(6.0, 36.0);

    public final double verticalHeight;
    public final double horizontalExtension;
    public final double shoulderAngle;
    public final double wristAngle;

    public SuperstructurePosition(double verticalHeight, double horizontalExtension,
            double shoulderAngle, double wristAngle) {
        this.verticalHeight = verticalHeight;
        this.horizontalExtension = horizontalExtension;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
    }

    public Translation2d getShoulderJointPosition() {
        return kShoulderJointRetractedPosition.translateBy(new Translation2d(horizontalExtension, verticalHeight));
    }

    public Translation2d getWristJointPosition() {
        Pose2d shoulderJointPose = new Pose2d(getShoulderJointPosition(), Rotation2d.fromDegrees(shoulderAngle));
        Pose2d wristJointPose = shoulderJointPose.transformBy(Pose2d.fromTranslation(
                new Translation2d(kShoulderJointToWristJointLength, 0)));

        return wristJointPose.getTranslation();
    }

    public Translation2d getWristTipPosition() {
        Rotation2d wristDirection = Rotation2d.fromDegrees(wristAngle).rotateBy(Rotation2d.fromDegrees(shoulderAngle));
        Pose2d wristJointPose = new Pose2d(getWristJointPosition(), wristDirection);
        Pose2d wristTipPose = wristJointPose.transformBy(Pose2d.fromTranslation(
                new Translation2d(kWristLength, 0)));

        return wristTipPose.getTranslation();
    }
}
