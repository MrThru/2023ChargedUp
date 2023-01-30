package com.team1323.frc2023.subsystems.superstructure;

import com.team1323.frc2023.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class SuperstructurePosition {
    /*
     * The origin of the superstructure's coordinate system is the center of
     * the robot, projected onto the ground. +x is forward (toward the front
     * the robot), and +y is upward.
     */

    // The shoulder joint's position in space when both elevators are fully retracted
    private static final Translation2d kShoulderJointRetractedPosition = new Translation2d(12.75, 22.626);
    // The length from the shoulder joint to the very end of the shoulder
    private static final double kShoulderLength = 20.625;
    // The length from the shoulder joint to the wrist joint
    private static final double kShoulderJointToWristJointLength = 19.5;
    // The length from the wrist joint to the very end of the claw
    private static final double kWristLength = 15.0; // TODO: Update this when claw is finalized
    // The position in space of the center of the bottom of the elevator's top bar (the bar is a 2x1)
    private static final Translation2d kElevatorTopBarPosition = new Translation2d(6.25, 46.876);
    // The topmost and forwardmost point of the bumper
    private static final Translation2d kBumperCornerPosition = new Translation2d(Constants.kRobotHalfLength, 6.75);

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

    public SuperstructurePosition withVerticalHeight(double newVerticalHeight) {
        return new SuperstructurePosition(
            newVerticalHeight,
            this.horizontalExtension,
            this.shoulderAngle,
            this.wristAngle
        );
    }

    public SuperstructurePosition withHorizontalExtension(double newHorizontalExtension) {
        return new SuperstructurePosition(
            this.verticalHeight,
            newHorizontalExtension,
            this.shoulderAngle,
            this.wristAngle
        );
    }

    public SuperstructurePosition withShoulderAngle(double newShoulderAngle) {
        return new SuperstructurePosition(
            this.verticalHeight,
            this.horizontalExtension,
            newShoulderAngle,
            this.wristAngle
        );
    }

    public SuperstructurePosition withWristAngle(double newWristAngle) {
        return new SuperstructurePosition(
            this.verticalHeight,
            this.horizontalExtension,
            this.shoulderAngle,
            newWristAngle
        );
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

    /**
     * @return Whether or not the shoulder would collide with the top of the elevator
     * at any point throughout its travel, given the position of the shoulder joint. 
     */
    public boolean canShoulderCollideWithElevator() {
        final double clearanceInches = 2.0;

        return getShoulderJointPosition().distance(kElevatorTopBarPosition) < (kShoulderLength + clearanceInches);
    }

    /**
     * @return The angle at which the shoulder would be most likely to collide with
     * the elevator, if the shoulder were to move from its current position.
     */
    public Rotation2d getElevatorCollisionAngle() {
        Translation2d shoulderJointToElevatorBar = new Translation2d(getShoulderJointPosition(), kElevatorTopBarPosition);

        return shoulderJointToElevatorBar.direction();
    }

    public boolean isShoulderJointHigherThanElevatorBar() {
        return getShoulderJointPosition().y() >= kElevatorTopBarPosition.y();
    }

    /**
     * @return Whether or not the shoulder or wrist intersect the bumper or ground
     * in the current position.
     */
    public boolean collidesWithBumperOrGround() {
        Translation2d wristJointPosition = getWristJointPosition();
        boolean shoulderCollides = intersectsBumper(wristJointPosition) || intersectsGround(wristJointPosition);
        Translation2d wristTipPosition = getWristTipPosition();
        boolean wristCollides = intersectsBumper(wristTipPosition) || intersectsGround(wristTipPosition);

        return shoulderCollides || wristCollides;
    }

    private boolean intersectsBumper(Translation2d point) {
        return point.x() <= kBumperCornerPosition.x() &&
                point.y() <= kBumperCornerPosition.y();
    }

    private boolean intersectsGround(Translation2d point) {
        return point.y() <= 0.0;
    }

    /**
     * @return Whether or not the wrist can collide with the bumper at any point
     * throughout the shoulder's travel.
     */
    public boolean canWristCollideWithShoulderRotation() {
        Translation2d shoulderJointPosition = getShoulderJointPosition();
        double shoulderJointToWristTip = shoulderJointPosition.distance(getWristTipPosition());
        double shoulderJointToBumper = shoulderJointPosition.distance(kBumperCornerPosition);

        return shoulderJointToWristTip >= shoulderJointToBumper;
    }

    /**
     * @return The shoulder angle at which the wrist is most likely to collide
     * with the bumper.
     */
    public Rotation2d getShoulderBumperCollisionAngle() {
        Translation2d shoulderJointPosition = getShoulderJointPosition();
        double shoulderJointToWristTipLength = shoulderJointPosition.distance(getWristTipPosition());
        Rotation2d shoulderJointToBumperDirection = new Translation2d(shoulderJointPosition, kBumperCornerPosition).direction();

        Rotation2d shoulderAngleDelta = getLawOfCosinesAngle(kWristLength, kShoulderJointToWristJointLength, shoulderJointToWristTipLength);
        if (wristAngle > 0.0) {
            shoulderAngleDelta = shoulderAngleDelta.inverse();
        }

        return shoulderJointToBumperDirection.rotateBy(shoulderAngleDelta);
    }

    private Rotation2d getLawOfCosinesAngle(double oppositeSideLength, double adjacentSideLength1, double adjacentSideLength2) {
        double numerator = adjacentSideLength1 * adjacentSideLength1 + adjacentSideLength2 * adjacentSideLength2 -
                oppositeSideLength * oppositeSideLength;
        double denominator = 2 * adjacentSideLength1 * adjacentSideLength2;
        double radians = Math.acos(numerator / denominator);

        return Rotation2d.fromRadians(radians);
    }

    /**
     * @return Whether or not the wrist will collide with the bumper or ground
     * at any point throughout its own travel.
     */
    public boolean canWristCollideWithRotation() {
        return canWristCollideWithBumper() || canWristCollideWithGround();
    }

    private boolean canWristCollideWithBumper() {
        return getShoulderJointPosition().distance(kBumperCornerPosition) < kWristLength;
    }

    private boolean canWristCollideWithGround() {
        return getShoulderJointPosition().y() < kWristLength;
    }

    public Rotation2d getWristCollisionAngle() {
        if (canWristCollideWithGround()) {
            // Return the wrist angle that would result in the wrist pointing straight down
            return Rotation2d.fromDegrees(-90).rotateBy(Rotation2d.fromDegrees(shoulderAngle).inverse());
        }

        return new Translation2d(getShoulderJointPosition(), kBumperCornerPosition).direction()
                .rotateBy(Rotation2d.fromDegrees(shoulderAngle).inverse());
    }
}
