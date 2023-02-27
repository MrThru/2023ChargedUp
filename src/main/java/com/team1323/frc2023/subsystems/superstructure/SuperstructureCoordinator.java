package com.team1323.frc2023.subsystems.superstructure;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.requests.EmptyRequest;
import com.team1323.frc2023.subsystems.requests.LambdaRequest;
import com.team1323.frc2023.subsystems.requests.ParallelRequest;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.requests.SequentialRequest;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;

/**
 * This class serves to coordinate movements between the various subsystems
 * (vertical elevator, horizontal elevator, shoulder, and wrist).
 * 
 * Each method in this class corresponds to one final position that the
 * superstructure can be asked to reach. Each method analyzes the superstructure's
 * current position and choreographs a series of movements (expressed as a
 * Request) that will get the Superstructure to its desired state in a safe
 * manner. Any choreography should abide by these three main constraints:
 * 
 * 1. The wrist should not collide with the ground, bumper, or top of the elevator
 * at any point throughout its travel.
 * 
 * 2. Similarly, the shoulder should not collide with the ground, bumper, or top
 * of the elevator at any point throughout its travel.
 * 
 * 3. Outward extension of the shoulder should be minimized when it is traveling
 * in the range of [-90deg, 90deg].
 */
public class SuperstructureCoordinator {
    private static SuperstructureCoordinator instance = null;
    public static SuperstructureCoordinator getInstance() {
        if (instance == null) {
            instance = new SuperstructureCoordinator();
        }
        return instance;
    }

    private static final double kVerticalHeightForTopBarClearance = 0.5;
    private static final double kVerticalHeightForBumperClearance = 16.0;
    private static final double kVerticalHeightForStow = 4.25;
    private static final double kShoulderAngleForHorizontalRetraction = -60.0;
    private static final double kShoulderAngleForHorizontalExtension = 20.0;
    private static final double kShoulderAngleForEscapingElevator = 90.0;
    private static final double kHorizontalExtensionForMinShoulderReach = 0.25;
    private static final double kHorizontalExtensionForUprightShoulder = 6.0;
    private static final double kHorizontalExtensionForGridClearance = 15.0;

    public static final double kCubeMidScoringHorizontalExtension = 16.5;
    public static final double kCubeHighScoringHorizontalExtension = 31.0;

    private final VerticalElevator verticalElevator;
    private final HorizontalElevator horizontalElevator;
    private final Shoulder shoulder;
    private final Wrist wrist;

    private SuperstructureCoordinator() {
        verticalElevator = VerticalElevator.getInstance();
        horizontalElevator = HorizontalElevator.getInstance();
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

	public SuperstructurePosition getPosition() {
		return new SuperstructurePosition(
			verticalElevator.getPosition(),
			horizontalElevator.getPosition(),
			shoulder.getPosition(),
			wrist.getPosition()
		);
	}

    /**
     * @return Whether or not the shoulder will collide with the top of the elevator when we
     * move the shoulder to its final position first.
     */
    private boolean willCollideWithElevator(SuperstructurePosition currentPosition, SuperstructurePosition finalPosition) {
        final double kShoulderStowAngle = 170.0;

        boolean isShoulderCloseEnoughForCollision = currentPosition.canShoulderCollideWithElevator();
        Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
        boolean willCollideWithElevator = isShoulderCloseEnoughForCollision && 
                Util.isInRange(elevatorCollisionAngle.getDegrees(), currentPosition.shoulderAngle, finalPosition.shoulderAngle);
        willCollideWithElevator |= currentPosition.isShoulderJointHigherThanElevatorBar() &&
                finalPosition.shoulderAngle >= kShoulderStowAngle;

        return willCollideWithElevator;
    }

    private boolean loweringElevatorCollidesWithGround(SuperstructurePosition currentPosition) {
        SuperstructurePosition elevatorClearancePosition = currentPosition.withVerticalHeight(kVerticalHeightForTopBarClearance);
        boolean elevatorClearanceCollidesWithGround = elevatorClearancePosition.collidesWithBumperOrGround();

        return elevatorClearanceCollidesWithGround;
    }

    private boolean rotatingShoulderCollidesWithBumper(SuperstructurePosition currentPosition, SuperstructurePosition finalPosition) {
        Rotation2d bumperCollisionAngle = currentPosition.getShoulderBumperCollisionAngle();
        boolean shoulderRotationCollidesWithBumper = currentPosition.canWristCollideWithShoulderRotation() && 
                Util.isInRange(bumperCollisionAngle.getDegrees(), currentPosition.shoulderAngle, finalPosition.shoulderAngle);

        return shoulderRotationCollidesWithBumper;
    }

    private boolean rotatingWristCausesCollision(SuperstructurePosition currentPosition, SuperstructurePosition finalPosition) {
        Rotation2d wristCollisionAngle = currentPosition.getWristCollisionAngle();
        boolean wristRotationCausesCollision = currentPosition.canWristCollideWithRotation() &&
                Util.isInRange(wristCollisionAngle.getDegrees(), currentPosition.wristAngle, finalPosition.wristAngle);

        return wristRotationCausesCollision;
    }

    private void updateShoulderAcceleration(SuperstructurePosition currentPosition, SuperstructurePosition finalPosition) {
        if (Math.abs(currentPosition.horizontalExtension - finalPosition.horizontalExtension) < 1.0) {
            shoulder.setAccelerationScalar(Constants.Shoulder.kFastAccelerationScalar);
        } else {
            shoulder.setAccelerationScalar(Constants.Shoulder.kAccelerationScalar);
        }
    }

    private Request getStowChoreography(SuperstructurePosition finalPosition) {
        SuperstructurePosition currentPosition = getPosition();
        Request finalElevatorRequest = new ParallelRequest(
            verticalElevator.heightRequest(finalPosition.verticalHeight),
            horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
        );

        updateShoulderAcceleration(currentPosition, finalPosition);

        Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
        if (Util.isInRange(currentPosition.shoulderAngle, 0.0, elevatorCollisionAngle.getDegrees()) &&
                (currentPosition.verticalHeight >= kVerticalHeightForStow || 
                        currentPosition.horizontalExtension >= kHorizontalExtensionForUprightShoulder)) {
            System.out.println(String.format("Branch 1, elevator collision angle is %.2f", elevatorCollisionAngle.getDegrees()));
            return new SequentialRequest(
                new ParallelRequest(
                    shoulder.angleRequest(90.0),
                    horizontalElevator.extensionRequest(kHorizontalExtensionForUprightShoulder),
                    new ParallelRequest(
                        verticalElevator.heightRequest(kVerticalHeightForTopBarClearance),
                        wrist.angleRequest(finalPosition.wristAngle)
                    ).withPrerequisites(() -> horizontalElevator.getPosition() < kHorizontalExtensionForGridClearance,
                            shoulder.anglePrerequisite(90.0))
                ),
                new ParallelRequest(
                    shoulder.angleRequest(finalPosition.shoulderAngle),
                    horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                ),
                verticalElevator.heightRequest(finalPosition.verticalHeight)
            );
        }

        if (willCollideWithElevator(currentPosition, finalPosition)) {
            System.out.println("Branch 2");
            return new SequentialRequest(
                new ParallelRequest(
                    shoulder.angleRequest(finalPosition.shoulderAngle),
                    wrist.angleRequest(finalPosition.wristAngle),
                    verticalElevator.heightRequest(kVerticalHeightForTopBarClearance)
                            .withPrerequisite(() -> shoulder.getPosition() > kShoulderAngleForHorizontalRetraction),
                    horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach)
                            .withPrerequisite(verticalElevator.heightPrerequisite(kVerticalHeightForTopBarClearance))
                ),
                finalElevatorRequest
            );
        }

        System.out.println("Branch 3");
        return new SequentialRequest(
            wrist.angleRequest(finalPosition.wristAngle),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            finalElevatorRequest
        );
    }

    public Request getFullStowChoreography(boolean zeroHorizontalElevator) {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            0.25,
            170.0,
            -146.0
        );
        Request zeroingRequest = zeroHorizontalElevator ?
                new LambdaRequest(() -> horizontalElevator.startCurrentZeroing()) :
                new EmptyRequest();

        return getStowChoreography(finalPosition);
    }

    public Request getConeStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            0.25,
            147.0,
            95.0
        );

        return getStowChoreography(finalPosition);
    }

    public Request getCubeStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            0.25,
            158.0,
            98.0
        );

        return getStowChoreography(finalPosition);
    }

    public Request getCommunityConeHoldChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            0.25,
            124.0,
            95.0
        );

        return getStowChoreography(finalPosition);
    }

    private Request getLowChoreography(SuperstructurePosition finalPosition) {
        SuperstructurePosition currentPosition = getPosition();

        updateShoulderAcceleration(currentPosition, finalPosition);

        Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
        if (currentPosition.shoulderAngle >= elevatorCollisionAngle.getDegrees()) {
            Prerequisite shoulderEscapedPrereq = () -> shoulder.getPosition() < kShoulderAngleForEscapingElevator;

            System.out.println("Branch 1");
            return new SequentialRequest(
                new ParallelRequest(
                    verticalElevator.heightRequest(kVerticalHeightForTopBarClearance),
                    horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach)
                ),
                new ParallelRequest(
                    shoulder.angleRequest(finalPosition.shoulderAngle),
                    new ParallelRequest(
                        verticalElevator.heightRequest(finalPosition.verticalHeight),
                        wrist.angleRequest(finalPosition.wristAngle)
                    ).withPrerequisite(shoulderEscapedPrereq),
                    horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                            .withPrerequisite(() -> shoulder.getPosition() < kShoulderAngleForHorizontalExtension)
                )
            );
        }

        if (currentPosition.shoulderAngle > kShoulderAngleForHorizontalExtension) {
            System.out.println("Branch 2");
            return new ParallelRequest(
                horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                wrist.angleRequest(finalPosition.wristAngle),
                verticalElevator.heightRequest(finalPosition.verticalHeight),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                        .withPrerequisite(() -> shoulder.getPosition() < kShoulderAngleForHorizontalExtension)
            );
        }

        System.out.println("Branch 3");
        return new ParallelRequest(
            verticalElevator.heightRequest(finalPosition.verticalHeight),
            horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            wrist.angleRequest(finalPosition.wristAngle)
        );
    }

    public Request getConeIntakeChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.25, //0.875
            0.25, //0.25  230.26
            -55.76, //-50.76
            85.26 //90 38.6 35
        );

        return getLowChoreography(finalPosition);
    }

    public Request getCubeIntakeChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            4.6875, //7.0
            -87.4,
            93.0
        );
        
        return getLowChoreography(finalPosition);
    }

    private Request getHighChoreography(SuperstructurePosition finalPosition) {
        return getHighChoreography(finalPosition, 0.0);
    }

    private Request getHighChoreography(SuperstructurePosition finalPosition, double preemptiveExtensionSeconds) {
        SuperstructurePosition currentPosition = getPosition();

        updateShoulderAcceleration(currentPosition, finalPosition);

        Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
        if (currentPosition.shoulderAngle >= elevatorCollisionAngle.getDegrees() &&
                (currentPosition.verticalHeight >= kVerticalHeightForStow || finalPosition.verticalHeight >= kVerticalHeightForStow)) {
            double clearanceHeight = finalPosition.verticalHeight > kVerticalHeightForStow ? 
                    kVerticalHeightForStow : kVerticalHeightForTopBarClearance;
            System.out.println("Branch 1");
            return new SequentialRequest(
                new ParallelRequest(
                    verticalElevator.heightRequest(clearanceHeight),
                    new ParallelRequest(
                        shoulder.angleRequest(90.0),
                        horizontalElevator.extensionRequest(kHorizontalExtensionForUprightShoulder)
                    ).withPrerequisite(() -> verticalElevator.getPosition() <= kVerticalHeightForStow || 
                            verticalElevator.isAtPosition(kVerticalHeightForStow))
                ),
                new ParallelRequest(
                    verticalElevator.heightRequest(finalPosition.verticalHeight),
                    new ParallelRequest(
                        horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
                        shoulder.angleRequest(finalPosition.shoulderAngle),
                        wrist.angleRequest(finalPosition.wristAngle)   
                    ).withPrerequisite(verticalElevator.willReachHeightWithinTime(finalPosition.verticalHeight, preemptiveExtensionSeconds))
                )
            );
        }

        if (currentPosition.shoulderAngle >= elevatorCollisionAngle.getDegrees()) {
            System.out.println("Branch 2");
            return new ParallelRequest(
                verticalElevator.heightRequest(finalPosition.verticalHeight),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                wrist.angleRequest(finalPosition.wristAngle)
                        .withPrerequisite(() -> shoulder.getPosition() < 135.0)
            );
        }

        if (currentPosition.shoulderAngle < kShoulderAngleForHorizontalExtension) {
            System.out.println("Branch 3");
            return new ParallelRequest(
                horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                wrist.angleRequest(finalPosition.wristAngle),
                verticalElevator.heightRequest(finalPosition.verticalHeight),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                        .withPrerequisite(verticalElevator.willReachHeightWithinTime(finalPosition.verticalHeight, preemptiveExtensionSeconds))
            );
        }

        if (currentPosition.shoulderAngle >= kShoulderAngleForHorizontalExtension &&
                (currentPosition.verticalHeight >= kVerticalHeightForStow || finalPosition.verticalHeight >= kVerticalHeightForStow)) {
            System.out.println("Branch 4");
            return new SequentialRequest(
                new ParallelRequest(
                    shoulder.angleRequest(90.0),
                    horizontalElevator.extensionRequest(kHorizontalExtensionForUprightShoulder)
                ),
                new ParallelRequest(
                    verticalElevator.heightRequest(finalPosition.verticalHeight),
                    new ParallelRequest(
                        horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
                        shoulder.angleRequest(finalPosition.shoulderAngle),
                        wrist.angleRequest(finalPosition.wristAngle)
                    ).withPrerequisite(verticalElevator.willReachHeightWithinTime(finalPosition.verticalHeight, preemptiveExtensionSeconds))
                )
            );
        }

        System.out.println("Branch 5");
        return new ParallelRequest(
            verticalElevator.heightRequest(finalPosition.verticalHeight),
            horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            wrist.angleRequest(finalPosition.wristAngle)
        );
    }

    public Request getConeScanChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.625,
            0.25,
            37.0,
            95.0
        );

        return getHighChoreography(finalPosition);
    }

    public Request getHalfCubeStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            0.25,
            97.0,
            98.0
        );

        return getHighChoreography(finalPosition);
    }

    public Request getShelfChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            10.0,
            4.0,
            45.0,
            -10.0
        );

        return getHighChoreography(finalPosition);
    }
    public Request getShuttleChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.50,
            0.25,
            0.75,
            104.5
        );
        return getHighChoreography(finalPosition);
    }

    public Request getConeHighScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            16.0,
            25.0,
            34.75,
            10.25 //-4.75
        );

        return new SequentialRequest(
            getHighChoreography(finalPosition, 1.15),
            Superstructure.getInstance().waitRequest(0.125)
        );
    }

    public Request getConeMidScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            3.0,
            7.5,
            49.5,
            -4.4 //19.4
        );

        return getHighChoreography(finalPosition);
    }

    public Request getCubeMidScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            1,
            kCubeMidScoringHorizontalExtension,
            60.,
            90.0
        );

        return getHighChoreography(finalPosition);
    }

    public Request getCubeHighScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            16.0,
            kCubeHighScoringHorizontalExtension,
            60.0,
            90.0
        );

        return getHighChoreography(finalPosition, 3.0);
    }
}
