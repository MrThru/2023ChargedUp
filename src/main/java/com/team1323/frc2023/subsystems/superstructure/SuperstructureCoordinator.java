package com.team1323.frc2023.subsystems.superstructure;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Settings;
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
    public static final double kConeIntakingWristAngle = 85.26; //85.26
    public static final double kCubeHoldingWristAngle = 105.0;

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

    private Request getStowChoreography(SuperstructurePosition finalPosition) {
        SuperstructurePosition currentPosition = getPosition();
        Request finalElevatorRequest = new ParallelRequest(
            verticalElevator.heightRequest(finalPosition.verticalHeight),
            horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
        );

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
                    ).withPrerequisites(horizontalElevator.willReachExtensionWithinTime(kHorizontalExtensionForUprightShoulder, 1.0) /*horizontalElevator.getPosition() < kHorizontalExtensionForGridClearance,*/
                            /*shoulder.anglePrerequisite(90.0)*/)
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
            Constants.Wrist.kMinControlAngle
        );
        Request zeroingRequest = zeroHorizontalElevator ?
                new LambdaRequest(() -> {
                    horizontalElevator.startCurrentZeroing();
                    verticalElevator.startCurrentZeroing();
                }) :
                new EmptyRequest();

        System.out.println("Full stow choreo");

        return new SequentialRequest(
            getStowChoreography(finalPosition),
            zeroingRequest
        );
    }

    public Request getConeStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            0.25,
            147.0,
            95.0
        );

        System.out.println("Cone stow choreo");

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
            -55.76, //-50.76 : -55.76
            kConeIntakingWristAngle //90 38.6 35 
        );

        System.out.println("Cone intake choreo");

        return getLowChoreography(finalPosition);
    }

    public Request getCubeIntakeChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            1.0,
            4.6875, //7.0
            -87.4,
            kCubeHoldingWristAngle
        );

        System.out.println("Cube intake choreo");
        
        return getLowChoreography(finalPosition);
    }

    public Request getConeFlipChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.1,
            6.1,
            Settings.kIsUsingCompBot ? -95.0 : -97.5,
            133.5
        );

        System.out.println("Cone flip choreo");
        
        return getLowChoreography(finalPosition);
    }

    private Request getHighChoreography(SuperstructurePosition finalPosition) {
        return getHighChoreography(finalPosition, 0.0);
    }

    private Request getHighChoreography(SuperstructurePosition finalPosition, double preemptiveExtensionSeconds) {
        SuperstructurePosition currentPosition = getPosition();

        Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
        if (currentPosition.shoulderAngle >= elevatorCollisionAngle.getDegrees() &&
                (currentPosition.verticalHeight >= kVerticalHeightForStow || finalPosition.verticalHeight >= kVerticalHeightForStow)) {
            double clearanceHeight = finalPosition.verticalHeight > kVerticalHeightForStow ? 
                    kVerticalHeightForStow : kVerticalHeightForTopBarClearance;
            double shoulderClearanceAngle = finalPosition.shoulderAngle <= 45.0 ? 45.0 : 90.0;
            System.out.println("Branch 1");
            return new SequentialRequest(
                new ParallelRequest(
                    verticalElevator.heightRequest(clearanceHeight),
                    new ParallelRequest(
                        shoulder.angleRequest(shoulderClearanceAngle),
                        horizontalElevator.extensionRequest(kHorizontalExtensionForUprightShoulder)
                    ).withPrerequisite(() -> verticalElevator.getPosition() <= kVerticalHeightForStow || 
                            verticalElevator.isAtPosition(kVerticalHeightForStow))
                ),
                new LambdaRequest(() -> System.out.println("Put the shoulder out.")),
                new ParallelRequest(
                    verticalElevator.heightRequest(finalPosition.verticalHeight),
                    shoulder.angleRequest(finalPosition.shoulderAngle)
                            .withPrerequisite(verticalElevator.willReachHeightWithinTime(finalPosition.verticalHeight, preemptiveExtensionSeconds + 0.5)),
                    new ParallelRequest(
                        horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
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
                        .withPrerequisites(verticalElevator.willReachHeightWithinTime(finalPosition.verticalHeight, preemptiveExtensionSeconds),
                                () -> shoulder.getPosition() > 0.0)
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
            kCubeHoldingWristAngle
        );

        return getHighChoreography(finalPosition);
    }

    public Request getAutoCubeHoldChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            1,
            0.25,
            55.0,
            kCubeHoldingWristAngle
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
            17.5,
            25.0,
            34.75,
            -4.75 //-4.75
        );

        System.out.println("Cone high scoring choreo");

        return new SequentialRequest(
            getHighChoreography(finalPosition, 1.15),
            Superstructure.getInstance().waitRequest(0.125)
        );
    }

    public Request getConeMidScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.5,
            11.5,
            49.5,
            -14.4 //19.4
        );

        System.out.println("Cone mid scoring choreo");

        return getHighChoreography(finalPosition);
    }

    public Request getCubeMidScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            1,
            kCubeMidScoringHorizontalExtension,
            55.0,
            kCubeHoldingWristAngle
        );

        System.out.println("Cube mid scoring choreo");

        return getHighChoreography(finalPosition);
    }

    public Request getCubeHighScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            16.0,
            kCubeHighScoringHorizontalExtension,
            55.0,
            kCubeHoldingWristAngle
        );

        System.out.println("Cube high scoring choreo");

        return getHighChoreography(finalPosition, 3.0);
    }
}
