package com.team1323.frc2023.subsystems.superstructure;

import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
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

    private static final double kVerticalHeightForTopBarClearance = 0.0;
    private static final double kVerticalHeightForBumperClearance = 16.0;
    private static final double kShoulderAngleForHorizontalRetraction = -60.0;
    private static final double kShoulderAngleForHorizontalExtension = 0.0;
    private static final double kHorizontalExtensionForMinShoulderReach = 0.0;

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

	private SuperstructurePosition getPosition() {
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

        if (!willCollideWithElevator(currentPosition, finalPosition) && 
                willCollideWithElevator(currentPosition.withHorizontalExtension(finalPosition.horizontalExtension), finalPosition)) {
            // The superstructure is likely in a scoring position, and it would be dangerous to 
            // bring the elevator down while everything is extended.
            return new SequentialRequest(
                new ParallelRequest(
                    shoulder.angleRequest(finalPosition.shoulderAngle),
                    wrist.angleRequest(finalPosition.wristAngle)
                ),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
                verticalElevator.heightRequest(finalPosition.verticalHeight)
            );
        }

        if (willCollideWithElevator(currentPosition, finalPosition)) {
            Request elevatorClearanceRequest = verticalElevator.heightRequest(kVerticalHeightForTopBarClearance);

            if (currentPosition.shoulderAngle < 0.0) {
                Prerequisite shoulderEscapedPrereq = () -> shoulder.getPosition() > kShoulderAngleForHorizontalRetraction;
                return new SequentialRequest(
                    new ParallelRequest(
                        shoulder.angleRequest(finalPosition.shoulderAngle),
                        wrist.angleRequest(finalPosition.wristAngle),
                        elevatorClearanceRequest.withPrerequisite(shoulderEscapedPrereq)
                    ),
                    finalElevatorRequest
                );
            }

            return new SequentialRequest(
                new ParallelRequest(
                    elevatorClearanceRequest,
                    wrist.angleRequest(finalPosition.wristAngle)
                ),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                finalElevatorRequest
            );
        }

        return new SequentialRequest(
            wrist.angleRequest(finalPosition.wristAngle),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            finalElevatorRequest
        );
    }

    public Request getFullStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.0,
            0.0,
            180.0,
            -135.0
        );

        return getStowChoreography(finalPosition);
    }

    public Request getConeStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.0,
            0.0,
            135.0,
            90.0
        );

        return getStowChoreography(finalPosition);
    }

    private Request getLowChoreography(SuperstructurePosition finalPosition) {
        SuperstructurePosition currentPosition = getPosition();

        if (willCollideWithElevator(currentPosition, finalPosition) ||
                willCollideWithElevator(currentPosition.withVerticalHeight(finalPosition.verticalHeight), finalPosition)) {
            Prerequisite shoulderEscapedPrereq = () -> shoulder.getPosition() < kShoulderAngleForHorizontalExtension;

            return new SequentialRequest(
                new ParallelRequest(
                    verticalElevator.heightRequest(kVerticalHeightForTopBarClearance),
                    horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach)
                ),
                new ParallelRequest(
                    shoulder.angleRequest(finalPosition.shoulderAngle),
                    new ParallelRequest(
                        verticalElevator.heightRequest(finalPosition.verticalHeight),
                        horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
                        wrist.angleRequest(finalPosition.wristAngle)
                    ).withPrerequisite(shoulderEscapedPrereq)
                )
            );
        }

        if (currentPosition.shoulderAngle > kShoulderAngleForHorizontalExtension) {
            return new ParallelRequest(
                horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                wrist.angleRequest(finalPosition.wristAngle),
                verticalElevator.heightRequest(finalPosition.verticalHeight),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                        .withPrerequisite(() -> shoulder.getPosition() < kShoulderAngleForHorizontalExtension)
            );
        }

        return new ParallelRequest(
            verticalElevator.heightRequest(finalPosition.verticalHeight),
            horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            wrist.angleRequest(finalPosition.wristAngle)
        );
    }

    public Request getConeIntakeChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            2.0,
            6.0,
            -90.0,
            30.0
        );

        return getLowChoreography(finalPosition);
    }

    public Request getCubeIntakeChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            12.0,
            16.0,
            -90.0,
            30.0
        );
        
        return getLowChoreography(finalPosition);
    }

    private Request getHighChoreography(SuperstructurePosition finalPosition) {
        SuperstructurePosition currentPosition = getPosition();

        if (willCollideWithElevator(currentPosition, finalPosition) ||
                willCollideWithElevator(currentPosition.withVerticalHeight(finalPosition.verticalHeight), finalPosition)) {
            Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
            Prerequisite shoulderEscapedPrereq = () -> shoulder.getPosition() < elevatorCollisionAngle.getDegrees();

            return new SequentialRequest(
                new ParallelRequest(
                    verticalElevator.heightRequest(kVerticalHeightForTopBarClearance),
                    horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach)
                ),
                new ParallelRequest(
                    shoulder.angleRequest(finalPosition.shoulderAngle),
                    new ParallelRequest(
                        verticalElevator.heightRequest(finalPosition.verticalHeight),
                        horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
                        wrist.angleRequest(finalPosition.wristAngle)
                    ).withPrerequisite(shoulderEscapedPrereq)
                )
            );
        }

        if (currentPosition.shoulderAngle < kShoulderAngleForHorizontalExtension) {
            return new ParallelRequest(
                horizontalElevator.extensionRequest(kHorizontalExtensionForMinShoulderReach),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                wrist.angleRequest(finalPosition.wristAngle),
                verticalElevator.heightRequest(finalPosition.verticalHeight),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                        .withPrerequisite(() -> shoulder.getPosition() > kShoulderAngleForHorizontalExtension)
            );
        }

        return new ParallelRequest(
            verticalElevator.heightRequest(finalPosition.verticalHeight),
            horizontalElevator.extensionRequest(finalPosition.horizontalExtension),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            wrist.angleRequest(finalPosition.wristAngle)
        );
    }

    public Request getConeHighScoringChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            20.0,
            16.0,
            45.0,
            -45.0
        );

        return getHighChoreography(finalPosition);
    }
}
