package com.team1323.frc2023.subsystems.superstructure;

import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.requests.EmptyRequest;
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

    private boolean willCollideWithElevator(SuperstructurePosition currentPosition, SuperstructurePosition finalPosition) {
        final double kShoulderStowAngle = 170.0;

        Rotation2d elevatorCollisionAngle = currentPosition.getElevatorCollisionAngle();
        boolean willCollideWithElevator = currentPosition.canShoulderCollideWithElevator() && 
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
        Rotation2d bumperCollisionAngle = currentPosition.getBumperCollisionAngle();
        boolean shoulderRotationCollidesWithBumper = currentPosition.canWristCollideWithBumper() && 
                Util.isInRange(bumperCollisionAngle.getDegrees(), currentPosition.shoulderAngle, finalPosition.shoulderAngle);

        return shoulderRotationCollidesWithBumper;
    }

    public Request getStowChoreography() {
        SuperstructurePosition finalPosition = new SuperstructurePosition(
            0.0,
            0.0,
            180.0,
            -180.0
        );
        SuperstructurePosition currentPosition = getPosition();

        if (willCollideWithElevator(currentPosition, finalPosition)) {
            Request elevatorClearanceRequest = verticalElevator.heightRequest(kVerticalHeightForTopBarClearance);

            if (loweringElevatorCollidesWithGround(currentPosition)) {
                Rotation2d bumperCollisionAngle = currentPosition.getBumperCollisionAngle();
                Prerequisite shoulderPastBumperPrereq = () -> shoulder.getPosition() > bumperCollisionAngle.getDegrees();
                Request bumperClearanceRequest = rotatingShoulderCollidesWithBumper(currentPosition, finalPosition) ? 
                        verticalElevator.heightRequest(kVerticalHeightForBumperClearance) :
                        new EmptyRequest();

                return new SequentialRequest(
                    bumperClearanceRequest,
                    new ParallelRequest(
                        shoulder.angleRequest(finalPosition.shoulderAngle),
                        wrist.angleRequest(finalPosition.wristAngle).withPrerequisite(shoulderPastBumperPrereq),
                        elevatorClearanceRequest.withPrerequisite(shoulderPastBumperPrereq)
                    ),
                    new ParallelRequest(
                        verticalElevator.heightRequest(finalPosition.verticalHeight),
                        horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                    )   
                );
            }

            return new SequentialRequest(
                new ParallelRequest(
                    elevatorClearanceRequest,
                    wrist.angleRequest(finalPosition.wristAngle)
                ),
                shoulder.angleRequest(finalPosition.shoulderAngle),
                new ParallelRequest(
                    verticalElevator.heightRequest(finalPosition.verticalHeight),
                    horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
                )   
            );
        }

        return new SequentialRequest(
            wrist.angleRequest(finalPosition.wristAngle),
            shoulder.angleRequest(finalPosition.shoulderAngle),
            new ParallelRequest(
                verticalElevator.heightRequest(finalPosition.verticalHeight),
                horizontalElevator.extensionRequest(finalPosition.horizontalExtension)
            )
        );
    }
}
