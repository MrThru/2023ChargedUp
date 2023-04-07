package com.team1323.frc2023.auto.modes;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.auto.actions.SetTrajectoryAction;
import com.team1323.frc2023.auto.actions.WaitForRemainingTimeAction;
import com.team1323.frc2023.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2023.auto.actions.WaitToIntakeAction;
import com.team1323.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class HighLinkMode extends HighLinkBaseMode {
    public HighLinkMode(Quadrant quadrant) {
        super(quadrant);
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        super.routine();

        // Finish intaking the second cone
        Pose2d coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition(AutoZones.mirror(Constants.kSecondPickupConePosition, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            coneIntakingPosition = coneIntakingPosition.transformBy(Pose2d.fromTranslation(new Translation2d(quadrant.hasBump() ? 4 : 0, (quadrant == Quadrant.TOP_RIGHT) ? 3 : 0)));
            Swerve.getInstance().startVisionPID(coneIntakingPosition, coneIntakingPosition.getRotation(), false,
                    new VisionPIDBuilder()
                            .withLateralPID(new SynchronousPIDF(0.07, 0.0, 0.0))
                            .withForwardPID(new SynchronousPIDF(0.02, 0.0, 0.0))
                            .withDecelerationRamp(new TwoPointRamp(
                                new Translation2d(1.0, 0.2),
                                new Translation2d(60.0, 0.4),
                                1.0,
                                true
                            ))
                            .build());
            runAction(new WaitToIntakeAction(Claw.HoldingObject.Cone, 4.0));
        } else {
            runAction(new WaitToFinishPathAction(4.0));
        }

        // Score the second cone
        runAction(new SetTrajectoryAction(trajectories.thirdPieceToEdgeColumn, Rotation2d.fromDegrees(180), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(120, quadrant));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cone) {
            NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.MIDDLE, Column.LEFT), quadrant);
            Superstructure.getInstance().scoringSequence(nodeLocation);
            runAction(new WaitForRemainingTimeAction(0.5, startTime));
            if (Swerve.getInstance().getDistanceToTargetPosition() <= 6.0) {
                Claw.getInstance().conformToState(Claw.ControlState.CONE_OUTAKE);
            }
            runAction(new WaitForRemainingTimeAction(0.125, startTime));
            if (Claw.getInstance().getState() == Claw.ControlState.CONE_OUTAKE) {
                Claw.getInstance().setCurrentHoldingObject(HoldingObject.None);
            }
        } else {
            runAction(new WaitToFinishPathAction());
            Superstructure.getInstance().objectAwareStowSequence();
        }

        System.out.println("Auto Done in: " + currentTime());
    }
}
