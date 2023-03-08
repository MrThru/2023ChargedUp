// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.auto.AutoModeBase;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.auto.actions.ResetPoseAction;
import com.team1323.frc2023.auto.actions.SetTrajectoryAction;
import com.team1323.frc2023.auto.actions.WaitForRemainingTimeAction;
import com.team1323.frc2023.auto.actions.WaitForSuperstructureAction;
import com.team1323.frc2023.auto.actions.WaitToEjectObjectAction;
import com.team1323.frc2023.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2023.auto.actions.WaitToIntakeAction;
import com.team1323.frc2023.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TwoConesOneCubeMidMode extends AutoModeBase {
    private final Quadrant quadrant;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.secondPiecePickupPath.get(Quadrant.TOP_LEFT), trajectories.secondPieceToCubeScore.get(Quadrant.TOP_LEFT),
                    trajectories.cubeScoreToThirdPiece.get(Quadrant.TOP_LEFT), trajectories.thirdPieceToSecondConeColumn.get(Quadrant.TOP_LEFT), 
                    trajectories.thirdPieceToBridgePath.get(Quadrant.TOP_LEFT));
    }
    
    public TwoConesOneCubeMidMode(Quadrant quadrant) {
        this.quadrant = quadrant;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.kAutoStartingPose, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        Superstructure.getInstance().coneMidScoreManual();
        runAction(new WaitForSuperstructureAction());
        Rotation2d targetHeading = Rotation2d.fromDegrees(quadrant.hasBump() ? -170 : 180);
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath, targetHeading, 0.75, quadrant));
        
        runAction(new WaitToEjectObjectAction(5.0));
        Superstructure.getInstance().request(SuperstructureCoordinator.getInstance().getFullStowChoreography(false));

        // Cube Intake
        runAction(new WaitToPassXCoordinateAction(200.0, quadrant));
        Superstructure.getInstance().intakeState(Tunnel.State.SINGLE_INTAKE);
        runAction(new WaitToIntakeCubeAction(5.0));
        runAction(new SetTrajectoryAction(trajectories.secondPieceToCubeScore, Rotation2d.fromDegrees(180), 0.75, quadrant));
        Superstructure.getInstance().postIntakeState(0);
        runAction(new WaitForSuperstructureAction());
        Superstructure.getInstance().handOffCubeState(SuperstructureCoordinator.getInstance()::getAutoCubeHoldChoreography);
        runAction(new WaitToPassXCoordinateAction(110, quadrant));
        runAction(new WaitToIntakeAction(HoldingObject.Cube));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cube) {
            Superstructure.getInstance().cubeMidScoringSequence(ScoringPoses.getCenterScoringPose(Swerve.getInstance().getPose()), false);
            runAction(new WaitToEjectObjectAction(5.0));
        } else {
            runAction(new WaitToFinishPathAction());
        }

        // Cone Intake
        LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR);
        runAction(new SetTrajectoryAction(trajectories.cubeScoreToThirdPiece, Rotation2d.fromDegrees(45), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(140.0, quadrant));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(quadrant.hasBump() ? 240.0 : 220.0, quadrant));
        Pose2d coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition(AutoZones.mirror(Constants.kSecondPickupConePosition, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            coneIntakingPosition = coneIntakingPosition.transformBy(Pose2d.fromTranslation(new Translation2d(quadrant.hasBump() ? 4 : 0, 0)));
            Swerve.getInstance().startVisionPID(coneIntakingPosition, coneIntakingPosition.getRotation(), false,
                    new SynchronousPIDF(0.07, 0.0, 0.0),
                    new SynchronousPIDF(0.02, 0.0, 0.0),
                    new TwoPointRamp(
                        new Translation2d(1.0, 0.1), 
                        new Translation2d(60.0, 0.4), 
                        1.0, 
                        true
                    ));
            runAction(new WaitToIntakeAction(Claw.HoldingObject.Cone));
        } else {
            runAction(new WaitToFinishPathAction());
        }
        runAction(new SetTrajectoryAction(trajectories.thirdPieceToSecondConeColumn, Rotation2d.fromDegrees(180), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(110, quadrant));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cone) {
            NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.MIDDLE, Column.RIGHT), quadrant);
            Superstructure.getInstance().scoringSequence(nodeLocation);
            runAction(new WaitForRemainingTimeAction(0.5, startTime));
            if (Swerve.getInstance().getDistanceToTargetPosition() <= 4.0) {
                Claw.getInstance().conformToState(Claw.ControlState.CONE_OUTAKE);
            }
            runAction(new WaitForRemainingTimeAction(0.1, startTime));
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
