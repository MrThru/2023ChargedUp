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
import com.team1323.frc2023.auto.actions.WaitAction;
import com.team1323.frc2023.auto.actions.WaitForSuperstructureAction;
import com.team1323.frc2023.auto.actions.WaitToEjectObjectAction;
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
public class ThreeConesMode extends AutoModeBase {

    private final Quadrant quadrant;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.secondPiecePickupPath.topLeft, trajectories.secondPiecePickupPath.topLeft, trajectories.secondPieceToEdgeColumn.topLeft,
                trajectories.edgeColumnToThirdPiece.topLeft, trajectories.thirdPieceToSecondConeColumn.topLeft);
    }

    public ThreeConesMode(Quadrant quadrant) {
        this.quadrant = quadrant;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseAction(Constants.kAutoStartingPose, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR);
        runAction(new WaitAction(0.5));
        startTime = Timer.getFPGATimestamp();
        Superstructure.getInstance().coneHighScoreManual();
        runAction(new WaitForSuperstructureAction());
        runAction(new WaitToEjectObjectAction());
        Superstructure.getInstance().request(SuperstructureCoordinator.getInstance().getFullStowChoreography(false));
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath, Rotation2d.fromDegrees(10), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(130.0, quadrant));
        Swerve.getInstance().setPathHeading(AutoZones.mirror(Rotation2d.fromDegrees(0), quadrant));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(207.0, quadrant));
        Pose2d coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition(AutoZones.mirror(Constants.kFirstPickupConePosition, quadrant));
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            coneIntakingPosition = coneIntakingPosition.transformBy(Pose2d.fromTranslation(new Translation2d(0, 3)));
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
        runAction(new SetTrajectoryAction(trajectories.secondPieceToEdgeColumn, Rotation2d.fromDegrees(170), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(110, quadrant));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cone) {
            NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.MIDDLE, Column.LEFT), quadrant);
            Superstructure.getInstance().scoringSequence(nodeLocation);
            runAction(new WaitToEjectObjectAction());
        } else {
            runAction(new WaitToFinishPathAction());
            Superstructure.getInstance().objectAwareStowSequence();
        }

        //Second Cone Intake
        LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR);
        runAction(new SetTrajectoryAction(trajectories.edgeColumnToThirdPiece, Rotation2d.fromDegrees(45), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(140.0, quadrant));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(220.0, quadrant));
        coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition(AutoZones.mirror(Constants.kSecondPickupConePosition, quadrant));
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            coneIntakingPosition = coneIntakingPosition.transformBy(Pose2d.fromTranslation(new Translation2d(0, 3)));
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
            runAction(new WaitToEjectObjectAction());
        } else {
            runAction(new WaitToFinishPathAction());
        }
        System.out.println("Auto Done in: " + currentTime());
    }
    
}
