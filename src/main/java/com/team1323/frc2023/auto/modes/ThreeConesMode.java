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
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class ThreeConesMode extends AutoModeBase {

    private final Quadrant quadrant;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.secondPiecePickupPath.topLeft, trajectories.secondConeHighScorePath.topLeft);
    }

    public ThreeConesMode(Quadrant quadrant) {
        this.quadrant = quadrant;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.kAutoStartingPose, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR);
        runAction(new WaitAction(0.5));
        Superstructure.getInstance().coneMidScoreManual();
        runAction(new WaitForSuperstructureAction());
        Superstructure.getInstance().objectAwareStowSequence();
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath, Rotation2d.fromDegrees(20), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(154.0, quadrant));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(185.0, quadrant));
        Pose2d coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition();
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            Swerve.getInstance().startVisionPID(coneIntakingPosition, coneIntakingPosition.getRotation(), false);
            runAction(new WaitToIntakeAction(Claw.HoldingObject.Cone));
        } else {
            runAction(new WaitToFinishPathAction());
        }
        runAction(new SetTrajectoryAction(trajectories.secondConeHighScorePath, Rotation2d.fromDegrees(180), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(110, quadrant));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cone) {
            NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.MIDDLE, Column.RIGHT), quadrant);
            Superstructure.getInstance().scoringSequence(nodeLocation);
            runAction(new WaitForSuperstructureAction());
        } else {
            runAction(new WaitToFinishPathAction());
        }

        //Second Cone Intake
        runAction(new SetTrajectoryAction(trajectories.secondConeScoreToThirdConePickup, Rotation2d.fromDegrees(45), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(153.0, quadrant));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(173, quadrant));
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            Swerve.getInstance().startVisionPID(coneIntakingPosition, coneIntakingPosition.getRotation(), false);
            runAction(new WaitToIntakeAction(Claw.HoldingObject.Cone));
        } else {
            runAction(new WaitToFinishPathAction());
        }
        runAction(new SetTrajectoryAction(trajectories.thirdConePickupToScorePath, Rotation2d.fromDegrees(180), 0.75, quadrant));
        runAction(new WaitToPassXCoordinateAction(110, quadrant));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cone) {
            NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.TOP, Column.RIGHT), quadrant);
            Superstructure.getInstance().scoringSequence(nodeLocation);
            runAction(new WaitForSuperstructureAction());
        } else {
            runAction(new WaitToFinishPathAction());
        }
        runAction(new WaitForSuperstructureAction());
        Superstructure.getInstance().objectAwareStowSequence();
        runAction(new WaitForSuperstructureAction());
        System.out.println("Auto Done in: " + currentTime());        
    }
    
}
