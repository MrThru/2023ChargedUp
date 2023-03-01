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
import com.team1323.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TwoConesOneCubeMidMode extends AutoModeBase {
    private final Quadrant quadrant;
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.secondPiecePickupPath.topLeft, trajectories.secondConeHighScorePath.topLeft,
                    trajectories.secondConeScoreToThirdConePickup.topLeft, trajectories.thirdPiecePickupToThirdPole.topLeft);
    }
    
    public TwoConesOneCubeMidMode(Quadrant quadrant) {
        this.quadrant = quadrant;
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseAction(Constants.kAutoStartingPose, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        runAction(new WaitAction(0.5));
        startTime = Timer.getFPGATimestamp();
        Superstructure.getInstance().coneMidScoreManual();
        runAction(new WaitForSuperstructureAction());
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath, Rotation2d.fromDegrees(180), 0.5, quadrant));
        runAction(new WaitToEjectObjectAction());
        runAction(new WaitToPassXCoordinateAction(200, quadrant));
        Superstructure.getInstance().intakeState(Tunnel.State.SINGLE_INTAKE);
    }

}
