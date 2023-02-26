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
import com.team1323.frc2023.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** A hella mid auto*/
public class ThreeMidConesMode extends AutoModeBase {
    Quadrant quadrant;
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.thirdPoleToSecondPiecePickup.topLeft, trajectories.thirdPoleToSecondPiecePickup.topLeft, trajectories.secondConeHighScorePath.topLeft,
                        trajectories.secondPoleToThirdPiecePickup.topLeft, trajectories.thirdPiecePickupToFirstPole.topLeft);
    }

    public ThreeMidConesMode(Quadrant quadrant) {
        this.quadrant = quadrant;
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.kThirdPoleStartingPose, quadrant));
        runAction(new SetTrajectoryAction(trajectories.thirdPoleToSecondPiecePickup, Rotation2d.fromDegrees(20), 0.75, quadrant));
        runAction(new WaitToFinishPathAction(5));
        runAction(new SetTrajectoryAction(trajectories.secondConeHighScorePath, Rotation2d.fromDegrees(180), 1.0, quadrant));
        runAction(new WaitToFinishPathAction(5));
        runAction(new SetTrajectoryAction(trajectories.secondPoleToThirdPiecePickup, Rotation2d.fromDegrees(45), 0.75, quadrant));
        runAction(new WaitToFinishPathAction(5));
        runAction(new SetTrajectoryAction(trajectories.thirdPiecePickupToFirstPole, Rotation2d.fromDegrees(180), 0.75, quadrant));
        runAction(new WaitToFinishPathAction(5));

        
        System.out.println(String.format("Auto Path Finished in %f", super.currentTime()));        
    
    }

}
