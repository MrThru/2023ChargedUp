package com.team1323.frc2023.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.auto.AutoModeBase;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.auto.SmartDashboardInteractions.StartingSide;
import com.team1323.frc2023.auto.actions.ResetPoseAction;
import com.team1323.frc2023.auto.actions.SetTrajectoryAction;
import com.team1323.frc2023.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class TwoConesAndRampMode extends AutoModeBase {
    private final StartingSide startingSide;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.secondPiecePickupPath.topLeft, trajectories.secondConeHighScorePath.topLeft);
    }

    public TwoConesAndRampMode(StartingSide startingSide) {
        this.startingSide = startingSide;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseAction(new Pose2d(new Translation2d(71, 20), Rotation2d.fromDegrees(180)).mirrorAboutY(108.19)));
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath.topLeft, Rotation2d.fromDegrees(-20), 0.75));
        runAction(new WaitToPassXCoordinateAction(154.0));
        Superstructure.getInstance().coneIntakeSequence();
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.secondConeHighScorePath.topLeft, Rotation2d.fromDegrees(180), 0.75));
        runAction(new WaitToFinishPathAction());
    }
}
