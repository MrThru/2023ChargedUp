package com.team1323.frc2023.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.auto.AutoModeBase;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.auto.SmartDashboardInteractions.StartingSide;
import com.team1323.frc2023.auto.actions.ResetPoseAction;
import com.team1323.frc2023.auto.actions.SetTrajectoryAction;
import com.team1323.frc2023.auto.actions.WaitAction;
import com.team1323.frc2023.auto.actions.WaitForSuperstructureAction;
import com.team1323.frc2023.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2023.auto.actions.WaitToIntakeAction;
import com.team1323.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2023.auto.actions.WaitToPassYCoordinateAction;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.ScoringPoses;
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
        LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR);
        runAction(new WaitAction(0.5));
        Superstructure.getInstance().coneHighScoreManual();
        runAction(new WaitForSuperstructureAction());
        Superstructure.getInstance().objectAwareStowSequence();
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath.topLeft, Rotation2d.fromDegrees(-20), 0.75));
        runAction(new WaitToPassXCoordinateAction(154.0));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(185.0));
        Pose2d coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition();
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            Swerve.getInstance().startVisionPID(coneIntakingPosition, coneIntakingPosition.getRotation(), false);
            runAction(new WaitToIntakeAction(Claw.HoldingObject.Cone));
        } else {
            runAction(new WaitToFinishPathAction());
        }
        runAction(new SetTrajectoryAction(trajectories.secondConeHighScorePath.topLeft, Rotation2d.fromDegrees(180), 0.75));
        runAction(new WaitToPassXCoordinateAction(110));
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cone) {
            NodeLocation nodeLocation = new NodeLocation(Grid.RIGHT, Row.TOP, Column.LEFT);
            Superstructure.getInstance().scoringSequence(nodeLocation);
            runAction(new WaitForSuperstructureAction());
        } else {
            runAction(new WaitToFinishPathAction());
        }
    }
}
