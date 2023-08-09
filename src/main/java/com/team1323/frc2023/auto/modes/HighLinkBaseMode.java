package com.team1323.frc2023.auto.modes;

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
import com.team1323.frc2023.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.superstructure.ChoreographyProvider;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class HighLinkBaseMode extends AutoModeBase {
    protected final Quadrant quadrant;
    protected final boolean scoreCubeHigh;
    private boolean scoredCube = false;

    public HighLinkBaseMode(Quadrant quadrant, boolean scoreCubeHigh) {
        this.quadrant = quadrant;
        this.scoreCubeHigh = scoreCubeHigh;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.kAutoStartingPose, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);

        // Score first cone and drive off
        Claw.getInstance().conformToState(Claw.ControlState.AUTO_CONE_HOLD);
        Superstructure.getInstance().coneHighScoreManual();
        runAction(new WaitForSuperstructureAction(2.0));
        Rotation2d targetHeading = Rotation2d.fromDegrees(quadrant.hasBump() ? -171 : 180);
        runAction(new SetTrajectoryAction(trajectories.secondPiecePickupPath, targetHeading, 0.75, quadrant));
        runAction(new WaitToEjectObjectAction(1.5));
        Superstructure.getInstance().request(SuperstructureCoordinator.getInstance().getCommunityConeHoldChoreography());

        // Intake cube and score it
        runAction(new WaitToPassXCoordinateAction(160.0, quadrant));
        Superstructure.getInstance().intakeState(Tunnel.State.SINGLE_INTAKE);
        runAction(new WaitToIntakeCubeAction(1.5, true));
        CubeIntake.getInstance().setPosition(Constants.CubeIntake.kIntakeAngle - 5.0);
        Claw.getInstance().conformToState(Claw.ControlState.CUBE_INTAKE);
        runAction(new SetTrajectoryAction(trajectories.secondPieceToCubeScore, Rotation2d.fromDegrees(180), 0.75, quadrant));
        runAction(new WaitToIntakeCubeAction(1.0));
        final ChoreographyProvider postHandoffChoreo = scoreCubeHigh ?
                SuperstructureCoordinator.getInstance()::getHalfCubeStowChoreography :
                SuperstructureCoordinator.getInstance()::getHalfCubeStowChoreography;
        Superstructure.getInstance().request(new SequentialRequest(
            Superstructure.getInstance().getPostIntakeState(0),
            Superstructure.getInstance().getHandOffCubeState(postHandoffChoreo)
        ));
        if (quadrant.hasBump()) {
            runAction(new WaitToFinishPathAction(1.5));
            runAction(new WaitToIntakeAction(HoldingObject.Cube, 0.75));
        } else {
            runAction(new WaitToPassXCoordinateAction(100, quadrant, 1.5));
            runAction(new WaitToIntakeAction(HoldingObject.Cube, 1.5));
        }
        if (Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cube) {
            if (scoreCubeHigh) {
                Superstructure.getInstance().cubeHighScoringSequence(
                    ScoringPoses.getCenterScoringPose(Swerve.getInstance().getPose()), 
                    new VisionPIDBuilder()
                            .withDecelerationRamp(new TwoPointRamp(
                                new Translation2d(1.0, 0.1),
                                new Translation2d(60.0, 0.5),
                                1.0,
                                true
                            ))
                            .build(),
                    false
                );
            } else {
                Superstructure.getInstance().cubeMidScoringSequence(ScoringPoses.getCenterScoringPose(Swerve.getInstance().getPose()), false);
            }
            scoredCube = true;
            runAction(new WaitToEjectObjectAction(4.0));
        } else {
            runAction(new WaitToFinishPathAction(2.0));
            scoredCube = false;
        }

        // Intake second cone
        LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR);
        runAction(new SetTrajectoryAction(trajectories.cubeScoreToThirdPiece, Rotation2d.fromDegrees(45), 0.75, quadrant));
        if(!scoredCube) {
            runAction(new WaitAction(0.5));
            Superstructure.getInstance().request(SuperstructureCoordinator.getInstance().getFullStowChoreography(false));
        }
        runAction(new WaitToPassXCoordinateAction(140.0, quadrant));
        Superstructure.getInstance().coneIntakeWithoutScanSequence();
        runAction(new WaitToPassXCoordinateAction(quadrant.hasBump() ? 234.0 : 230.0, quadrant));
    }
}
