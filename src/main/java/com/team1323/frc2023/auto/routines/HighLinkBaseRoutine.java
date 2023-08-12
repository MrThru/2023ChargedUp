package com.team1323.frc2023.auto.routines;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.auto.ResetPoseRequest;
import com.team1323.frc2023.auto.SetTrajectoryRequest;
import com.team1323.frc2023.auto.WaitForSuperstructureRequest;
import com.team1323.frc2023.auto.WaitToEjectObjectRequest;
import com.team1323.frc2023.auto.WaitToFinishPathRequest;
import com.team1323.frc2023.auto.WaitToIntakeCubeRequest;
import com.team1323.frc2023.auto.WaitToIntakeRequest;
import com.team1323.frc2023.auto.WaitToPassXCoordinateRequest;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.requests.IfRequest;
import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.requests.WaitRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class HighLinkBaseRoutine extends AutoRoutine {
    private final Quadrant quadrant;
    private final boolean scoreCubeHigh;
    private final Swerve swerve;
    private final Claw claw;
    private final CubeIntake cubeIntake;
    private final Superstructure s;
    private boolean scoredCube = false;

    public HighLinkBaseRoutine(Quadrant quadrant, boolean scoreCubeHigh) {
        this.quadrant = quadrant;
        this.scoreCubeHigh = scoreCubeHigh;
        swerve = Swerve.getInstance();
        claw = Claw.getInstance();
        cubeIntake = CubeIntake.getInstance();
        s = Superstructure.getInstance();
    }

    private void setScoredCube(boolean scoredCube) {
        this.scoredCube = scoredCube;
    }

    private boolean getScoredCube() {
        return scoredCube;
    }

    @Override
    public Request getRoutine() {
        final SequentialRequest setUp = new SequentialRequest(
            new ResetPoseRequest(Constants.kAutoStartingPose, quadrant),
            new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL))
        );

        final SequentialRequest scoreFirstConeAndLeave = new SequentialRequest(
            claw.stateRequest(Claw.ControlState.AUTO_CONE_HOLD),
            new LambdaRequest(() -> s.coneHighScoreManual()),
            new WaitForSuperstructureRequest(2.0),
            new SetTrajectoryRequest(trajectories.secondPiecePickupPath, Rotation2d.fromDegrees(quadrant.hasBump() ? -171 : 180), 0.75, quadrant),
            new WaitToEjectObjectRequest(1.5),
            new LambdaRequest(() -> s.request(SuperstructureCoordinator.getInstance().getCommunityConeHoldChoreography()))
        );

        final SequentialRequest intakeCubeAndScore = new SequentialRequest(
            new WaitToPassXCoordinateRequest(160.0, quadrant, 4.0),
            new LambdaRequest(() -> s.intakeState(Tunnel.State.SINGLE_INTAKE)),
            new WaitToIntakeCubeRequest(true, 1.5),
            new LambdaRequest(() -> cubeIntake.setPosition(Constants.CubeIntake.kIntakeAngle - 5.0)),
            claw.stateRequest(Claw.ControlState.CUBE_INTAKE),
            new SetTrajectoryRequest(trajectories.secondPieceToCubeScore, Rotation2d.fromDegrees(180), 0.75, quadrant),
            new WaitToIntakeCubeRequest(false, 1.0),
            new LambdaRequest(() -> s.request(new SequentialRequest(s.getPostIntakeState(0),
                    s.getHandOffCubeState(SuperstructureCoordinator.getInstance()::getHalfCubeStowChoreography)))),
            new IfRequest(
                () -> quadrant.hasBump(),
                new SequentialRequest(
                    new WaitToFinishPathRequest(1.5),
                    new WaitToIntakeRequest(HoldingObject.Cube, 0.75)
                ),
                new SequentialRequest(
                    new WaitToPassXCoordinateRequest(100, quadrant, 1.5),
                    new WaitToIntakeRequest(HoldingObject.Cube, 1.5)
                )
            ),
            new IfRequest(
                () -> claw.getCurrentHoldingObject() == HoldingObject.Cube,
                new SequentialRequest(
                    new LambdaRequest(() -> {
                        if (scoreCubeHigh) {
                            s.cubeHighScoringSequence(
                                ScoringPoses.getCenterScoringPose(swerve.getPose()), 
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
                            s.cubeMidScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()), false);
                        }

                        setScoredCube(true);
                    }),
                    new WaitToEjectObjectRequest(4.0)
                ),
                new SequentialRequest(
                    new WaitToFinishPathRequest(2.0),
                    new LambdaRequest(() -> setScoredCube(false))
                )
            )
        );

        final SequentialRequest intakeSecondCone = new SequentialRequest(
            new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR)),
            new SetTrajectoryRequest(trajectories.cubeScoreToThirdPiece, Rotation2d.fromDegrees(45), 0.75, quadrant),
            new IfRequest(
                () -> !getScoredCube(),
                new SequentialRequest(
                    new WaitRequest(0.5),
                    new LambdaRequest(() -> s.request(SuperstructureCoordinator.getInstance().getFullStowChoreography(false)))
                )
            ),
            new WaitToPassXCoordinateRequest(140.0, quadrant, 4.0),
            new LambdaRequest(() -> s.coneIntakeWithoutScanSequence()),
            new WaitToPassXCoordinateRequest(quadrant.hasBump() ? 234.0 : 230.0, quadrant, 4.0)
        );

        return new SequentialRequest(setUp, scoreFirstConeAndLeave, intakeCubeAndScore, intakeSecondCone);
    }
}
