package com.team1323.frc2023.requests.auto.routines;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.requests.IfRequest;
import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.requests.WaitRequest;
import com.team1323.frc2023.requests.auto.ResetPoseRequest;
import com.team1323.frc2023.requests.auto.SetTrajectoryRequest;
import com.team1323.frc2023.requests.auto.WaitForSuperstructureRequest;
import com.team1323.frc2023.requests.auto.WaitToEjectObjectRequest;
import com.team1323.frc2023.requests.auto.WaitToFinishPathRequest;
import com.team1323.frc2023.requests.auto.WaitToIntakeCubeRequest;
import com.team1323.frc2023.requests.auto.WaitToIntakeRequest;
import com.team1323.frc2023.requests.auto.WaitToPassXCoordinateRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Rotation2d;

public class MidLinkBaseRoutine extends AutoRoutine {
    private final Quadrant quadrant;
    private final Superstructure s;
    private boolean scoredCube = false;

    public MidLinkBaseRoutine(Quadrant quadrant) {
        this.quadrant = quadrant;
        this.s = Superstructure.getInstance();
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
            new LambdaRequest(() -> s.coneMidScoreManual()),
            new WaitForSuperstructureRequest(2.0),
            new SetTrajectoryRequest(trajectories.slowSecondPiecePickupPath, Rotation2d.fromDegrees(quadrant.hasBump() ? -170 : 180), 0.75, quadrant),
            new WaitToEjectObjectRequest(1.5),
            new LambdaRequest(() -> s.request(SuperstructureCoordinator.getInstance().getCommunityConeHoldChoreography()))
        );

        final SequentialRequest intakeCubeAndScore = new SequentialRequest(
            new WaitToPassXCoordinateRequest(200.0, quadrant, 3.0),
            new LambdaRequest(() -> s.intakeState(Tunnel.State.SINGLE_INTAKE)),
            new WaitToIntakeCubeRequest(false, 1.5),
            new LambdaRequest(() -> Claw.getInstance().conformToState(Claw.ControlState.CUBE_INTAKE)),
            new SetTrajectoryRequest(trajectories.slowSecondPieceToCubeScore, Rotation2d.fromDegrees(180), 0.75, quadrant),
            new LambdaRequest(() -> s.postIntakeState(0)),
            new WaitToPassXCoordinateRequest(242.0, quadrant, 1.5),
            new LambdaRequest(() -> s.handOffCubeState(SuperstructureCoordinator.getInstance()::getAutoCubeHoldChoreography)),
            new WaitToPassXCoordinateRequest(110.0, quadrant, 1.5),
            new WaitToIntakeRequest(HoldingObject.Cube, 1.5),
            new IfRequest(
                () -> Claw.getInstance().getCurrentHoldingObject() == HoldingObject.Cube,
                new SequentialRequest(
                    new LambdaRequest(() -> s.cubeMidScoringSequence(ScoringPoses.getCenterScoringPose(Swerve.getInstance().getPose()), false)),
                    new WaitToEjectObjectRequest(4.0),
                    new LambdaRequest(() -> setScoredCube(true))
                ),
                new SequentialRequest(
                    new WaitToFinishPathRequest(2.0),
                    new LambdaRequest(() -> setScoredCube(false))
                )
            )
        );

        final SequentialRequest intakeSecondCone = new SequentialRequest(
            new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.DETECTOR)),
            new SetTrajectoryRequest(trajectories.slowCubeScoreToThirdPiece, Rotation2d.fromDegrees(45), 0.75, quadrant),
            new IfRequest(
                () -> !getScoredCube(),
                new SequentialRequest(
                    new WaitRequest(0.5),
                    new LambdaRequest(() -> s.request(SuperstructureCoordinator.getInstance().getFullStowChoreography(false)))
                )
            ),
            new WaitToPassXCoordinateRequest(140.0, quadrant, 3.0),
            new LambdaRequest(() -> s.coneIntakeWithoutScanSequence()),
            new WaitToPassXCoordinateRequest(quadrant.hasBump() ? 240.0 : 230.0, quadrant, 3.0)
        );

        return new SequentialRequest(setUp, scoreFirstConeAndLeave, intakeCubeAndScore, intakeSecondCone);
    }
}
