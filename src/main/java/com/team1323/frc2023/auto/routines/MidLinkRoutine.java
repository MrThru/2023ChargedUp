package com.team1323.frc2023.auto.routines;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.auto.SetTrajectoryRequest;
import com.team1323.frc2023.auto.WaitForRemainingTimeRequest;
import com.team1323.frc2023.auto.WaitToFinishPathRequest;
import com.team1323.frc2023.auto.WaitToIntakeRequest;
import com.team1323.frc2023.auto.WaitToPassXCoordinateRequest;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.requests.IfRequest;
import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class MidLinkRoutine extends AutoRoutine {
    private final Quadrant quadrant;
    private final Swerve swerve;
    private final Claw claw;
    private final Superstructure s;
    private Pose2d coneIntakingPosition;

    public MidLinkRoutine(Quadrant quadrant) {
        this.quadrant = quadrant;
        swerve = Swerve.getInstance();
        claw = Claw.getInstance();
        s = Superstructure.getInstance();
    }

    private void setConeIntakingPosition(Pose2d pose) {
        coneIntakingPosition = pose;
    }

    private Pose2d getConeIntakingPosition() {
        return coneIntakingPosition;
    }

    @Override
    public Request getRoutine() {
        final Request baseRoutine = new MidLinkBaseRoutine(quadrant).getRoutine();

        final SequentialRequest finishIntakingSecondCone = new SequentialRequest(
            new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL)),
            new IfRequest(
                () -> {
                    Pose2d intakingPosition = LimelightProcessor.getInstance()
                            .getRobotConePickupPosition(AutoZones.mirror(Constants.kSecondPickupConePosition, quadrant));
                    setConeIntakingPosition(intakingPosition);

                    return !intakingPosition.equals(Pose2d.identity());
                },
                new SequentialRequest(
                    new LambdaRequest(() -> {
                        Pose2d adjustedIntakingPosition = getConeIntakingPosition().transformBy(Pose2d.fromTranslation(
                                new Translation2d(quadrant.hasBump() ? 4 : 0, (quadrant == Quadrant.TOP_RIGHT) ? 3 : 0)));
                        
                        swerve.startVisionPID(adjustedIntakingPosition, adjustedIntakingPosition.getRotation(), false,
                                new VisionPIDBuilder()
                                        .withLateralPID(new SynchronousPIDF(0.07, 0.0, 0.0))
                                        .withForwardPID(new SynchronousPIDF(0.02, 0.0, 0.0))
                                        .withDecelerationRamp(new TwoPointRamp(
                                            new Translation2d(1.0, 0.2),
                                            new Translation2d(60.0, 0.4),
                                            1.0,
                                            true
                                        ))
                                        .build());
                    }),
                    new WaitToIntakeRequest(HoldingObject.Cone, 4.0)
                ),
                new WaitToFinishPathRequest(4.0)
            ),
            new IfRequest(
                () -> claw.getCurrentHoldingObject() != HoldingObject.Cone,
                new LambdaRequest(() -> s.request(SuperstructureCoordinator.getInstance().getConeStowChoreography()))
            )
        );

        final SequentialRequest scoreSecondCone = new SequentialRequest(
            new SetTrajectoryRequest(trajectories.thirdPieceToSecondConeColumn, Rotation2d.fromDegrees(180), 0.75, quadrant),
            new WaitToPassXCoordinateRequest(110.0, quadrant, 4.0),
            new IfRequest(
                () -> claw.getCurrentHoldingObject() == HoldingObject.Cone,
                new SequentialRequest(
                    new LambdaRequest(() -> {
                        NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.MIDDLE, Column.RIGHT), quadrant);
                        s.scoringSequence(nodeLocation);
                    }),
                    new WaitForRemainingTimeRequest(0.625, runtimeStopwatch),
                    new IfRequest(
                        () -> swerve.getDistanceToTargetPosition() <= 6.0,
                        claw.stateRequest(Claw.ControlState.CONE_OUTAKE)
                    ),
                    new WaitForRemainingTimeRequest(0.25, runtimeStopwatch),
                    new IfRequest(
                        () -> claw.getState() == Claw.ControlState.CONE_OUTAKE,
                        new LambdaRequest(() -> claw.setCurrentHoldingObject(HoldingObject.None))
                    )
                ),
                new SequentialRequest(
                    new WaitToFinishPathRequest(4.0),
                    new LambdaRequest(() -> s.objectAwareStowSequence())
                )
            )
        );
        
        return new SequentialRequest(
            getStartStopwatchRequest(),
            baseRoutine,
            finishIntakingSecondCone,
            scoreSecondCone,
            getPrintRuntimeRequest()
        );
    }
}
