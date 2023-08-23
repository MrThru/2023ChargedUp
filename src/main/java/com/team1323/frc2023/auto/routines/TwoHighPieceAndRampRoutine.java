package com.team1323.frc2023.auto.routines;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.auto.SetTrajectoryRequest;
import com.team1323.frc2023.auto.WaitForRemainingTimeRequest;
import com.team1323.frc2023.auto.WaitToFinishPathRequest;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.requests.IfRequest;
import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.LimelightManager;
import com.team1323.frc2023.vision.LimelightManager.ProcessingMode;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class TwoHighPieceAndRampRoutine extends AutoRoutine {
    private static final double kTimeToStartBalancing = 11.5;

    private final Quadrant quadrant;
    private final boolean scoreCubeHigh;
    private final Swerve swerve;
    private final Claw claw;
    private final Shoulder shoulder;
    private final CubeIntake cubeIntake;
    private final Superstructure s;
    private Pose2d coneIntakingPosition;

    public TwoHighPieceAndRampRoutine(Quadrant quadrant, boolean scoreCubeHigh) {
        this.quadrant = quadrant;
        this.scoreCubeHigh = scoreCubeHigh;
        swerve = Swerve.getInstance();
        claw = Claw.getInstance();
        shoulder = Shoulder.getInstance();
        cubeIntake = CubeIntake.getInstance();
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
        final Request baseRoutine = new HighLinkBaseRoutine(quadrant, scoreCubeHigh).getRoutine();

        final SequentialRequest finishIntakingSecondCone = new SequentialRequest(
            new LambdaRequest(() -> LimelightManager.getInstance().setProcessingMode(ProcessingMode.CENTER_FIDUCIAL)),
            new IfRequest(
                () -> {
                    Pose2d intakingPosition = LimelightManager.getInstance()
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
                    new WaitForPrereqRequest(() -> claw.getCurrentHoldingObject() == HoldingObject.Cone ||
                            runtimeStopwatch.getTime() >= kTimeToStartBalancing, 4.0)
                ),
                new WaitForPrereqRequest(() -> swerve.hasFinishedPath() || runtimeStopwatch.getTime() >= kTimeToStartBalancing, 4.0)
            )
        );

        final SequentialRequest getOnBridgeAndBalance = new SequentialRequest(
            new LambdaRequest(() -> swerve.resetGyroRoll()),
            new SetTrajectoryRequest(trajectories.thirdPieceToBridgePath, Rotation2d.fromDegrees(0), 0.75, quadrant),
            new IfRequest(
                () -> claw.getCurrentHoldingObject() != HoldingObject.Cone,
                new LambdaRequest(() -> s.request(SuperstructureCoordinator.getInstance().getConeStowChoreography()))
            ),
            new WaitForPrereqRequest(() -> shoulder.getPosition() >= 60.0, 2.0),
            cubeIntake.stateRequest(CubeIntake.State.FLOOR),
            new WaitToFinishPathRequest(2.0),
            new LambdaRequest(() -> swerve.startBalancePID()),
            cubeIntake.stateRequest(CubeIntake.State.STOWED),
            new WaitForRemainingTimeRequest(0.25, runtimeStopwatch),
            new LambdaRequest(() -> swerve.zukLockDrivePosition())
        );

        return new SequentialRequest(
            getStartStopwatchRequest(),
            baseRoutine,
            finishIntakingSecondCone,
            getOnBridgeAndBalance,
            getPrintRuntimeRequest()
        );
    }
}
