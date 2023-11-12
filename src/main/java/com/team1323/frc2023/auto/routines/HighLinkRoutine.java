package com.team1323.frc2023.auto.routines;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Settings;
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
import com.team1323.frc2023.requests.IfRequest;
import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.ParallelRequest;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.LimelightManager;
import com.team1323.frc2023.vision.LimelightManager.ProcessingMode;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class HighLinkRoutine extends AutoRoutine {
    private final Quadrant quadrant;
    private final boolean scoreCubeHigh;
    private final Swerve swerve;
    private final Claw claw;
    private final Superstructure s;
    private Pose2d coneIntakingPosition;

    public HighLinkRoutine(Quadrant quadrant, boolean scoreCubeHigh) {
        this.quadrant = quadrant;
        this.scoreCubeHigh = scoreCubeHigh;
        swerve = Swerve.getInstance();
        claw = Claw.getInstance();
        s = Superstructure.getInstance();
    }

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        final List<MirroredTrajectory> mirroredTrajectories = Arrays.asList(trajectories.secondPiecePickupPath, trajectories.secondPieceToCubeScore,
                        trajectories.cubeScoreToThirdPiece, trajectories.thirdPieceToEdgeColumn, trajectories.finalBackupPath);
        return Arrays.stream(Quadrant.values())
                .flatMap(quadrant -> mirroredTrajectories.stream().map(mirroredTrajectory -> mirroredTrajectory.get(quadrant)))
                .toList();
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
                        double yOffset = 0.0;
                        if (quadrant == Quadrant.BOTTOM_LEFT) { 
                            yOffset = 6.0;
                        } else if (quadrant == Quadrant.TOP_RIGHT) {
                            yOffset = Settings.kIsUsingCompBot ? 4.0 : 2.0;
                        } else if (quadrant == Quadrant.BOTTOM_RIGHT) {
                            yOffset = -4.0;
                        } else if (quadrant == Quadrant.TOP_LEFT) {
                            yOffset = -2.0;
                        }
                        Pose2d adjustedIntakingPosition = getConeIntakingPosition().transformBy(Pose2d.fromTranslation(
                                new Translation2d(0, yOffset)));
                        
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
            new SetTrajectoryRequest(trajectories.thirdPieceToEdgeColumn, Rotation2d.fromDegrees(180), 0.75, quadrant),
            new WaitToPassXCoordinateRequest(120, quadrant, 4.0),
            new IfRequest(
                () -> claw.getCurrentHoldingObject() == HoldingObject.Cone,
                new SequentialRequest(
                    new LambdaRequest(() -> {
                        NodeLocation nodeLocation = AutoZones.mirror(new NodeLocation(Grid.LEFT, Row.MIDDLE, Column.LEFT), quadrant);
                        s.scoringSequence(nodeLocation);
                    }),
                    new WaitForPrereqRequest(() -> 15.0 - runtimeStopwatch.getTime() < 0.5 || claw.getCurrentHoldingObject() == HoldingObject.None),
                    new IfRequest(
                        () -> claw.getCurrentHoldingObject() == HoldingObject.None,
                        new ParallelRequest(
                            new SetTrajectoryRequest(trajectories.finalBackupPath, Rotation2d.fromDegrees(180), 0.75, quadrant),
                            new LambdaRequest(() -> Netlink.setBooleanValue("Swerve Coast Mode", true))
                        ),
                        new SequentialRequest(
                            new IfRequest(
                                () -> {
                                    double distanceToTarget = swerve.getDistanceToTargetPosition();
                                    System.out.println(String.format("Distance to target at 0.5s left was %.2f.", distanceToTarget));

                                    return distanceToTarget <= 8.0;
                                },
                                new SequentialRequest(
                                    new LambdaRequest(() -> System.out.println("Ejecting cone early.")),
                                    claw.stateRequest(Claw.ControlState.CONE_OUTAKE)
                                )
                            ),
                            new WaitForRemainingTimeRequest(0.125, runtimeStopwatch),
                            new IfRequest(
                                () -> claw.getState() == Claw.ControlState.CONE_OUTAKE,
                                new LambdaRequest(() -> claw.setCurrentHoldingObject(HoldingObject.None))
                            )
                        )
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
