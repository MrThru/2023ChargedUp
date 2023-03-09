// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.modes;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.auto.actions.SetTrajectoryAction;
import com.team1323.frc2023.auto.actions.WaitForRemainingTimeAction;
import com.team1323.frc2023.auto.actions.WaitForShoulderToPassAngleAction;
import com.team1323.frc2023.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2023.auto.actions.WaitToIntakeAction;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.SynchronousPIDF;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/** Add your docs here. */
public class TwoPieceAndRampMode extends TwoConesOneCubeBaseMode {
    private static final double kTimeToStartBalancing = 11.5;

    public TwoPieceAndRampMode(Quadrant quadrant) {
        super(quadrant);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.routine();

        // Finish intaking the second cone
        Pose2d coneIntakingPosition = LimelightProcessor.getInstance().getRobotConePickupPosition(AutoZones.mirror(Constants.kSecondPickupConePosition, quadrant));
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        if(!coneIntakingPosition.equals(Pose2d.identity())) {
            coneIntakingPosition = coneIntakingPosition.transformBy(Pose2d.fromTranslation(new Translation2d(quadrant.hasBump() ? 4 : 0, (quadrant == Quadrant.TOP_RIGHT) ? 6 : 0)));
            Swerve.getInstance().startVisionPID(coneIntakingPosition, coneIntakingPosition.getRotation(), false,
                    new SynchronousPIDF(0.07, 0.0, 0.0),
                    new SynchronousPIDF(0.02, 0.0, 0.0),
                    new TwoPointRamp(
                        new Translation2d(1.0, 0.1), 
                        new Translation2d(60.0, 0.4), 
                        1.0, 
                        true
                    ));
            runAction(new WaitToIntakeAction(Claw.HoldingObject.Cone, Util.limit(kTimeToStartBalancing - currentTime(), 0, 4)));
        } else {
            runAction(new WaitToFinishPathAction(Util.limit(kTimeToStartBalancing - currentTime(), 0, 4)));
        }

        // Get on the bridge and balance
        runAction(new SetTrajectoryAction(trajectories.thirdPieceToBridgePath, Rotation2d.fromDegrees(0), 0.75, quadrant));
        if (Claw.getInstance().getCurrentHoldingObject() != HoldingObject.Cone) {
            Superstructure.getInstance().request(SuperstructureCoordinator.getInstance().getConeStowChoreography());
        }
        runAction(new WaitForShoulderToPassAngleAction(90.0, 2.0));
        CubeIntake.getInstance().conformToState(CubeIntake.State.FLOOR);
        runAction(new WaitToFinishPathAction(2.0));
        Swerve.getInstance().startBalancePID();
        CubeIntake.getInstance().conformToState(CubeIntake.State.STOWED);
        runAction(new WaitForRemainingTimeAction(0.25, startTime));
        Swerve.getInstance().zukLockDrivePosition();
        System.out.println("Auto Done in: " + currentTime());
    }

}
