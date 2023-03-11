// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems.swerve;

import com.team1323.frc2023.subsystems.gyros.Pigeon2IMU;
import com.team1323.lib.util.Stopwatch;
import com.team1323.lib.util.SynchronousPIDF;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/** Add your docs here. */
public class BalancePIDController {
    private final double kPitchDeadband = 7.5;
    private final double kOnTargetTime = 0.5;
    private final double kScalarTolerance = 4.0;
    private double kPIDScalar = 1.0;
    SynchronousPIDF mainPIDF = new SynchronousPIDF(0.007, 0, 0, 0);
    Rotation2d targetPitch = Rotation2d.fromDegrees(0);
    boolean isOnTarget = false;
    Stopwatch onTargetStopwatch = new Stopwatch();

    public BalancePIDController() {

    }

    public void start(Rotation2d targetPitch) {
        isOnTarget = false;
        this.targetPitch = targetPitch;
        mainPIDF.setSetpoint(this.targetPitch.getDegrees());
        kPIDScalar = 1.0;
    }
    
    public Translation2d update(Rotation2d robotPitch, double timestamp) {
        double error = Math.toDegrees(targetPitch.distance(robotPitch));
        if(Math.abs(error) < kScalarTolerance) {
            kPIDScalar = 0.5;
        }
        error = Util.deadBand(error, kPitchDeadband);
        if (error == 0.0) {
            onTargetStopwatch.startIfNotRunning();
            if (onTargetStopwatch.getTime() >= kOnTargetTime) {
                isOnTarget = true;
            }
        }

        double output = -mainPIDF.calculate(error, timestamp) * kPIDScalar;
        return new Translation2d(output, 0);
    }
    
    public boolean isOnTarget() {
        return isOnTarget;
    }


}
