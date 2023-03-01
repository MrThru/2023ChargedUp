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
    private final double kPitchDeadband = 5;
    private final double kOnTargetTime = 0.5;

    SynchronousPIDF mainPIDF = new SynchronousPIDF(0.005, 0, 0, 0);
    Rotation2d targetPitch = Rotation2d.fromDegrees(0);
    boolean isOnTarget = false;
    Stopwatch onTargetStopwatch = new Stopwatch();

    public BalancePIDController() {

    }

    public void start(Rotation2d targetPitch) {
        isOnTarget = false;
        this.targetPitch = targetPitch;
        mainPIDF.setSetpoint(this.targetPitch.getDegrees());
    }
    
    public Translation2d update(Rotation2d robotPitch, double timestamp) {
        double error = Math.toDegrees(targetPitch.distance(robotPitch));
        error = Util.deadBand(error, kPitchDeadband);
        if (error == 0.0) {
            onTargetStopwatch.startIfNotRunning();
            if (onTargetStopwatch.getTime() >= kOnTargetTime) {
                isOnTarget = true;
            }
        }
        double output = -mainPIDF.calculate(error, timestamp);
        return new Translation2d(output, 0);
    }
    
    public boolean isOnTarget() {
        return isOnTarget;
    }


}
