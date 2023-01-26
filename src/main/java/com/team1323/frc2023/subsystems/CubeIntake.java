// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import java.io.Console;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class CubeIntake extends ServoSubsystemWithAbsoluteEncoder {

    public CubeIntake() {
        super(Ports.CUBE_INTAKE, Constants.kCanBus, Constants.CubeIntake.kEncUnitsPerDegree, 
                Constants.CubeIntake.kMinControlAngle, Constants.CubeIntake.kMaxControlAngle,
                Constants.CubeIntake.kAngleTolerance, Constants.CubeIntake.kVelocityScalar, 
                Constants.CubeIntake.kAccelerationScalar, Constants.CubeIntake.kEncoderInfo);
        setPIDF(Constants.CubeIntake.kStandardPID);
        setSupplyCurrentLimit(Constants.CubeIntake.kSupplyCurrentLimit);
        zeroPosition();
        stop();
    }

    private void updateArbitraryFeedForward() {
        periodicIO.arbitraryFeedForward = Math.cos(Math.toRadians(getPosition())) * Constants.CubeIntake.kArbitraryFeedForward;
    }

    @Override
    public void outputTelemetry() {

    }
    

}
