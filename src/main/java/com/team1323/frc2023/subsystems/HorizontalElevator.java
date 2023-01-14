// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.requests.Request;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class HorizontalElevator extends ServoSubsystemWithCurrentZeroing {
    private static HorizontalElevator instance = null;
    public static HorizontalElevator getInstance() {
        if(instance == null)
            instance = new HorizontalElevator();
        return instance;
    }

    public HorizontalElevator() {
        super(Ports.HORIZONTAL_ELEVATOR_LEADER, null, 
                Constants.HorizontalElevator.kTicksPerInch, Constants.HorizontalElevator.kMinExtension, Constants.HorizontalElevator.kMaxExtension, 
                Constants.HorizontalElevator.kExtensionTolerance, Constants.HorizontalElevator.kVelocityScalar, Constants.HorizontalElevator.kAccelerationScalar,
                Constants.HorizontalElevator.kCurrentZeroingConfig);

        setPIDF(0, Constants.HorizontalElevator.kP, Constants.HorizontalElevator.kI, Constants.HorizontalElevator.kD, Constants.HorizontalElevator.kF);
        setSupplyCurrentLimit(Constants.HorizontalElevator.kSupplyLimit);
        zeroPosition();
    }

    public Request extensionRequest(double inches) {
        return new Request() {
            @Override
            public void act() {
                setPosition(inches);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    @Override
    public void outputTelemetry() {
       SmartDashboard.putNumber("Horizontal Elevator Height", getPosition());
       SmartDashboard.putNumber("Horizontal Elevator Encoder Position", periodicIO.position); 
    }

}
