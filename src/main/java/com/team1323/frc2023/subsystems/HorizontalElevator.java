// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import java.util.Arrays;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.requests.Request;

/** Add your docs here. */
public class HorizontalElevator extends ServoSubsystem {
    private static HorizontalElevator instance = null;
    public static HorizontalElevator getInstance() {
        if(instance == null)
            instance = new HorizontalElevator();
        return instance;
    }

    public HorizontalElevator() {
        super(Ports.HORIZONTAL_ELEVATOR_LEADER, Arrays.asList(Ports.HORIZONTAL_ELEVATOR_FOLLOWER), null, 
                Constants.HorizontalElevator.kTicksPerInch, Constants.HorizontalElevator.kMinExtension, Constants.HorizontalElevator.kMaxExtension, 
                Constants.HorizontalElevator.kExtensionTolerance, Constants.HorizontalElevator.kVelocityScalar, Constants.HorizontalElevator.kAccelerationScalar);

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

    }

}
