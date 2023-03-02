// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithCurrentZeroing;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.Stopwatch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HorizontalElevator extends ServoSubsystemWithCurrentZeroing {
    private static HorizontalElevator instance = null;
    public static HorizontalElevator getInstance() {
        if(instance == null)
            instance = new HorizontalElevator();
        return instance;
    }

    public HorizontalElevator() {
        super(Ports.HORIZONTAL_ELEVATOR_LEADER, Ports.CANBUS, 
                Constants.HorizontalElevator.kTicksPerInch, Constants.HorizontalElevator.kMinExtension, Constants.HorizontalElevator.kMaxExtension, 
                Constants.HorizontalElevator.kExtensionTolerance, Constants.HorizontalElevator.kVelocityScalar, Constants.HorizontalElevator.kAccelerationScalar,
                Constants.HorizontalElevator.kCurrentZeroingConfig);

        setSupplyCurrentLimit(Constants.HorizontalElevator.kSupplyLimit);
        zeroPosition();
        isZeroed = true;
        leader.setInverted(TalonFXInvertType.Clockwise);
        leader.setPIDF(Constants.HorizontalElevator.kPIDF);
    }


    Stopwatch currentDrawStopwatch = new Stopwatch();

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            
        }

        @Override
        public void onLoop(double timestamp) {
        }

        @Override
        public void onStop(double timestamp) {
            
        }
        
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
        super.registerEnabledLoops(enabledLooper);
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

    public Prerequisite willReachExtensionWithinTime(double inches, double seconds) {
        return () -> {
            double inchesPerSecond = getVelocityOutputUnitsPerSecond();
            boolean isHeadingTowardHeight = Math.signum(inches - getPosition()) == Math.signum(inchesPerSecond);
            double secondsUntilHeightReached = Double.POSITIVE_INFINITY;
            if (inchesPerSecond != 0.0) {
                secondsUntilHeightReached = Math.abs(inches - getPosition()) / Math.abs(inchesPerSecond);
            }

            boolean willReachExtension = (isHeadingTowardHeight && secondsUntilHeightReached <= seconds) || isAtPosition(inches);

            if (willReachExtension) {
                System.out.println("Horizontal elevator will reach extension.");
            }

            return willReachExtension;
        };
    }

    @Override
    public void setPosition(double outputUnits) {
        System.out.println(String.format("Horizontal elevator being set to %.2f", outputUnits));
        super.setPosition(outputUnits);
    }

    private boolean neutralModeIsBrake = true;
    @Override
    public void outputTelemetry() {
        if(Netlink.getBooleanValue("Subsystems Coast Mode") && neutralModeIsBrake) {
			leader.setNeutralMode(NeutralMode.Coast);
			neutralModeIsBrake = false;
		} else if(!neutralModeIsBrake && !Netlink.getBooleanValue("Subsystems Coast Mode")) {
            leader.setNeutralMode(NeutralMode.Brake);
			neutralModeIsBrake = true;
		}
        SmartDashboard.putNumber("Horizontal Elevator Height", getPosition());
        SmartDashboard.putNumber("Horizontal Elevator Encoder Position", periodicIO.position); 
        SmartDashboard.putBoolean("Horizontal Elevator on Target", isOnTarget());
    }

}
