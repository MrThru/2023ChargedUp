// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithCurrentZeroing;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.Stopwatch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HorizontalElevator extends ServoSubsystemWithCurrentZeroing<Phoenix5FXMotorController> {
    private static HorizontalElevator instance = null;
    public static HorizontalElevator getInstance() {
        if(instance == null)
            instance = new HorizontalElevator();
        return instance;
    }

    public HorizontalElevator() {
        super(new Phoenix5FXMotorController(Constants.HorizontalElevator.kConfig.leaderPortNumber, Constants.HorizontalElevator.kConfig.canBus),
                new ArrayList<>(), Constants.HorizontalElevator.kConfig, Constants.HorizontalElevator.kCurrentZeroingConfig);
        leader.useIntegratedSensor();
        leader.setInverted(TalonFXInvertType.Clockwise);
        leader.setPIDF(Constants.HorizontalElevator.kPIDF);
        setSupplyCurrentLimit(Constants.HorizontalElevator.kSupplyLimit);
        setStatorCurrentLimit(Constants.HorizontalElevator.kStatorLimit);
        zeroPosition();
        isZeroed = true;
    }

    private Stopwatch onTargetStopwatch = new Stopwatch();
    private boolean weakPIDEnabled = false;
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            setStatorCurrentLimit(Constants.HorizontalElevator.kStatorLimit);
        }

        @Override
        public void onLoop(double timestamp) {
            if(isOnTarget() && getPosition() > 2.0) {
                onTargetStopwatch.startIfNotRunning();
            } else {
                onTargetStopwatch.reset();
            }
            if(!weakPIDEnabled && onTargetStopwatch.getTime() > 0.5) {
                weakPIDEnabled = true;
                setStatorCurrentLimit(Constants.HorizontalElevator.kWeakStatorLimit);
                System.out.println("Horizontal Weak Current");
            }

        }

        @Override
        public void onStop(double timestamp) {
            setStatorCurrentLimit(Constants.HorizontalElevator.kStatorLimit);
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

            @Override
            public String toString() {
                return String.format("HorizontalElevatorRequest(target = %.2f)", inches);
            }
        };
    }

    @Override
    public void setPosition(double outputUnits) {
        System.out.println(String.format("Horizontal elevator being set to %.2f", outputUnits));
        setStatorCurrentLimit(Constants.HorizontalElevator.kStatorLimit);
        weakPIDEnabled = false;
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
