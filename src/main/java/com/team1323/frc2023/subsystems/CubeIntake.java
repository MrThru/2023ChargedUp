// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;


import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.DriverControls;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.encoders.MagEncoder;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.Stopwatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class CubeIntake extends ServoSubsystemWithAbsoluteEncoder<Phoenix5FXMotorController> {

    Phoenix5FXMotorController intakeRoller;
    DigitalInput banner;
    
    private static CubeIntake instance = null;
    public static CubeIntake getInstance() {
        if(instance == null)
            instance = new CubeIntake();
        return instance;
    }

    
    public CubeIntake() {
        super(new Phoenix5FXMotorController(Constants.CubeIntake.kConfig.leaderPortNumber, Constants.CubeIntake.kConfig.canBus),
                new ArrayList<>(), Constants.CubeIntake.kConfig, Constants.CubeIntake.kCurrentZeroingConfig,
                new MagEncoder(Ports.INTAKE_WRIST_ENCODER, true), Constants.CubeIntake.kEncoderInfo);
        leader.setPIDF(Constants.CubeIntake.kStandardPID);
        setSupplyCurrentLimit(Constants.CubeIntake.kSupplyCurrentLimit);
        setPositionToAbsolute();
        stop();
        disableStatorCurrentLimit();

        
        
        intakeRoller = new Phoenix5FXMotorController(Ports.CUBE_INTAKE, Ports.CANBUS);
        intakeRoller.configureAsRoller();
        intakeRoller.setInverted(TalonFXInvertType.CounterClockwise);
        intakeRoller.setNeutralMode(NeutralMode.Brake);
        setIntakeCurrent(Constants.CubeIntake.kStandardIntakeCurrentLimit);

        banner = new DigitalInput(Ports.INTAKE_BANNER);
    }

    public static enum GameObject {
        
    }

    public static enum State {
        STOWED(Constants.CubeIntake.kMaxControlAngle, 0), INTAKE(Constants.CubeIntake.kIntakeAngle, 0.55),
                            FLOOR(Constants.CubeIntake.kFloorAngle, 0);
        public final double intakeAngle;
        public final double intakeSpeed;
        State(double intakeAngle,double intakeSpeed) {
            this.intakeAngle = intakeAngle;
            this.intakeSpeed = intakeSpeed;
        }
    }
    private State currentState = State.STOWED;
    public State getState() {
        return currentState;
    }
    private void setState(State desiredState) {
        currentState = desiredState;
    }

    public void conformToState(State desiredState) {
        setPosition(desiredState.intakeAngle);
        setIntakeSpeed(desiredState.intakeSpeed);
        setState(desiredState);
    }

    public void setIntakeSpeed(double intakeSpeed) {
        intakeRoller.set(ControlMode.PercentOutput, intakeSpeed);   
    }
    public void setIntakeCurrent(double amps) {
        intakeRoller.setStatorCurrentLimit(amps, 0.1);
    }
    public void setHoldMode() {
        conformToState(State.FLOOR);
    }

    public boolean getBanner() {
        return banner.get();
    }

    private boolean isCurrentLimited = false;
    private Stopwatch onTargetStopwatch = new Stopwatch();
    @Override
    public void setPosition(double outputUnits) {
        if (isCurrentLimited) {
            disableStatorCurrentLimit();
            isCurrentLimited = false;
        }
        super.setPosition(outputUnits);
    }

    private void updateArbitraryFeedForward() {
        periodicIO.arbitraryFeedForward = (Math.cos(Math.toRadians(getPosition())) * Constants.CubeIntake.kArbitraryFeedForward);
    }

    private Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            resetCurrentLimit();
            disableStatorCurrentLimit();
            updateArbitraryFeedForward();
            setIntakeSpeed(0);
        }

        @Override
        public void onLoop(double timestamp) {
            updateArbitraryFeedForward();
            if (isOnTarget() && getPosition() < 100) {
                onTargetStopwatch.startIfNotRunning();
            } else {
                onTargetStopwatch.reset();
            }
            if(onTargetStopwatch.getTime() > (DriverControls.getInstance().getInAuto() ? 0.5 : 0.1) && !isCurrentLimited) {
                isCurrentLimited = true;
                onTargetStopwatch.reset();
                setStatorCurrentLimit(20.0);
            }
            /*if(getBanner() && !intakeCurrentLimitSet) {
                setIntakeCurrent(Constants.CubeIntake.kLowerIntakeCurrentLimit);
                intakeCurrentLimitSet = true;
            } else if(!getBanner() && intakeCurrentLimitSet) {
                setIntakeCurrent(Constants.CubeIntake.kStandardIntakeCurrentLimit);
                intakeCurrentLimitSet = false;
            }*/
        }

        @Override
        public void onStop(double timestamp) {}
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        super.registerEnabledLoops(enabledLooper);
        enabledLooper.register(loop);
    }

    public void resetCurrentLimit() {
        setSupplyCurrentLimit(Constants.CubeIntake.kSupplyCurrentLimit);
    }

    
    public Request angleRequest(double degrees) {
        return new Request() {
            @Override
            public void act() {
                setPosition(degrees);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }
    public Request stateRequest(State desiredState) {
        return new Request() {
            @Override
            public void act() {
                conformToState(desiredState);
            }

            public String toString() {
                return String.format("CubeIntakeRequest(state = %s)", desiredState);
            }
        };
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
        SmartDashboard.putNumber("Cube Intake Angle", getPosition());
        SmartDashboard.putNumber("Cube Intake Encoder Position", periodicIO.position);
        SmartDashboard.putNumber("Cube Intake Absolute Encoder", absoluteEncoder.getDegrees());
        SmartDashboard.putNumber("Cube Intake RPM", intakeRoller.getSelectedSensorVelocity() * 600 / 2048);
        SmartDashboard.putNumber("Cube Intake Target Angle", encoderUnitsToOutputUnits(periodicIO.demand));
        SmartDashboard.putBoolean("Cube Intake Banner", getBanner());
        SmartDashboard.putNumber("Cube Intake Stator Current", leader.getStatorAmps());
    }
    @Override
    public void stop() {
        super.stop();
    }

}
