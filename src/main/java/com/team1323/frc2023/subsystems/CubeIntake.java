// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.lib.drivers.TalonFXFactory;
import com.team254.drivers.LazyPhoenix5TalonFX;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class CubeIntake extends ServoSubsystemWithAbsoluteEncoder {

    LazyPhoenix5TalonFX intakeRoller;
    DigitalInput banner;
    
    private static CubeIntake instance = null;
    public static CubeIntake getInstance() {
        if(instance == null)
            instance = new CubeIntake();
        return instance;
    }

    
    public CubeIntake() {
        super(Ports.CUBE_INTAKE, Ports.CANBUS, Constants.CubeIntake.kEncUnitsPerDegree, 
                Constants.CubeIntake.kMinControlAngle, Constants.CubeIntake.kMaxControlAngle,
                Constants.CubeIntake.kAngleTolerance, Constants.CubeIntake.kVelocityScalar, 
                Constants.CubeIntake.kAccelerationScalar, Constants.CubeIntake.kEncoderInfo);
        setPIDF(Constants.CubeIntake.kStandardPID);
        setSupplyCurrentLimit(Constants.CubeIntake.kSupplyCurrentLimit);
        zeroPosition();
        stop();

        
        
        intakeRoller = TalonFXFactory.createRollerTalon(Ports.CUBE_INTAKE, Ports.CANBUS);
        intakeRoller.setInverted(TalonFXInvertType.Clockwise);
        intakeRoller.setNeutralMode(NeutralMode.Brake);

        banner = new DigitalInput(Ports.INTAKE_BANNER);
    }

    public static enum State {
        STOWED(Constants.CubeIntake.kStartingAngle, 0);
        double intakeAngle;
        double intakeSpeed;
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

    private void conformToState(State desiredState) {
        setPosition(desiredState.intakeAngle);
        setIntakeSpeed(desiredState.intakeSpeed);
        setState(desiredState);
    }

    private void setIntakeSpeed(double intakeSpeed) {
        intakeRoller.set(ControlMode.PercentOutput, intakeSpeed);   
    }

    public boolean getBanner() {
        return banner.get();
    }

    private void updateArbitraryFeedForward() {
        periodicIO.arbitraryFeedForward = Math.cos(Math.toRadians(getPosition())) * Constants.CubeIntake.kArbitraryFeedForward;
    }

    private Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            updateArbitraryFeedForward();
        }

        @Override
        public void onLoop(double timestamp) {
            updateArbitraryFeedForward();
        }

        @Override
        public void onStop(double timestamp) {}
    };
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
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
            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Cube Intake Angle", getPosition());
        SmartDashboard.putNumber("Cube Intake Encoder Position", periodicIO.position);
        SmartDashboard.putNumber("Cube Intake Absolute Encoder", getAbsoluteEncoderDegrees());
    }
    @Override
    public void stop() {
        super.stop();

    }

}
