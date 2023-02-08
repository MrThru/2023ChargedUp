// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.lib.drivers.TalonFXFactory;
import com.team254.drivers.LazyPhoenix5TalonFX;

/** Add your docs here. */
public class Claw extends Subsystem {

    LazyPhoenix5TalonFX claw;

    private static Claw instance = null;
    public static Claw getInstance() {
        if(instance == null)
            instance = new Claw();
        return instance;
    }
    

    public Claw() {
        claw = TalonFXFactory.createRollerTalon(Ports.CLAW, Ports.CANBUS);
        claw.setSupplyCurrentLimit(120, 0.1);
        claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorCurrentLimit, 0.01);
        claw.setInverted(TalonFXInvertType.Clockwise);
        claw.configNeutralDeadband(0);

        claw.setPIDF(Constants.Claw.kPID);
    }

    public enum ControlState {
        OFF(0.0), CUBE_INTAKE(-0.5), CUBE_OUTAKE(0.5), CONE_INTAKE(0.5), CONE_OUTAKE(-0.5);
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }

    private boolean stateChanged = false;
    private ControlState currentState = ControlState.OFF;
    private void setState(ControlState desiredState) {
        currentState = desiredState;
        stateChanged = true;
    }
    public ControlState getState() {
        return currentState;
    }

    private void setPercentSpeed(double speed) {
        setRPM(6380.0 * speed);
    }
    private void setRPM(double rpm) {
        claw.set(ControlMode.Velocity, rpmToEncUnits(rpm));
    }
    public void conformToState(ControlState state) {
        setState(state);
        setPercentSpeed(state.speed);
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            reset();            
        }

        @Override
        public void onLoop(double timestamp) {
            if(currentState == ControlState.CONE_INTAKE) {
                if(stateChanged)
                    claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorCurrentLimit, 0.01);
                if(claw.getOutputCurrent() > Constants.Claw.kIntakeConeAmpThreshold) {
                    claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorHoldCurrent, 0.01);
                }
            } else if(currentState == ControlState.CUBE_INTAKE) {
                if(stateChanged)
                    claw.setStatorCurrentLimit(Constants.Claw.kIntakeCubeStatorCurrentLimit, 0.01);
            }
            if(stateChanged)
                stateChanged = false;
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    } 




    public double rpmToEncUnits(double rpm) {
        return (rpm / 600 * 2048.0);
    }
    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }

    @Override
    public void outputTelemetry() {

    }

    public Request stateRequest(ControlState desiredState) {
        return new Request() {
            @Override
            public void act() {
                conformToState(desiredState);
            }
        };
    }

    public void reset() {
        claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorCurrentLimit, 0.001);
    }
    @Override
    public void stop() {
        reset();
        conformToState(ControlState.OFF);
    }

}
