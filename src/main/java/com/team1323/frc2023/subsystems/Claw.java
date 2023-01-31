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
        claw.setStatorCurrentLimit(Constants.Claw.kIntakeStatorCurrentLimit, 0.01);
        claw.setInverted(TalonFXInvertType.Clockwise);
    }

    public enum ControlState {
        OFF(0.0), INTAKE(0.5);
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }
    private ControlState currentState = ControlState.OFF;
    private void setState(ControlState desiredState) {
        currentState = desiredState;
    }
    public ControlState getState() {
        return currentState;
    }

    private void setSpeed(double speed) {
        claw.set(ControlMode.PercentOutput, speed);
    }
    public void conformToState(ControlState state) {
        setState(state);
        setSpeed(state.speed);
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            reset();            
        }

        @Override
        public void onLoop(double timestamp) {
            if(currentState == ControlState.INTAKE) {
                if(claw.getOutputCurrent() > Constants.Claw.kIntakeHoldTargetCurrent) {
                    claw.setStatorCurrentLimit(Constants.Claw.kIntakeStatorHoldCurrent, 0.01);
                }
            }
        }

        @Override
        public void onStop(double timestamp) {

        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
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
        claw.setStatorCurrentLimit(Constants.Claw.kIntakeStatorCurrentLimit, 0.001);
    }
    @Override
    public void stop() {
        reset();
        conformToState(ControlState.OFF);
    }

}
