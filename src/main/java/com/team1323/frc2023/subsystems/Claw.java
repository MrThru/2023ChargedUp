// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Ports;
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
        claw.setSupplyCurrentLimit(20, 0.1);
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
    @Override
    public void stop() {
        conformToState(ControlState.OFF);
    }

}
