// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.lib.drivers.TalonFXFactory;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class Intake extends Subsystem {
    private static Intake instance = null;
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }

    LazyPhoenix5TalonFX leftTalon, rightTalon;
    Solenoid solenoid;
    public Intake() {
        leftTalon = TalonFXFactory.createRollerTalon(Ports.INTAKE_LEFT);
        rightTalon = TalonFXFactory.createRollerTalon(Ports.INTAKE_RIGHT);

        TalonFXFactory.setSupplyCurrentLimit(leftTalon, 10);
        TalonFXFactory.setSupplyCurrentLimit(rightTalon, 10);

        leftTalon.setInverted(TalonFXInvertType.Clockwise);
        rightTalon.setInverted(TalonFXInvertType.CounterClockwise);

        solenoid = new Solenoid(Ports.PNEUMATIC_HUB, PneumaticsModuleType.CTREPCM, Ports.INTAKE_CLAMPER);
    }
    public enum ControlState {
        OFF(0.0, false), INTAKE_CUBE(-0.5, true), INTAKE_CONE(0.0, true),
        HOLD_CUBE(-0.1, true), HOLD_CONE(-0.1, false), EJECT(0.2, true);
        double speed;
        boolean clampOpened;
        ControlState(double speed, boolean clampOpened) {
            this.speed = speed;
            this.clampOpened = clampOpened;
        }
    }
    private ControlState currentState = ControlState.OFF;
    public void setState(ControlState state) {
        currentState = state;
    }
    public ControlState getState() {
        return currentState;
    }
    public void conformToState(ControlState state) {
        setSpeed(state.speed);
        openClamp(state.clampOpened);
    }
    public void setSpeed(double demand) {
        leftTalon.set(ControlMode.PercentOutput, demand);
        rightTalon.set(ControlMode.PercentOutput, demand);
    }
    public void openClamp(boolean openClamp) {
        solenoid.set(openClamp);
        System.out.println("Clamp Solenoid set to " + openClamp);
    }
    @Override
    public void outputTelemetry() {

    }
    public Request setSpeedRequest(double speed) {
        return new Request() {
            @Override
            public void act() {
                setSpeed(speed);
            }
        };
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
