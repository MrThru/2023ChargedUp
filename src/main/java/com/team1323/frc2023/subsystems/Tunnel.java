// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.lib.drivers.TalonFXFactory;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Have a single state named "detect" that will handle
 * the tunnels indexing.
 */
public class Tunnel extends Subsystem {

    LazyPhoenix5TalonFX conveyorTalon, rollerTalon;
    DigitalInput frontBanner, rearBanner;

    private static Tunnel instance = null;
    public static Tunnel getInstance() {
        if(instance == null)
            instance = new Tunnel();
        return instance;
    }


    public Tunnel() {
        conveyorTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_CONVEYOR_TALON, Ports.CANBUS);
        rollerTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_ROLLER_TALON, Ports.CANBUS);

        //frontBanner = new DigitalInput(Ports.TUNNEL_FRONT_BANNER);
        //rearBanner = new DigitalInput(Ports.TUNNEL_REAR_BANNER);

        conveyorTalon.setInverted(TalonFXInvertType.CounterClockwise);
        rollerTalon.setInverted(TalonFXInvertType.Clockwise);
    }

    public enum State {
        OFF, DETECT, SPIT, HOLD;
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State state) {
        currentState = state;
    }


    private void setRollerSpeed(double speed) {
        rollerTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setConveyorSpeed(double speed) {
        conveyorTalon.set(ControlMode.PercentOutput, speed);
    }

    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
                        
        }
        

        @Override
        public void onLoop(double timestamp) {
            if(currentState == State.DETECT) {
                if(frontBanner.get()) {
                    setConveyorSpeed(0.0);
                } else {
                    setConveyorSpeed(1.0);
                }
                if(rearBanner.get() && frontBanner.get()){
                    setRollerSpeed(0.0);
                } else if(!frontBanner.get()) {
                    setRollerSpeed(1.0);
                }
            } else if(currentState == State.SPIT) {
                setRollerSpeed(0.4);
                setConveyorSpeed(0.2);
            } else if(currentState == State.HOLD) {
                setRollerSpeed(-0.1);
                setConveyorSpeed(0.05);
            } else if(currentState == State.OFF) {
                setRollerSpeed(0);
                setConveyorSpeed(0);
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
        SmartDashboard.putNumber("Conveyor RPM", encUnitsToRPM(conveyorTalon.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("Roller RPM", encUnitsToRPM(rollerTalon.getSelectedSensorVelocity()));
    }

    public Request stateRequest(State desiredState) {
        return new Request() {
            @Override
            public void act() {
                setState(desiredState);
            }
        };
    }

    @Override
    public void stop() {
        setConveyorSpeed(0);
        setRollerSpeed(0);        
    }
}
