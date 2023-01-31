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

/** Have a single state named "detect" that will handle
 * the tunnels indexing.
 */
public class Tunnel extends Subsystem {

    LazyPhoenix5TalonFX frontTalon, rearTalon;
    DigitalInput frontBanner, rearBanner;

    private static Tunnel instance = null;
    public static Tunnel getInstance() {
        if(instance == null)
            instance = new Tunnel();
        return instance;
    }


    public Tunnel() {
        frontTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_FRONT_TALON, Ports.CANBUS);
        rearTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_REAR_TALON, null);

        //frontBanner = new DigitalInput(Ports.TUNNEL_FRONT_BANNER);
        //rearBanner = new DigitalInput(Ports.TUNNEL_REAR_BANNER);

        frontTalon.setInverted(TalonFXInvertType.CounterClockwise);
    }

    public enum State {
        OFF, DETECT;
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State state) {
        currentState = state;
    }


    private void setRearSpeed(double speed) {
        rearTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setFrontSpeed(double speed) {
        frontTalon.set(ControlMode.PercentOutput, speed);
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
                        
        }
        

        @Override
        public void onLoop(double timestamp) {
            if(currentState == State.DETECT) {
                if(frontBanner.get()) {
                    setFrontSpeed(0.0);
                } else {
                    setFrontSpeed(1.0);
                }
                if(rearBanner.get() && frontBanner.get()){
                    setRearSpeed(0.0);
                } else if(!frontBanner.get()) {
                    setRearSpeed(1.0);
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
        setFrontSpeed(0);
        setRearSpeed(0);        
    }
}
