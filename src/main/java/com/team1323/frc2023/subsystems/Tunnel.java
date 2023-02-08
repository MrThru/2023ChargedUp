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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Have a single state named "detect" that will handle
 * the tunnels indexing.
 */
public class Tunnel extends Subsystem {

    LazyPhoenix5TalonFX conveyorTalon, frontRollerTalon;
    DigitalInput frontBanner, rearBanner;

    private boolean cubeEjected = false;

    private static Tunnel instance = null;
    public static Tunnel getInstance() {
        if(instance == null)
            instance = new Tunnel();
        return instance;
    }


    public Tunnel() {
        conveyorTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_CONVEYOR_TALON, Ports.CANBUS);
        frontRollerTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_ROLLER_TALON, Ports.CANBUS);

        //frontBanner = new DigitalInput(Ports.TUNNEL_FRONT_BANNER);
        //rearBanner = new DigitalInput(Ports.TUNNEL_REAR_BANNER);

        conveyorTalon.setInverted(TalonFXInvertType.CounterClockwise);
        frontRollerTalon.setInverted(TalonFXInvertType.Clockwise);
        
        conveyorTalon.setPIDF(Constants.Tunnel.kConveyorPID);
        frontRollerTalon.setPIDF(Constants.Tunnel.kFrontRollerPID);
        
    }

    public enum State {
        OFF, DETECT, SPIT, HOLD, EJECT_ONE;
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State state) {
        currentState = state;
    }


    private void setRollerSpeed(double speed) {
        frontRollerTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setConveyorSpeed(double speed) {
        conveyorTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setRollerSpeeds(double frontRoller, double conveyorSpeed) {
        setRollerSpeed(frontRoller);
        setConveyorSpeed(conveyorSpeed);
    }


    public boolean getFrontBanner() {
        return frontBanner.get();
    }
    public boolean getRearBanner() {
        return rearBanner.get();
    }

    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }
    public double rpmToEncUnits(double rpm) {
        return (rpm / 600) * 2048.0;
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
                        
        }
        

        @Override
        public void onLoop(double timestamp) {
            switch(currentState) {
                case DETECT:
                    if(!getFrontBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kIntakeFrontRollerSpeed, Constants.Tunnel.kIntakeConveyorSpeed);
                    } else if(!getRearBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kHoldFrontRollerSpeed, Constants.Tunnel.kHoldConveyorSpeed);
                    } else {
                        setRollerSpeeds(0, 0);
                    }
                    break;
                case EJECT_ONE:
                    if(!cubeEjected) {
                        setRollerSpeeds(Constants.Tunnel.kScoreFrontRollerSpeed, Constants.Tunnel.kScoreConveyorSpeed);
                        cubeEjected = !getFrontBanner();
                    } else {
                        if(getFrontBanner()) {
                            setState(State.OFF);
                            cubeEjected = false;
                        }
                    }
                    break;
                case SPIT:
                    setRollerSpeed(0.75);
                    setConveyorSpeed(0.75);
                    break;
                case HOLD:
                    setRollerSpeed(-0.1);
                    setConveyorSpeed(0.05);
                    break;
                case OFF:
                    setRollerSpeed(0);
                    setConveyorSpeed(0);    
                break;
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


    
    public double getCubeCount() {
        return (frontBanner.get() ? 1 : 0) + (rearBanner.get() ? 1 : 0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Conveyor RPM", encUnitsToRPM(conveyorTalon.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("Roller RPM", encUnitsToRPM(frontRollerTalon.getSelectedSensorVelocity()));
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
