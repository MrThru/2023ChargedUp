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
import com.team1323.lib.util.Stopwatch;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Have a single state named "detect" that will handle
 * the tunnels indexing.
 */
public class Tunnel extends Subsystem {
    CubeIntake cubeIntake;

    LazyPhoenix5TalonFX tunnelEntrance, conveyorTalon, frontRollerTalon;
    DigitalInput frontBanner, rearBanner;

    private boolean cubeEjected = false;

    private static Tunnel instance = null;
    public static Tunnel getInstance() {
        if(instance == null)
            instance = new Tunnel();
        return instance;
    }


    public Tunnel() {
        cubeIntake = CubeIntake.getInstance();

        tunnelEntrance = TalonFXFactory.createRollerTalon(Ports.TUNNEL_ENTRANCE_TALON, Ports.CANBUS);
        conveyorTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_CONVEYOR_TALON, Ports.CANBUS);
        frontRollerTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_ROLLER_TALON, Ports.CANBUS);

        frontBanner = new DigitalInput(Ports.TUNNEL_FRONT_BANNER);
        rearBanner = new DigitalInput(Ports.TUNNEL_REAR_BANNER);

        conveyorTalon.setInverted(TalonFXInvertType.CounterClockwise);
        frontRollerTalon.setInverted(TalonFXInvertType.Clockwise);
        
        conveyorTalon.setPIDF(Constants.Tunnel.kConveyorPID);
        frontRollerTalon.setPIDF(Constants.Tunnel.kFrontRollerPID);

        tunnelEntrance.setInverted(TalonFXInvertType.CounterClockwise);
        
    }

    public enum State {
        OFF, DETECT, SINGLE_INTAKE, SPIT, HOLD, EJECT_ONE, COMMUNITY;
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State state) {
        currentState = state;
    }

    public void setTunnelEntranceSpeed(double speed) {
        tunnelEntrance.set(ControlMode.PercentOutput, speed);
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

    private boolean pendingShutdown = false;//Shuts down a state if the states requirement(s) have been met
    public void queueShutdown(boolean pendingShutdown) {
        this.pendingShutdown = pendingShutdown;
    }
    public boolean getFrontBanner() {
        return frontBanner.get();
    }
    public boolean getRearBanner() {
        return rearBanner.get();
    }
    public boolean getCubeIntakeBanner() {
        return cubeIntake.getBanner();
    }

    public boolean allowSingleIntakeMode() {
        return !(getFrontBanner() || getRearBanner() || getCubeIntakeBanner());
    }

    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }
    public double rpmToEncUnits(double rpm) {
        return (rpm / 600) * 2048.0;
    }

    Stopwatch cubeEjectedStopwatch = new Stopwatch();
    private double lastCubeDetectedtimestamp = Double.POSITIVE_INFINITY;
    
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
                        
        }
        

        @Override
        public void onLoop(double timestamp) {
            switch(currentState) {
                //ToDo: Send straight through to claw, if the claw is empty
                case COMMUNITY:
                    if(Claw.getInstance().getCurrentHoldingObject() == Claw.HoldingObject.None) {
                        setRollerSpeeds(-Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.50);
                    } else if(!getFrontBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.50);
                    } else if(!getRearBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.30);
                    } else {
                        if(CubeIntake.getInstance().getBanner() && Double.isInfinite(lastCubeDetectedtimestamp)) {
                            lastCubeDetectedtimestamp = timestamp;
                        } else if(Double.isInfinite(lastCubeDetectedtimestamp)) {
                            setRollerSpeeds(0, 0);
                            setTunnelEntranceSpeed(0.05);
                        }
                        if(timestamp - lastCubeDetectedtimestamp > 0.5) {
                            setTunnelEntranceSpeed(0.0);
                            lastCubeDetectedtimestamp = Double.POSITIVE_INFINITY;
                            CubeIntake.getInstance().setHoldMode();
                            setState(State.OFF);
                        }
                        
                    }
                    break;
                case DETECT:
                    if(!getFrontBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.50);
                    } else if(!getRearBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.50);
                    } else {
                        if(CubeIntake.getInstance().getBanner() && Double.isInfinite(lastCubeDetectedtimestamp)) {
                            lastCubeDetectedtimestamp = timestamp;
                        } else if(Double.isInfinite(lastCubeDetectedtimestamp)) {
                            setRollerSpeeds(0, 0);
                            setTunnelEntranceSpeed(0.05);
                        }
                        if(timestamp - lastCubeDetectedtimestamp > 0.5) {
                            setTunnelEntranceSpeed(0.0);
                            lastCubeDetectedtimestamp = Double.POSITIVE_INFINITY;
                            CubeIntake.getInstance().setHoldMode();
                            setState(State.OFF);
                        }
                        
                    }
                    break;
                case SINGLE_INTAKE:
                    if(pendingShutdown) {
                        if(allowSingleIntakeMode()) {
                            setState(State.HOLD);
                            pendingShutdown = false;
                        } else if(getFrontBanner()) {
                            setState(State.HOLD);
                            pendingShutdown = false;
                        }
                    }
                    setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                    setTunnelEntranceSpeed(0.25);
                    break;
                case EJECT_ONE:
                    if(!cubeEjected) {
                        setRollerSpeeds(Constants.Tunnel.kScoreFrontRollerSpeed, Constants.Tunnel.kScoreConveyorSpeed);
                        cubeEjected = !getFrontBanner();
                        cubeEjectedStopwatch.start();
                    } else {
                        if(getFrontBanner() || cubeEjectedStopwatch.getTime() > 2.0) {
                            if(!allowSingleIntakeMode())
                                setState(State.HOLD);
                            else
                                setState(State.OFF);
                            cubeEjected = false;
                            cubeEjectedStopwatch.reset();
                        }
                    }
                    break;
                case SPIT:
                    setRollerSpeed(0.25);
                    setConveyorSpeed(0.25);
                    setTunnelEntranceSpeed(0.25);
                    break;
                case HOLD:
                    setRollerSpeed(Constants.Tunnel.kHoldFrontRollerSpeed);
                    setConveyorSpeed(Constants.Tunnel.kHoldConveyorSpeed);
                    setTunnelEntranceSpeed(0.10);
                    break;
                case OFF:
                    setRollerSpeed(0);
                    setConveyorSpeed(0); 
                    setTunnelEntranceSpeed(0);   
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
        SmartDashboard.putBoolean("Tunnel Front Banner", getFrontBanner());
        SmartDashboard.putBoolean("Tunnel Rear Banner", getRearBanner());
        SmartDashboard.putString("Tunnel Control State", currentState.toString());
    }

    public Request stateRequest(State desiredState) {
        return new Request() {
            @Override
            public void act() {
                setState(desiredState);
            }
        };
    }
    public Request queueShutdownRequest() {
        return new Request() {  
            @Override
            public void act() {
                queueShutdown(true);
            }
        };
    }
    @Override
    public void stop() {
        setState(State.OFF);
        setConveyorSpeed(0);
        setRollerSpeed(0);        
    }
}
