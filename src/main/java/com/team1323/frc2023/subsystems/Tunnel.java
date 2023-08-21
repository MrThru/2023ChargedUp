// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.digitalinputs.IDigitalInput;
import com.team1323.frc2023.subsystems.digitalinputs.RioDigitalInput;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.util.Stopwatch;

public class Tunnel extends Subsystem {
    private static Tunnel instance = null;
    public static Tunnel getInstance() {
        if (instance == null) {
            instance = new Tunnel();
        }
        return instance;
    }

    private final CubeIntake cubeIntake;

    private final MotorController tunnelEntrance, conveyorTalon, frontRollerTalon;
    private final IDigitalInput frontBanner, rearBanner, intakeBanner;

    private final TunnelInputsAutoLogged inputs = new TunnelInputsAutoLogged();

    private boolean rearBannerDetected = false;

    private Tunnel() {
        cubeIntake = CubeIntake.getInstance();

        tunnelEntrance = Phoenix5FXMotorController.createRealOrSimulatedController(Ports.TUNNEL_ENTRANCE_TALON, Ports.CANBUS);
        conveyorTalon = Phoenix5FXMotorController.createRealOrSimulatedController(Ports.TUNNEL_CONVEYOR_TALON, Ports.CANBUS);
        frontRollerTalon = Phoenix5FXMotorController.createRealOrSimulatedController(Ports.TUNNEL_ROLLER_TALON, Ports.CANBUS);
        tunnelEntrance.configureAsRoller();
        conveyorTalon.configureAsRoller();
        frontRollerTalon.configureAsRoller();

        this.frontBanner = RioDigitalInput.createRealOrSimulatedInput(Ports.TUNNEL_FRONT_BANNER);
        this.rearBanner = RioDigitalInput.createRealOrSimulatedInput(Ports.TUNNEL_REAR_BANNER);
        this.intakeBanner = RioDigitalInput.createRealOrSimulatedInput(Ports.INTAKE_BANNER);

        conveyorTalon.setInverted(TalonFXInvertType.CounterClockwise);
        frontRollerTalon.setInverted(TalonFXInvertType.Clockwise);
        
        conveyorTalon.setPIDF(Constants.Tunnel.kConveyorPID);
        frontRollerTalon.setPIDF(Constants.Tunnel.kFrontRollerPID);

        tunnelEntrance.setInverted(TalonFXInvertType.Clockwise);

        frontRollerTalon.setNeutralMode(NeutralMode.Brake);
    }

    public enum State {
        OFF, SINGLE_INTAKE, SPIT, SPIT_HANDOFF, HOLD, EJECT_ONE, COMMUNITY, MANUAL, REVERSE, TRIPLE_CUBE_HOLD;
    }
    private State currentState = State.MANUAL;
    private boolean stateChanged = false;
    public State getState() {
        return currentState;
    }
    public void setState(State state) {
        currentState = state;
        stateChanged = true;
    }

    public void setTunnelEntranceSpeed(double speed) {
        tunnelEntrance.set(ControlMode.PercentOutput, speed);
    }
    public void setRollerSpeed(double speed) {
        frontRollerTalon.set(ControlMode.PercentOutput, speed);
    }
    double conveyorSpeed = 0;
    public void setConveyorSpeed(double speed) {
        conveyorSpeed = speed;
        conveyorTalon.set(ControlMode.PercentOutput, speed);
    }
    public void setRollerSpeeds(double frontRoller, double conveyorSpeed) {
        setRollerSpeed(frontRoller);
        setConveyorSpeed(conveyorSpeed);
    }

    public void setAllSpeeds(double speed) {
        setRollerSpeed(speed);
        setConveyorSpeed(speed);
        setTunnelEntranceSpeed(speed);
    }

    private boolean pendingShutdown = false;//Shuts down a state if the states requirement(s) have been met
    public void queueShutdown(boolean pendingShutdown) {
        this.pendingShutdown = pendingShutdown;
    }
    private State shutdownState = State.COMMUNITY;
    public void queueShutdownAndChangestate(State state) {
        shutdownState = state;
        queueShutdown(true);
    }

    private boolean autoAdjustElevator = false;
    public void setAutoAdjustElevator(boolean enable) {
        autoAdjustElevator = enable;
    }

    
    private boolean cubeEnteredNotifier = false;
    private boolean driverNotified = false;
    public boolean getCubeEnteredNotifier() {
        if(cubeEnteredNotifier) {
            cubeEnteredNotifier = false;
            driverNotified = true;
            return true;
        }
        return false;
    }

    public boolean getFrontBanner() {
        return inputs.frontBannerValue;
    }
    public boolean getRearBanner() {
        return inputs.rearBannerValue;
    }
    public boolean getCubeIntakeBanner() {
        return inputs.intakeBannerValue;
    }
    /*True if all the banners(front, rear, and entrance) are false*/
    public boolean allowSingleIntakeMode() {
        return !(getFrontBanner() || getRearBanner() || getCubeIntakeBanner());
    }

    public boolean isEmpty() {
        return allowSingleIntakeMode();
    }
    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }
    public double rpmToEncUnits(double rpm) {
        return (rpm / 600) * 2048.0;
    }

    Stopwatch bannerActivatedStopwatch = new Stopwatch();
    Stopwatch cubeEjectedStopwatch = new Stopwatch();
    Stopwatch queueShutdownStopwatch = new Stopwatch();
    Stopwatch firstBannerActivatedStopwatch = new Stopwatch(); //The Cube Intake banner

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
                        
        }

        @Override
        public void onLoop(double timestamp) {
            if(stateChanged) {
                pendingShutdown = false;
                rearBannerDetected = false;
                queueShutdownStopwatch.reset();
                bannerActivatedStopwatch.reset();
                firstBannerActivatedStopwatch.reset();
            }
            switch(currentState) {
                //ToDo: Send straight through to claw, if the claw is empty
                case COMMUNITY:
                    if(allowSingleIntakeMode()) {
                        setRollerSpeeds(0.1, 0.5);
                        setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                        bannerActivatedStopwatch.reset();
                    } else {
                        if(getFrontBanner()) {
                            bannerActivatedStopwatch.startIfNotRunning();
                            if(bannerActivatedStopwatch.getTime() > Constants.Tunnel.kFrontBannerStopTime) {
                                setRollerSpeeds(0, 0);
                            }
                            if(getRearBanner()) {
                                if(getCubeIntakeBanner()) {
                                    setState(State.TRIPLE_CUBE_HOLD);
                                } else if(cubeIntake.getState() == CubeIntake.State.FLOOR && bannerActivatedStopwatch.getTime() > Constants.Tunnel.kFrontBannerStopTime) {
                                    //setState(State.OFF);
                                    cubeIntake.setIntakeSpeed(0);
                                } else {
                                    setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed/2);
                                    cubeIntake.setIntakeSpeed(CubeIntake.State.INTAKE.intakeSpeed/2);
                                }
                            } else {
                                if(!getCubeIntakeBanner() && cubeIntake.getState() == CubeIntake.State.FLOOR) {
                                    //setState(State.OFF);
                                }
                                setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                            }

                        } else {
                            bannerActivatedStopwatch.reset();
                            setRollerSpeeds(0.1, 0.2);
                            setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                            if(getRearBanner()) {
                                if(cubeIntake.getState() == CubeIntake.State.FLOOR) {
                                    cubeIntake.setIntakeSpeed(0);
                                }
                            } else {
                                if(cubeIntake.getState() == CubeIntake.State.FLOOR) {
                                    cubeIntake.setIntakeSpeed(CubeIntake.State.INTAKE.intakeSpeed);
                                }
                            }
                        }
                    }
                    break;
                case SINGLE_INTAKE:
                    if(pendingShutdown) {
                        queueShutdownStopwatch.startIfNotRunning();
                        if(queueShutdownStopwatch.getTime() > 2.0) {
                            pendingShutdown = false;
                            setState(State.OFF);
                        } else {
                            if(getFrontBanner() || getRearBanner()) {
                                pendingShutdown = false;
                                queueShutdownStopwatch.reset();
                            }
                        }
                    }
                    if(getRearBanner() || !getFrontBanner()) {
                        bannerActivatedStopwatch.reset();
                    }
                    if (getRearBanner() && !getFrontBanner()) {
                        rearBannerDetected = true;
                    }

                    if(bannerActivatedStopwatch.getTime() > Constants.Tunnel.kFrontBannerStopTime) { //0.05
                        // if(frontRollerTalon.getStatorCurrent() > 7.0)
                        bannerActivatedStopwatch.reset();
                        setAllSpeeds(0);
                    }
                    if(getFrontBanner()) {
                        bannerActivatedStopwatch.startIfNotRunning();
                    } else {
                        setRollerSpeeds(Settings.kIsUsingCompBot ? 0.08 : 0.1, rearBannerDetected ? 0.75 : 1.0);
                        setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                        bannerActivatedStopwatch.reset();
                    }
                    break;
                case EJECT_ONE:
                    if(getFrontBanner()) {
                        setRollerSpeeds(0.225, 0.075); //0.2 : 0.1 : 0.15 0.05
                        cubeEjectedStopwatch.reset();
                    } else {
                        cubeEjectedStopwatch.startIfNotRunning();
                        if(cubeEjectedStopwatch.getTime() > 0.01) { //0.01
                            if(!allowSingleIntakeMode()) {
                                setState(State.COMMUNITY);
                            } else {
                                setState(State.SINGLE_INTAKE);
                                queueShutdown(true);
                            }
                            cubeEjectedStopwatch.reset();
                        }
                    }
                    break;
                case TRIPLE_CUBE_HOLD:
                    firstBannerActivatedStopwatch.startIfNotRunning();
                    if(firstBannerActivatedStopwatch.getTime() > 0.5) {
                        firstBannerActivatedStopwatch.reset();
                        setState(State.OFF);
                    } else if(firstBannerActivatedStopwatch.getTime() > 0.25) {
                        cubeIntake.setHoldMode();
                    }
                    break;
                case SPIT:
                    setRollerSpeed(0.25); //0.15
                    setConveyorSpeed(1.0); //0.25
                    setTunnelEntranceSpeed(0.25);
                    break;
                case SPIT_HANDOFF:
                    if(getFrontBanner()) {
                        setRollerSpeeds(0.2, 0.5); //Top Roller = 0.2
                        cubeEjectedStopwatch.reset();
                    } else {
                        cubeEjectedStopwatch.startIfNotRunning();
                        if(cubeEjectedStopwatch.getTime() > 0.1) {
                            if(!allowSingleIntakeMode()) {
                                setState(State.COMMUNITY);
                            } else {
                                setState(State.SINGLE_INTAKE);
                                queueShutdown(true);
                            }
                            cubeEjectedStopwatch.reset();
                        }
                    }
                    break;
                case MANUAL:
                    break;
                case REVERSE:
                    setRollerSpeeds(-0.25, -1.0);
                    setTunnelEntranceSpeed(-0.25);
                    break;
                case OFF:
                    setRollerSpeed(0);
                    setConveyorSpeed(0); 
                    setTunnelEntranceSpeed(0);   
                    break;
                default:
                    break;
            }
            stateChanged = false;
            if(getRearBanner() && !cubeEnteredNotifier && !driverNotified) {
                cubeEnteredNotifier = true;
            } else if(!getRearBanner()) {
                cubeEnteredNotifier = false;
                driverNotified = false;
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
    public void readPeriodicInputs() {
        inputs.frontBannerValue = frontBanner.get();
        inputs.rearBannerValue = rearBanner.get();
        inputs.intakeBannerValue = intakeBanner.get();
        Logger.getInstance().processInputs("Tunnel", inputs);
    }
    
    public double getCubeCount() {
        return (getFrontBanner() ? 1 : 0) + (getRearBanner() ? 1 : 0);
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput("Tunnel/Control State", currentState.toString());
        Logger.getInstance().recordOutput("Tunnel/Floor Percent Output", conveyorSpeed);
    }

    public Request stateRequest(State desiredState) {
        return new Request() {
            @Override
            public void act() {
                setState(desiredState);
            }

            @Override
            public String toString() {
                return String.format("TunnelRequest(state = %s)", desiredState);
            }
        };
    }

    public Request ejectOneRequest() {
        return new Request() {
            @Override
            public void act() {
                setState(State.EJECT_ONE);
            }

            @Override
            public boolean isFinished() {
                return getState() == State.OFF;
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

    @AutoLog
    public static class TunnelInputs {
        public boolean frontBannerValue;
        public boolean rearBannerValue;
        public boolean intakeBannerValue;
    }
}
