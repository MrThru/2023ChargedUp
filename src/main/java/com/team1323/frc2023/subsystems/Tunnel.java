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
import com.team1323.lib.util.Stopwatch;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Have a single state named "detect" that will handle
 * the tunnels indexing.
 */
public class Tunnel extends Subsystem {
    CubeIntake cubeIntake;
    Claw claw;

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
        claw = Claw.getInstance();

        tunnelEntrance = TalonFXFactory.createRollerTalon(Ports.TUNNEL_ENTRANCE_TALON, Ports.CANBUS);
        conveyorTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_CONVEYOR_TALON, Ports.CANBUS);
        frontRollerTalon = TalonFXFactory.createRollerTalon(Ports.TUNNEL_ROLLER_TALON, Ports.CANBUS);

        frontBanner = new DigitalInput(Ports.TUNNEL_FRONT_BANNER);
        rearBanner = new DigitalInput(Ports.TUNNEL_REAR_BANNER);

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
    public void setConveyorSpeedOfTopRoller(double percent) {
        setConveyorSpeed(frontRollerTalon.getMotorOutputPercent() * Constants.Tunnel.kTopRollerRatio / Constants.Tunnel.kFloorRatio);
    }
    public void setTopRollerSpeedOfConveyor(double perecent) {
        setRollerSpeed(conveyorTalon.getMotorOutputPercent() * Constants.Tunnel.kFloorRatio / Constants.Tunnel.kTopRollerRatio);
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
        return frontBanner.get();
    }
    public boolean getRearBanner() {
        return rearBanner.get();
    }
    public boolean getCubeIntakeBanner() {
        return cubeIntake.getBanner();
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
                queueShutdownStopwatch.reset();
                bannerActivatedStopwatch.reset();
                firstBannerActivatedStopwatch.reset();
                cubeEjected = false;
            }
            switch(currentState) {
                //ToDo: Send straight through to claw, if the claw is empty
                case COMMUNITY:
                    if(allowSingleIntakeMode()) {
                        setRollerSpeeds(0.1, 1.0);
                        setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                        bannerActivatedStopwatch.reset();
                    } else {
                        if(getFrontBanner()) {
                            bannerActivatedStopwatch.startIfNotRunning();
                            if(bannerActivatedStopwatch.getTime() > 0.06) {
                                setRollerSpeeds(0, 0);
                            }
                            if(getRearBanner()) {
                                if(getCubeIntakeBanner()) {
                                    setState(State.TRIPLE_CUBE_HOLD);
                                } else if(cubeIntake.getState() == CubeIntake.State.FLOOR && bannerActivatedStopwatch.getTime() > 0.06) {
                                    //setState(State.OFF);
                                    cubeIntake.setIntakeSpeed(0);
                                }
                                setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                            } else {
                                if(!getCubeIntakeBanner() && cubeIntake.getState() == CubeIntake.State.FLOOR) {
                                    //setState(State.OFF);
                                }
                                setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                            }

                        } else {
                            bannerActivatedStopwatch.reset();
                            setRollerSpeeds(0.1, 1.0);
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
                    if(bannerActivatedStopwatch.getTime() > 0.05) {
                        // if(frontRollerTalon.getStatorCurrent() > 7.0)
                        bannerActivatedStopwatch.reset();
                        setAllSpeeds(0);
                    }
                    if(getFrontBanner()) {
                        bannerActivatedStopwatch.startIfNotRunning();
                    } else {
                        setRollerSpeeds(0.1, 1.0);
                        setTunnelEntranceSpeed(Constants.Tunnel.kTunnelEntranceSpeed);
                        bannerActivatedStopwatch.reset();
                    }
                    break;
                case EJECT_ONE:
                    if(getFrontBanner()) {
                        setRollerSpeeds(0.2, 0.10);
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
                    setConveyorSpeed(0.25); //0.25
                    setTunnelEntranceSpeed(0.25);
                    break;
                case SPIT_HANDOFF:
                    if(getFrontBanner()) {
                        setRollerSpeeds(0.15, 1.0);
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
                case HOLD:
                    setRollerSpeed(Constants.Tunnel.kHoldFrontRollerSpeed);
                    setConveyorSpeed(Constants.Tunnel.kHoldConveyorSpeed);
                    setTunnelEntranceSpeed(0.10);
                    break;
                case MANUAL:
                    break;
                case REVERSE:
                    setRollerSpeeds(-0.25, -0.35);
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


    
    public double getCubeCount() {
        return (frontBanner.get() ? 1 : 0) + (rearBanner.get() ? 1 : 0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Conveyor RPM", encUnitsToRPM(conveyorTalon.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("Roller RPM", encUnitsToRPM(frontRollerTalon.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("Cube Entrance RPM", encUnitsToRPM(tunnelEntrance.getSelectedSensorVelocity()));
        SmartDashboard.putBoolean("Tunnel Front Banner", getFrontBanner());
        SmartDashboard.putBoolean("Tunnel Rear Banner", getRearBanner());
        SmartDashboard.putString("Tunnel Control State", currentState.toString());
        SmartDashboard.putNumber("Tunnel Top Roller Current", frontRollerTalon.getStatorCurrent());
        SmartDashboard.putNumber("Tunnel Floor Percent Output", conveyorSpeed);
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
}
