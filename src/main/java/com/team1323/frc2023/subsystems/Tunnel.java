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

        tunnelEntrance.setInverted(TalonFXInvertType.Clockwise);

        frontRollerTalon.setNeutralMode(NeutralMode.Brake);
        
    }

    public enum State {
        OFF, DETECT, SINGLE_INTAKE, SPIT, HOLD, EJECT_ONE, COMMUNITY, MANUAL, REVERSE;
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
    public void setConveyorSpeed(double speed) {
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

    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }
    public double rpmToEncUnits(double rpm) {
        return (rpm / 600) * 2048.0;
    }

    Stopwatch cubeEjectedStopwatch = new Stopwatch();
    Stopwatch queueShutdownStopwatch = new Stopwatch();
    private double lastCubeDetectedtimestamp = Double.POSITIVE_INFINITY;
    private boolean frontBannerActivated = false;
    public boolean cubeOnBumper() {
        return frontBannerActivated;
    }
    
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
                        
        }
        

        @Override
        public void onLoop(double timestamp) {
            if(stateChanged) {
                pendingShutdown = false;
            }
            switch(currentState) {
                //ToDo: Send straight through to claw, if the claw is empty
                case COMMUNITY:
                    if(Claw.getInstance().getCurrentHoldingObject() == Claw.HoldingObject.None) {
                        if(Claw.getInstance().getState() == Claw.ControlState.CUBE_INTAKE) {
                            setRollerSpeeds(-Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                            System.out.println("Tunnel Outtaking");
                        } else {
                            setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                            System.out.println("Tunnel Holding");
                        }
                        setTunnelEntranceSpeed(0.65);
                    } else {
                        setRollerSpeeds(0, 0);
                        setTunnelEntranceSpeed(0.65);
                    }
                    
                    
                    
                    /*else if(!getFrontBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.65);
                    } else if(!getRearBanner()) {
                        setRollerSpeeds(Constants.Tunnel.kFeedFrontRollerSpeed, Constants.Tunnel.kFeedConveyorSpeed);
                        setTunnelEntranceSpeed(0.65);
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
                    }*/
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
                    if(pendingShutdown && allowSingleIntakeMode() && !frontBannerActivated) {
                        queueShutdownStopwatch.startIfNotRunning();
                        if(queueShutdownStopwatch.getTime() > 2.0)
                        {
                            setState(State.HOLD);
                            queueShutdownStopwatch.reset();
                            pendingShutdown = false;
                        }
                    } else {
                        queueShutdownStopwatch.reset();
                    }
                    if(frontBannerActivated && allowSingleIntakeMode()) {
                        setAllSpeeds(0);
                    } else if(frontBannerActivated) {
                        setRollerSpeeds(0.25, 0.4);
                        setTunnelEntranceSpeed(0.25);
                    } else if(allowSingleIntakeMode()) {
                        setRollerSpeeds(-0.6, 1.0);
                        setTunnelEntranceSpeed(0.50);
                    } else if(getRearBanner()) {
                        setRollerSpeeds(-0.6, 1.0);
                        setTunnelEntranceSpeed(0.25);
                    } else if(getFrontBanner()) {
                        setRollerSpeeds(-0.25, 1.0);
                        setTunnelEntranceSpeed(0.25);
                    }

                    if(getFrontBanner() && !frontBannerActivated) {
                        frontBannerActivated = true;
                    }
                    
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
                    frontBannerActivated = false;
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
