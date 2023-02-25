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
import com.team1323.frc2023.subsystems.LEDs.LEDColors;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.lib.drivers.TalonFXFactory;
import com.team1323.lib.util.Stopwatch;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Claw extends Subsystem {

    LazyPhoenix5TalonFX claw;

    private double targetRPM = 0;
    private Stopwatch stopwatch = new Stopwatch();
    private Stopwatch stopwatch2 = new Stopwatch();
    private static Claw instance = null;
    public static Claw getInstance() {
        if(instance == null)
            instance = new Claw();
        return instance;
    }
    
     
    public Claw() {
        claw = TalonFXFactory.createRollerTalon(Ports.CLAW, Ports.CANBUS);
        claw.setSupplyCurrentLimit(200, 0.1);
        claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorCurrentLimit, 0.01);
        claw.setInverted(TalonFXInvertType.Clockwise);
        claw.configNeutralDeadband(0);

        claw.setPIDF(Constants.Claw.kPID);
    }

    public enum HoldingObject {
        Cone(ControlState.CONE_INTAKE, ControlState.CONE_OUTAKE), Cube(ControlState.CUBE_INTAKE, ControlState.CUBE_OUTAKE), 
                None(ControlState.OFF, ControlState.OFF);
        ControlState intakeState;
        ControlState outtakeState;
        HoldingObject(ControlState intakeState, ControlState outtakeState) {
            this.intakeState = intakeState;
            this.outtakeState = outtakeState;
        }
    }

    private PeriodicIO periodicIO = new PeriodicIO();
    private HoldingObject currentHoldingObject = HoldingObject.None;
    public HoldingObject getCurrentHoldingObject() {
        return currentHoldingObject;
    }
    private void setCurrentHoldingObject(HoldingObject holdingObject) {
        currentHoldingObject = holdingObject;
    }

    public enum ControlState {
        OFF(0.0), CUBE_INTAKE(-0.5), CUBE_OUTAKE(0.3), CONE_INTAKE(1.0), CONE_OUTAKE(-1.0);
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }

    private boolean stateChanged = false;
    private ControlState currentState = ControlState.OFF;
    private void setState(ControlState desiredState) {
        currentState = desiredState;
        stateChanged = true;
    }
    public ControlState getState() {
        return currentState;
    }

    public void setPercentSpeed(double speed) {
        claw.set(ControlMode.PercentOutput, speed);
        //setRPM(6380.0 * speed);
    }
    private void setRPM(double rpm) {
        targetRPM = rpm;
        claw.set(ControlMode.Velocity, rpmToEncUnits(rpm));
    }
    public void conformToState(ControlState state) {
        if(state == ControlState.CUBE_OUTAKE || state == ControlState.CONE_OUTAKE) {
            claw.setStatorCurrentLimit(100, 0.001);
        }
        setState(state);
        setPercentSpeed(state.speed);
    }

    private double looperStartTime = 0;

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            //conformToState(getCurrentHoldingObject().intakeState);
            looperStartTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            // if((timestamp - looperStartTime) > 2.0 && ((timestamp - looperStartTime) < 3.0) && claw.getOutputCurrent() < 5 && 
            //             (currentState == ControlState.CONE_INTAKE || currentState == ControlState.CUBE_INTAKE)) {
            //     conformToState(ControlState.OFF);
            // }

            if(currentState == ControlState.CONE_INTAKE) {
                if(stateChanged) {
                    claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorCurrentLimit, 0.01);
                    stopwatch.start();
                    stopwatch2.reset();
                }
                if(encUnitsToRPM(periodicIO.velocity) < Constants.Claw.kIntakeConeVelocityThreshold && stopwatch.getTime() > 0.25) {
                    stopwatch2.startIfNotRunning();
                    if(stopwatch2.getTime() > 0.25) {
                        setCurrentHoldingObject(HoldingObject.Cone);
                        LEDs.getInstance().configLEDs(LEDColors.YELLOW);
                        claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorHoldCurrent, 0.01);
                        stopwatch.reset();
                    }
                }

            } else if(currentState == ControlState.CUBE_INTAKE) {
                if(stateChanged) {
                    claw.setStatorCurrentLimit(Constants.Claw.kIntakeCubeStatorCurrentLimit, 0.01);
                    stopwatch.start();
                }
                if(encUnitsToRPM(periodicIO.velocity) > -Constants.Claw.kIntakeCubeVelocityThreshold && stopwatch.getTime() > 0.25) {
                    setCurrentHoldingObject(HoldingObject.Cube);
                    LEDs.getInstance().configLEDs(LEDColors.PURPLE);
                    stopwatch.reset();
                }

                
            } else if(currentState == ControlState.CONE_OUTAKE) {
                if(stateChanged)
                    stopwatch.start();
                if(stopwatch.getTime() > 0.375) {
                    conformToState(ControlState.OFF);
                    stopwatch.reset();
                    setCurrentHoldingObject(HoldingObject.None);
                }

            } else if(currentState == ControlState.CUBE_OUTAKE) {
                if(stateChanged)
                    stopwatch.start();
                if(stopwatch.getTime() > 0.6) {
                    setCurrentHoldingObject(HoldingObject.None);
                }
                if(getCurrentHoldingObject() == HoldingObject.None && stopwatch.getTime() > 3.0) {
                    conformToState(ControlState.OFF);
                    stopwatch.reset();
                }
            } else if(currentState == ControlState.OFF) {
                conformToState(ControlState.OFF);
                setCurrentHoldingObject(HoldingObject.None);
            }
            if(stateChanged)
                stateChanged = false;
        }

        @Override
        public void onStop(double timestamp) {
            //stop();
        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    } 




    public double rpmToEncUnits(double rpm) {
        return (rpm / 600 * 2048.0);
    }
    public double encUnitsToRPM(double encUnits) {
        return (encUnits * 600) / 2048.0;
    }

    public double getRPM() {
        return encUnitsToRPM(periodicIO.velocity);
    }
    private double previousTimestamp = 0;

    @Override
    public void readPeriodicInputs() {
        double currentVelocity = claw.getSelectedSensorVelocity();
        periodicIO.dv = (currentVelocity - periodicIO.velocity) / (Timer.getFPGATimestamp() - previousTimestamp);        
        periodicIO.velocity = currentVelocity;
        
        periodicIO.supplyCurrent = claw.getSupplyCurrent();
        periodicIO.statorCurrent = claw.getStatorCurrent();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Claw Target RPM", targetRPM);
        SmartDashboard.putNumber("Claw RPM", encUnitsToRPM(claw.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("Claw RPM detla", periodicIO.dv);
        SmartDashboard.putString("Current Holding Object", getCurrentHoldingObject().toString());
        SmartDashboard.putNumber("Claw Current", claw.getOutputCurrent());
        SmartDashboard.putString("Claw State", getState().toString());
        SmartDashboard.putNumber("Claw Stator Current", claw.getStatorCurrent());

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
        claw.setStatorCurrentLimit(Constants.Claw.kIntakeConeStatorCurrentLimit, 0.001);
    }
    public void resetCurrentHolding() {
        setCurrentHoldingObject(HoldingObject.None);
    }
    @Override
    public void stop() {
        conformToState(ControlState.OFF);
    }

    private class PeriodicIO {
        double velocity = 0;
        double dv = 0;
        double supplyCurrent = 0;
        double statorCurrent = 0;
    }

}
