// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team254.drivers.LazyTalonFX;

/** Add your docs here. */
public class Intake extends Subsystem {
    private static Intake instance = null;
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }

    LazyTalonFX intake1, intake2, intake3;
    List<LazyTalonFX> motors;
    public Intake() {
        intake1 = new LazyTalonFX(Ports.INTAKE_1);
        intake2 = new LazyTalonFX(Ports.INTAKE_2);
        intake3 = new LazyTalonFX(Ports.INTAKE_3);

        motors = Arrays.asList(intake1, intake2, intake3);
        for(LazyTalonFX motor : motors) {
            motor.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
            motor.enableVoltageCompensation(true);

            motor.setNeutralMode(NeutralMode.Brake);

            motor.configClosedloopRamp(0.0, Constants.kCANTimeoutMs);
            motor.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);

            motor.configPeakOutputForward(1.0, Constants.kCANTimeoutMs);
            motor.configPeakOutputReverse(-1.0, Constants.kCANTimeoutMs);

            motor.configNominalOutputForward(0.0, Constants.kCANTimeoutMs);
            motor.configNominalOutputReverse(0.0, Constants.kCANTimeoutMs);

            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            motor.configForwardSoftLimitEnable(false);
            motor.configReverseSoftLimitEnable(false);

            
            motor.setInverted(TalonFXInvertType.CounterClockwise);

        }
        setSupplyCurrentLimit(10);
        
    }
    protected void setSupplyCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 0.01);
        motors.forEach(m -> m.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs));
    }
    public void setSpeed(double speed1, double speed2, double speed3) {
        intake1.set(ControlMode.PercentOutput, speed1);
        intake2.set(ControlMode.PercentOutput, speed2);
        intake3.set(ControlMode.PercentOutput, speed3);
    }
    public void setSpeed(double speed) {
        setSpeed(speed, speed, speed);
    }
    @Override
    public void outputTelemetry() {
        
    }
    @Override
    public void stop() {
        setSpeed(0);
    }
}
