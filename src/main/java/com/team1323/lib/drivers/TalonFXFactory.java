package com.team1323.lib.drivers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1323.frc2023.Constants;
import com.team254.drivers.LazyPhoenix5TalonFX;

public class TalonFXFactory {
    public static LazyPhoenix5TalonFX createRollerTalon(int deviceId, String canBus) {
        LazyPhoenix5TalonFX talon = constructTalon(deviceId, canBus);
        configureRollerTalon(talon);

        return talon;
    }

    public static LazyPhoenix5TalonFX createRollerTalon(int deviceId) {
        return createRollerTalon(deviceId, null);
    }

    public static LazyPhoenix5TalonFX createServoTalon(int deviceId, String canBus) {
        LazyPhoenix5TalonFX talon = constructTalon(deviceId, canBus);
        configureServoTalon(talon);

        return talon;
    }

    public static LazyPhoenix5TalonFX createServoTalon(int deviceId) {
        return createServoTalon(deviceId, null);
    }

    private static LazyPhoenix5TalonFX constructTalon(int deviceId, String canBus) {
        if (canBus == null) {
            return new LazyPhoenix5TalonFX(deviceId);
        }
        return new LazyPhoenix5TalonFX(deviceId, canBus);
    }

    private static void configureRollerTalon(LazyPhoenix5TalonFX talon) {
        configureTalon(talon);
        talon.configForwardSoftLimitEnable(false, Constants.kCANTimeoutMs);
        talon.configReverseSoftLimitEnable(false, Constants.kCANTimeoutMs);
        
    }
    
    public static void setSupplyCurrentLimit(LazyPhoenix5TalonFX talon, double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 10, 10, 0.25);
        talon.configSupplyCurrentLimit(currentLimitConfiguration);
    }

    private static void configureServoTalon(LazyPhoenix5TalonFX talon) {
        configureTalon(talon);
        talon.configNeutralDeadband(0.001, Constants.kCANTimeoutMs);
    }

    private static void configureTalon(LazyPhoenix5TalonFX talon) {
        talon.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        talon.enableVoltageCompensation(true);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.configClosedloopRamp(0.0, Constants.kCANTimeoutMs);
        talon.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);

        talon.configPeakOutputForward(1.0, Constants.kCANTimeoutMs);
        talon.configPeakOutputReverse(-1.0, Constants.kCANTimeoutMs);

        talon.configNominalOutputForward(0.0, Constants.kCANTimeoutMs);
        talon.configNominalOutputReverse(0.0, Constants.kCANTimeoutMs);

        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
}
