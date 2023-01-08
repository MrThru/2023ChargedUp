package com.team1323.lib.drivers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2023.Constants;
import com.team254.drivers.LazyTalonFX;

public class TalonFXFactory {
    public static LazyTalonFX createRollerTalon(int deviceId) {
        LazyTalonFX talon = new LazyTalonFX(deviceId);
        configureRollerTalon(talon);

        return talon;
    }

    public static LazyTalonFX createRollerTalon(int deviceId, String canBus) {
        LazyTalonFX talon = new LazyTalonFX(deviceId, canBus);
        configureRollerTalon(talon);

        return talon;
    }

    public static LazyTalonFX createServoTalon(int deviceId) {
        LazyTalonFX talon = new LazyTalonFX(deviceId);
        configureServoTalon(talon);

        return talon;
    }

    public static LazyTalonFX createServoTalon(int deviceId, String canBus) {
        LazyTalonFX talon = new LazyTalonFX(deviceId, canBus);
        configureServoTalon(talon);

        return talon;
    }

    private static void configureRollerTalon(LazyTalonFX talon) {
        configureTalon(talon);
        talon.configForwardSoftLimitEnable(false, Constants.kCANTimeoutMs);
        talon.configReverseSoftLimitEnable(false, Constants.kCANTimeoutMs);
    }

    private static void configureServoTalon(LazyTalonFX talon) {
        configureTalon(talon);
    }

    private static void configureTalon(LazyTalonFX talon) {
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
