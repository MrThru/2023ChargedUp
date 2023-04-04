package com.team1323.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.team1323.frc2023.Constants;
import com.team254.drivers.LazyPhoenix5TalonFX;

public class Phoenix5FXMotorController extends LazyPhoenix5TalonFX implements MotorController {
    public Phoenix5FXMotorController(int deviceNumber) {
        super(deviceNumber);
    }

    public Phoenix5FXMotorController(int deviceNumber, String canBus) {
        super(deviceNumber, canBus);
    }

    private void configureDefaultSettings() {
        this.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        this.enableVoltageCompensation(true);

        this.setNeutralMode(NeutralMode.Brake);

        this.configClosedloopRamp(0.0, Constants.kCANTimeoutMs);
        this.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);

        this.configPeakOutputForward(1.0, Constants.kCANTimeoutMs);
        this.configPeakOutputReverse(-1.0, Constants.kCANTimeoutMs);

        this.configNominalOutputForward(0.0, Constants.kCANTimeoutMs);
        this.configNominalOutputReverse(0.0, Constants.kCANTimeoutMs);

        this.configNeutralDeadband(0.001, Constants.kCANTimeoutMs);
    }

    @Override
    public void configureAsRoller() {
        configureDefaultSettings();
        useIntegratedSensor();
        this.configForwardSoftLimitEnable(false, Constants.kCANTimeoutMs);
        this.configReverseSoftLimitEnable(false, Constants.kCANTimeoutMs);
    }

    @Override
    public void configureAsServo() {
        configureDefaultSettings();
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 0.25);
        return this.configSupplyCurrentLimit(currentLimitConfiguration);
    }

    @Override
    public ErrorCode configStatorCurrentLimit(double amps) {
        StatorCurrentLimitConfiguration currentLimitConfiguration = new StatorCurrentLimitConfiguration(true, amps, amps, 0.1);
        return this.configStatorCurrentLimit(currentLimitConfiguration);
    }

    @Override
    public ErrorCode disableStatorCurrentLimit() {
        StatorCurrentLimitConfiguration currentLimitConfiguration = new StatorCurrentLimitConfiguration(false, 200.0, 200.0, 0.1);
        return this.configStatorCurrentLimit(currentLimitConfiguration);
    }

    @Override
    public double getSupplyAmps() {
        return Math.abs(this.getSupplyCurrent());
    }

    @Override
    public double getStatorAmps() {
        return Math.abs(this.getStatorCurrent());
    }

    @Override
    public double getVelocityEncoderUnitsPer100Ms() {
        return this.getSelectedSensorVelocity();
    }

    @Override
    public void useIntegratedSensor() {
        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
    }

    @Override
    public void useCANCoder(int cancoderId) {
        this.configRemoteFeedbackFilter(cancoderId, RemoteSensorSource.CANCoder, 0);
        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, Constants.kCANTimeoutMs);
    }

    @Override
    public void set(ControlMode mode, double demand, double arbitraryFeedForward) {
        this.set(mode, demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }
}
