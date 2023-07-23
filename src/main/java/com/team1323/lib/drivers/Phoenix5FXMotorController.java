package com.team1323.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team1323.frc2023.Constants;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.RobotBase;

public class Phoenix5FXMotorController extends LazyPhoenix5TalonFX implements MotorController {
    public static MotorController createRealOrSimulatedController(int deviceNumber) {
        return RobotBase.isReal() ? new Phoenix5FXMotorController(deviceNumber) : new SimulatedMotorController();
    }

    public static MotorController createRealOrSimulatedController(int deviceNumber, String canBus) {
        return RobotBase.isReal() ? new Phoenix5FXMotorController(deviceNumber, canBus) : new SimulatedMotorController();
    }

    private Phoenix5FXMotorController(int deviceNumber) {
        super(deviceNumber);
    }

    private Phoenix5FXMotorController(int deviceNumber, String canBus) {
        super(deviceNumber, canBus);
    }

    private void configureDefaultSettings(double voltageCompensationValue) {
        this.configVoltageCompSaturation(voltageCompensationValue, Constants.kCANTimeoutMs);
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
        configureDefaultSettings(12.0);
        useIntegratedSensor();
        this.configForwardSoftLimitEnable(false, Constants.kCANTimeoutMs);
        this.configReverseSoftLimitEnable(false, Constants.kCANTimeoutMs);
    }

    @Override
    public void configureAsServo() {
        configureDefaultSettings(12.0);
    }

    @Override
    public void configureAsCoaxialSwerveRotation() {
        configureDefaultSettings(7.0);

        useIntegratedSensor();
		this.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 10);

		this.configForwardSoftLimitEnable(false, 10);
		this.configReverseSoftLimitEnable(false, 10);

		this.configMotionAcceleration(Constants.kSwerveRotationMaxSpeedEncUnits*12.5, 10);
		this.configMotionCruiseVelocity(Constants.kSwerveRotationMaxSpeedEncUnits, 10);
        setPIDF(0, 1.55, 0.0, 5.0, 1023.0 / Constants.kSwerveRotationMaxSpeedEncUnits);
		this.selectProfileSlot(0, 0);

        // TODO: Check if we can remove the allowable closed-loop error given the lower neutral deadband
		this.configAllowableClosedloopError(0, 50, Constants.kCANTimeoutMs);
    }

    @Override
    public void configureAsCoaxialSwerveDrive() {
        useIntegratedSensor();
		this.setSelectedSensorPosition(0, 0, 10);
		this.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 10);
		this.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms, 10);
		this.configVelocityMeasurementWindow(32, 10);

		this.configForwardSoftLimitEnable(false, 10);
		this.configReverseSoftLimitEnable(false, 10);

		this.configNominalOutputForward(0.0, 10);
		this.configNominalOutputReverse(0.0, 10);

        this.configPeakOutputForward(1.0, Constants.kCANTimeoutMs);
        this.configPeakOutputReverse(-1.0, Constants.kCANTimeoutMs);

		this.configVoltageCompSaturation(12.0, 10);
		this.enableVoltageCompensation(true);

		this.configOpenloopRamp(0.2, 10);
		this.configClosedloopRamp(0.0);

		this.configAllowableClosedloopError(0, 0, 10);

        this.configNeutralDeadband(0.001, Constants.kCANTimeoutMs);

		this.setNeutralMode(NeutralMode.Brake);

		// Slot 0 is reserved for MotionMagic
        setPIDF(0, 0.18, 0.0, 3.6, 1023.0 / Constants.kMaxFalconEncoderSpeed);
		this.configMotionCruiseVelocity(Constants.kMaxFalconEncoderSpeed*0.9, 10);
		this.configMotionAcceleration(Constants.kMaxFalconEncoderSpeed, 10);
		this.selectProfileSlot(0, 0);

		// Slot 1 corresponds to velocity mode
        setPIDF(1, 0.11, 0.0, 0.0, 1023.0 / Constants.kMaxFalconEncoderSpeed);
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
    public double getAppliedVoltage() {
        return Math.abs(this.getMotorOutputVoltage());
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
    public void selectProfileSlot(int slotIndex) {
        this.selectProfileSlot(slotIndex, 0);
    }

    @Override
    public void set(ControlMode mode, double demand, double arbitraryFeedForward) {
        this.set(mode, demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    @Override
    public boolean isConnected() {
        // Try to configure an effectively inconsequential setting on the Talon
        ErrorCode status =  this.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        return status == ErrorCode.OK;
    }
}
