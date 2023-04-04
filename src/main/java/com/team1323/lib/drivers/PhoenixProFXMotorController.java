package com.team1323.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team1323.frc2023.Constants;

public class PhoenixProFXMotorController extends TalonFX implements MotorController {
    private static final double kTalonFXEncoderResolution = 2048.0;
    private static final double kCANCoderResolution = 4096.0;

    private final TalonFXConfiguration configuration = new TalonFXConfiguration();

    public PhoenixProFXMotorController(int deviceId) {
        super(deviceId);
    }

    public PhoenixProFXMotorController(int deviceId, String canBus) {
        super(deviceId, canBus);
    }

    private double getEncoderResolution() {
        return configuration.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder ?
                kCANCoderResolution : kTalonFXEncoderResolution;
    }

    private double encoderUnitsToEncoderRotations(double encoderUnits) {
        return encoderUnits / getEncoderResolution();
    }

    private double encoderRotationsToEncoderUnits(double rotations) {
        return rotations * getEncoderResolution();
    }

    private double encoderVelocityToRotationsPerSecond(double encUnitsPer100Ms) {
        return encUnitsPer100Ms * 10.0 / getEncoderResolution();
    }

    private double rotationsPerSecondToEncoderVelocity(double rps) {
        return rps * getEncoderResolution() / 10.0;
    }

    private ErrorCode convertToErrorCode(StatusCode statusCode) {
        if (statusCode != StatusCode.OK) {
            return ErrorCode.GENERAL_ERROR;
        }

        return ErrorCode.OK;
    }

    private ErrorCode applyConfig() {
        StatusCode statusCode = this.getConfigurator().apply(configuration, Constants.kCANTimeoutMs);
        return convertToErrorCode(statusCode);
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double encoderUnits) {
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = encoderUnitsToEncoderRotations(encoderUnits);
        return applyConfig();
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double encoderUnits) {
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = encoderUnitsToEncoderRotations(encoderUnits);
        return applyConfig();
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable) {
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
        return applyConfig();
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable) {
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
        return applyConfig();
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double encoderUnitsPer100Ms) {
        configuration.MotionMagic.MotionMagicCruiseVelocity = encoderVelocityToRotationsPerSecond(encoderUnitsPer100Ms);
        return applyConfig();
    }

    @Override
    public ErrorCode configMotionAcceleration(double encoderUnitsPer100MsPerSecond) {
        configuration.MotionMagic.MotionMagicAcceleration = encoderVelocityToRotationsPerSecond(encoderUnitsPer100MsPerSecond);
        return applyConfig();
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(double amps) {
        configuration.CurrentLimits.SupplyCurrentLimit = amps;
        configuration.CurrentLimits.SupplyCurrentThreshold = amps;
        configuration.CurrentLimits.SupplyTimeThreshold = 0.25;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

        return applyConfig();
    }

    @Override
    public ErrorCode configStatorCurrentLimit(double amps) {
        configuration.CurrentLimits.StatorCurrentLimit = amps;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        return applyConfig();
    }

    @Override
    public ErrorCode disableStatorCurrentLimit() {
        configuration.CurrentLimits.StatorCurrentLimitEnable = false;
        return applyConfig();
    }

    @Override
    public double getSupplyCurrent() {
        
    }

    @Override
    public double getStatorCurrent() {

    }

    @Override
    public double getVelocityEncoderUnitsPer100Ms() {
        return rotationsPerSecondToEncoderVelocity(this.getVelocity().getValue());
    }

    @Override
    public double getSelectedSensorPosition() {
        return encoderRotationsToEncoderUnits(this.getPosition().getValue());
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double encoderUnits) {
        return convertToErrorCode(this.setRotorPosition(encoderUnitsToEncoderRotations(encoderUnits)));
    }

    @Override
    public void useIntegratedSensor() {
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        applyConfig();
    }

    @Override
    public void useCANCoder(int cancoderId) {
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        configuration.Feedback.FeedbackRemoteSensorID = cancoderId;
        applyConfig();
    }

    @Override
    public void setInverted(TalonFXInvertType invertType) {
        configuration.MotorOutput.Inverted = invertType == TalonFXInvertType.Clockwise ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        applyConfig();
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        configuration.MotorOutput.NeutralMode = neutralMode == NeutralMode.Coast ?
                NeutralModeValue.Coast : NeutralModeValue.Brake;
        applyConfig();
    }

    @Override
    public void setPIDF(MotorPIDF pidf) {
        switch (pidf.slotIndex) {
            case 0:
                configuration.Slot0.kP = pidf.kP;
                configuration.Slot0.kI = pidf.kI;
                configuration.Slot0.kD = pidf.kD;
                configuration.Slot0.kV = pidf.kF;
                break;
            case 1:
                configuration.Slot1.kP = pidf.kP;
                configuration.Slot1.kI = pidf.kI;
                configuration.Slot1.kD = pidf.kD;
                configuration.Slot1.kV = pidf.kF;
                break;
            case 2:
                configuration.Slot2.kP = pidf.kP;
                configuration.Slot2.kI = pidf.kI;
                configuration.Slot2.kD = pidf.kD;
                configuration.Slot2.kV = pidf.kF;
                break;
        }

        applyConfig();
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIndex, double integralZoneEncoderUnits) {
        
    }

    @Override
    public void set(ControlMode mode, double demand) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void set(ControlMode mode, double demand, double arbitraryFeedForward) {
        // TODO Auto-generated method stub
        
    }
}
