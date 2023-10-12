package com.team1323.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class SimulatedMotorController implements MotorController {

    @Override
    public void configureAsRoller() {
    }

    @Override
    public void configureAsServo() {
    }

    @Override
    public void configureAsCoaxialSwerveRotation() {
    }

    @Override
    public void configureAsCoaxialSwerveDrive() {
    }

    @Override
    public void configureAsDifferentialSwerveMotor() {
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double encoderUnits) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double encoderUnits) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double encoderUnitsPer100Ms) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionAcceleration(double encoderUnitsPer100MsPerSecond) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(double amps) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currentLimitConfig) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatorCurrentLimit(double amps) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode disableStatorCurrentLimit() {
        return ErrorCode.OK;
    }

    @Override
    public double getSupplyAmps() {
        return 0.0;
    }

    @Override
    public double getStatorAmps() {
        return 0.0;
    }

    @Override
    public double getAppliedVoltage() {
        return 0.0;
    }
    @Override
    public double getMotorTemperature() {
        return 0.0;
    }

    @Override
    public double getVelocityEncoderUnitsPer100Ms() {
        return 0.0;
    }

    @Override
    public double getSelectedSensorPosition() {
        return 0.0;
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double encoderUnits) {
        return ErrorCode.OK;
    }

    @Override
    public void useIntegratedSensor() {
    }

    @Override
    public void useCANCoder(int cancoderId) {
    }

    @Override
    public void setInverted(TalonFXInvertType invertType) {
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
    }

    @Override
    public void setPIDF(MotorPIDF pidf) {
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIndex, double integralZoneEncoderUnits) {
        return ErrorCode.OK;
    }

    @Override
    public void selectProfileSlot(int slotIndex) {
    }

    @Override
    public void set(ControlMode mode, double demand) {
    }

    @Override
    public void set(ControlMode mode, double demand, double arbitraryFeedForward) {
    }

    @Override
    public boolean isConnected() {
        return true;
    }
    
}
