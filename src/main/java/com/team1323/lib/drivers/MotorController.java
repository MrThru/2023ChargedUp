package com.team1323.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public interface MotorController {
    public ErrorCode configForwardSoftLimitThreshold(double encoderUnits);
    public ErrorCode configReverseSoftLimitThreshold(double encoderUnits);
    public ErrorCode configForwardSoftLimitEnable(boolean enable);
    public ErrorCode configReverseSoftLimitEnable(boolean enable);

    public ErrorCode configMotionCruiseVelocity(double encoderUnitsPer100Ms);
    public ErrorCode configMotionAcceleration(double encoderUnitsPer100MsPerSecond);

    public ErrorCode configSupplyCurrentLimit(double amps);
    public ErrorCode configStatorCurrentLimit(double amps);
    public ErrorCode disableStatorCurrentLimit();

    public double getSupplyCurrent();
    public double getStatorCurrent();

    public double getVelocityEncoderUnitsPer100Ms();
    public double getSelectedSensorPosition();
    public ErrorCode setSelectedSensorPosition(double encoderUnits);

    public void useIntegratedSensor();
    public void useCANCoder(int cancoderId);

    public void setInverted(TalonFXInvertType invertType);
    public void setNeutralMode(NeutralMode neutralMode);

    public void setPIDF(MotorPIDF pidf);
    public ErrorCode config_IntegralZone(int slotIndex, double integralZoneEncoderUnits);

    public void set(ControlMode mode, double demand);
    public void set(ControlMode mode, double demand, double arbitraryFeedForward);

    public static class MotorPIDF {
        public final int slotIndex;
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;

        public MotorPIDF(int slotIndex, double kP, double kI, double kD, double kF) {
            this.slotIndex = slotIndex;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }
}
