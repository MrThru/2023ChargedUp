package com.team1323.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public interface MotorController {
    public void configForwardSoftLimitThreshold(double encoderUnits);
    public void configReverseSoftLimitThreshold(double encoderUnits);
    public void configForwardSoftLimitEnable(boolean enable);
    public void configReverseSoftLimitEnable(boolean enable);

    public void configMotionCruiseVelocity(double encoderUnitsPer100Ms);
    public void configMotionAcceleration(double encoderUnitsPer100MsPerSecond);

    public void configSupplyCurrentLimit(double amps);
    public void configStatorCurrentLimit(double amps);
    public void disableStatorCurrentLimit();

    public double getSupplyCurrent();
    public double getStatorCurrent();

    public double getVelocityEncoderUnitsPer100Ms();
    public double getSelectedSensorPosition();
    public double setSelectedSensorPosition(double encoderUnits);

    public void useIntegratedSensor();
    public void useCANCoder(int cancoderId);

    public void setInverted(TalonFXInvertType invertType);
    public void setNeutralMode(NeutralMode neutralMode);

    public void setPIDF(MotorPIDF pidf);
    public void configIntegralZone(int slotIndex, double integralZoneEncoderUnits);

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
