package com.team254.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Settings;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class is a thin wrapper around the TalonFX that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyPhoenix5TalonFX extends TalonFX {
    private static final String kDefaultCanBus = "rio";

    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    protected DemandType mLastDemandType = null;
    protected double mLastDemandValue = Double.NaN;

    protected double mLastSetPosition = Double.NaN;
    protected ErrorCode mLastSetPositionErrorCode = ErrorCode.GeneralError;
    
    boolean log = false;
    int id;

    boolean kSimulated = !RobotBase.isReal();

    //Simulation variables
    double simPercentOutput = 0.0;
    double simVoltage = 0.0;
    double simVoltageCompSat = 12.0;
    double simCurrent = 0.0;
    double simRampRate = 0.0;
    int simSensorPosition = 0;

    public LazyPhoenix5TalonFX(int deviceNumber) {
        super(deviceNumber);
        if(Settings.kResetTalons) super.configFactoryDefault();
        id = deviceNumber;
        if (kSimulated) {
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000, Constants.kLongCANTimeoutMs);
        }
    }

    public LazyPhoenix5TalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
        if(Settings.kResetTalons) super.configFactoryDefault();
        id = deviceNumber;
        if (kSimulated) {
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, Constants.kLongCANTimeoutMs);
            super.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000, Constants.kLongCANTimeoutMs);
        }
    }

    public void enableLogging(boolean on) {
        log = on;
    }

    public void simulate(boolean simulate){
        kSimulated = simulate;
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;

            if(!kSimulated){
                super.set(mode, value);
                if (log) {
                    System.out.println(String.format("Talon %d set to %.2f in %s mode", id, value, mode.toString()));
                    System.out.println("Closed loop target: " + getClosedLoopTarget());
                }
            }
        }
    }

    @Override
    public void set(ControlMode mode, double value, DemandType demandType, double demandValue) {
        if (mode != mLastControlMode || value != mLastSet || demandType != mLastDemandType || demandValue != mLastDemandValue) {
            mLastControlMode = mode;
            mLastSet = value;
            mLastDemandType = demandType;
            mLastDemandValue = demandValue;

            if (!kSimulated) {
                if (log) {
                    System.out.println(String.format("Talon %d set to %.2f in %s mode with demand type and value (%s, %.2f)", 
                            id, value, mode.toString(), demandType.toString(), demandValue));
                }
                super.set(mode, value, demandType, demandValue);
            }
        }
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos, int pidIdx, int timeout) {
        mLastSetPosition = sensorPos;
        mLastSetPositionErrorCode = super.setSelectedSensorPosition(sensorPos, pidIdx, timeout);
        return mLastSetPositionErrorCode;
    }

    @Override
    public double getMotorOutputVoltage(){
        if(kSimulated) return simVoltage;
        return super.getMotorOutputVoltage();
    }

    @Override
    public double getOutputCurrent(){
        if(kSimulated) return simCurrent;
        return super.getSupplyCurrent();
    }

    @Override
    public double getSelectedSensorPosition(int pidIdx){
        if(kSimulated) return simSensorPosition;
        return super.getSelectedSensorPosition(pidIdx);
    }

    @Override
    public double getSelectedSensorVelocity() {
        if(kSimulated) return 0;
        return super.getSelectedSensorVelocity();
    }
}