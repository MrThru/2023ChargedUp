package com.team1323.frc2023.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1323.frc2023.Constants;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

/**
 * A class which can serve as the base for any subsystem that is primarily controlled
 * with MotionMagic running on a Talon FX.
 */
public abstract class ServoSubsystem extends Subsystem {
    private static final double kMaxFalconEncoderVelocity = 6380.0 * 2048.0 / 600.0;

    protected LazyTalonFX leader;
    protected List<LazyTalonFX> allMotors;
    protected List<LazyTalonFX> followers;

    protected PeriodicIO periodicIO = new PeriodicIO();

    private final double encoderUnitsPerOutputUnit;
    private final double minOutputUnits;
    private final double maxOutputUnits;

    public ServoSubsystem(int portNumber, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double cruiseVelocityScalar, double accelerationScalar) {
        this(portNumber, new ArrayList<>(), canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits,
                cruiseVelocityScalar, accelerationScalar);
    }

    public ServoSubsystem(int portNumber, List<Integer> followerPortNumbers, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double cruiseVelocityScalar, double accelerationScalar) {
        this.encoderUnitsPerOutputUnit = encoderUnitsPerOutputUnit;
        this.minOutputUnits = minOutputUnits;
        this.maxOutputUnits = maxOutputUnits;

        leader = new LazyTalonFX(portNumber, canBus);
        followers = followerPortNumbers.stream()
                .map(port -> new LazyTalonFX(port, canBus))
                .collect(Collectors.toList());
        allMotors = Arrays.asList(leader);
        allMotors.addAll(followers);
        configureMotors(portNumber, cruiseVelocityScalar, accelerationScalar);
    }

    private void configureMotors(int leaderPortNumber, double cruiseVelocityScalar, double accelerationScalar) {
        for (LazyTalonFX motor : allMotors) {
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
        }

        leader.configForwardSoftLimitThreshold(outputUnitsToEncoderUnits(maxOutputUnits), Constants.kCANTimeoutMs);
        leader.configReverseSoftLimitThreshold(outputUnitsToEncoderUnits(minOutputUnits), Constants.kCANTimeoutMs);
        enableLimits(true);

        leader.configMotionCruiseVelocity(kMaxFalconEncoderVelocity * cruiseVelocityScalar, Constants.kCANTimeoutMs);
        leader.configMotionAcceleration(kMaxFalconEncoderVelocity * accelerationScalar, Constants.kCANTimeoutMs);

        followers.forEach(f -> f.set(ControlMode.Follower, leaderPortNumber));
    }

    protected void setPIDF(int slotIndex, double p, double i, double d, double f) {
        leader.config_kP(slotIndex, p, Constants.kCANTimeoutMs);
        leader.config_kI(slotIndex, i, Constants.kCANTimeoutMs);
        leader.config_kD(slotIndex, d, Constants.kCANTimeoutMs);
        leader.config_kF(slotIndex, f, Constants.kCANTimeoutMs);
    }
    
    protected void enableLimits(boolean enable) {
        leader.configForwardSoftLimitEnable(enable, Constants.kCANTimeoutMs);
        leader.configReverseSoftLimitEnable(enable, Constants.kCANTimeoutMs);
    }

    protected void setSupplyCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 0.25);
        allMotors.forEach(m -> m.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs));
    }

    protected double encoderUnitsToOutputUnits(double encoderUnits) {
        return encoderUnits / encoderUnitsPerOutputUnit;
    }

    protected double outputUnitsToEncoderUnits(double outputUnits) {
        return outputUnits * encoderUnitsPerOutputUnit;
    }

    protected void zeroPosition() {
        leader.setSelectedSensorPosition(0.0);
    }

    protected double getPosition() {
        return encoderUnitsToOutputUnits(periodicIO.position);
    }

    protected void setPosition(double outputUnits) {
        double boundedOutputUnits = Util.limit(outputUnits, minOutputUnits, maxOutputUnits);
        periodicIO.demand = outputUnitsToEncoderUnits(boundedOutputUnits);
        periodicIO.controlMode = ControlMode.MotionMagic;
    }

    protected void setPositionWithCruiseVelocity(double outputUnits, double cruiseVelocityScalar) {
        leader.configMotionCruiseVelocity(kMaxFalconEncoderVelocity * cruiseVelocityScalar);
        setPosition(outputUnits);
    }

    protected void lockPosition() {
        periodicIO.demand = periodicIO.position;
        periodicIO.controlMode = ControlMode.MotionMagic;
    }

    protected void setOpenLoop(double percent) {
        periodicIO.demand = percent;
        periodicIO.controlMode = ControlMode.PercentOutput;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = leader.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        leader.set(periodicIO.controlMode, periodicIO.demand);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public class PeriodicIO {
        public double position;

        public double demand = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
    }
}
