package com.team1323.frc2023.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1323.frc2023.Constants;
import com.team1323.lib.drivers.TalonFXFactory;
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
    private final double outputUnitTolerance;

    public ServoSubsystem(int portNumber, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, 
            double cruiseVelocityScalar, double accelerationScalar) {
        this(portNumber, new ArrayList<>(), canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits,
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar);
    }

    public ServoSubsystem(int portNumber, List<Integer> followerPortNumbers, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, double cruiseVelocityScalar, double accelerationScalar) {
        this.encoderUnitsPerOutputUnit = encoderUnitsPerOutputUnit;
        this.minOutputUnits = minOutputUnits;
        this.maxOutputUnits = maxOutputUnits;
        this.outputUnitTolerance = outputUnitTolerance;

        leader = TalonFXFactory.createServoTalon(portNumber, canBus);
        followers = followerPortNumbers.stream()
                .map(port -> TalonFXFactory.createServoTalon(port, canBus))
                .collect(Collectors.toList());
        allMotors = Arrays.asList(leader);
        allMotors.addAll(followers);
        configureMotors(portNumber, cruiseVelocityScalar, accelerationScalar);
    }

    private void configureMotors(int leaderPortNumber, double cruiseVelocityScalar, double accelerationScalar) {
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

    public boolean isOnTarget() {
        return periodicIO.controlMode == ControlMode.MotionMagic && 
                Math.abs(encoderUnitsToOutputUnits(periodicIO.demand) - getPosition()) <= outputUnitTolerance;
    }

    public void acceptManualInput(double input) {
        if (input != 0.0) {
            setOpenLoop(input);
        } else if (periodicIO.controlMode == ControlMode.PercentOutput) {
            lockPosition();
        }
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
