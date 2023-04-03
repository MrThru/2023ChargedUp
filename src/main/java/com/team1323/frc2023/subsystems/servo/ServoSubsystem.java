package com.team1323.frc2023.subsystems.servo;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.drivers.TalonFXFactory;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyPhoenix5TalonFX;

/**
 * A class which can serve as the base for any subsystem that is primarily controlled
 * with MotionMagic running on a Talon FX.
 */
public abstract class ServoSubsystem extends Subsystem {
    protected final ServoSubsystemConfig config;

    protected MotorController leader;
    protected List<MotorController> allMotors;
    protected List<MotorController> followers;

    protected PeriodicIO periodicIO = new PeriodicIO();

    public ServoSubsystem(ServoSubsystemConfig config) {
        this.config = config;
        leader = TalonFXFactory.createServoTalon(config.leaderPortNumber, config.canBus);
        followers = config.followerPortNumbers.stream()
                .map(port -> TalonFXFactory.createServoTalon(port, config.canBus))
                .collect(Collectors.toList());
        allMotors = new ArrayList<>();
        allMotors.add(leader);
        allMotors.addAll(followers);
        configureMotors();
    }

    private void configureMotors() {
        leader.configForwardSoftLimitThreshold(outputUnitsToEncoderUnits(config.maxOutputUnits));
        leader.configReverseSoftLimitThreshold(outputUnitsToEncoderUnits(config.minOutputUnits));
        enableLimits(true);

        leader.configMotionCruiseVelocity(config.maxEncoderVelocity * config.cruiseVelocityScalar);
        leader.configMotionAcceleration(config.maxEncoderVelocity * config.accelerationScalar);

        followers.forEach(f -> f.set(ControlMode.Follower, config.leaderPortNumber));
    }

    protected void enableLimits(boolean enable) {
        leader.configForwardSoftLimitEnable(enable);
        leader.configReverseSoftLimitEnable(enable);
    }

    protected void setSupplyCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 0.25);
        allMotors.forEach(m -> m.configSupplyCurrentLimit(amps));
    }

    protected void setStatorCurrentLimit(double amps) {
        StatorCurrentLimitConfiguration currentLimitConfiguration = new StatorCurrentLimitConfiguration(true, amps, amps, 0.1);
        allMotors.forEach(m -> m.configStatorCurrentLimit(amps));
    }

    protected void disableStatorCurrentLimit() {
        StatorCurrentLimitConfiguration currentLimitConfiguration = new StatorCurrentLimitConfiguration(false, 200.0, 200.0, 0.1);
        allMotors.forEach(m -> m.disableStatorCurrentLimit());
    }

    protected double encoderUnitsToOutputUnits(double encoderUnits) {
        return encoderUnits / config.encoderUnitsPerOutputUnit;
    }

    protected double outputUnitsToEncoderUnits(double outputUnits) {
        return outputUnits * config.encoderUnitsPerOutputUnit;
    }

    public double getVelocityOutputUnitsPerSecond() {
        double encoderUnitsPer100Ms = leader.getVelocityEncoderUnitsPer100Ms();
        double encoderUnitsPerSecond = encoderUnitsPer100Ms * 10.0;
        return encoderUnitsToOutputUnits(encoderUnitsPerSecond);
    }

    protected void zeroPosition() {
        leader.setSelectedSensorPosition(0.0);
    }

    public double getPosition() {
        return encoderUnitsToOutputUnits(periodicIO.position);
    }

    public void setPosition(double outputUnits) {
        double boundedOutputUnits = Util.limit(outputUnits, config.minOutputUnits, config.maxOutputUnits);
        periodicIO.demand = outputUnitsToEncoderUnits(boundedOutputUnits);
        periodicIO.controlMode = ControlMode.MotionMagic;
    }

    public void setPositionWithCruiseVelocity(double outputUnits, double cruiseVelocityScalar) {
        leader.configMotionCruiseVelocity(config.maxEncoderVelocity * cruiseVelocityScalar);
        setPosition(outputUnits);
    }

    public void lockPosition() {
        periodicIO.demand = periodicIO.position;
        periodicIO.controlMode = ControlMode.MotionMagic;
    }

    protected void setOpenLoop(double percent) {
        periodicIO.demand = percent;
        periodicIO.controlMode = ControlMode.PercentOutput;
    }

    public boolean isAtPosition(double position) {
        return periodicIO.controlMode == ControlMode.MotionMagic &&
                Math.abs(position - getPosition()) <= config.outputUnitTolerance;
    }

    public boolean isOnTarget() {
        return isAtPosition(encoderUnitsToOutputUnits(periodicIO.demand));
    }

    public boolean isWithinTolerance(double tolerance) {
        return Math.abs(encoderUnitsToOutputUnits(periodicIO.demand) - getPosition()) <= tolerance &&
            periodicIO.controlMode == ControlMode.MotionMagic;
    }

    public Prerequisite willReachPositionWithinTime(double outputUnits, double seconds) {
        return () -> {
            double outputUnitsPerSecond = getVelocityOutputUnitsPerSecond();
            boolean isHeadingTowardPosition = Math.signum(outputUnits - getPosition()) == Math.signum(outputUnitsPerSecond);
            double secondsUntilPositionReached = Double.POSITIVE_INFINITY;
            if (outputUnitsPerSecond != 0.0) {
                secondsUntilPositionReached = Math.abs(outputUnits - getPosition()) / Math.abs(outputUnitsPerSecond);
            }

            boolean willReachPosition = (isHeadingTowardPosition && secondsUntilPositionReached <= seconds) || isAtPosition(outputUnits);

            return willReachPosition;
        };
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
        leader.set(periodicIO.controlMode, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.arbitraryFeedForward);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public class PeriodicIO {
        public double position;

        public double demand = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
        public double arbitraryFeedForward = 0.0;
    }
}
