package com.team1323.frc2023.subsystems.servo;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.util.Util;

/**
 * A class which can serve as the base for any subsystem that is primarily controlled
 * with MotionMagic running on a Talon FX.
 */
public abstract class ServoSubsystem<Inputs extends ServoSubsystemInputsAutoLogged> extends Subsystem {
    protected final ServoSubsystemConfig config;

    protected final MotorController leader;
    protected final List<MotorController> allMotors;
    protected final List<MotorController> followers;

    protected final Inputs inputs;
    protected final ServoSubsystemOutputs outputs = new ServoSubsystemOutputs();

    public ServoSubsystem(MotorController leader, List<MotorController> followers, 
            ServoSubsystemConfig config, Inputs inputs) {
        this.config = config;
        this.inputs = inputs;
        this.leader = leader;
        this.followers = followers;
        allMotors = new ArrayList<>();
        allMotors.add(leader);
        allMotors.addAll(followers);
        configureMotors();
    }

    private void configureMotors() {
        allMotors.forEach(MotorController::configureAsServo);

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
        allMotors.forEach(m -> m.configSupplyCurrentLimit(amps));
    }

    protected void setStatorCurrentLimit(double amps) {
        allMotors.forEach(m -> m.configStatorCurrentLimit(amps));
    }

    protected void disableStatorCurrentLimit() {
        allMotors.forEach(m -> m.disableStatorCurrentLimit());
    }

    protected double encoderUnitsToOutputUnits(double encoderUnits) {
        return encoderUnits / config.encoderUnitsPerOutputUnit;
    }

    protected double outputUnitsToEncoderUnits(double outputUnits) {
        return outputUnits * config.encoderUnitsPerOutputUnit;
    }

    public double getVelocityOutputUnitsPerSecond() {
        double encoderUnitsPer100Ms = inputs.velocity;
        double encoderUnitsPerSecond = encoderUnitsPer100Ms * 10.0;
        return encoderUnitsToOutputUnits(encoderUnitsPerSecond);
    }

    protected void zeroPosition() {
        leader.setSelectedSensorPosition(0.0);
    }

    public double getPosition() {
        return encoderUnitsToOutputUnits(inputs.position);
    }

    public void setPosition(double outputUnits) {
        double boundedOutputUnits = Util.limit(outputUnits, config.minOutputUnits, config.maxOutputUnits);
        outputs.demand = outputUnitsToEncoderUnits(boundedOutputUnits);
        outputs.controlMode = ControlMode.MotionMagic;
    }

    public void setPositionWithCruiseVelocity(double outputUnits, double cruiseVelocityScalar) {
        leader.configMotionCruiseVelocity(config.maxEncoderVelocity * cruiseVelocityScalar);
        setPosition(outputUnits);
    }

    public void lockPosition() {
        outputs.demand = inputs.position;
        outputs.controlMode = ControlMode.MotionMagic;
    }

    protected void setOpenLoop(double percent) {
        outputs.demand = percent;
        outputs.controlMode = ControlMode.PercentOutput;
    }

    public boolean isAtPosition(double position) {
        return outputs.controlMode == ControlMode.MotionMagic &&
                Math.abs(position - getPosition()) <= config.outputUnitTolerance;
    }

    public boolean isOnTarget() {
        return isAtPosition(encoderUnitsToOutputUnits(outputs.demand));
    }

    public boolean isWithinTolerance(double tolerance) {
        return Math.abs(encoderUnitsToOutputUnits(outputs.demand) - getPosition()) <= tolerance &&
            outputs.controlMode == ControlMode.MotionMagic;
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
        } else if (outputs.controlMode == ControlMode.PercentOutput) {
            lockPosition();
        }
    }

    protected void updateInputsFromIO() {
        inputs.position = leader.getSelectedSensorPosition();
        inputs.velocity = leader.getVelocityEncoderUnitsPer100Ms();
    }

    @Override
    public void readPeriodicInputs() {
        updateInputsFromIO();
        Logger.getInstance().processInputs(config.logTableName, inputs);
    }

    @Override
    public void writePeriodicOutputs() {
        leader.set(outputs.controlMode, outputs.demand, outputs.arbitraryFeedForward);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @AutoLog
    public static class ServoSubsystemInputs {
        public double position;
        public double velocity;
    }

    public static class ServoSubsystemOutputs {
        public double demand = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
        public double arbitraryFeedForward = 0.0;
    }
}
