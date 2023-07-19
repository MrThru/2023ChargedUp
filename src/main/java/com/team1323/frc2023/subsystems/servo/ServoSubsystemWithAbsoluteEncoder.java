package com.team1323.frc2023.subsystems.servo;

import java.util.List;

import com.team1323.frc2023.subsystems.encoders.AbsoluteEncoder;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class ServoSubsystemWithAbsoluteEncoder<Inputs extends ServoSubsystemWithAbsoluteEncoderInputs> extends ServoSubsystemWithCurrentZeroing<Inputs> {
    private static final int kMaxPositionResets = 50;

    protected final AbsoluteEncoder absoluteEncoder;
    private final AbsoluteEncoderInfo absoluteEncoderInfo;
    private int numPositionResets = 0;

    public ServoSubsystemWithAbsoluteEncoder(MotorController leader, List<MotorController> followers, ServoSubsystemConfig servoConfig, CurrentZeroingConfig currentZeroingConfig, 
            AbsoluteEncoder encoder, AbsoluteEncoderInfo encoderInfo, Inputs inputs) {
        super(leader, followers, servoConfig, currentZeroingConfig, inputs);
        absoluteEncoder = encoder;
        absoluteEncoderInfo = encoderInfo;
        isZeroed = true;
    }

    @Override
    protected void updateInputsFromIO() {
        super.updateInputsFromIO();
        inputs.absoluteEncoderDegrees = absoluteEncoder.getDegrees();
    }

    protected void setSensorPosition(double encoderUnits) {
        leader.setSelectedSensorPosition(encoderUnits);
    }

    public void setPositionToAbsolute() {
        double absoluteEncoderOffset = Util.boundAngle0to360Degrees(absoluteEncoder.getDegrees() - absoluteEncoderInfo.encoderZeroingAngle);
        double absoluteSubsystemAngle = absoluteEncoderInfo.subsystemZeroingAngle + (absoluteEncoderOffset / absoluteEncoderInfo.encoderToOutputRatio);
        if (absoluteSubsystemAngle > absoluteEncoderInfo.maxInitialSubsystemAngle) {
            absoluteEncoderOffset -= 360.0;
            absoluteSubsystemAngle = absoluteEncoderInfo.subsystemZeroingAngle + (absoluteEncoderOffset / absoluteEncoderInfo.encoderToOutputRatio);
        } else if (absoluteSubsystemAngle < absoluteEncoderInfo.minInitialSubsystemAngle) {
            absoluteEncoderOffset += 360.0;
            absoluteSubsystemAngle = absoluteEncoderInfo.subsystemZeroingAngle + (absoluteEncoderOffset / absoluteEncoderInfo.encoderToOutputRatio);
        }   

        if (absoluteSubsystemAngle > absoluteEncoderInfo.maxInitialSubsystemAngle || absoluteSubsystemAngle < absoluteEncoderInfo.minInitialSubsystemAngle) {
            DriverStation.reportError(String.format("%s position is out of bounds", config.logTableName), false);
            hasEmergency = true;
        } else {
            hasEmergency = false;
        }

        setSensorPosition(outputUnitsToEncoderUnits(absoluteSubsystemAngle));
    }

    public void setAbsolutePositionWithCounter() {
        if (numPositionResets < kMaxPositionResets) {
            setPositionToAbsolute();
            numPositionResets++;
        }
    }

    public static class AbsoluteEncoderInfo {
        public final double encoderToOutputRatio;
        public final double encoderZeroingAngle;
        public final double subsystemZeroingAngle;
        public final double minInitialSubsystemAngle;
        public final double maxInitialSubsystemAngle;

        public AbsoluteEncoderInfo(double encoderToOutputRatio, 
                double encoderZeroingAngle, double subsystemZeroingAngle, double minInitialSubsystemAngle, double maxInitialSubsystemAngle) {
            this.encoderToOutputRatio = encoderToOutputRatio;
            this.encoderZeroingAngle = encoderZeroingAngle;
            this.subsystemZeroingAngle = subsystemZeroingAngle;
            this.minInitialSubsystemAngle = minInitialSubsystemAngle;
            this.maxInitialSubsystemAngle = maxInitialSubsystemAngle;
        }
    }
}
