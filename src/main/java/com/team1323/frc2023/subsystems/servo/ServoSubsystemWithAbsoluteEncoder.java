package com.team1323.frc2023.subsystems.servo;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2023.subsystems.encoders.AbsoluteEncoder;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class ServoSubsystemWithAbsoluteEncoder extends ServoSubsystem {
    private static final int kMaxPositionResets = 50;

    protected final AbsoluteEncoder absoluteEncoder;
    private final AbsoluteEncoderInfo absoluteEncoderInfo;
    private int numPositionResets = 0;

    public ServoSubsystemWithAbsoluteEncoder(int portNumber, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, 
            double cruiseVelocityScalar, double accelerationScalar, AbsoluteEncoder encoder, AbsoluteEncoderInfo encoderInfo) {
        this(portNumber, new ArrayList<>(), canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits,
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar, encoder, encoderInfo);
    }

    public ServoSubsystemWithAbsoluteEncoder(int portNumber, List<Integer> followerPortNumbers, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, double cruiseVelocityScalar, double accelerationScalar, 
            AbsoluteEncoder encoder, AbsoluteEncoderInfo encoderInfo) {
        super(portNumber, followerPortNumbers, canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits, 
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar);
        absoluteEncoder = encoder;
        absoluteEncoderInfo = encoderInfo;
    }

    @Override
    protected void zeroPosition() {
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
            DriverStation.reportError("Servo subsystem is out of bounds", true);
            hasEmergency = true;
        } else {
            hasEmergency = false;
        }

        leader.setSelectedSensorPosition(outputUnitsToEncoderUnits(absoluteSubsystemAngle));
    }

    public void zeroPositionWithCounter() {
        if (numPositionResets < kMaxPositionResets) {
            zeroPosition();
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
