package com.team1323.frc2023.subsystems.servo;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2023.subsystems.encoders.AbsoluteEncoder;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class ServoSubsystemWithAbsoluteEncoder extends ServoSubsystemWithCurrentZeroing {
    private static final int kMaxPositionResets = 50;

    protected final AbsoluteEncoder absoluteEncoder;
    private final AbsoluteEncoderInfo absoluteEncoderInfo;
    private int numPositionResets = 0;

    public ServoSubsystemWithAbsoluteEncoder(int portNumber, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, 
            double cruiseVelocityScalar, double accelerationScalar, CurrentZeroingConfig currentZeroingConfig, 
            AbsoluteEncoder encoder, AbsoluteEncoderInfo encoderInfo) {
        this(portNumber, new ArrayList<>(), canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits,
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar, currentZeroingConfig, encoder, encoderInfo);
    }

    public ServoSubsystemWithAbsoluteEncoder(int portNumber, List<Integer> followerPortNumbers, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, double cruiseVelocityScalar, double accelerationScalar, 
            CurrentZeroingConfig currentZeroingConfig, AbsoluteEncoder encoder, AbsoluteEncoderInfo encoderInfo) {
        super(portNumber, followerPortNumbers, canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits, 
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar, currentZeroingConfig);
        absoluteEncoder = encoder;
        absoluteEncoderInfo = encoderInfo;
        isZeroed = true;
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
            DriverStation.reportError("Servo subsystem is out of bounds", true);
            hasEmergency = true;
        } else {
            hasEmergency = false;
        }

        leader.setSelectedSensorPosition(outputUnitsToEncoderUnits(absoluteSubsystemAngle));
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
