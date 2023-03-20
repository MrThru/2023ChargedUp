package com.team1323.frc2023.subsystems.servo;

import java.util.List;

public class ServoSubsystemConfig {
    public final int leaderPortNumber;
    public final List<Integer> followerPortNumbers;
    public final String canBus;
    public final double maxEncoderVelocity;
    public final double encoderUnitsPerOutputUnit;
    public final double minOutputUnits;
    public final double maxOutputUnits;
    public final double outputUnitTolerance;
    public final double cruiseVelocityScalar;
    public final double accelerationScalar;

    public ServoSubsystemConfig(int leaderPortNumber, List<Integer> followerPortNumbers,
            String canBus, double maxEncoderVelocity, double encoderUnitsPerOutputUnit,
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance,
            double cruiseVelocityScalar, double accelerationScalar) {
        this.leaderPortNumber = leaderPortNumber;
        this.followerPortNumbers = followerPortNumbers;
        this.canBus = canBus;
        this.maxEncoderVelocity = maxEncoderVelocity;
        this.encoderUnitsPerOutputUnit = encoderUnitsPerOutputUnit;
        this.minOutputUnits = minOutputUnits;
        this.maxOutputUnits = maxOutputUnits;
        this.outputUnitTolerance = outputUnitTolerance;
        this.cruiseVelocityScalar = cruiseVelocityScalar;
        this.accelerationScalar = accelerationScalar;
    }
}
