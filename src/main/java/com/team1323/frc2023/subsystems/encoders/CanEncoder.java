package com.team1323.frc2023.subsystems.encoders;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1323.frc2023.Ports;

public class CanEncoder implements AbsoluteEncoder {
    private final CANCoder encoder;

    public CanEncoder(int deviceId, boolean isReversed) {
        encoder = new CANCoder(deviceId, Ports.CANBUS);
        encoder.configSensorDirection(isReversed);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition();
    }
}
