package com.team1323.frc2023.subsystems.encoders;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.team1323.frc2023.Ports;

public class CanEncoder implements AbsoluteEncoder {
    private final CANCoder encoder;

    public CanEncoder(int deviceId, boolean isReversed) {
        encoder = new CANCoder(deviceId, Ports.CANBUS);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorDirection = isReversed;
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.sensorTimeBase = SensorTimeBase.Per100Ms_Legacy;
        encoder.configAllSettings(config);
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }
}
