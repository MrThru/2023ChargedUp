package com.team1323.frc2023.subsystems.encoders;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.team1323.frc2023.Ports;

public class PhoenixProCANCoder implements AbsoluteEncoder {
    private final CANcoder encoder;

    public PhoenixProCANCoder(int deviceId, boolean isReversed) {
        encoder = new CANcoder(deviceId, Ports.CANBUS);
        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.SensorDirection = isReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder.getConfigurator().apply(configuration);
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition().getValue() * 360.0;
    }

    @Override
    public void setPosition(double degrees) {
        encoder.setPosition(degrees / 360.0);
    }
}
