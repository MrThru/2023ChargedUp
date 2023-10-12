package com.team1323.frc2023.subsystems.encoders;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team1323.frc2023.Ports;

import edu.wpi.first.wpilibj.RobotBase;

public class PhoenixProCANCoder implements AbsoluteEncoder {
    private final CANcoder encoder;

    public static AbsoluteEncoder createRealOrSimulatedEncoder(int deviceId, boolean isReversed, double magnetOffset) {
        return RobotBase.isReal() ? new PhoenixProCANCoder(deviceId, isReversed, magnetOffset) : new SimulatedAbsoluteEncoder();
    }

    private PhoenixProCANCoder(int deviceId, boolean isReversed, double magnetOffset) {
        encoder = new CANcoder(deviceId, Ports.CANBUS);
        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.SensorDirection = isReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configuration.MagnetSensor.MagnetOffset = magnetOffset;
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

    @Override
    public boolean isConnected() {
        return encoder.getAbsolutePosition().getStatus() == StatusCode.OK;
    }
}
