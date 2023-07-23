package com.team1323.frc2023.subsystems.encoders;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.team1323.frc2023.Ports;

import edu.wpi.first.wpilibj.RobotBase;

public class Phoenix5CANCoder implements AbsoluteEncoder {
    private final CANCoder encoder;
    private final CANCoderConfiguration configuration;

    public static AbsoluteEncoder createRealOrSimulatedEncoder(int deviceId, boolean isReversed) {
        return RobotBase.isReal() ? new Phoenix5CANCoder(deviceId, isReversed) : new SimulatedAbsoluteEncoder();
    }

    private Phoenix5CANCoder(int deviceId, boolean isReversed) {
        encoder = new CANCoder(deviceId, Ports.CANBUS);
        configuration = new CANCoderConfiguration();
        configuration.sensorDirection = isReversed;
        configuration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        configuration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        configuration.sensorTimeBase = SensorTimeBase.Per100Ms_Legacy;
        encoder.configAllSettings(configuration);
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public void setPosition(double degrees) {
        encoder.setPosition(degrees);
    }

    @Override
    public boolean isConnected() {
        return encoder.configAllSettings(configuration, 10) == ErrorCode.OK;
    }
}
