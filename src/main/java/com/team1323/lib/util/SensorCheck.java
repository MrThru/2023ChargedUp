// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/** A Sensor Check that works for encoders and digital input devices such as a banner sensor */
public class SensorCheck {
    public static enum SensorType {
        CANCoder, MAGCoder, FalconFX, Banner, None;
    }
    public static enum SensorStatus {
        DISCONNECTED, IDLE, OPERATIONAL
    }

    Object sensorInstance;
    public SensorType selectedSensorType = SensorType.None;
    public SensorStatus currentSensorStatus = SensorStatus.DISCONNECTED;


    public interface VoidInterface {
        void f();
    }
    VoidInterface mF;

    public SensorCheck(CANCoder sensor, VoidInterface f) {
        selectedSensorType = SensorType.CANCoder;
        sensorInstance = sensor;
        mF = f;
    }
    public SensorCheck(DutyCycle sensor, VoidInterface f) {
        selectedSensorType = SensorType.MAGCoder;
        sensorInstance = sensor;
        mF = f;
    }
    public SensorCheck(LazyPhoenix5TalonFX falcon, VoidInterface f) {
        selectedSensorType = SensorType.FalconFX;
        sensorInstance = falcon;
        mF = f;
    }
    public SensorCheck(DigitalInput banner, VoidInterface f) {
        selectedSensorType = SensorType.Banner;
        sensorInstance = banner;
        mF = f;
    }
    public void update() {
        switch(selectedSensorType) {
            case CANCoder:
                if (((CANCoder)sensorInstance).getBusVoltage() > 0) {
                    currentSensorStatus = SensorStatus.OPERATIONAL;
                } else {
                    currentSensorStatus = SensorStatus.DISCONNECTED;
                }
                break;
            case MAGCoder:
                if(((DutyCycle)sensorInstance).getFrequency() > 0) {
                    currentSensorStatus = SensorStatus.OPERATIONAL;
                } else {
                    currentSensorStatus = SensorStatus.DISCONNECTED;
                }
            break;
            case FalconFX:
                if(((LazyPhoenix5TalonFX)sensorInstance).getBusVoltage() > 0) {
                    currentSensorStatus = SensorStatus.OPERATIONAL;
                } else {
                    currentSensorStatus = SensorStatus.DISCONNECTED;
                }
            break;
            case Banner:
            break;
            default:
            break;
        }
        if(currentSensorStatus == SensorStatus.DISCONNECTED) {
            mF.f();
        }
    }

}
