// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems.gyros;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.RobotBase;

public class Pigeon2IMU implements Gyro {
    Pigeon2 pigeon;
    private double rollOffset = 0.0;

    public static Gyro createRealOrSimulatedGyro(int deviceId, String canBus) {
        return RobotBase.isReal() ? new Pigeon2IMU(deviceId, canBus) : new SimulatedGyro();
    }

    private Pigeon2IMU(int deviceId, String canBus) {
        pigeon = new Pigeon2(deviceId, canBus);
    }

    @Override
    public void setYaw(double angle) {
        pigeon.setYaw(angle);
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch();
    }

    @Override
    public void resetRoll() {
        rollOffset = pigeon.getRoll();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll() - rollOffset;
    }
    
    @Override
    public double[] getYPR(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}

}
