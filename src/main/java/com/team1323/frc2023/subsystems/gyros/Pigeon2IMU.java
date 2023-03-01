// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems.gyros;

import com.ctre.phoenix.sensors.Pigeon2;
import com.team1323.frc2023.Ports;
import com.team254.lib.geometry.Rotation2d;

/** Add your docs here. */
public class Pigeon2IMU extends Gyro {
    Pigeon2 pigeon;

    private static Pigeon2IMU instance = null;
    public static Pigeon2IMU getInstance() {
        if(instance == null)
            instance = new Pigeon2IMU();
        return instance;
    }

    public Pigeon2IMU() {
        pigeon = new Pigeon2(Ports.PIGEON, Ports.CANBUS);

    }

    @Override
    public void setAngle(double angle) {
        pigeon.setYaw(angle);
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pigeon.getPitch());
    }

    private double rollOffset = 0;
    public void resetRoll() {
        rollOffset = getRoll().getDegrees();
    }
    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(pigeon.getRoll() - rollOffset);
    }
    
    @Override
    public double[] getYPR(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}

}
