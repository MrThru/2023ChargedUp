// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems.gyros;

public interface Gyro {
    
    public void setYaw(double angle);

    public double getYaw();

    public double getPitch();

    public double getRoll();

    public double[] getYPR();

    public void resetRoll();

}
