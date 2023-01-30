// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems.gyros;

import com.team254.lib.geometry.Rotation2d;

/** Add your docs here. */
public abstract class Gyro {
    

    public abstract void setAngle(double angle);

    public abstract Rotation2d getYaw();

    public abstract Rotation2d getPitch();

    public abstract Rotation2d getRoll();

    public abstract double[] getYPR();
}
