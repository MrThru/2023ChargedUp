// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.math.geometry;

/** Add your docs here. */
public class Rotation3d {
    private final double yaw_;
    private final double pitch_;
    private final double roll_;
    public Rotation3d() {
        this(0, 0, 0);
    }
    public Rotation3d(double yaw, double pitch, double roll) {
        yaw_ = yaw;
        pitch_ = pitch;
        roll_ = roll;
    }

    public double getYaw() {
        return this.yaw_;
    }
    public double getPitch() {
        return this.pitch_;
    }
    public double getRoll() {
        return this.roll_;
    }
}
