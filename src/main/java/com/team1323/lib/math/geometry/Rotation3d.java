// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.math.geometry;

import com.team254.lib.geometry.Rotation2d;

/** Add your docs here. */
public class Rotation3d {
    private final Rotation2d yaw_;
    private final Rotation2d pitch_;
    private final Rotation2d roll_;
    public Rotation3d() {
        this(0, 0, 0);
    }
    public Rotation3d(double yaw, double pitch, double roll) {
        this(Rotation2d.fromDegrees(yaw), Rotation2d.fromDegrees(pitch), Rotation2d.fromDegrees(roll));
    }
    public Rotation3d(Rotation2d yaw, Rotation2d pitch, Rotation2d roll) {
        this.yaw_ = yaw;
        this.pitch_ = pitch;
        this.roll_ = roll;
    }

    public Rotation2d getYaw() {
        return this.yaw_;
    }
    public Rotation2d getPitch() {
        return this.pitch_;
    }
    public Rotation2d getRoll() {
        return this.roll_;
    }
}
