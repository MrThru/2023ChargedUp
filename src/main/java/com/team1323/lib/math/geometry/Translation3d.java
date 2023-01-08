// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.math.geometry;

/** Add your docs here. */
public class Translation3d {
    protected double x_;
    protected double y_;
    protected double z_;

    public Translation3d() {
        x_ = 0;
        y_ = 0;
        z_ = 0;
    }
    public Translation3d(double x, double y, double z) {
        x_ = x;
        y_ = y;
        z_ = z;
    }

    public double getX() {
        return this.x_;
    }
    public double getY() {
        return this.y_;
    }
    public double getZ() {
        return this.z_;
    }

    public Translation3d translateBy(Translation3d other) {
        return new Translation3d(this.x_ + other.getX(), this.y_ + other.getY(),
                            this.z_ + other.getZ());
    }
}
