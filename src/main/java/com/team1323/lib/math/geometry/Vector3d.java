/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.lib.math.geometry;

/**
 * Add your docs here.
 */
public class Vector3d {
    private double x_, y_, z_;

    public Vector3d(double x, double y, double z) {
        x_ = x;
        y_ = y;
        z_ = z;
    }
    public Vector3d() {
        x_ = 0;
        y_ = 0;
        z_ = 0;
    }

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }

    public double z() {
        return z_;
    }

    public Vector3d inverse() {
        return new Vector3d(-x_, -y_, -z_);
    }

    public Vector3d add(Vector3d other) {
        return new Vector3d(x_ + other.x_, y_ + other.y_, z_ + other.z_);
    }

    public Vector3d subtract(Vector3d other) {
        return this.add(other.inverse());
    }

    public Vector3d scale(double scalar) {
        return new Vector3d(x_ * scalar, y_ * scalar, z_ * scalar);
    }

    public double magnitude() {
        return Math.sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    }

    public Vector3d direction() {
        return this.scale(1.0 / magnitude());
    }

    public Vector3d crossProduct(Vector3d other) {
        double i_determinant = y_ * other.z_ - z_ * other.y_;
        double j_determinant = x_ * other.z_ - z_ * other.x_;
        double k_determinant = x_ * other.y_ - y_ * other.x_;

        return new Vector3d(i_determinant, -j_determinant, k_determinant);
    }

    public static Vector3d average(Vector3d... vectors) {
        double x = 0.0, y = 0.0, z = 0.0;
        
        for (Vector3d vector : vectors) {
            x += vector.x_;
            y += vector.y_;
            z += vector.z_;
        }

        return new Vector3d(x / vectors.length, y / vectors.length, z / vectors.length);
    }
}
