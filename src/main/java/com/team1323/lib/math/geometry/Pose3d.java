// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.math.geometry;

import com.team254.lib.geometry.Rotation2d;



/** Add your docs here. */
public class Pose3d {
    protected Vector3d vector3d;
    protected Rotation2d rotation2d;

    public Pose3d() {
        vector3d = new Vector3d();
        rotation2d = new Rotation2d();
    }
    public Pose3d(Vector3d vector, Rotation2d rotation) {
        vector3d = vector;
        rotation2d = rotation;
    }

    public Vector3d getVector3d() {
        return this.vector3d;
    }
    public Rotation2d getRotation2d() {
        return this.rotation2d;
    }
    
}
