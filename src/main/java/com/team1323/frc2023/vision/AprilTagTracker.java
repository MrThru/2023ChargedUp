// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import com.team1323.lib.math.geometry.Plane;
import com.team1323.lib.math.geometry.Pose3d;
import com.team1323.lib.math.geometry.Translation3d;
import com.team1323.lib.math.geometry.Vector3d;
import com.team254.lib.geometry.Rotation2d;

/** Add your docs here. */
public class AprilTagTracker {
    private static AprilTagTracker instance = null;
    public AprilTagTracker getInstance() {
        if(instance == null)
            instance = new AprilTagTracker();
        return instance;
    }
    public enum AprilTags {
        ZERO(0, new Pose3d(new Vector3d(), new Rotation2d()));
        Pose3d pose3d;
        int id;
        AprilTags(int id, Pose3d pose3d) {
            this.pose3d = pose3d;
            this.id = id;
        }

    }

    private AprilTags currentDetectedAprilTag = AprilTags.ZERO;
    

    public AprilTags getCurrentDetectedAprilTag() {
        return currentDetectedAprilTag;
    }

    public AprilTagTracker() {

    }
    /**
     * 
     * @param x - Lateral distance
     * @param y - Vertical distance
     * @param z - Distance
     * @param yaw - In Radians
     * @param pitch - In Radians
     * @param roll - In Radians
     * @return A 3d plane of the April Tag
     */
    public Plane aprilTagInfoTo3dPlane(double x, double y, double z, double yaw, double pitch, double roll) {
        Vector3d pointVector = new Vector3d(x, y, z);
        Plane plane = new Plane(pointVector.add(new Vector3d(Math.cos(yaw), Math.cos(pitch), Math.sin(roll))), pointVector);
        return plane;
    }


    
    public void update() {

    }
}
