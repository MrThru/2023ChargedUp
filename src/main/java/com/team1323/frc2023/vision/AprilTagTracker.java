// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import com.team1323.lib.math.geometry.Plane;
import com.team1323.lib.math.geometry.Pose3d;
import com.team1323.lib.math.geometry.Rotation3d;
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

    
    public void update() {

    }


    public static class AprilTag {
        private Vector3d tagPosition;
        private Rotation3d tagOrientation;
        public AprilTag() {
            this(new Vector3d(), new Rotation3d()); 
        }
        public AprilTag(Vector3d tagPosition, Rotation3d tagOrientation) {
            this.tagPosition = tagPosition;
            this.tagOrientation = tagOrientation;
        }
        public Rotation3d getRotation() {
            return this.tagOrientation;
        }
        public Vector3d getPosition() {
            return this.tagPosition;
        }
    }
}
