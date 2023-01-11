// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.team1323.lib.math.geometry.Pose3d;
import com.team1323.lib.math.geometry.Rotation3d;
import com.team1323.lib.math.geometry.Vector3d;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class AprilTagTracker {
    private static AprilTagTracker instance = null;
    public AprilTagTracker getInstance() {
        if(instance == null)
            instance = new AprilTagTracker();
        return instance;
    }
    
    public enum AprilTags {
        ZERO(new AprilTag(0, new Pose3d(new Vector3d(), new Rotation2d()))),
        ONE(new AprilTag(1, new Pose3d(new Vector3d(610.77, 42.19, 18.22), Rotation2d.fromDegrees(180)))),
        TWO(new AprilTag(2, new Pose3d(new Vector3d(610.77, 108.19, 18.22), Rotation2d.fromDegrees(180)))),
        THREE(new AprilTag(3, new Pose3d(new Vector3d(610.77, 174.19, 18.22), Rotation2d.fromDegrees(180)))),
        FOUR(new AprilTag(4, new Pose3d(new Vector3d(636.96, 265.74, 27.38), Rotation2d.fromDegrees(180)))),
        FIVE(new AprilTag(5, new Pose3d(new Vector3d(14.25, 265.74, 27.38), Rotation2d.fromDegrees(0)))),
        SIX(new AprilTag(6, new Pose3d(new Vector3d(40.45, 147.19, 18.22), Rotation2d.fromDegrees(0)))),
        SEVEN(new AprilTag(7, new Pose3d(new Vector3d(40.45, 108.19, 18.22), Rotation2d.fromDegrees(0)))),
        EIGHT(new AprilTag(8, new Pose3d(new Vector3d(40.45, 42.19, 18.22), Rotation2d.fromDegrees(0))));

        AprilTag tag;
        AprilTags(AprilTag tag) {
            this.tag = tag;
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
        private Rotation2d tagOrientation;
        private int id;
        public AprilTag() {
            this(0, new Vector3d(), new Rotation2d()); 
        }
        public AprilTag(int id, Vector3d tagPosition, Rotation2d tagOrientation) {
            this.id = id;
            this.tagPosition = tagPosition;
            this.tagOrientation = tagOrientation;
        }
        public AprilTag(int id, Pose3d pose3d) {
            this(id, pose3d.getVector3d(), pose3d.getRotation2d());
        }
        public Rotation2d getRotation() {
            return this.tagOrientation;
        }
        public Vector3d getPosition() {
            return this.tagPosition;
        }
        public int getId() {
            return this.id;
        }
    }
}
