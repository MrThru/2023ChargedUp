// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import com.team1323.lib.math.geometry.Pose3d;
import com.team1323.lib.math.geometry.Vector3d;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class AprilTagTracker {
    private static AprilTagTracker instance = null;
    public AprilTagTracker getInstance() {
        if(instance == null)
            instance = new AprilTagTracker();
        return instance;
    }
    
    public enum AprilTag {
        ZERO(0, new Vector3d(), new Rotation2d()),
        ONE(1, new Vector3d(610.77, 42.19, 18.22), Rotation2d.fromDegrees(0)),
        TWO(2, new Vector3d(610.77, 108.19, 18.22), Rotation2d.fromDegrees(0)),
        THREE(3, new Vector3d(610.77, 174.19, 18.22), Rotation2d.fromDegrees(0)),
        FOUR(4, new Vector3d(636.96, 265.74, 27.38), Rotation2d.fromDegrees(0)),
        FIVE(5, new Vector3d(14.25, 265.74, 27.38), Rotation2d.fromDegrees(180)),
        SIX(6, new Vector3d(40.45, 174.19, 18.22), Rotation2d.fromDegrees(180)),
        SEVEN(7, new Vector3d(40.45, 108.19, 18.22), Rotation2d.fromDegrees(180)),
        EIGHT(8, new Vector3d(40.45, 42.19, 18.22), Rotation2d.fromDegrees(180));

        private final int id;
        private final Vector3d position;
        private final Rotation2d orientation;

        private AprilTag(int id, Vector3d tagPosition, Rotation2d tagOrientation) {
            this.id = id;
            this.position = tagPosition;
            this.orientation = tagOrientation;
        }

        private AprilTag(int id, Pose3d pose3d) {
            this(id, pose3d.getVector3d(), pose3d.getRotation2d());
        }

        public Rotation2d getRotation() {
            return this.orientation;
        }

        public Vector3d getVector3d() {
            return this.position;
        }

        public Translation2d getTranslation2d() {
            return new Translation2d(position.x(), position.y());
        }

        public Pose2d getPose2d() {
            return new Pose2d(getTranslation2d(), getRotation());
        }

        public int getId() {
            return this.id;
        }
    }

    private AprilTag currentDetectedAprilTag = AprilTag.ZERO;
    

    public AprilTag getCurrentDetectedAprilTag() {
        return currentDetectedAprilTag;
    }

    public AprilTagTracker() {
    }

    
    public void update() {

    }
}
