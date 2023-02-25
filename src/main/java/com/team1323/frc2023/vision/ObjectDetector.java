// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import java.util.Arrays;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightHelpers.LimelightTarget_Detector;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.geometry.Raycast3d;
import com.team1323.lib.math.geometry.Vector3d;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class ObjectDetector {
    private GridTracker gridTracker;

    public ObjectDetector() {
        gridTracker = new GridTracker();

    }

    public void addDetectedObjects() {
        
    }


    public class DetectedObject {
        private String classString = "";

        private double confidence = 0.0;
        private double[] cornerPoints;
        private double tx = 0;
        private double ty = 0;

        private GoalTracker groundTracker;

        public DetectedObject(String clasString, double confidence, double[] cornerPoints,
                    double tx, double ty) {
            this.classString = clasString;
            this.confidence = confidence;
            this.cornerPoints = cornerPoints;
            this.tx = tx;
            this.ty = ty;

            groundTracker = new GoalTracker();
        }
    }

    public void addDetectedObject(LimelightTarget_Detector detectedTarget) {
        GameObject detectedGameObject;
        Pose2d swervePose = Swerve.getInstance().getPose();
        if(detectedTarget.className == "cube") {
            detectedGameObject = new Cube();
        } else if(detectedTarget.className == "cone") {
            detectedGameObject = new Cone();
        } else {
            detectedGameObject = new None();
        }
        Raycast3d raycast = Raycast3d.fromVisionDegrees(detectedTarget.tx, detectedTarget.ty);
        Vector3d ray = raycast.getRayVector().scale(1);
        
        TargetInfo targetInfo = new TargetInfo(Rotation2d.fromDegrees(-detectedTarget.tx).tan(),
                            Rotation2d.fromDegrees(detectedTarget.ty).tan());
        Vector3d objectPosition = LimelightProcessor.getInstance()
                        .getRetroTargetPosition(targetInfo, detectedGameObject.getHeight(), swervePose).toVector3d();
        detectedGameObject.setPosition(objectPosition);

    }


//    public void addDetectedGroudObject()

    public abstract class GameObject {
        public double width = 1;
        public double height = 1;
        public double length = 1;

        public String objectName = "";

        private Vector3d position = new Vector3d(); //The mid-point of the object

        public void setCube(double cubeLength) {
            this.width = cubeLength;
            this.height = cubeLength;
            this.length = cubeLength;
        }
        public double getWidth() {
            return this.getWidth();
        }
        public double getLength() {
            return this.length;
        }
        public double getHeight() {
            return this.height;
        }
        public String getName() {
            return this.objectName;
        }

        public void setPosition(Vector3d position) {
            this.position = position;
        }
        public Vector3d getPosition() {
            return this.position;
        }

    }
    public class None extends GameObject {

    }
    public class Cube extends GameObject {
        public Cube() {
            this.setCube(9.5);
            objectName = "cube";
        }

    }

    public class Cone extends GameObject {
        public Cone() {
            this.height = 12.8125;
            this.length = 8.375;
            this.width = 8.375;
            objectName = "cone";
        }
    }
}
