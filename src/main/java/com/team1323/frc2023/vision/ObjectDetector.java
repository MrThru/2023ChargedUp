// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import java.util.Arrays;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.team1323.lib.math.geometry.Vector3d;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class ObjectDetector {
    private GridTracker gridTracker;

    public ObjectDetector() {
        gridTracker = new GridTracker();

    }

    public void addDetectedObjects(JSONObject jsonDump) {
        JSONArray results = null;
        try {
            jsonDump.getJSONObject("results").toJSONArray(results);
            for(int i = 0; i < results.length(); i++) {
                JSONArray result = results.getJSONArray(i);
                
            }
        } catch(JSONException error) {
            DriverStation.reportError(error.getMessage(), null);
        }
        

    }


    public class DetectedObject {
        private String classString = "";

        private double confidence = 0.0;
        private double[] cornerPoints;
        private double tx = 0;
        private double ty = 0;

        public DetectedObject(String clasString, double confidence, double[] cornerPoints,
                    double tx, double ty) {
            this.classString = clasString;
            this.confidence = confidence;
            this.cornerPoints = cornerPoints;
            this.tx = tx;
            this.ty = ty;
        }
    }


    public abstract class GameObject {
        public double width = 0;
        public double height = 0;
        public double length = 0;

        private Vector3d position = new Vector3d();

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

        public void setPosition(Vector3d position) {
            this.position = position;
        }
        public Vector3d getPosition() {
            return this.position;
        }

    }

    public class Cube extends GameObject {
        public Cube() {
            this.setCube(9.5);
        }

    }

    public class Cone extends GameObject {
        public Cone() {
            this.height = 12.8125;
            this.length = 8.375;
            this.width = 8.375;
        }
    }
}
