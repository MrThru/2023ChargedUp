// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.vision.LimelightHelpers.LimelightTarget_Detector;
import com.team1323.frc2023.vision.ObjectDetector.Cube;
import com.team1323.frc2023.vision.ObjectDetector.GameObject;
import com.team1323.lib.math.geometry.Vector3d;
import com.team254.lib.geometry.Pose2d;

/** Contains the Grid tracking system. This is fed detected objects
 * every cycle, and is factored into the "current detected grid" which
 * handles the grid system
 * 
 */
public class GridTracker {
    
    private static GridTracker instance = null;
    public static GridTracker getInstance() {
        if(instance == null)
            instance = new GridTracker();
        return instance;
    }


    /*private Map<String, List<PieceLocations>> gridPositions;
    {
      gridPositions.put("Top", Arrays.asList());
      gridPositions.put("Mid", Arrays.asList());
      gridPositions.put("Low", Arrays.asList());
    }*/
    public GridTracker() {
        constructGridLocations();
    }

    public void constructGridLocations() {

    }

    public void addRobotScorePosition(Grid grid, Row row, Column column) {
        
    }

    public void checkDetections(LimelightTarget_Detector detectedObject) {

    }

   

    public class PieceLocations {
        int id = 0;
        Vector3d position;
        GameObject objectType;
        public PieceLocations(int id, Vector3d position, GameObject objectType) {
            this.id = id;
            this.position = position;
            this.objectType = objectType;
        }
    }
}
