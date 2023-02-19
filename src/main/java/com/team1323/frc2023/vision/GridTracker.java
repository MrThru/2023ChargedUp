// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.vision;

import java.util.List;

import com.team1323.frc2023.loops.LimelightHelpers.LimelightTarget_Detector;
import com.team1323.frc2023.vision.ObjectDetector.GameObject;
import com.team1323.lib.math.geometry.Vector3d;

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


    private List<PieceLocations> gridPositions;
    public GridTracker() {
        constructGridLocations();
    }

    public void constructGridLocations() {
        
    }

    public void addDetectedObject(LimelightTarget_Detector detectedTarget) {
        
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
