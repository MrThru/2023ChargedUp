// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import com.team1323.lib.math.geometry.Vector3d;


/** Add your docs here. */
public class FieldConversions {
    private final static double xOffset = 0;
    private final static double yOffset = 315.5;
    private final static double zOffset = 0;
    private final static double rotationOffset = 0; //In radians
    /**
     * Converts a WPI Field Coordinate to our cordinates
     * @param wpiXCord
     * @return
     */
    public static Vector3d convertWPICordTo3dCord(Vector3d wpiVectorCord) {
        return wpiVectorCord.add(new Vector3d(xOffset, yOffset, zOffset));
    }
    /**
     * Converts our cordinates to WPIs Field Coordinates
     * @param wpiXCord
     * @return
     */
    public static Vector3d convert3dCordToWPICord(Vector3d cord3d) {
        return cord3d.subtract(new Vector3d(xOffset, yOffset, zOffset));
    }

    public static Vector3d convertToField(Vector3d centerPoint, Vector3d cord) {
        return cord.subtract(centerPoint);
    }

}
