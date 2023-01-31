// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.math.geometry;


/** Creates a Raycast in 3d space on the field */
public class Raycast3d {
    private Vector3d startingLocation;
    private Vector3d endingLocation;

    private Vector3d ray;
    public Raycast3d() {
        this(new Vector3d(), new Vector3d());        
    }
    public Raycast3d(Vector3d startingLocation, Vector3d endingLocation) {
        this.startingLocation = startingLocation;
        this.endingLocation = endingLocation;
        this.ray = this.startingLocation.add(this.endingLocation.inverse());
    }

    public Rotation3d getRotation3d() {
        return this.ray.getRotation3d();
    }

  
    public static Raycast3d fromVisionDegrees(double xDegrees, double yDegrees) {
        return fromRotation3d(new Rotation3d(xDegrees, yDegrees, 0));
    }
    public static Raycast3d fromRotation3d(Rotation3d rotation3d) {
        return new Raycast3d(new Vector3d(), new Vector3d(rotation3d.getPitch().cos(), 
                    rotation3d.getPitch().sin(), rotation3d.getPitch().cos()));
    }


}
