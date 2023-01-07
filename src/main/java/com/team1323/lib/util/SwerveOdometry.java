
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.wpilib.SwerveModuleState;

/** 
 * Goals(Possibly be accurate):
 *   -Implement Skid Detection
 *   -Tighten Averaged Module Vectors to get a better result
 * The Structure
 *   -We create a box with each of the corners being a Swerve Drive Module. 
 *    From there, if the "square" ever becomes out of shape, there was a possible 
 *    skid from one or multiple drive wheels.
 * 
 * 
 */
public class SwerveOdometry {
    private int numberOfModules;
    private List<Translation2d> modulePositions;

    private double prevTimestamp = 0;

    public SwerveOdometry(List<Translation2d> modulePositions) {
        numberOfModules = modulePositions.size();
        this.modulePositions = modulePositions;
    }
    //The angle of the SwerveModuleStates are field oriented
    public Pose2d update(List<SwerveModuleState> moduleStates, Rotation2d robotHeading, double timestamp) {
        double dt = timestamp - prevTimestamp;
        List<Translation2d> moduleVectors = Arrays.asList();
        List<Translation2d> moduleToModuleVectors = Arrays.asList();
        for(int i = 0; i < moduleStates.size(); i++) { //Moves the position of the module relative to the field
            Translation2d modulePosition = modulePositions.get(i);
            Translation2d currentModuleVector = Translation2d.fromPolar(moduleStates.get(i).angle, moduleStates.get(i).speedMetersPerSecond * dt);
            modulePosition = modulePosition.translateBy(currentModuleVector);
            moduleVectors.add(currentModuleVector);

        }
        for(int i = 0; i < moduleStates.size(); i++) { //Creates a vector from the Left Module To the Right Module
            moduleToModuleVectors.add(moduleVectors.get((i + 1) % 4).translateBy(moduleVectors.get(i).inverse()));
        }
        for(int i = 0; i < moduleToModuleVectors.size(); i++) {
            
        }



        prevTimestamp = timestamp;
        return new Pose2d();
    }

}
