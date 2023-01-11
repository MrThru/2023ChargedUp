// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An easier way to connect robot code to the custom dashboard */
public class Netlink {
 
    private static List<String> initializedSD = new ArrayList<>();


    public static double getNumberValue(String name) {
        for(int i = 0; i < initializedSD.size(); i++) {
            if(initializedSD.get(i) == name)
                return SmartDashboard.getNumber(name, 0);
        }
        initializedSD.add(name);
        SmartDashboard.putNumber(name, 0);
        return SmartDashboard.getNumber(name, 0);
    }
    public static boolean getBooleanValue(String name) {
        for(int i = 0; i < initializedSD.size(); i++) {
            if(initializedSD.get(i) == name)
                return SmartDashboard.getBoolean(name, false);
        }
        initializedSD.add(name);
        SmartDashboard.putBoolean(name, false);
        return SmartDashboard.getBoolean(name, false);
    } 

}