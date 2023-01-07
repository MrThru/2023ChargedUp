// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An easier way to connect robot code to the custom dashboard */
public class Netlink {
    private boolean defaultBooleanValue = false;
    private String defaultStringValue = "";
    private double defaultDoubleValue = 0;

    private String name = "";
    private static List<String> initializedSD = new ArrayList<>();
    
    private ValueTypes valueType = ValueTypes.NONE;
    public enum ValueTypes {
        STRING, DOUBLE, BOOLEAN, NONE;
    }
    public Netlink(String name) {
        this.name = name;
        valueType = ValueTypes.NONE;
        initializeDashboard();
    }
    public Netlink(String name, boolean defaultValue) {
        this.name = name;
        this.defaultBooleanValue = defaultValue;
        valueType = ValueTypes.BOOLEAN;
        initializeDashboard();
    }
    public Netlink(String name, String defaultValue) {
        this.name = name;
        this.defaultStringValue = defaultValue;
        valueType = ValueTypes.STRING;
        initializeDashboard();
    }
    public Netlink(String name, double defaultValue) {
        this.name = name;
        this.defaultDoubleValue = defaultValue;
        valueType = ValueTypes.DOUBLE;
        initializeDashboard();
    }

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

    private void initializeDashboard() {
        switch(valueType) {
            case BOOLEAN:
                boolean dashboardBooleanValue = SmartDashboard.getBoolean(name, defaultBooleanValue);
                SmartDashboard.putBoolean(name, dashboardBooleanValue);
                break;
            case STRING:
                String dashboardStringValue = SmartDashboard.getString(name, defaultStringValue);
                SmartDashboard.putString(name, dashboardStringValue);
                break;
            case DOUBLE:
                double dashboardDoubleValue = SmartDashboard.getNumber(name, defaultDoubleValue);
                SmartDashboard.putNumber(name, dashboardDoubleValue);
                
                break;
            case NONE:
                break;
            default:
                break;
        }
    }
}