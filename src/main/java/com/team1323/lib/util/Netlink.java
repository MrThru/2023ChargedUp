// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/** An easier way to connect robot code to the custom dashboard */
public class Netlink {
    
    
    private static Netlink instance = null;
    public static Netlink getInstance() {
        if(instance == null)
            instance = new Netlink();
        return instance;
    }
    
    private Netlink() {

    }

    private boolean dashboardAlive = false;
    private double lastDashboardTimestamp = 0;
    public boolean isDashboardAlive() {
        return dashboardAlive;
    }
    public void updateDriveStationStatus() {
        double dashboardTimestamp = getNumberValue("Dashboard Robot Timestamp");
        dashboardAlive = (dashboardTimestamp != lastDashboardTimestamp);
        lastDashboardTimestamp = dashboardTimestamp;
    }
    
    public void update() {
        updateDriveStationStatus();
        if(isDashboardAlive()) {
            // Send any data needed by the dashboard here
        }
    }
    

    private static Map<String, LoggedDashboardNumber> dashboardNumbers = new HashMap<>();
    private static Map<String, LoggedDashboardBoolean> dashboardBooleans = new HashMap<>();

    private static void addNumberIfNotPresent(String name) {
        if (!dashboardNumbers.containsKey(name)) {
            LoggedDashboardNumber newDashboardNumber = new LoggedDashboardNumber(name, 0);
            newDashboardNumber.set(0);
            dashboardNumbers.put(name, newDashboardNumber);
        }
    }
    
    public static double getNumberValue(String name) {
        addNumberIfNotPresent(name);
        return dashboardNumbers.get(name).get();
    }

    public static void setNumberValue(String name, double value) {
        addNumberIfNotPresent(name);
        dashboardNumbers.get(name).set(value);
    }

    private static void addBooleanIfNotPresent(String name) {
        if (!dashboardBooleans.containsKey(name)) {
            LoggedDashboardBoolean newDashboardBoolean = new LoggedDashboardBoolean(name, false);
            newDashboardBoolean.set(false);
            dashboardBooleans.put(name, newDashboardBoolean);
        }
    }

    public static boolean getBooleanValue(String name) {
        addBooleanIfNotPresent(name);
        return dashboardBooleans.get(name).get();
    } 

    public static void setBooleanValue(String name, boolean value) {
        addBooleanIfNotPresent(name);
        dashboardBooleans.get(name).set(value);
    }
}