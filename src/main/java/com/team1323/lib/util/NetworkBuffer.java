// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

/** 
 * This class is intended to reduce the network strain between the robot and
 * the connected computer(s) by sectioning off the "queue" into multiple
 * groups and sending those groups through NetworkTables
 */
public class NetworkBuffer {
    static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private static NetworkTable table;
    
    static {
        initializeTable(networkTableInstance);
    }

    private static void initializeTable(NetworkTableInstance netTableInstance) {
        table = netTableInstance.getTable("dashPipe");
    }

    private static int GroupSize = 10;
    private static final int NumberOfPartitions = 5;

    private static List<List> entryQueue = new ArrayList<>();

    public static void replaceValueInQueue(String valueName, List value) {
        for(int currentEntryQueue = 0; currentEntryQueue < entryQueue.size(); currentEntryQueue++) {
            if(entryQueue.get(currentEntryQueue).get(0) == valueName) {
                entryQueue.set(currentEntryQueue, value);
            }
        }
    }
    public static synchronized void queueNumber(String valueName, double number) {
        replaceValueInQueue(valueName, Arrays.asList(valueName, number));
    }
    public static synchronized void queueString(String valueName, String string) {
        replaceValueInQueue(valueName, Arrays.asList(valueName, string));
    }
    public static synchronized void queueBoolean(String valueName, boolean bool) {
        replaceValueInQueue(valueName, Arrays.asList(valueName, bool));
    }
    
    private static void addValueToTable(String valueName, NetworkTableValue valueType) {
        table.putValue(valueName, valueType);
        networkTableInstance.flush();
    }

    public static void update() {
        for(int currentValue = 0; currentValue < GroupSize; currentValue++) {
            if(entryQueue.size() >= currentValue) 
                break;
            
            List currentObject = entryQueue.get(currentValue);
            Object valueClassType = currentObject.get(1).getClass();
            Object objectValue = currentObject.get(0);
            String objectName = (String) (currentObject.get(1));
            

            if(valueClassType == Double.class) {
                addValueToTable(objectName, NetworkTableValue.makeDouble((Double) objectValue));
            } else if(valueClassType == String.class) {
                addValueToTable(objectName, NetworkTableValue.makeString((String) objectValue));
            } else if(valueClassType == Boolean.class) {
                addValueToTable(objectName, NetworkTableValue.makeBoolean((Boolean) objectValue));
            }
            entryQueue.remove(currentValue);
        }
    }
}
