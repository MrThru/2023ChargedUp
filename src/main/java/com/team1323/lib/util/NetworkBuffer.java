// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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

    private static final int kPartitionSize = 5;

    private static Map<String, Object> globalData;
    
    private static List<List<String>> partitionsNamesList = new ArrayList<>();

    static {
        for(int i = 0; i < kPartitionSize; i++) {
            partitionsNamesList.add(new ArrayList<>());
        }
    }

    private static int currentPartitionAdded = 0;
    private static int currentPartitionUpdated = 0;


    private static void addValueToPartition(String valueName) {
        partitionsNamesList.get(currentPartitionAdded).add(valueName);
        currentPartitionAdded++;
        currentPartitionAdded %= kPartitionSize - 1;
    }


    public static void replaceValueInQueue(String valueName, Object value) {
        if(globalData.containsKey(valueName)) 
            addValueToPartition(valueName);
        globalData.put(valueName, value);
    }

    public static synchronized void queueNumber(String valueName, double number) {
        replaceValueInQueue(valueName, number);
    }   
    public static synchronized void queueString(String valueName, String string) {
        replaceValueInQueue(valueName, string);
    }
    public static synchronized void queueBoolean(String valueName, boolean bool) {
        replaceValueInQueue(valueName, bool);
    }
    public static synchronized void queueNetworkValue(String valueName, NetworkTableValue value) {
        replaceValueInQueue(valueName, value);
    }
    
    private static void addValueToTable(String valueName, NetworkTableValue valueType) {
        table.putValue(valueName, valueType);
    }

    public static void update() {
        List<String> currentPartition = partitionsNamesList.get(currentPartitionUpdated);
        for(int currentEntry = 0; currentEntry < currentPartition.size(); currentEntry++) {
            String objectName = currentPartition.get(currentEntry);
            Object objectValue = globalData.get(objectName);
            Object objectClassType = objectValue.getClass();

            if(objectClassType == Double.class) {
                addValueToTable(objectName, NetworkTableValue.makeDouble((Double) objectValue));
            } else if(objectClassType == String.class) {
                addValueToTable(objectName, NetworkTableValue.makeString((String) objectValue));
            } else if(objectClassType == Boolean.class) {
                addValueToTable(objectName, NetworkTableValue.makeBoolean((Boolean) objectValue));
            }
        }
        networkTableInstance.flush();
        currentPartitionUpdated++;
        currentPartitionUpdated  %= kPartitionSize - 1;
    }
}
