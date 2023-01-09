package com.team1323.frc2023.subsystems;

import java.util.Arrays;

import com.team1323.frc2023.Ports;

public class VerticalElevator extends ServoSubsystem {
    private static VerticalElevator instance = null;
    public static VerticalElevator getInstance() {
        if(instance == null)
            instance = new VerticalElevator();
        return instance;
    }
    public VerticalElevator() {
        super(Ports.VERTICAL_ELEVATOR_LEADER, Arrays.asList(Ports.VERTICAL_ELEVATOR_FOLLOWER), 
                null, getPosition(), getPosition(), getPosition(), getPosition(), getPosition())
    }

    @Override
    public void outputTelemetry() {
        
    }
    
}
