package com.team1323.frc2023;

public class Ports {
    //TODO: Update Ports with IRL Number
    
    // CAN Devices 
    // Falcons
    public static final int FRONT_RIGHT_ROTATION= 4;
    public static final int FRONT_RIGHT_DRIVE   = 7;

    public static final int FRONT_LEFT_ROTATION = 11;
    public static final int FRONT_LEFT_DRIVE    = 16;

    public static final int REAR_LEFT_ROTATION  = 8;
    public static final int REAR_LEFT_DRIVE     = 13;

    public static final int REAR_RIGHT_ROTATION = 1;
    public static final int REAR_RIGHT_DRIVE    = 5;
    

    public static final int VERTICAL_ELEVATOR_LEADER = 40;
    public static final int VERTICAL_ELEVATOR_FOLLOWER = 50;

    public static final int HORIZONTAL_ELEVATOR_LEADER = 41;

    public static final int WRIST = 43;
    public static final int INTAKE_LEFT = 12;
    public static final int INTAKE_RIGHT = 3;
    // MISC CAN
    public static final int PIGEON = 45;
    public static final int CANDLE = 46;
    
    //PWM


    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = Settings.kIsUsingCompBot ? 3: 30; //30
    public static final int FRONT_LEFT_ENCODER = Settings.kIsUsingCompBot ? 2: 31; //31
    public static final int REAR_LEFT_ENCODER = Settings.kIsUsingCompBot ? 1: 32; //32
    public static final int REAR_RIGHT_ENCODER = Settings.kIsUsingCompBot ? 0: 33; //33
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};

    public static final int WRIST_ENCODER = 9;

    //CANCoders
    
    // Pneumatics
    public static final int PNEUMATIC_HUB = 1;
    public static final int INTAKE_LEFT_CLAMPER = 0;
    public static final int INTAKE_RIGHT_CLAMPER = 1;

    //Canifier
    public static final int CANIFIER = 30;
        
    }
    