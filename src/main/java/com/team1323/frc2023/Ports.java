package com.team1323.frc2023;

public class Ports {
    public static final String CANBUS = "main";
    // CAN Devices 
    // Falcons
    public static final int FRONT_RIGHT_ROTATION= 0;
    public static final int FRONT_RIGHT_DRIVE   = 1;

    public static final int FRONT_LEFT_ROTATION = 2;
    public static final int FRONT_LEFT_DRIVE    = 3;

    public static final int REAR_LEFT_ROTATION  = 4;
    public static final int REAR_LEFT_DRIVE     = 5;

    public static final int REAR_RIGHT_ROTATION = 6;
    public static final int REAR_RIGHT_DRIVE    = 7;
    
    public static final int VERTICAL_ELEVATOR_LEADER = 30;

    public static final int HORIZONTAL_ELEVATOR_LEADER = 31;

    public static final int SHOULDER = 42;

    public static final int WRIST = 43;
    public static final int CLAW = 44;

    public static final int CUBE_INTAKE_WRIST = 10;
    public static final int CUBE_INTAKE = 11;

    public static final int TUNNEL_ENTRANCE_TALON = 14;
    public static final int TUNNEL_CONVEYOR_TALON = 15;
    public static final int TUNNEL_ROLLER_TALON = 16;


    public static final int WINCH = 0;
    // MISC CAN
    public static final int PIGEON = 45;
    public static final int CANDLE = 46;
    
    //PWM


    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = Settings.kIsUsingCompBot ? 3: 0; //30
    public static final int FRONT_LEFT_ENCODER = Settings.kIsUsingCompBot ? 2: 1; //31
    public static final int REAR_LEFT_ENCODER = Settings.kIsUsingCompBot ? 1: 2; //32
    public static final int REAR_RIGHT_ENCODER = Settings.kIsUsingCompBot ? 0: 3; //33
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};

    public static final int INTAKE_WRIST_ENCODER = 5;
    
    public static final int INTAKE_BANNER = 4;
    
    public static final int TUNNEL_FRONT_BANNER = 6;
    public static final int TUNNEL_REAR_BANNER = 7;
    
    //CANCoders
    
    public static final int WRIST_ENCODER = 20;
    public static final int SHOULDER_ENCODER = 21;
    // Pneumatics



    //Canifier
    public static final int CANIFIER = 30;
        
    }
    