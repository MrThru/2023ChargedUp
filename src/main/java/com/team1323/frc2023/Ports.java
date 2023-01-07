package com.team1323.frc2023;

public class Ports {
    //TODO: Update Ports with IRL Number
    
    // CAN Devices 
    // Falcons
    public static final int FRONT_RIGHT_ROTATION= 1; 
    public static final int FRONT_RIGHT_DRIVE   = 0;

    public static final int FRONT_LEFT_ROTATION = 17;
    public static final int FRONT_LEFT_DRIVE    = 18;

    public static final int REAR_LEFT_ROTATION  = 11;
    public static final int REAR_LEFT_DRIVE     = 10;

    public static final int REAR_RIGHT_ROTATION = 8;
    public static final int REAR_RIGHT_DRIVE    = 9;
    

    public static final int INTAKE = 2;
    public static final int WRIST = 3;
    public static final int BALL_SPLITTER = 12;
    public static final int BALL_FEEDER = 13;
    public static final int COLUMN = 5;
    public static final int TURRET = 14;
    //public static final int SHOOTER_TOP = 7;
    public static final int HOOD_TALON = 15;
    public static final int SHOOTER_LEFT = 20;
    public static final int SHOOTER_RIGHT = 7;

    public static final int TELESCOPE_LEFT = 41;
    public static final int TELESCOPE_RIGHT = 40;

    // MISC CAN
    public static final int PIGEON = 45;
    public static final int CANDLE = 46;
    
    //PWM


    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = Settings.kIsUsingCompBot ? 6 : 0; //30
    public static final int FRONT_LEFT_ENCODER = Settings.kIsUsingCompBot ? 7 : 1; //31
    public static final int REAR_LEFT_ENCODER = Settings.kIsUsingCompBot ? 8 : 2; //32
    public static final int REAR_RIGHT_ENCODER = Settings.kIsUsingCompBot ? 9 : 3; //33
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
    //CANCoders
    public static final int TURRET_ENCODER = Settings.kIsUsingCompBot ? 2 : 8;
    public static final int FEEDER_BANNER = Settings.kIsUsingCompBot ? 3 : 4; //4 - 3
    public static final int COLUMN_BANNER = Settings.kIsUsingCompBot ? 5 : 6; //6 - 5
    public static final int COLOR_SENSOR = Settings.kIsUsingCompBot ? 4 : 5; //5 - 4
    public static final int WRIST_ENCODER = Settings.kIsUsingCompBot ? 1 : 7;

    // Pneumatics

    //Canifier
    public static final int CANIFIER = 30;
    public static final int HOOD_ENCODER = 9;
        
    }
    