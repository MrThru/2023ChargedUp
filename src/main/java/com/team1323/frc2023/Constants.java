package com.team1323.frc2023;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.subsystems.ServoSubsystemWithAbsoluteEncoder.AbsoluteEncoderInfo;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
    /*All distance measurements are in inches, unless otherwise noted.*/
    
    public static final double kLooperDt = 0.01;
    public static final double kAutoAimPredictionTime = 0.14; // 0.14

    public static final double kEpsilon = 0.0001;
    
    //Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 36.5; //35.5
    public static final double kRobotLength = 32.25; //35.5
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    
    public static final double kFieldLength = 629.25;
    
    //Field Landmarks
    public static final Translation2d kCenterOfField = new Translation2d(324.0, 0.0);
    public static final Pose2d kRobotStartingPose = Pose2d.identity();

    /**
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 104.625; //81.0 to bottom
    public static final double kVisionTargetRadius = 26.6875;
    public static final double kVisionTargetRelativeHeight = 38.25; // From opening of shooter to bottom of goal
    
    //Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 24.75;
    public static final double kWheelbaseWidth = 24.75;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = 7.236; //8.5 //9.586
    public static final double kCameraZOffset = 44.467; //26.776 24.524 //42.095
    public static final double kCameraYawAngleDegrees = 0.0;//-12.7
    public static final double kCameraPitchAngleDegrees = Settings.kIsUsingCompBot ? 39.5 : 36.5; //37.5

    //Limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    
    //Goal tracker constants
    public static final double kMaxGoalTrackAge = 0.5;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kTrackStabilityWeight = 1.0;
    public static final double kTrackAgeWeight = 1.0;
    public static final double kTrackSwitchingWeight = 0.0;
    public static final double kClosestVisionDistance = 26.0;//36.0
    
    public static final double kVisionPIDOutputPercent = 0.5;

    public static final double kPosePredictionTime = 0.125; // seconds 0.25
    
    public static final double kDistanceToTargetTolerance = 1.0;

    public static final double kGyroDriftPerRotation = -0.25; // degrees
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 22000.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
    public static final double kSwerveRotationMaxSpeed = 12720.0 * 0.8; //The 0.8 is to request a speed that is always achievable
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    public static final double kSwerveXInputRate = 0.5;
    public static final double kSwerveYInputRate = 0.5;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
    /**
    * To Zero: Rotate module so that bevel gear is face out. Rotate module 90° CW from the top
    * Enter angle read by the absolute encoder. Insert as degrees and subtract or add 90° to the value
    * based on where the bevel ended up.
    */
    public static final double kFrontRightEncoderStartingPos = Settings.kIsUsingCompBot ? 196.2: 0;
    public static final double kFrontLeftEncoderStartingPos = Settings.kIsUsingCompBot ? 89.2 : 0;
    public static final double kRearLeftEncoderStartingPos = Settings.kIsUsingCompBot ? 5.3 : 0;
    public static final double kRearRightEncoderStartingPos = Settings.kIsUsingCompBot ? 193.0 : 0;
    
    //Swerve Module Positions (relative to the center of the drive base)
    public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    
    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
    kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);

   


    
    //Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
    public static final double kXScrubFactor = 1.0;//1.0 / (1.0 - (9549.0 / 293093.0));
    public static final double kYScrubFactor = 1.0;//1.0 / (1.0 - (4.4736 / 119.9336));
    
    //Voltage-Velocity equation constants {m, b, x-intercept}
    //First set is the positive direction, second set is negative
    public static final double[][][] kVoltageVelocityEquations = new double[][][]{
        {{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}},
        {{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}},
        {{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}},
        {{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}
    };
    
    //Swerve Odometry Constants
    public static final double kSwerveWheelDiameter = 4.0587; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
    public static final double kSwerveDriveEncoderResolution = 2048.0; //2048.0 for falcon 500
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    /** The number of rotations the swerve rotation motor undergoes for every rotation of the module. */
    public static final double kSwerveRotationReduction = 10.2857;
    /** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
    public static final double kSwerveEncoderToWheelRatio = 6.55; //7.132867133
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    
    
    
    public static class LEDs {
        
        //LED Colors
        public static final List<Double> pink = Arrays.asList(255.0, 20.0, 30.0);
        public static final List<Double> blue = Arrays.asList(0.0, 0.0, 255.0);
        public static final List<Double> red = Arrays.asList(255.0, 0.0, 0.0);
        public static final List<Double> orange = Arrays.asList(255.0, 20.0, 0.0);
        public static final List<Double> yellow = Arrays.asList(255.0, 60.0, 0.0);
        public static final List<Double> green = Arrays.asList(0.0, 255.0, 0.0);
        public static final List<Double> purple = Arrays.asList(255.0, 0.0, 255.0);
        
        //LED Arrays
        public static final List<List<Double>> rainbow = Arrays.asList(red, orange, yellow, green, blue, pink, purple);
        
    }

    public static final double kMaxFalconEncoderSpeed = 6380.0 * 2048.0 / 600.0;
    public static final double kFalconMotionMagicFeedForward = 1023.0 / kMaxFalconEncoderSpeed;

    public static class VerticalElevator {
        public static final double kTicksPerInch = 37327.0 / 13.0;

        public static final double kMinControlHeight = 0.0;
        public static final double kMaxControlHeight = 31.0;

        public static final double kHeightTolerance = 1.0;

        public static final double kVelocityScalar = 1.0;
        public static final double kAccelerationScalar = 1.0;

        public static final double kSupplyCurrentLimit = 40.0;

        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = kFalconMotionMagicFeedForward;

        public static final double kArbitraryFeedForward = 0.0;
    }

    public static class HorizontalElevator {
        public static final double kTicksPerInch = 25890.0 / 9.0;

        public static final double kMinExtension = 0.0;
        public static final double kMaxExtension = 12.25;

        public static final double kExtensionTolerance = 1.0;

        public static final double kVelocityScalar = 0.25;
        public static final double kAccelerationScalar = 1.0;

        public static final double kSupplyLimit = 40.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
    }

    public static class Wrist {
        public static final double kMotorRotationsPerWristRotation = 81.57;
        public static final double kEncoderUnitsPerWristRotation = kMotorRotationsPerWristRotation * 2048.0;
        public static final double kEncoderUnitsPerDegree = 35858.0 / 90.0; // kEncoderUnitsPerWristRotation / 360.0;

        public static final double kMinControlAngle = -27.0;
        public static final double kMaxControlAngle = 148.0;

        public static final double kStartingAngle = 148.0;

        public static final double kAngleTolerance = 2.0;

        public static final double kVelocityScalar = 0.25;
        public static final double kAccelerationScalar = 1.0;

        public static final double kSupplyCurrentLimit = 30.0;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = kFalconMotionMagicFeedForward;

        public static final double kArbitraryFeedForward = 0.05;

        public static final AbsoluteEncoderInfo kAbsoluteEncoderInfo = new AbsoluteEncoderInfo(
            Ports.WRIST_ENCODER, 
            false, 
            1.0, 
            0.0, 
            0.0, 
            0.0, 
            45.0
        );
    }
}
