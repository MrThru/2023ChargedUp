package com.team1323.frc2023;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.subsystems.servo.ServoSubsystemConfig;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder.AbsoluteEncoderInfo;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithCurrentZeroing.CurrentZeroingConfig;
import com.team1323.lib.drivers.MotorController.MotorPIDF;
import com.team1323.lib.math.geometry.Vector3d;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
    /*All distance measurements are in inches, unless otherwise noted.*/

    public static final double kMainThreadDt = 0.02;
    public static final double kLooperDt = 0.01;
    public static final double kAutoAimPredictionTime = 0.14;

    public static final double kEpsilon = 0.0001;
    
    //Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 36.625;
    public static final double kRobotLength = 36.125;
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    
    public static final double kFieldLength = 629.25;
    
    //Field Landmarks
    public static final Translation2d kCenterOfField = new Translation2d(324.0, 0.0);
    public static final Pose2d kRobotStartingPose = Pose2d.identity();
    public static final Pose2d kAutoStartingPose = new Pose2d(new Translation2d(71, 20), Rotation2d.fromDegrees(180));
    public static final Pose2d kThirdPoleStartingPose = new Pose2d(new Translation2d(71, 87), Rotation2d.fromDegrees(180));
    
    public static final Translation2d kFirstPickupConePosition = new Translation2d(277.7, 38.25);
    public static final Translation2d kSecondPickupConePosition = new Translation2d(277.7, 85.25);

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

    //Limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    // The origin of the limelight's field coordinate system, in terms of our coordinate system.
    // Units here are meters.
    public static final Vector3d kLimelightFieldOrigin = new Vector3d(8.270875, 4.00685, 0.0);
    
    //Goal tracker constants
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kTrackStabilityWeight = 1.0;
    public static final double kTrackAgeWeight = 1.0;
    public static final double kTrackSwitchingWeight = 0.0;
    public static final double kClosestVisionDistance = 26.0;//36.0

    public static final double kPosePredictionTime = 0.125; // seconds 0.25
    
    public static final double kGyroDriftPerRotation = -0.25; // degrees
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveMaxSpeedInchesPerSecond = 230.0;
    public static final double kSwerveRotationMaxSpeedRps = 106.33 * 0.46728;
    public static final double kSwerveRotationMaxSpeedEncUnits = 12720.0 * 0.8;
    public static final double kSwerveRotationMaxAcceleration = kSwerveRotationMaxSpeedRps * 12.5;
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
    public static final double kFrontRightEncoderStartingPos = Settings.kIsUsingCompBot ? 11.94 : 228.1;
    public static final double kFrontLeftEncoderStartingPos = Settings.kIsUsingCompBot ? 350.67 : 98.1;
    public static final double kRearLeftEncoderStartingPos = Settings.kIsUsingCompBot ? 289.43 : 356.7;
    public static final double kRearRightEncoderStartingPos = Settings.kIsUsingCompBot ? 318.81 : 165.6;
    
    //Swerve Module Positions (relative to the center of the drive base)
    public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    
    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
    kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
   
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kElevatorHeightToSwerveSpeedMap = new InterpolatingTreeMap<>();
    static {
        kElevatorHeightToSwerveSpeedMap.put(new InterpolatingDouble(-1.0), new InterpolatingDouble(1.0));
        kElevatorHeightToSwerveSpeedMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(1.0));
        kElevatorHeightToSwerveSpeedMap.put(new InterpolatingDouble(VerticalElevator.kMaxControlHeight), new InterpolatingDouble(0.3));
        kElevatorHeightToSwerveSpeedMap.put(new InterpolatingDouble(VerticalElevator.kMaxControlHeight + 1.0), new InterpolatingDouble(0.3));
    }

    

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
    public static final double kSwerveWheelDiameter = 3.85; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
    public static final double kSwerveDriveEncoderResolution = 2048.0; //2048.0 for falcon 500
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    /** The number of rotations the swerve rotation motor undergoes for every rotation of the module. */
    public static final double kSwerveRotationReduction = 15.42857143;
    /** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
    public static final double kSwerveEncoderToWheelRatio = Settings.kIsUsingCompBot ? 5.142857143 : 6.53; // 6.53 : 5.142857143
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    public static final double kSwerveModuleRotationTolerance = 10.0;
    public static final double kSwerveMotionMagicTolerance = 2.0;
    
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    
    
    

    public static final double kMaxFalconRotationsPerSecond = 6380.0 / 60.0;
    public static final double kMaxFalconEncoderSpeed = 6380.0 * 2048.0 / 600.0;
    public static final double kFalconMotionMagicFeedForward = 1023.0 / kMaxFalconEncoderSpeed;

    public static final double kMaxKrakenRotationsPerSecond = 6000.0 / 60.0;
    public static final double kMaxKrakenEncoderSpeed = 6000.0 * 2048.0 / 600.0;
    public static final double kKrakenMotionMagicFeedForward = 12.0 / kMaxKrakenRotationsPerSecond;

    public static class CubeIntake {
        public static final double kMotorRotationsPerWristRotation = 41.66666667;
        public static final double kEncoderUnitsPerWristRotation = kMotorRotationsPerWristRotation * 2048.0;
        public static final double kEncoderUnitsPerDegree = kEncoderUnitsPerWristRotation / 360.0;

        public static final double kMinControlAngle = -40.0;
        public static final double kMaxControlAngle = 110;

        public static final double kStartingAngle = 0;
        public static final double kIntakeAngle = Settings.kIsUsingCompBot ? -3.75 : -3.5;
        public static final double kFloorAngle = -38.75;

        public static final double kAngleTolerance = Settings.kIsUsingCompBot ? 2.0 : 1.0;
        public static final double kVelocityScalar = Settings.kIsUsingCompBot ? 1.0 : 1.0;
        public static final double kAccelerationScalar = Settings.kIsUsingCompBot ? 4.0 : 4.0;
        
        public static final double kSupplyCurrentLimit = 25.0;

        public static final double kStandardIntakeCurrentLimit = 50.0; //30
        public static final double kLowerIntakeCurrentLimit = 20.0;

        public static final ServoSubsystemConfig kConfig = new ServoSubsystemConfig(
            Ports.CUBE_INTAKE_WRIST,
            new ArrayList<>(),
            Ports.CANBUS,
            kMaxFalconEncoderSpeed,
            kEncoderUnitsPerDegree,
            kMinControlAngle,
            kMaxControlAngle,
            kAngleTolerance,
            kVelocityScalar,
            kAccelerationScalar,
            "CubeIntake"
        );

        public static final double kArbitraryFeedForward = 0.027; //0.025
        public static final AbsoluteEncoderInfo kEncoderInfo = new AbsoluteEncoderInfo(
            1,
            Settings.kIsUsingCompBot ? -334.13 : -208.001705, //124.287924 
            109, 
            -45,
            115
        );

        private static final MotorPIDF kPracticePID = new MotorPIDF(0, 
            0.0174999237, 
            0,
            0.6, 
            kFalconMotionMagicFeedForward
        );

        private static final MotorPIDF kCompPID = new MotorPIDF(0, 
            0.0174999237, 
            0,
            0.6, 
            0.044
        );

        public static final MotorPIDF kStandardPID = Settings.kIsUsingCompBot ? kCompPID : kPracticePID;

        public static final CurrentZeroingConfig kCurrentZeroingConfig = new CurrentZeroingConfig(
            0.1,
            100.0,
            109.0,
            108.0
        );
    }

    public static class Tunnel {

        public static final double kIntakeSpeed = 0.5;
        public static final double kIntakeConveyorSpeed = 0.2;

        public static final double kFloorRatio = 9.722222222;
        public static final double kTopRollerRatio = 2.333333333;
        public static final double kTunnelEntranceRatio = 2.333333333;

        public static final double kIntakeFrontRollerSpeed = kIntakeConveyorSpeed * -2.0;

        public static final double kFeedConveyorSpeed = 0.4;
        public static final double kFeedFrontRollerSpeed = kFeedConveyorSpeed * -1.5;

        public static final double kHoldConveyorSpeed = 0.1;
        public static final double kHoldFrontRollerSpeed = kHoldConveyorSpeed * -2.0;

        public static final double kScoreConveyorSpeed = 0.25;
        public static final double kScoreFrontRollerSpeed = 0.25;

        public static final double kTunnelEntranceSpeed = 0.75;

        public static final double kFrontBannerStopTime = 0.02;
        public static final MotorPIDF kConveyorPID = new MotorPIDF(
            0,
            0,
            0,
            0,
            0.048
        );

        public static final MotorPIDF kFrontRollerPID = new MotorPIDF(
            0,
            0,
            0,
            0,
            0.048
        );
        
        public static final MotorPIDF kTunnelEntrance = new MotorPIDF(
            0, 
            0, 
            0,
            0, 
            0.048
        );
    }

    public static class VerticalElevator {
        public static final double kTicksPerInch = 84646.0 / 18.9375;

        public static final double kMinControlHeight = 0.0;
        public static final double kMaxControlHeight = 20.0;

        public static final double kHeightTolerance = 2.0;

        public static final double kVelocityScalar = Settings.kIsUsingCompBot ? 1.0 : 1.0;
        public static final double kAccelerationScalar = Settings.kIsUsingCompBot ? 4.0 : 4.0;

        public static final double kSupplyCurrentLimit = 60.0;

        public static final ServoSubsystemConfig kConfig = new ServoSubsystemConfig(
            Ports.VERTICAL_ELEVATOR_LEADER,
            new ArrayList<>(),
            Ports.CANBUS,
            kMaxFalconEncoderSpeed,
            kTicksPerInch,
            kMinControlHeight,
            kMaxControlHeight,
            kHeightTolerance,
            kVelocityScalar,
            kAccelerationScalar,
            "VerticalElevator"
        );
        
        private static final MotorPIDF kPracticePIDF = new MotorPIDF(
            0,
            0.01,
            0.0,
            0.0,
            kFalconMotionMagicFeedForward
        );

        private static final MotorPIDF kCompPIDF = new MotorPIDF(
            0,
            0.01,
            0.0,
            0.0,
            kFalconMotionMagicFeedForward
        );
            
        public static final MotorPIDF kPIDF = Settings.kIsUsingCompBot ? kCompPIDF : kPracticePIDF;

        public static final double kArbitraryFeedForward = Settings.kIsUsingCompBot ? 0.043750 : 0.048438;

        public static final CurrentZeroingConfig kCurrentZeroingConfig = new CurrentZeroingConfig(
            -0.15,
            3.5,
            -0.3,
            0.5
        );
    }

    public static class HorizontalElevator {
        public static final double kTicksPerInch = 48714.0 / 15;

        public static final double kMinExtension = 0.0;
        public static final double kMaxExtension = 30.0;

        public static final double kExtensionTolerance = 2.0;

        public static final double kVelocityScalar = Settings.kIsUsingCompBot ? 1.0 : 1.0;
        public static final double kAccelerationScalar = Settings.kIsUsingCompBot ? 4.0 : 4.0;

        public static final double kSupplyLimit = 40.0;
        public static final double kStatorLimit = 200.0;
        public static final double kWeakStatorLimit = 10.0;

        public static final ServoSubsystemConfig kConfig = new ServoSubsystemConfig(
            Ports.HORIZONTAL_ELEVATOR_LEADER,
            new ArrayList<>(),
            Ports.CANBUS,
            kMaxFalconEncoderSpeed,
            kTicksPerInch,
            kMinExtension,
            kMaxExtension,
            kExtensionTolerance,
            kVelocityScalar,
            kAccelerationScalar,
            "HorizontalElevator"
        );

        public static final MotorPIDF kWeakPID = new MotorPIDF(
            0,
            0.0025,
            0.0,
            0.0,
            kFalconMotionMagicFeedForward
        );

        private static final MotorPIDF kPracticePIDF = new MotorPIDF(
            0,
            0.01,
            0.0,
            0.0,
            kFalconMotionMagicFeedForward
        );

        private static final MotorPIDF kCompPIDF = new MotorPIDF(
            0,
            0.01,
            0.0,
            0.0,
            kFalconMotionMagicFeedForward
        );
            
        public static final MotorPIDF kPIDF = Settings.kIsUsingCompBot ? kCompPIDF : kPracticePIDF;

        public static final CurrentZeroingConfig kCurrentZeroingConfig = new CurrentZeroingConfig(
            -0.1,
            5.0, //0.8
            -0.2,
            0.25
        );
    }

    public static class Shoulder {
        public static final double kMotorRotationsPerShoulderRotation = 69.444444; //61.728395;// 69.444444;
        public static final double kEncoderUnitsPerShoulderRotation = kMotorRotationsPerShoulderRotation * 2048.0;
        public static final double kEncoderUnitsPerDegree = Settings.kIsUsingShoulderCANCoder ? 4096.0 / 360.0 : kEncoderUnitsPerShoulderRotation / 360.0;

        public static final double kMaxCANCoderVelocity = kMaxFalconRotationsPerSecond / 10.0 / kMotorRotationsPerShoulderRotation * 4096.0;

        public static final double kMinControlAngle = -97.5;
        public static final double kMaxControlAngle = 180.0;

        public static final double kAngleTolerance = 6.0;

        public static final double kVelocityScalar = Settings.kIsUsingCompBot ? 1.0 : 1.0;
        public static final double kAccelerationScalar = Settings.kIsUsingCompBot ? 5.0 : 5.0;

        public static final double kTriggerSupplyCurrentLimit = 150.0; //30.0 - 40
        public static final double kContinuousSupplyCurrentLimit = 60.0;

        public static final ServoSubsystemConfig kConfig = new ServoSubsystemConfig(
            Ports.SHOULDER,
            new ArrayList<>(),
            Ports.CANBUS,
            Settings.kIsUsingShoulderCANCoder ? kMaxCANCoderVelocity : (Settings.kIsUsingCompBot ? kMaxKrakenEncoderSpeed : kMaxFalconEncoderSpeed),
            kEncoderUnitsPerDegree,
            kMinControlAngle,
            kMaxControlAngle,
            kAngleTolerance,
            kVelocityScalar,
            kAccelerationScalar,
            "Shoulder"
        );

        private static final MotorPIDF kPracticePIDF = new MotorPIDF(
            0,
            Settings.kIsUsingShoulderCANCoder ? 1.0 : 0.04, // 1.0 : 55.0
            0.0,
            0.0,
            Settings.kIsUsingShoulderCANCoder ? 1.475 : kFalconMotionMagicFeedForward // 1.475 : 10.0
        );

        private static final MotorPIDF kCompPIDF = new MotorPIDF(
            0,
            Settings.kIsUsingShoulderCANCoder ? 1.0 : 1.0, //0.04
            0.0,
            0.0,
            Settings.kIsUsingShoulderCANCoder ? 1.475 : kKrakenMotionMagicFeedForward
        );
            
        public static final MotorPIDF kPIDF = Settings.kIsUsingCompBot ? kCompPIDF : kPracticePIDF;

        public static final double kArbitraryFeedForward = Settings.kIsUsingCompBot ? 0.02 : 0.025;

        public static final double kCANCoderMagnetOffset = Settings.kIsUsingCompBot ? 0.0 : 0.6794106;

        public static final AbsoluteEncoderInfo kAbsoluteEncoderInfo = new AbsoluteEncoderInfo(
            1.0, 
            Settings.kIsUsingCompBot ? 26.015625 : 286.7, 
            Settings.kIsUsingCompBot ? 173.0 : 177.0, 
            -95.0,
            185.0
        );

        public static final CurrentZeroingConfig kCurrentZeroingConfig = new CurrentZeroingConfig(
            0.1,
            100.0,
            176.0,
            175.0
        );
    }

    public static class Wrist {
        public static final double kMotorRotationsPerWristRotation = 40.333333;
        public static final double kEncoderUnitsPerWristRotation = kMotorRotationsPerWristRotation * 2048.0;
        public static final double kEncoderUnitsPerDegree = kEncoderUnitsPerWristRotation / 360.0;

        public static final double kMinControlAngle = -144;
        public static final double kMaxControlAngle = 134.6;

        public static final double kAngleTolerance = 10.0;

        public static final double kVelocityScalar = Settings.kIsUsingCompBot ? 1.0 : 1.0;
        public static final double kAccelerationScalar = Settings.kIsUsingCompBot ? 4.0 : 4.0;

        public static final double kSupplyCurrentLimit = 30.0;

        public static final double kLowCurrentMode = 3.5;

        public static final ServoSubsystemConfig kConfig = new ServoSubsystemConfig(
            Ports.WRIST,
            new ArrayList<>(),
            Ports.CANBUS,
            kMaxFalconEncoderSpeed,
            kEncoderUnitsPerDegree,
            kMinControlAngle,
            kMaxControlAngle,
            kAngleTolerance,
            kVelocityScalar,
            kAccelerationScalar,
            "Wrist"
        );

        private static final MotorPIDF kPracticePIDF = new MotorPIDF(
            0,
            0.08,
            0.00005,
            0.3,
            kFalconMotionMagicFeedForward
        );

        private static final MotorPIDF kCompPIDF = new MotorPIDF(
            0,
            0.06, // TODO: Check this
            0.00005,
            0.6,
            kFalconMotionMagicFeedForward
        );

        public static MotorPIDF kPIDF = Settings.kIsUsingCompBot ? kCompPIDF : kPracticePIDF;

        public static final double kArbitraryFeedForward = 0.0;

        public static final AbsoluteEncoderInfo kAbsoluteEncoderInfo = new AbsoluteEncoderInfo(
            1.0, 
            Settings.kIsUsingCompBot ? 208.8 : 283.3, 
            0.0, 
            -149, 
            139.6
        );

        public static final CurrentZeroingConfig kCurrentZeroingConfig = new CurrentZeroingConfig(
            -0.1,
            100.0,
            -148.0,
            -147.0
        );
    }

    public static class Claw {
        
        public static final double kIntakeConeStatorCurrentLimit = Settings.kIsUsingCompBot ? 180.0 : 200.0; //200.0
        public static final double kIntakeConeStatorHoldCurrent = Settings.kIsUsingCompBot ? 15.0 : 20.0; //30.0
        public static final double kIntakeConeVelocityThreshold = 200.0;


        public static final double kIntakeCubeStatorCurrentLimit = Settings.kIsUsingCompBot ? 15.0 : 20.0;
        public static final double kIntakeCubeWeakStatorCurrentLimit = 8.0; //10
        public static final double kIntakeCubeVelocityThreshold = Settings.kIsUsingCompBot ? 2100.0 : 1500.0; //1500.0
        
        
        public static final double kIntakeCubeSpeed = 0.25;
        public static final double kIntakeConeSpeed = 0.80;

        public static final MotorPIDF kPID = new MotorPIDF(
            0, 
            0.0, 
            0.0, 
            0.0, 
            0.045
        );

        public static final double kConeOffset = 3;
    }


    public static class Winch {
        public static final double kMotorRotationsPerWinchRotation = 1.0;
        public static final double kEncoderUnitsPerWinchRotation = kMotorRotationsPerWinchRotation * 2048.0;
        public static final double kEncoderUnitsPerDegree = kEncoderUnitsPerWinchRotation / 360.0;

        public static final double kVelocityScalar = 0.25;
        public static final double kAccelerationScalar = 1.0;

        public static final double kSupplyCurrentLimit = 30.0;

        public static final ServoSubsystemConfig kConfig = new ServoSubsystemConfig(
            Ports.WINCH,
            new ArrayList<>(),
            Ports.CANBUS,
            kMaxFalconEncoderSpeed,
            kEncoderUnitsPerDegree,
            0.0,
            90.0,
            2.0,
            kVelocityScalar,
            kAccelerationScalar,
            "Winch"
        );

        public static final MotorPIDF kPIDF = new MotorPIDF(
            0,
            0.0,
            0.0,
            0.0,
            0.0 //kFalconMotionMagicFeedForward
        );
    }
}
