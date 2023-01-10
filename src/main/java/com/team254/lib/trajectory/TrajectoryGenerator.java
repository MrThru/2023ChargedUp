package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 120.0; // 120 
    private static final double kMaxDecel = 72.0; //72
    private static final double kMaxVoltage = 9.0;
    
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;
    
    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }
    
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    
    public void generateTrajectories() {
        if(mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }
    
    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double start_vel,  // inches/s
    double end_vel,  // inches/s
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    // CRITICAL POSES
    // Origin is the bottom left corner of the field, when viewed from above with the blue alliance on the left side.
    // +x is towards the center of the field.
    // +y is to the left.
    static final Pose2d autoStartingPose = new Pose2d(new Translation2d(297.7142857142857, 92.85714285714286), Rotation2d.fromDegrees(90));
    static final Pose2d autoEjectStartingPose = new Pose2d(new Translation2d(233.71428571428572, -40.28571428571429), Rotation2d.fromDegrees(-135.0));

    static final Pose2d firstBallPickupPose = new Pose2d(new Translation2d(297.7142857142857, 133.42857142857144), Rotation2d.fromDegrees(90));
    static final Pose2d firstOpponentBallPickupPose = new Pose2d(new Translation2d(360.0, 140.42857142857144), Rotation2d.fromDegrees(90));
    static final Pose2d secondBallPickupPose = new Pose2d(new Translation2d(215.42857142857142, 95.0), Rotation2d.fromDegrees(180));
    static final Pose2d postTerminalShotPose = new Pose2d(new Translation2d(283.42857142857144, 102), Rotation2d.fromDegrees(0));
    static final Pose2d secondOpponentBallPickupPose = new Pose2d(new Translation2d(162.0, 30.0), Rotation2d.fromDegrees(-90));
    static final Pose2d humanPlayerPickupPose = new Pose2d(new Translation2d(54.285714285714285, 104.28571428571428), Rotation2d.fromDegrees(135.0)).transformBy(Pose2d.fromTranslation(new Translation2d(3.0, 0.0))); //(11, 12)
    static final Pose2d humanPlayerBackupPose = humanPlayerPickupPose.transformBy(Pose2d.fromTranslation(new Translation2d(-24.0, 0)));
    static final Pose2d thirdBallPickupPose = new Pose2d(new Translation2d(205, -68), Rotation2d.fromDegrees(-90)).transformBy(Pose2d.fromTranslation(Translation2d.fromPolar(Rotation2d.fromDegrees(-135.0), 0.0)));
    static final Pose2d thirdOpponentBallPickupPose = new Pose2d(new Translation2d(234, -107.71428571428572), Rotation2d.fromDegrees(-90));

    static final Pose2d opponentBallEjectPosition = new Pose2d(new Translation2d(318.0, -130.42857142857143), Rotation2d.fromDegrees(-45.0));
    static final Pose2d wallRideStartPosition = new Pose2d(new Translation2d(290, -140), Rotation2d.fromDegrees(-50.8263420296 + -90));
    static final Pose2d wallRideEndPosition = new Pose2d(new Translation2d(180, -103), Rotation2d.fromDegrees(90));

    static final Pose2d closeWallEjectPosition = new Pose2d(new Translation2d(38.285714285714285, -4.0), Rotation2d.fromDegrees(180.0));
    static final Pose2d backSideEjectPosition = new Pose2d(new Translation2d(246, -30), Rotation2d.fromDegrees(80.0));    

    static final Pose2d leftMidlinePosition = new Pose2d(new Translation2d(247, -116), Rotation2d.fromDegrees(0));
    
    public class TrajectorySet {
        public class MirroredTrajectory {
            private static final double kXMirror = 325.625;
            private static final double kYMirror = 108.5;

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> bottomLeft) {
                this.bottomLeft = bottomLeft;
                double defaultVelocity = bottomLeft.defaultVelocity();
                topLeft = TrajectoryUtil.mirrorAboutYTimed(bottomLeft, kYMirror, defaultVelocity);
                topRight = TrajectoryUtil.mirrorAboutXTimed(topLeft, kXMirror, defaultVelocity);
                bottomRight = TrajectoryUtil.mirrorAboutYTimed(topRight, kYMirror, defaultVelocity);
            }
            
            public final Trajectory<TimedState<Pose2dWithCurvature>> bottomLeft;
            public final Trajectory<TimedState<Pose2dWithCurvature>> topLeft;
            public final Trajectory<TimedState<Pose2dWithCurvature>> topRight;
            public final Trajectory<TimedState<Pose2dWithCurvature>> bottomRight;
        }
        
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        
        // Auto Paths
        public final MirroredTrajectory secondPiecePickupPath;
        public final MirroredTrajectory secondPieceToAprilTag;
        public final MirroredTrajectory thirdPiecePickupPath;
        public final MirroredTrajectory thirdPieceToAprilTagPath;
        public final MirroredTrajectory frontBridgePath;
        public final MirroredTrajectory thirdPieceToBridgePath;
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();

            // Auto Paths
            secondPiecePickupPath = new MirroredTrajectory(getSecondPiecePickupPath());
            secondPieceToAprilTag = new MirroredTrajectory(getSecondPieceToAprilTagPath());
            thirdPiecePickupPath = new MirroredTrajectory(getThirdPiecePickupPath());
            thirdPieceToAprilTagPath = new MirroredTrajectory(getThirdPieceToAprilTagPath());
            frontBridgePath = new MirroredTrajectory(getFrontBridgePath());
            thirdPieceToBridgePath = new MirroredTrajectory(getThirdPieceToBridgePath());
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(60.0, 0.0)), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPiecePickupPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74, 22.25), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(260, 38.25), Rotation2d.fromDegrees(0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPieceToAprilTagPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(260, 38.25), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74, 44), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPiecePickupPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74, 44), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(265.75, 73), Rotation2d.fromDegrees(45)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToAprilTagPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(265.75, 73), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(74, 44), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFrontBridgePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74, 44), Rotation2d.fromDegrees(90)));
            waypoints.add(new Pose2d(new Translation2d(152.5, 85), Rotation2d.fromDegrees(0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToBridgePath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(265.75, 73), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(152.5, 85), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
    }
}
