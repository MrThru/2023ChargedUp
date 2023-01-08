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
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
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
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }
            
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }
        
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        
        // Auto Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondPiecePickupPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondPieceToAprilTag;
        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdPiecePickupPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdPieceToAprilTagPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontBridgePath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdPieceToBridgePath;
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();

            // Auto Paths
            secondPiecePickupPath = getSecondPiecePickupPath();
            secondPieceToAprilTag = getSecondPieceToAprilTagPath();
            thirdPiecePickupPath = getThirdPiecePickupPath();
            thirdPieceToAprilTagPath = getThirdPieceToAprilTagPath();
            frontBridgePath = getFrontBridgePath();
            thirdPieceToBridgePath = getThirdPieceToBridgePath();
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(60.0, 0.0)), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPiecePickupPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74, 132), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(260, 114.5), Rotation2d.fromDegrees(0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPieceToAprilTagPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(260, 114.5), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74, 111), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPiecePickupPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74, 111), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(265.75, 79.75), Rotation2d.fromDegrees(-45)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToAprilTagPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(265.75, 79.75), Rotation2d.fromDegrees(135)));
            waypoints.add(new Pose2d(new Translation2d(74, 111), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFrontBridgePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74, 111), Rotation2d.fromDegrees(-90)));
            waypoints.add(new Pose2d(new Translation2d(152.5, 76), Rotation2d.fromDegrees(0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToBridgePath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(265.75, 79.75), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(152.5, 76), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
    }
}
