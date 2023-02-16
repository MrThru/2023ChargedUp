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
    private static final double kMaxAccel = 120.0; 
    private static final double kMaxDecel = 72.0;
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
    private Pose2d communityEntrancePose = new Pose2d(new Translation2d(40.45 + 13.8 + 98.75, 29.695), Rotation2d.fromDegrees(180.0));
    private Pose2d communitySweepMidPose = new Pose2d(new Translation2d(40.45 + 13.8 + 30.345 + 6.0, 108.015), Rotation2d.fromDegrees(90.0));
    private Pose2d communitySweepEndPose = new Pose2d(new Translation2d(40.45 + 13.8 + 30.345 + 6.0, 174.19 + 22.0), Rotation2d.fromDegrees(90.0));
    
    public class TrajectorySet {
        public class MirroredTrajectory {
            private static final double kXMirror = 325.625;
            private static final double kYMirror = 108.19;

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

        // Teleop Paths
        public final MirroredTrajectory communitySweepPath;
        
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

            // Teleop paths
            communitySweepPath = new MirroredTrajectory(getCommunitySweepPath());
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d());
            waypoints.add(new Pose2d(new Translation2d(72.0, 0.0), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCommunitySweepPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(communityEntrancePose);
            waypoints.add(communityEntrancePose.transformBy(Pose2d.fromTranslation(new Translation2d(32.0, 0.0))));
            waypoints.add(communitySweepMidPose);
            waypoints.add(communitySweepEndPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
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
