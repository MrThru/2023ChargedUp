package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.DriveMotionPlanner;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 96.0;
    private static final double kMaxAccel = 120.0; 
    private static final double kMaxDecel = 120.0;
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

    private Pose2d secondConePickupPose = new Pose2d(new Translation2d(269.5, 30.5), Rotation2d.fromDegrees(0));
    private Pose2d thirdConePickupPose = new Pose2d(new Translation2d(278.0, 80.5), Rotation2d.fromDegrees(45))
                .transformBy(Pose2d.fromTranslation(new Translation2d(-12, 4)));
    
    public class TrajectorySet {
        public class MirroredTrajectory {
            public final Trajectory<TimedState<Pose2dWithCurvature>> bottomLeft;
            public final Trajectory<TimedState<Pose2dWithCurvature>> topLeft;
            public final Trajectory<TimedState<Pose2dWithCurvature>> topRight;
            public final Trajectory<TimedState<Pose2dWithCurvature>> bottomRight;

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> bottomLeft) {
                this.bottomLeft = bottomLeft;
                double defaultVelocity = bottomLeft.defaultVelocity();
                topLeft = TrajectoryUtil.mirrorAboutYTimed(bottomLeft, AutoZones.kYMirror, defaultVelocity);
                topRight = TrajectoryUtil.mirrorAboutXTimed(topLeft, AutoZones.kXMirror, defaultVelocity);
                bottomRight = TrajectoryUtil.mirrorAboutYTimed(topRight, AutoZones.kYMirror, defaultVelocity);
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(Quadrant quadrant) {
                switch (quadrant) {
                    case TOP_LEFT:
                        return topLeft;
                    case TOP_RIGHT:
                        return topRight;
                    case BOTTOM_RIGHT:
                        return bottomRight;
                    default:
                        return bottomLeft;
                }
            }
        }
        
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        
        // Auto Paths
        public final MirroredTrajectory secondPiecePickupPath;
        public final MirroredTrajectory secondPieceToEdgeColumn;
        public final MirroredTrajectory secondPieceToCubeScore;
        public final MirroredTrajectory edgeColumnToThirdPiece;
        public final MirroredTrajectory cubeScoreToThirdPiece;
        public final MirroredTrajectory thirdPieceToSecondConeColumn;
        public final MirroredTrajectory thirdPieceToCubeScore;
        public final MirroredTrajectory secondConeHighScorePath;
        public final MirroredTrajectory secondPieceToAprilTag;
        public final MirroredTrajectory secondConeScoreToThirdConePickup;
        public final MirroredTrajectory thirdConePickupToScorePath;
        public final MirroredTrajectory thirdPiecePickupPath;
        public final MirroredTrajectory thirdPieceToAprilTagPath;
        public final MirroredTrajectory thirdPoleToSecondPiecePickup;
        public final MirroredTrajectory secondPoleToThirdPiecePickup;
        public final MirroredTrajectory thirdPiecePickupToFirstPole;
        public final MirroredTrajectory thirdPiecePickupToThirdPole;
        public final MirroredTrajectory frontBridgePath;
        public final MirroredTrajectory thirdPieceToBridgePath;

        // Teleop Paths
        public final MirroredTrajectory communitySweepPath;
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();

            // Auto Paths
            secondPiecePickupPath = new MirroredTrajectory(getSecondPiecePickupPath());
            secondPieceToEdgeColumn = new MirroredTrajectory(getSecondPieceToEdgeColumn());
            secondPieceToCubeScore = new MirroredTrajectory(getSecondPieceToCubeScore());
            edgeColumnToThirdPiece = new MirroredTrajectory(getEdgeColumnToThirdPiece());
            cubeScoreToThirdPiece = new MirroredTrajectory(getCubeScoreToThirdPiece());
            thirdPieceToSecondConeColumn = new MirroredTrajectory(getThirdPieceToSecondConeColumn());
            thirdPieceToCubeScore = new MirroredTrajectory(getThirdPieceToCubeScore());
            secondConeHighScorePath = new MirroredTrajectory(getSecondConeHighScorePath());
            secondPieceToAprilTag = new MirroredTrajectory(getSecondPieceToAprilTagPath());
            secondConeScoreToThirdConePickup = new MirroredTrajectory(getSecondScoreToThirdConePickupPath());
            thirdConePickupToScorePath = new MirroredTrajectory(getThirdConePickupToScorePath());
            thirdPiecePickupPath = new MirroredTrajectory(getThirdPiecePickupPath());
            thirdPieceToAprilTagPath = new MirroredTrajectory(getThirdPieceToAprilTagPath());
            thirdPoleToSecondPiecePickup = new MirroredTrajectory(getThirdPoleToFirstPiecePickup());
            secondPoleToThirdPiecePickup = new MirroredTrajectory(getSecondPoleToThirdPiecePickup());
            thirdPiecePickupToFirstPole = new MirroredTrajectory(getThirdPieceToFirstPole());
            thirdPiecePickupToThirdPole = new MirroredTrajectory(getThirdPiecePickupToThirdPole());
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
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(10)));
            waypoints.add(secondConePickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 120.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPieceToEdgeColumn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondConePickupPose.getTranslation(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(-170)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPieceToCubeScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondConePickupPose.getTranslation(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74.3125, 42.19), Rotation2d.fromDegrees(170)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEdgeColumnToThirdPiece(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(20)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 24.28), Rotation2d.fromDegrees(0)));
            waypoints.add(thirdConePickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCubeScoreToThirdPiece(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(74.3125, 42.19), Rotation2d.fromDegrees(-10)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 24.28), Rotation2d.fromDegrees(0)));
            waypoints.add(thirdConePickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToSecondConeColumn(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 34.28), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(71.0, 64.0), Rotation2d.fromDegrees(140)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToCubeScore(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 30.28), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74.3125, 42.19), Rotation2d.fromDegrees(160)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondConeHighScorePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondConePickupPose.getTranslation(), Rotation2d.fromDegrees(-180)));
            waypoints.add(new Pose2d(new Translation2d(129.0, 35.5), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(71.0, 64.0), Rotation2d.fromDegrees(140)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPieceToAprilTagPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondConePickupPose.getTranslation(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74, 44), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondScoreToThirdConePickupPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(71.0, 64.0), Rotation2d.fromDegrees(-45)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 30.28), Rotation2d.fromDegrees(0)));
            waypoints.add(thirdConePickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdConePickupToScorePath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 30.28), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(180)));
            
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

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPoleToFirstPiecePickup(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.kThirdPoleStartingPose.getTranslation(), Rotation2d.fromDegrees(-45)));
            waypoints.add(new Pose2d(new Translation2d(116.57142857142857, 33.71428571428572), Rotation2d.fromDegrees(-10)));
            waypoints.add(new Pose2d(new Translation2d(179.42857142857142, 33.71428571428572), Rotation2d.fromDegrees(0)));
            waypoints.add(secondConePickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPoleToThirdPiecePickup(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(71.0, 64.0), Rotation2d.fromDegrees(-40)));
            waypoints.add(new Pose2d(new Translation2d(120.57142857142857, 28), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(210.28571428571428, 32.571428571428555), Rotation2d.fromDegrees(27.5)));
            waypoints.add(new Pose2d(new Translation2d(248, 55.428571428571445), Rotation2d.fromDegrees(45)));
            waypoints.add(thirdConePickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPiecePickupToThirdPole(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(227.42857142857142, 32.0), Rotation2d.fromDegrees(-160)));
            waypoints.add(new Pose2d(new Translation2d(165.71428571428572, 26), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(112, 28), Rotation2d.fromDegrees(170)));
            waypoints.add(new Pose2d(new Translation2d(73.14285714285714, 87.42857142857142), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToFirstPole(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(234.28571428571428, 38.85714285714283), Rotation2d.fromDegrees(-170)));
            waypoints.add(new Pose2d(new Translation2d(147.42857142857142, 35.857142857142833), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(180)));
            
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
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(135)));
            waypoints.add(new Pose2d(new Translation2d(158.5, 108.19), Rotation2d.fromDegrees(180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 48.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
    }
}
