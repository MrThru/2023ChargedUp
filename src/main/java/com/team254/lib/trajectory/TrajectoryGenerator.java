package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.DriveMotionPlanner;
import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.WaypointList;
import com.team1323.frc2023.field.WaypointList.Pose2dWithQuadrantOffsets;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.VelocityLimitRegionConstraint;

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

    private Pose2d secondConePickupPose = new Pose2d(new Translation2d(269.5, 26.5), Rotation2d.fromDegrees(0));
    private Pose2d thirdConePickupPose = new Pose2d(new Translation2d(278.0, 80.5), Rotation2d.fromDegrees(45))
                .transformBy(Pose2d.fromTranslation(new Translation2d(-12, 0)));
    
    public class TrajectorySet {
        public class MirroredTrajectory {
            private final Map<Quadrant, Trajectory<TimedState<Pose2dWithCurvature>>> trajectoryMap = new HashMap<>();

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> bottomLeft) {
                for (Quadrant quadrant : Quadrant.values()) {
                    trajectoryMap.put(quadrant, AutoZones.mirror(bottomLeft, quadrant));
                }
            }

            public MirroredTrajectory(
                boolean reversed,
                final WaypointList waypoints,
                final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                double max_vel,  // inches/s
                double max_accel,  // inches/s^2
                double max_decel,
                double max_voltage,
                double default_vel,
                int slowdown_chunks
            ) {
                Function<List<Pose2d>, Trajectory<TimedState<Pose2dWithCurvature>>> trajectorySupplier = poses ->
                        generateTrajectory(reversed, poses, constraints, max_vel, max_accel, max_decel,
                                max_voltage, default_vel, slowdown_chunks);
                generateMirroredTrajectories(waypoints, trajectorySupplier);
            }

            public MirroredTrajectory(
                boolean reversed,
                final WaypointList waypoints,
                final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                double start_vel,  // inches/s
                double end_vel,  // inches/s
                double max_vel,  // inches/s
                double max_accel,  // inches/s^2
                double max_decel,
                double max_voltage,
                double default_vel,
                int slowdown_chunks
            ) {
                Function<List<Pose2d>, Trajectory<TimedState<Pose2dWithCurvature>>> trajectorySupplier = poses ->
                        generateTrajectory(reversed, poses, constraints, start_vel, end_vel, max_vel,
                                max_accel, max_decel, max_voltage, default_vel, slowdown_chunks);
                generateMirroredTrajectories(waypoints, trajectorySupplier);
            }

            private void generateMirroredTrajectories(final WaypointList waypoints, Function<List<Pose2d>, Trajectory<TimedState<Pose2dWithCurvature>>> trajectorySupplier) {
                if (waypoints.hasOffsetsForQuadrant(Quadrant.BOTTOM_LEFT)) {
                    for (Quadrant quadrant : Quadrant.values()) {
                        Trajectory<TimedState<Pose2dWithCurvature>> trajectory = trajectorySupplier.apply(waypoints.getWaypointsForQuadrant(quadrant));
                        trajectoryMap.put(quadrant, trajectory);
                    }
                } else {
                    Trajectory<TimedState<Pose2dWithCurvature>> bottomLeftTrajectory = trajectorySupplier.apply(waypoints.getWaypointsForQuadrant(Quadrant.BOTTOM_LEFT));
                    trajectoryMap.put(Quadrant.BOTTOM_LEFT, bottomLeftTrajectory);

                    for (Quadrant quadrant : Quadrant.values()) {
                        if (quadrant == Quadrant.BOTTOM_LEFT) {
                            continue;
                        }

                        if (waypoints.hasOffsetsForQuadrant(quadrant)) {
                            Trajectory<TimedState<Pose2dWithCurvature>> trajectory = trajectorySupplier.apply(waypoints.getWaypointsForQuadrant(quadrant));
                            trajectoryMap.put(quadrant, trajectory);
                        } else {
                            trajectoryMap.put(quadrant, AutoZones.mirror(bottomLeftTrajectory, quadrant));
                        }
                    }
                }
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(Quadrant quadrant) {
                return trajectoryMap.get(quadrant);
            }
        }
        
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        
        // Auto Paths
        public final MirroredTrajectory secondPiecePickupPath;
        public final MirroredTrajectory slowSecondPiecePickupPath;
        public final MirroredTrajectory secondPieceToCubeScore;
        public final MirroredTrajectory slowSecondPieceToCubeScore;
        public final MirroredTrajectory cubeScoreToThirdPiece;
        public final MirroredTrajectory slowCubeScoreToThirdPiece;
        public final MirroredTrajectory thirdPieceToSecondConeColumn;
        public final MirroredTrajectory thirdPieceToEdgeColumn;
        public final MirroredTrajectory thirdPieceToBridgePath;

        // Teleop Paths
        public final MirroredTrajectory communitySweepPath;
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();

            // Auto Paths
            secondPiecePickupPath = getSecondPiecePickupPath();
            slowSecondPiecePickupPath = getSlowSecondPiecePickupPath();
            secondPieceToCubeScore = new MirroredTrajectory(getSecondPieceToCubeScore());
            slowSecondPieceToCubeScore = new MirroredTrajectory(getSlowSecondPieceToCubeScore());
            cubeScoreToThirdPiece = getCubeScoreToThirdPiece();
            slowCubeScoreToThirdPiece = getSlowCubeScoreToThirdPiece();
            thirdPieceToSecondConeColumn = new MirroredTrajectory(getThirdPieceToSecondConeColumn());
            thirdPieceToEdgeColumn = new MirroredTrajectory(getThirdPieceToEdgeColumn());
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

        private MirroredTrajectory getSecondPiecePickupPath() {
            WaypointList waypoints = new WaypointList();
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(10)));
            waypoints.add(new Pose2dWithQuadrantOffsets(secondConePickupPose)
                    .withOffset(Quadrant.TOP_RIGHT, Pose2d.fromTranslation(new Translation2d(0.0, 3.0))));
            
            return new MirroredTrajectory(false, waypoints, Arrays.asList(), 24.0, 0.0, 144.0, 240.0, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private MirroredTrajectory getSlowSecondPiecePickupPath() {
            WaypointList waypoints = new WaypointList();
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(10)));
            waypoints.add(new Pose2dWithQuadrantOffsets(secondConePickupPose)
                    .withOffset(Quadrant.TOP_RIGHT, Pose2d.fromTranslation(new Translation2d(0.0, 3.0))));
            
            return new MirroredTrajectory(false, waypoints, Arrays.asList(), 24.0, 0.0, 144.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondPieceToCubeScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondConePickupPose.getTranslation(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(125, 30), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74.3125 + 12.0, 42.19), Rotation2d.fromDegrees(160)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 120, 240, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlowSecondPieceToCubeScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondConePickupPose.getTranslation(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(74.3125 + 12.0, 42.19), Rotation2d.fromDegrees(170)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 120, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private MirroredTrajectory getCubeScoreToThirdPiece() {
            WaypointList waypoints = new WaypointList();
            waypoints.add(new Pose2d(new Translation2d(74.3125, 42.19), Rotation2d.fromDegrees(-10)));
            waypoints.add(new Pose2dWithQuadrantOffsets(new Pose2d(new Translation2d(152.57, 24.28), Rotation2d.fromDegrees(0)))
                    .withOffset(Quadrant.BOTTOM_LEFT, Pose2d.fromTranslation(new Translation2d(0, 6)))
                    .withOffset(Quadrant.BOTTOM_RIGHT, Pose2d.fromTranslation(new Translation2d(0, 6))));
            waypoints.add(thirdConePickupPose);
            
            return new MirroredTrajectory(false, waypoints, Arrays.asList(), 24.0, 0.0, kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private MirroredTrajectory getSlowCubeScoreToThirdPiece() {
            WaypointList waypoints = new WaypointList();
            waypoints.add(new Pose2d(new Translation2d(74.3125, 42.19), Rotation2d.fromDegrees(-10)));
            waypoints.add(new Pose2dWithQuadrantOffsets(new Pose2d(new Translation2d(152.57, 24.28), Rotation2d.fromDegrees(0)))
                    .withOffset(Quadrant.BOTTOM_LEFT, Pose2d.fromTranslation(new Translation2d(0, 6)))
                    .withOffset(Quadrant.BOTTOM_RIGHT, Pose2d.fromTranslation(new Translation2d(0, 6))));
            waypoints.add(thirdConePickupPose);
            
            return new MirroredTrajectory(false, waypoints, Arrays.asList(), 24.0, 0.0, kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToSecondConeColumn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 34.28), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(71.0, 64.0), Rotation2d.fromDegrees(130)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToEdgeColumn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            waypoints.add(new Pose2d(new Translation2d(152.57, 34.28), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Constants.kAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(-170)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdPieceToBridgePath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdConePickupPose.getTranslation(), Rotation2d.fromDegrees(135)));
            waypoints.add(new Pose2d(new Translation2d(146.5 + 18.0, 108.19), Rotation2d.fromDegrees(180)));

            List<TimingConstraint<Pose2dWithCurvature>> constraints = Arrays.asList(
                new VelocityLimitRegionConstraint<>(
                    new Translation2d(177, 86),
                    new Translation2d(210, 131),
                    72.0
                )
            );
        
            return generateTrajectory(false, waypoints, constraints, 48.0, 36.0, 120.0, kMaxAccel, kMaxDecel, kMaxVoltage, 48.0, 1);
        }
    }
}
