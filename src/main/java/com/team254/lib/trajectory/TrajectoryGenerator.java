package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.VelocityLimitRegionConstraint;

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
        public final Trajectory<TimedState<Pose2dWithCurvature>> firstBallBackup;
        public final Trajectory<TimedState<Pose2dWithCurvature>> firstBallToHumanPlayer;
        public final Trajectory<TimedState<Pose2dWithCurvature>> firstBallToSecondBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondBallToHumanPlayer;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondBallToOpponentBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> humanPlayerToSecondBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> terminalToBackupPoint;
        public final Trajectory<TimedState<Pose2dWithCurvature>> backupPointToSecondBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> backupPointToPostTerminalShotPose;
        public final Trajectory<TimedState<Pose2dWithCurvature>> firstBallToOpponentBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> opponentBallToHumanPlayer;
        public final Trajectory<TimedState<Pose2dWithCurvature>> humanPlayerToSecondOpponentBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondOpponentBallToThirdBall;

        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdBallBackup;
        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdBallToSecondOpponentBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondOpponentBallToThirdOpponentBall;

        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdBallToThirdOpponentBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> opponentBallToEjectLocation;
        public final Trajectory<TimedState<Pose2dWithCurvature>> ejectLocationToWallRideStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallRideStartToWallRideEnd;
        public final Trajectory<TimedState<Pose2dWithCurvature>> wallRideEndToMidline;

        public final Trajectory<TimedState<Pose2dWithCurvature>> opponentBallToBackEjectLocation;
        public final Trajectory<TimedState<Pose2dWithCurvature>> backSideEjectToSecondOpponentBall;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondOpponentToBackSideEject;
        public final Trajectory<TimedState<Pose2dWithCurvature>> backSideToMidline;

        public final Trajectory<TimedState<Pose2dWithCurvature>> thirdOpponentBallToCloseEject;
        public final Trajectory<TimedState<Pose2dWithCurvature>> closeEjectToMidline;

        public final Trajectory<TimedState<Pose2dWithCurvature>> farLeftTaxiBackup;
    
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();

            // Auto Paths
            firstBallBackup = getFirstBallBackup();
            firstBallToHumanPlayer = getFirstBallToHumanPlayer();
            firstBallToSecondBall = getFirstBallToSecondBall();
            secondBallToHumanPlayer = getSecondBallToHumanPlayer();
            humanPlayerToSecondBall = getHumanPlayerToSecondBall();
            terminalToBackupPoint = getTerminalBackup();
            backupPointToSecondBall = getTerminalBackupPointToSecondBall();
            backupPointToPostTerminalShotPose = getTerminalBackupPointToPostTerminalShotPose();
            secondBallToOpponentBall = getSecondBallToOpponentBall();
            firstBallToOpponentBall = getFirstBallToOpponentBall();
            opponentBallToHumanPlayer = getOpponentBallToHumanPlayer();
            humanPlayerToSecondOpponentBall = getHumanPlayerToSecondOpponentBall();
            secondOpponentBallToThirdBall = getSecondOpponentBallToThirdBall();

            thirdBallBackup = getBackupEject();
            thirdBallToSecondOpponentBall = getThirdBallToSecondOpponentBall();
            secondOpponentBallToThirdOpponentBall = getSecondOpponentBallToThirdOpponentBall();

            thirdBallToThirdOpponentBall = getThirdBallToThirdOpponentBall();
            opponentBallToEjectLocation = getThirdOpponentBallToEjectPosition();
            ejectLocationToWallRideStart = getEjectPositionToWallRideStart();
            wallRideStartToWallRideEnd = getWallRideStartToWallRideEnd();
            wallRideEndToMidline = getWallRideEndToMidline();

            opponentBallToBackEjectLocation = getThirdOpponentBallToBackSideEject();
            backSideEjectToSecondOpponentBall = getBackSideToSecondOpponentBall();
            secondOpponentToBackSideEject = getSecondOpponentBallToBackSide();
            backSideToMidline = getBackSideToMidline();

            thirdOpponentBallToCloseEject = getThirdOpponentBallToCloseEject();
            closeEjectToMidline = getCloseEjectToMideline();

            farLeftTaxiBackup = getFarLeftTaxiBackup();
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(60.0, 0.0)), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFirstBallBackup(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(autoStartingPose);
            waypoints.add(firstBallPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFirstBallToSecondBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(firstBallPickupPose.getTranslation(), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(secondBallPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondBallToHumanPlayer(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(secondBallPickupPose);
            waypoints.add(humanPlayerPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 12.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getTerminalBackup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(humanPlayerPickupPose.getTranslation(), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(humanPlayerBackupPose.getTranslation(), Rotation2d.fromDegrees(-45)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);

        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getTerminalBackupPointToSecondBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(humanPlayerBackupPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(secondBallPickupPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 36.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getHumanPlayerToSecondBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(humanPlayerPickupPose.getTranslation(), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(secondBallPickupPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getTerminalBackupPointToPostTerminalShotPose() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(humanPlayerBackupPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(postTerminalShotPose);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 36.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondBallToOpponentBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondBallPickupPose.getTranslation(), Rotation2d.fromDegrees(90)));
            waypoints.add(firstOpponentBallPickupPose);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFirstBallToHumanPlayer(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(firstBallPickupPose.getTranslation(), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(secondBallPickupPose);
            waypoints.add(humanPlayerPickupPose);

            List<TimingConstraint<Pose2dWithCurvature>> constraints = Arrays.asList(new VelocityLimitRegionConstraint<>(new Translation2d(175, 65), new Translation2d(240, 115), 60.0));
            
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getFirstBallToOpponentBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(firstBallPickupPose.getTranslation(), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(firstOpponentBallPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel / 2.0, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getOpponentBallToHumanPlayer(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(firstOpponentBallPickupPose.getTranslation(), Rotation2d.fromDegrees(-145.0)));
            waypoints.add(secondBallPickupPose);
            waypoints.add(humanPlayerPickupPose);

            List<TimingConstraint<Pose2dWithCurvature>> constraints = Arrays.asList(new VelocityLimitRegionConstraint<>(new Translation2d(175, 65), new Translation2d(240, 115), 60.0));
            
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHumanPlayerToSecondOpponentBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(humanPlayerPickupPose.getTranslation(), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(secondOpponentBallPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondOpponentBallToThirdBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(secondOpponentBallPickupPose);
            waypoints.add(thirdBallPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getBackupEject() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(autoEjectStartingPose);
            waypoints.add(new Pose2d(thirdBallPickupPose.getTranslation(), Rotation2d.fromDegrees(-135)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 12.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdBallToSecondOpponentBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdBallPickupPose.getTranslation(), Rotation2d.fromDegrees(90)));
            waypoints.add(new Pose2d(secondOpponentBallPickupPose.getTranslation(), Rotation2d.fromDegrees(90)));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondOpponentBallToThirdOpponentBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(secondOpponentBallPickupPose);
            waypoints.add(thirdOpponentBallPickupPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdBallToThirdOpponentBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdBallPickupPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(thirdOpponentBallPickupPose.getTranslation().translateBy(new Translation2d(0, -17.0)), Rotation2d.fromDegrees(-90.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdOpponentBallToEjectPosition() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdOpponentBallPickupPose.getTranslation().translateBy(new Translation2d(0, -17.0)), Rotation2d.fromDegrees(45.0)));
            waypoints.add(opponentBallEjectPosition);
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEjectPositionToWallRideStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(opponentBallEjectPosition.getTranslation(), Rotation2d.fromDegrees(-90)));
            waypoints.add(wallRideStartPosition);
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getWallRideStartToWallRideEnd() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(wallRideStartPosition.getTranslation(), Rotation2d.fromDegrees(-180)));
            waypoints.add(wallRideEndPosition);
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getWallRideEndToMidline() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(wallRideEndPosition.getTranslation(), Rotation2d.fromDegrees(0)));
            waypoints.add(leftMidlinePosition);
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdOpponentBallToBackSideEject() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdOpponentBallPickupPose.getTranslation().translateBy(new Translation2d(0, -17.0)), Rotation2d.fromDegrees(90.0)));
            waypoints.add(backSideEjectPosition);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBackSideToSecondOpponentBall() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(backSideEjectPosition.getTranslation(), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(secondOpponentBallPickupPose.getTranslation().translateBy(new Translation2d(17.0, 4.0)), Rotation2d.fromDegrees(135.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondOpponentBallToBackSide() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(secondOpponentBallPickupPose.getTranslation().translateBy(new Translation2d(17.0, 4.0)), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(backSideEjectPosition.getTranslation().translateBy(new Translation2d(-7.0, 22.0)), Rotation2d.fromDegrees(0.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBackSideToMidline() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(backSideEjectPosition.getTranslation().translateBy(new Translation2d(-7.0, 22.0)), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(leftMidlinePosition.getTranslation(), Rotation2d.fromDegrees(-45.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        

        private Trajectory<TimedState<Pose2dWithCurvature>> getThirdOpponentBallToCloseEject(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdOpponentBallPickupPose.getTranslation(), Rotation2d.fromDegrees(90)));
            waypoints.add(new Pose2d(new Translation2d(167.42857142857142, -16.285714285714278), Rotation2d.fromDegrees(170)));
            waypoints.add(closeWallEjectPosition);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCloseEjectToMideline() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(closeWallEjectPosition.getTranslation(), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(170, -5.0), Rotation2d.fromDegrees(-10)));
            waypoints.add(leftMidlinePosition);
            return generateTrajectory(false, waypoints, Arrays.asList(), 72.0, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }



        private Trajectory<TimedState<Pose2dWithCurvature>> getFarLeftTaxiBackup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.autoLeftFarStartingPose);
            waypoints.add(Constants.autoLeftFarStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(36.0, 0.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
    }
}
