package com.team1323.frc2023.vision;

import com.team1323.frc2023.DriveMotionPlanner;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.subsystems.LEDs;
import com.team1323.frc2023.subsystems.LEDs.LEDColors;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.Stopwatch;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionPIDController {
    private static final double kLateralThresholdForRetroSwitch = 2.0;
    private static final double kDistanceThresholdForRetroSwitch = 12.0;

    private final double kOnTargetTime;
    private final double kDistanceToTargetTolerance;
	private final SynchronousPIDF lateralPID;
	private final SynchronousPIDF forwardPID;
    private final TwoPointRamp decelerationRamp;
    private final GoalTrack retroTargetTrack = GoalTrack.makeNewTrack(0.0, Translation2d.identity(), 0);
    private final DriveMotionPlanner motionPlanner = new DriveMotionPlanner();

    public enum TrackingPhase {
        TRAJECTORY, VISION_PID, RETRO
    }
    private TrackingPhase currentPhase = TrackingPhase.VISION_PID;

    private Translation2d targetPosition = Translation2d.identity();
    private Rotation2d targetHeading = Rotation2d.identity();
    private Rotation2d approachAngle = Rotation2d.identity();
    private double distanceToTargetPosition = Double.POSITIVE_INFINITY;
    private Stopwatch runtimeStopwatch = new Stopwatch();
    private Stopwatch onTargetStopwatch = new Stopwatch();
    private boolean targetReached = false;
    private boolean useRetroTarget = false;

    private VisionPIDController(VisionPIDBuilder builder) {
        kOnTargetTime = builder.onTargetTime;
        kDistanceToTargetTolerance = builder.distanceToTargetTolerance;
        lateralPID = builder.lateralPID;
        forwardPID = builder.forwardPID;
        decelerationRamp = builder.decelerationRamp;
    }

    public void start(Pose2d currentPose, Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useTrajectory, boolean useRetroTarget) {
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        lateralPID.setSetpoint(0.0);
        forwardPID.setSetpoint(0.0);
        targetPosition = desiredFieldPose.getTranslation();
        targetHeading = desiredFieldPose.getRotation();
        this.approachAngle = approachAngle;
        distanceToTargetPosition = Double.POSITIVE_INFINITY;
        this.useRetroTarget = useRetroTarget;
        runtimeStopwatch.start();
        onTargetStopwatch.reset();
        targetReached = false;

        Trajectory<TimedState<Pose2dWithCurvature>> communitySweepPath = getCommunitySweepPath(currentPose, targetPosition);
        if (communitySweepPath != null && useTrajectory) {
            LEDs.getInstance().configLEDs(LEDColors.BLUE);
            motionPlanner.reset();
            motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(communitySweepPath)));
            currentPhase = TrackingPhase.TRAJECTORY;
        } else {
            LEDs.getInstance().configLEDs(LEDColors.GREEN);
            currentPhase = TrackingPhase.VISION_PID;
        }
    }

    public Translation2d getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(Translation2d targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void resetDistanceToTargetPosition() {
        distanceToTargetPosition = Double.POSITIVE_INFINITY;
        targetReached = false;
    }

    private boolean canSwitchToVisionPID(Pose2d robotPose, Translation2d scoringPosition) {
        final double kYTolerance = 36.0;
        final double kBlueCloseRampX = 40.45 + 13.8 + 60.69;
        final double kRedCloseRampX = 610.77 - 13.8 - 60.69;
        boolean isPastRamp = AllianceChooser.getAlliance() == Alliance.Blue ?
                robotPose.getTranslation().x() < kBlueCloseRampX :
                robotPose.getTranslation().x() > kRedCloseRampX;
        boolean isWithinYTolerance = Math.abs(scoringPosition.y() - robotPose.getTranslation().y()) <= kYTolerance;
        
        return isPastRamp && isWithinYTolerance;
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getCommunitySweepPath(Pose2d robotPose, Translation2d scoringPosition) {
        if (canSwitchToVisionPID(robotPose, scoringPosition)) {
            return null;
        }

        MirroredTrajectory communitySweepPath = TrajectoryGenerator.getInstance().getTrajectorySet().communitySweepPath;
        if (AllianceChooser.getAlliance() == Alliance.Blue) {
            return robotPose.getTranslation().y() < scoringPosition.y() ?
                    communitySweepPath.get(Quadrant.BOTTOM_LEFT) :
                    communitySweepPath.get(Quadrant.TOP_LEFT);
        } else {
            return robotPose.getTranslation().y() < scoringPosition.y() ?
                    communitySweepPath.get(Quadrant.BOTTOM_RIGHT) :
                    communitySweepPath.get(Quadrant.TOP_RIGHT);
        }
    }

    public void addRetroObservation(Translation2d retroPosition, double timestamp) {
        retroTargetTrack.forceUpdate(timestamp, retroPosition);
    }

    private void updateTargetHeading(Pose2d robotPose, Translation2d error) {
        retroTargetTrack.emptyUpdate();
        Translation2d retroPosition = retroTargetTrack.getSmoothedPosition();
        if (retroPosition == null) {
            return;
        }

        Translation2d robotToRetro = new Translation2d(robotPose.getTranslation(), retroPosition);
        targetHeading = robotToRetro.direction();
    }

    private Translation2d getPIDError(Pose2d robotPose) {
		Translation2d adjustedTarget = targetPosition.rotateBy(approachAngle.inverse());
		Translation2d adjustedPose = robotPose.getTranslation().rotateBy(approachAngle.inverse());
		Translation2d error = adjustedTarget.translateBy(adjustedPose.inverse());

        return error;
    }

    private Translation2d getPIDDriveVector(Translation2d error, double dt) {
		Translation2d driveVector = new Translation2d(-forwardPID.calculate(error.x(), dt), -lateralPID.calculate(error.y(), dt));
		double adjustedMagnitude = Math.min(driveVector.norm(), decelerationRamp.calculate(error.norm()));
		driveVector = Translation2d.fromPolar(driveVector.direction(), adjustedMagnitude);
		driveVector = driveVector.rotateBy(approachAngle);

        return driveVector;
    }

    private Pose2d getTrajectoryOutput(Pose2d robotPose, double timestamp) {
        Translation2d driveVector = motionPlanner.update(timestamp, robotPose);

        if (canSwitchToVisionPID(robotPose, targetPosition)) {
            LEDs.getInstance().configLEDs(LEDColors.GREEN);
            currentPhase = TrackingPhase.VISION_PID;
        }

        return new Pose2d(driveVector, targetHeading);
    }

    private Pose2d getPIDOutput(Pose2d robotPose, double dt) {
        Translation2d error = getPIDError(robotPose);
        Translation2d driveVector = getPIDDriveVector(error, dt);

        boolean isCloseEnoughForRetro = error.norm() < kDistanceThresholdForRetroSwitch &&
                Math.abs(error.y()) < kLateralThresholdForRetroSwitch;
        if (useRetroTarget && isCloseEnoughForRetro) {
            LimelightProcessor.getInstance().setPipeline(Pipeline.RETRO);
            currentPhase = TrackingPhase.RETRO;
        }

        return new Pose2d(driveVector, targetHeading);
    }

    private Pose2d getRetroOutput(Pose2d robotPose, double dt) {
        Translation2d error = getPIDError(robotPose);
        Translation2d driveVector = getPIDDriveVector(error, dt);
        updateTargetHeading(robotPose, error);

        return new Pose2d(driveVector, targetHeading);
    }

    private void finish() {
        onTargetStopwatch.reset();
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        targetReached = true;
    }

    /**
     * @return A Pose2d whose translation represents a drive vector that should be applied
     * to the swerve, and whose rotation represents a target heading for the swerve's
     * heading controller.
     */
    public Pose2d update(Pose2d robotPose, double timestamp, double dt) {
        if (targetReached) {
            return Pose2d.fromRotation(targetHeading);
        }

        if (runtimeStopwatch.getTime() > 1.0 && Swerve.getInstance().areModulesStuck()) {
            finish();
            DriverStation.reportError("Vision PID ended due to stuck modules", false);
            return Pose2d.fromRotation(targetHeading);
        }

        Translation2d error = getPIDError(robotPose);
        distanceToTargetPosition = error.norm();
		if (distanceToTargetPosition <= kDistanceToTargetTolerance) {
            onTargetStopwatch.startIfNotRunning();
            if (onTargetStopwatch.getTime() >= kOnTargetTime) {
                finish();
                return Pose2d.fromRotation(targetHeading);
            }
		}

        Pose2d output = Pose2d.identity();
        switch (currentPhase) {
            case TRAJECTORY:
                output = getTrajectoryOutput(robotPose, timestamp);
                break;
            case VISION_PID:
                output = getPIDOutput(robotPose, dt);
                break;
            case RETRO:
                output = getRetroOutput(robotPose, dt);
                break;
        }

        return output;
    }

    public double getDistanceToTargetPosition() {
        return distanceToTargetPosition;
    }

    public boolean isDone() {
        return targetReached;
    }

    public static class VisionPIDBuilder {
        private SynchronousPIDF lateralPID = new SynchronousPIDF(0.05, 0.0, 0.0);
        private SynchronousPIDF forwardPID = new SynchronousPIDF(0.03, 0.0, 0.0);
        private TwoPointRamp decelerationRamp = new TwoPointRamp(
            new Translation2d(1.0, 0.1),
            new Translation2d(60.0, 0.4),
            1.0,
            true
        );
        private double distanceToTargetTolerance = 1.0;
        private double onTargetTime = 0.5;

        public VisionPIDBuilder withLateralPID(SynchronousPIDF lateralPID) {
            this.lateralPID = lateralPID;
            return this;
        }

        public VisionPIDBuilder withForwardPID(SynchronousPIDF forwardPID) {
            this.forwardPID = forwardPID;
            return this;
        }

        public VisionPIDBuilder withDecelerationRamp(TwoPointRamp decelerationRamp) {
            this.decelerationRamp = decelerationRamp;
            return this;
        }

        public VisionPIDBuilder withTolerance(double distanceToTargetTolerance) {
            this.distanceToTargetTolerance = distanceToTargetTolerance;
            return this;
        }

        public VisionPIDBuilder withOnTargetTime(double onTargetTime) {
            this.onTargetTime = onTargetTime;
            return this;
        }

        public VisionPIDController build() {
            return new VisionPIDController(this);
        }
    }
}
