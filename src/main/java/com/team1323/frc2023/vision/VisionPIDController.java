package com.team1323.frc2023.vision;

import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.Stopwatch;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class VisionPIDController {
    private static final double kOnTargetTime = 1.0;
    private static final double kDistanceToTargetTolerance = 1.0;
    private static final double kLateralThresholdForRetroSwitch = 2.0;
    private static final double kDistanceThresholdForRetroSwitch = 12.0;

	private final SynchronousPIDF lateralPID = new SynchronousPIDF(0.05, 0.0, 0.0);
	private final SynchronousPIDF forwardPID = new SynchronousPIDF(0.02, 0.0, 0.0);
    private final TwoPointRamp decelerationRamp = new TwoPointRamp(
        new Translation2d(1.0, 0.1),
        new Translation2d(60.0, 0.4),
        1.0,
        true
    );
    private final GoalTrack retroTargetTrack = GoalTrack.makeNewTrack(0.0, Translation2d.identity(), 0);

    private Translation2d targetPosition = Translation2d.identity();
    private Rotation2d targetHeading = Rotation2d.identity();
    private Rotation2d approachAngle = Rotation2d.identity();
    private Stopwatch onTargetStopwatch = new Stopwatch();
    private boolean targetReached = false;
    private boolean useRetroTarget = false;

    public void start(Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useRetroTarget) {
        LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        lateralPID.setSetpoint(0.0);
        forwardPID.setSetpoint(0.0);
        targetPosition = desiredFieldPose.getTranslation();
        targetHeading = desiredFieldPose.getRotation();
        this.approachAngle = approachAngle;
        onTargetStopwatch.reset();
        targetReached = false;
        this.useRetroTarget = useRetroTarget;
    }

    public void addRetroObservation(Translation2d retroPosition, double timestamp) {
        retroTargetTrack.forceUpdate(timestamp, retroPosition);
    }

    private void updateTargetHeading(Pose2d robotPose, Translation2d error) {
        if (!useRetroTarget) {
            return;
        }

        boolean isCloseEnoughForRetro = error.norm() < kDistanceThresholdForRetroSwitch &&
                Math.abs(error.y()) < kLateralThresholdForRetroSwitch;
        if (!isCloseEnoughForRetro) {
            return;
        }

        LimelightProcessor.getInstance().setPipeline(Pipeline.RETRO);
        retroTargetTrack.emptyUpdate();
        Translation2d retroPosition = retroTargetTrack.getSmoothedPosition();
        if (retroPosition == null) {
            return;
        }

        Translation2d robotToRetro = new Translation2d(robotPose.getTranslation(), retroPosition);
        targetHeading = robotToRetro.direction();
    }

    /**
     * @return A Pose2d whose translation represents a drive vector that should be applied
     * to the swerve, and whose rotation represents a target heading for the swerve's
     * heading controller.
     */
    public Pose2d update(Pose2d robotPose, double dt) {
        if (targetReached) {
            return Pose2d.fromRotation(targetHeading);
        }

		Translation2d adjustedTarget = targetPosition.rotateBy(approachAngle.inverse());
		Translation2d adjustedPose = robotPose.getTranslation().rotateBy(approachAngle.inverse());
		Translation2d error = adjustedTarget.translateBy(adjustedPose.inverse());
		Translation2d driveVector = new Translation2d(-forwardPID.calculate(error.x(), dt), -lateralPID.calculate(error.y(), dt));
		double adjustedMagnitude = Math.min(driveVector.norm(), decelerationRamp.calculate(error.norm()));
		driveVector = Translation2d.fromPolar(driveVector.direction(), adjustedMagnitude);
		driveVector = driveVector.rotateBy(approachAngle);

        updateTargetHeading(robotPose, error);

		if (error.norm() <= kDistanceToTargetTolerance) {
            onTargetStopwatch.startIfNotRunning();
            if (onTargetStopwatch.getTime() >= kOnTargetTime) {
                onTargetStopwatch.reset();
                LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
                targetReached = true;

                return Pose2d.fromRotation(targetHeading);
            }
		}

		return new Pose2d(driveVector, targetHeading);
    }

    public boolean isDone() {
        return targetReached;
    }
}
