package com.team1323.frc2023.vision;

import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.Stopwatch;
import com.team1323.lib.util.SynchronousPIDF;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class VisionPIDController {
    private static final double kOnTargetTime = 0.5;
    private static final double kDistanceToTargetTolerance = 1.0;

	private final SynchronousPIDF lateralPID = new SynchronousPIDF(0.05, 0.0, 0.0);
	private final SynchronousPIDF forwardPID = new SynchronousPIDF(0.05, 0.0, 0.0);
    private final TwoPointRamp decelerationRamp = new TwoPointRamp(
        new Translation2d(1.0, 0.05),
        new Translation2d(60.0, 0.3),
        2.0,
        true
    );

    private Translation2d targetPosition = Translation2d.identity();
    private Rotation2d approachAngle = Rotation2d.identity();
    private Stopwatch onTargetStopwatch = new Stopwatch();
    private boolean targetReached = false;

    public void start(Translation2d desiredFieldPosition, Rotation2d approachAngle) {
        lateralPID.setSetpoint(0.0);
        forwardPID.setSetpoint(0.0);
        targetPosition = desiredFieldPosition;
        this.approachAngle = approachAngle;
        onTargetStopwatch.reset();
        targetReached = false;
    }

    public Translation2d update(Pose2d robotPose, double dt) {
        if (targetReached) {
            return Translation2d.identity();
        }

		Translation2d adjustedTarget = targetPosition.rotateBy(approachAngle.inverse());
		Translation2d adjustedPose = robotPose.getTranslation().rotateBy(approachAngle.inverse());
		Translation2d error = adjustedTarget.translateBy(adjustedPose.inverse());
		Translation2d output = new Translation2d(-forwardPID.calculate(error.x(), dt), -lateralPID.calculate(error.y(), dt));
		double adjustedMagnitude = Math.min(output.norm(), decelerationRamp.calculate(error.norm()));
		output = Translation2d.fromPolar(output.direction(), adjustedMagnitude);
		output = output.rotateBy(approachAngle);

		if (error.norm() <= kDistanceToTargetTolerance) {
            onTargetStopwatch.startIfNotRunning();
            if (onTargetStopwatch.getTime() >= kOnTargetTime) {
                onTargetStopwatch.reset();
                targetReached = true;

                return Translation2d.identity();
            }
		}

		System.out.println("Vision PID output vector: " + output.toString() + ", Error Norm: " + error.norm());
		
		return output;
    }

    public boolean isDone() {
        return targetReached;
    }
}
