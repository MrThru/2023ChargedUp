package com.team1323.frc2023.loops;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.loops.LimelightHelpers.LimelightResults;
import com.team1323.frc2023.loops.LimelightHelpers.LimelightTarget_Fiducial;
import com.team1323.frc2023.loops.LimelightHelpers.LimelightTarget_Retro;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.math.Units;
import com.team1323.lib.util.FieldConversions;
import com.team1323.lib.util.Netlink;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightProcessor implements Loop {
	private static LimelightProcessor instance = new LimelightProcessor();
	public static LimelightProcessor getInstance() {
		return instance;
	}

	private static final String kLimelightName = "limelight";
	private static final double kMinFiducialArea = 0.115;

	private static final TwoPointRamp translationalStandardDeviationRamp = new TwoPointRamp(
		new Translation2d(60.0, 0.05),
		new Translation2d(100.0, 1.5),
		1.0,
		false
	);
	private static final TwoPointRamp rotationalStandardDeviationRamp = new TwoPointRamp(
		new Translation2d(60.0, 0.2),
		new Translation2d(100.0, 1.5),
		1.0,
		false
	);

	private double previousHeartbeat = -1.0;
	
	private LimelightProcessor() {}
	
	@Override 
	public void onStart(double timestamp) {}
	
	@Override 
	public void onLoop(double timestamp) {
		double currentHeartbeat = LimelightHelpers.getLimelightNTDouble(kLimelightName, "hb");
		if (currentHeartbeat > previousHeartbeat && !Netlink.getBooleanValue("Limelight Disabled")) {
			LimelightResults results = LimelightHelpers.getLatestResults(kLimelightName);
			handleFiducialTargets(results, timestamp);
			handleRetroTargets(results, timestamp);
			previousHeartbeat = currentHeartbeat;
		}
	}
	
	@Override
	public void onStop(double timestamp) {}

	private double getTotalLatencySeconds(LimelightResults results) {
		return (results.targetingResults.latency_capture + results.targetingResults.latency_pipeline +
				results.targetingResults.latency_jsonParse) / 1000.0;
	}

	private void handleFiducialTargets(LimelightResults results, double timestamp) {
		if (results.targetingResults.targets_Fiducials.length == 0) {
			return;
		}

		Pose2d robotPoseInLimelightCoordinates;
		Pose3d cameraPose;
		List<LimelightTarget_Fiducial> fiducials = Arrays.asList(results.targetingResults.targets_Fiducials);
		if (fiducials.stream().allMatch(fiducial -> fiducial.ta >= kMinFiducialArea)) {
			robotPoseInLimelightCoordinates = LimelightHelpers.getBotPose2d(kLimelightName);
			cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace(kLimelightName);
		} else {
			Comparator<LimelightTarget_Fiducial> areaComparator = (f1, f2) -> Double.compare(f1.ta, f2.ta);
			LimelightTarget_Fiducial largestFiducial = Collections.max(fiducials, areaComparator);
			robotPoseInLimelightCoordinates = largestFiducial.getRobotPose_FieldSpace2D();
			cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace(kLimelightName);
		}

		if (robotPoseInLimelightCoordinates.equals(Pose2d.identity()) || cameraPose.equals(new Pose3d())) {
			return;
		}

		Pose2d estimatedRobotPoseMeters = FieldConversions.convertToField(Constants.kLimelightFieldOrigin, robotPoseInLimelightCoordinates);
		Pose2d estimatedRobotPoseInches = Units.metersToInches(estimatedRobotPoseMeters);

		// Only accept vision updates if they place the robot within our own community or loading zone
		if (!AllianceChooser.getCommunityBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation()) &&
				!AllianceChooser.getLoadingZoneBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation())) {
			return;
		}

		double cameraDistanceInches = Units.metersToInches(cameraPose.getTranslation().getNorm());

		double translationalStdDev = translationalStandardDeviationRamp.calculate(cameraDistanceInches);
		double rotationalStdDev = rotationalStandardDeviationRamp.calculate(cameraDistanceInches);
		Matrix<N3, N1> standardDeviations = VecBuilder.fill(translationalStdDev, translationalStdDev, rotationalStdDev);
		Swerve.getInstance().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results), standardDeviations);

		// For debugging purposes
		if (Swerve.getInstance().getState() == Swerve.ControlState.VISION_PID) {
			SmartDashboard.putNumberArray("Path Pose", new double[]{estimatedRobotPoseInches.getTranslation().x(), estimatedRobotPoseInches.getTranslation().y(), estimatedRobotPoseInches.getRotation().getDegrees()});
		}
	}

	private void handleRetroTargets(LimelightResults results, double timestamp) {
		if (results.targetingResults.targets_Retro.length == 0) {
			return;
		}

		int pipelineIndex = (int) results.targetingResults.pipelineID;
		if (pipelineIndex == Pipeline.RETRO.index) {
			updateRobotPoseWithConePoles(results, timestamp);
		} else if (pipelineIndex == Pipeline.CONE.index) {

		}
	}

	private void updateConePolePosition(LimelightResults results, double timestamp) {
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(timestamp - getTotalLatencySeconds(results));
		double approximateDistanceToTarget = Math.abs(robotPose.getTranslation().x() - AllianceChooser.getAverageConePoleX());
		Rotation2d rotationToTarget = Rotation2d.fromDegrees(-LimelightHelpers.getTX(kLimelightName));

		Translation2d approximateTargetPosition = robotPose
				.transformBy(Pose2d.fromRotation(rotationToTarget))
				.transformBy(Pose2d.fromTranslation(new Translation2d(approximateDistanceToTarget, 0.0)))
				.getTranslation();
		Swerve.getInstance().addRetroObservation(approximateTargetPosition, timestamp);
	}

	private void updateRobotPoseWithConePoles(LimelightResults results, double timestamp) {
		final double kTYDecisionValue = 10.0;

		double observationTimestamp = timestamp - getTotalLatencySeconds(results);
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(observationTimestamp);

		List<LimelightTarget_Retro> retroTargets;
		if (results.targetingResults.targets_Retro.length > 2) {
			Comparator<LimelightTarget_Retro> areaComparator = (r1, r2) -> Double.compare(r2.ta, r1.ta);
			retroTargets = Arrays.stream(results.targetingResults.targets_Retro)
					.sorted(areaComparator)
					.limit(2)
					.toList();
		} else {
			retroTargets = Arrays.asList(results.targetingResults.targets_Retro);
		}

		double conePoleY = getNearestConePoleY(robotPose);
		for (LimelightTarget_Retro retroTarget : retroTargets) {
			double conePoleX = retroTarget.ty > kTYDecisionValue ? AllianceChooser.getHighConePoleX() : AllianceChooser.getMidConePoleX();
			Translation2d polePosition = new Translation2d(conePoleX, conePoleY);
			updateRobotPoseWithConePole(retroTarget, polePosition, robotPose, observationTimestamp);
		}
	}

	private void updateRobotPoseWithConePole(LimelightTarget_Retro retroTarget, Translation2d polePosition, Pose2d robotPose, double observationTimestamp) {
		// TODO: Implement this
		// Calculate the position of the retro target in the robot's coordinate system.
		// Use the difference between the calculated position and the true position to
		// estimate what the robot pose should actually be. Add this estimated robot
		// pose as a vision measurement to the swerve. Remember to use the same heading
		// as the current robot pose; retro targets don't provide good heading approximations.
	}

	private double getNearestConePoleY(Pose2d robotPose) {
		Comparator<Double> distanceToRobotComparator = (y1, y2) -> {
			double y1Distance = Math.abs(y1 - robotPose.getTranslation().y());
			double y2Distance = Math.abs(y2 - robotPose.getTranslation().y());

			return Double.compare(y1Distance, y2Distance);
		};

		return Collections.min(AllianceChooser.getConePoleYs(), distanceToRobotComparator);
	}

	public void setPipeline(Pipeline pipeline) {
		LimelightHelpers.setPipelineIndex(kLimelightName, pipeline.index);
	}

	public enum Pipeline {
		FIDUCIAL(0), RETRO(1), CONE(3);

		public final int index;

		private Pipeline(int index) {
			this.index = index;
		}
	}
}
