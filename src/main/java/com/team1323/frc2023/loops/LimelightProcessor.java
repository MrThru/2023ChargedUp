package com.team1323.frc2023.loops;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.math.Units;
import com.team1323.lib.math.geometry.Vector3d;
import com.team1323.lib.util.FieldConversions;
import com.team1323.lib.util.Netlink;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightProcessor implements Loop {
	private static LimelightProcessor instance = new LimelightProcessor();
	public static LimelightProcessor getInstance() {
		return instance;
	}

	private static final String kLimelightName = "limelight";
	private static final double kMinFiducialArea = 0.01; // TODO: update this

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
		double currentHeartbeat = LimelightHelper.getLimelightNTDouble(kLimelightName, "hb");
		if (currentHeartbeat > previousHeartbeat && !Netlink.getBooleanValue("Limelight Disabled")) {
			LimelightResults results = LimelightHelper.getLatestResults(kLimelightName);
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

		double[] robotPoseArray;
		double[] cameraPoseArray;
		List<LimelightTarget_Fiducial> fiducials = Arrays.asList(results.targetingResults.targets_Fiducials);
		if (fiducials.stream().allMatch(fiducial -> fiducial.ta >= kMinFiducialArea)) {
			robotPoseArray = LimelightHelper.getBotpose(kLimelightName);
			cameraPoseArray = LimelightHelper.getCameraPose_TargetSpace(kLimelightName);
		} else {
			Comparator<LimelightTarget_Fiducial> areaComparator = (f1, f2) -> Double.compare(f1.ta, f2.ta);
			LimelightTarget_Fiducial largestFiducial = Collections.max(fiducials, areaComparator);
			robotPoseArray = largestFiducial.robotPose_FieldSpace;
			cameraPoseArray = largestFiducial.cameraPose_TargetSpace;
		}

		if (robotPoseArray.length != 6 || cameraPoseArray.length != 6) {
			return;
		}

		Vector3d robotPositionInLimelightCoordinates = new Vector3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]);
		Vector3d robotPositionInWpiCoordinates = FieldConversions.convertToField(Constants.kLimelightFieldOrigin, robotPositionInLimelightCoordinates);
		Pose2d estimatedRobotPoseMeters = new Pose2d(robotPositionInWpiCoordinates.x(), robotPositionInWpiCoordinates.y(), Rotation2d.fromDegrees(robotPoseArray[5]));
		Pose2d estimatedRobotPoseInches = Units.metersToInches(estimatedRobotPoseMeters);

		// Only accept vision updates if they place the robot within our own community or loading zone
		if (!AllianceChooser.getCommunityBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation()) &&
				!AllianceChooser.getLoadingZoneBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation())) {
			return;
		}

		Vector3d cameraPoseVector = new Vector3d(cameraPoseArray[0], cameraPoseArray[1], cameraPoseArray[2]);
		double cameraDistanceInches = Units.metersToInches(cameraPoseVector.magnitude());

		double translationalStdDev = translationalStandardDeviationRamp.calculate(cameraDistanceInches);
		double rotationalStdDev = rotationalStandardDeviationRamp.calculate(cameraDistanceInches);
		Matrix<N3, N1> standardDeviations = VecBuilder.fill(translationalStdDev, translationalStdDev, rotationalStdDev);
		Swerve.getInstance().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(results), standardDeviations);

		// For debugging purposes
		if (Swerve.getInstance().getState() == Swerve.ControlState.VISION_PID) {
			System.out.println("Vision measurement added");
			SmartDashboard.putNumberArray("Path Pose", new double[]{estimatedRobotPoseInches.getTranslation().x(), estimatedRobotPoseInches.getTranslation().y(), estimatedRobotPoseInches.getRotation().getDegrees()});
		}
	}

	private void handleRetroTargets(LimelightResults results, double timestamp) {
		if (results.targetingResults.targets_Retro.length == 0) {
			return;
		}

		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(timestamp - getTotalLatencySeconds(results));
		double approximateDistanceToTarget = Math.abs(robotPose.getTranslation().x() - AllianceChooser.getConePoleX());
		Rotation2d rotationToTarget = Rotation2d.fromDegrees(LimelightHelper.getTX(kLimelightName)); // TODO: Check if this needs to be flipped

		Translation2d approximateTargetPosition = robotPose
				.transformBy(Pose2d.fromRotation(rotationToTarget))
				.transformBy(Pose2d.fromTranslation(new Translation2d(approximateDistanceToTarget, 0.0)))
				.getTranslation();
		Swerve.getInstance().addRetroObservation(approximateTargetPosition, timestamp);
	}

	public void setPipeline(Pipeline pipeline) {
		LimelightHelper.setPipelineIndex(kLimelightName, pipeline.index);
	}
	public enum Pipeline {
		FIDUCIAL(0), RETRO(1);

		public final int index;

		private Pipeline(int index) {
			this.index = index;
		}
	}
}
