package com.team1323.frc2023.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.ConeOffset;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.LimelightHelpers.LimelightResults;
import com.team1323.frc2023.vision.LimelightHelpers.LimelightTarget_Fiducial;
import com.team1323.frc2023.vision.LimelightHelpers.LimelightTarget_Retro;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.math.Units;
import com.team1323.lib.math.geometry.Vector3d;
import com.team1323.lib.util.CircularBuffer;
import com.team1323.lib.util.FieldConversions;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.Stopwatch;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

public class Limelight extends Subsystem {
	private static final double kMinFiducialArea = 0.115;
	private static final double kMinRetroArea = 0.0007;

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

	private static final double kMidConePoleHeight = 24.125;
	private static final double kHighConePoleHeight = 43.875;
	private static final double kConeHeight = 7.5;

	private final String name;
	private final Pose2d robotToCameraTransform;
	private final double cameraHeightInches;
	private final Rotation2d cameraPitch, cameraYaw;
	private final GoalTracker coneGoalTracker = new GoalTracker();
	private final LimelightInputs inputs = new LimelightInputs();
	private final LatencyInputsAutoLogged latencyInputs = new LatencyInputsAutoLogged();

	private final Stopwatch lastUpdateStopwatch = new Stopwatch();
	private double previousHeartbeat = -1.0;

	private boolean isConnected = false;
	
	public Limelight(String name, Translation2d robotToCameraTranslation, double cameraHeightInches, Rotation2d cameraPitch, Rotation2d cameraYaw) {
		this.name = name;
		this.robotToCameraTransform = new Pose2d(robotToCameraTranslation, Rotation2d.identity());
		this.cameraHeightInches = cameraHeightInches;
		this.cameraPitch = cameraPitch;
		this.cameraYaw = cameraYaw;
	}

	@Override
	public void readPeriodicInputs() {
		if (RobotBase.isReal()) {
			inputs.heartbeat = LimelightHelpers.getLimelightNTDouble(name, "hb");
			inputs.jsonDump = LimelightHelpers.getJSONDump(name);
			inputs.tx = LimelightHelpers.getTX(name);
			inputs.robotPose2d = LimelightHelpers.getBotPose2d(name);
			inputs.cameraPose3d_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(name);
		}
		Logger.getInstance().processInputs(name, inputs);
		inputs.results = LimelightHelpers.getLatestResults(name, inputs.jsonDump);

		latencyInputs.jsonParseLatencyMs = inputs.results.targetingResults.latency_jsonParse;
		Logger.getInstance().processInputs(name, latencyInputs);
		inputs.results.targetingResults.latency_jsonParse = latencyInputs.jsonParseLatencyMs;
	}

	@Override
	public void outputTelemetry() {
		Logger.getInstance().recordOutput(getLogKey("Is Connected"), isConnected);
	}

	@Override
	public void stop() {
	}
	
	public void processAllTargets(double timestamp) {
		if (inputs.heartbeat != previousHeartbeat) {
			lastUpdateStopwatch.reset();
			isConnected = true;

			if (!Netlink.getBooleanValue("Limelight Disabled")) {
				handleFiducialTargets(timestamp);
				handleRetroTargets(timestamp);
				handleDetectorTargets(timestamp);
			}

			previousHeartbeat = inputs.heartbeat;
		} else {
			lastUpdateStopwatch.startIfNotRunning();
			if (lastUpdateStopwatch.getTime() > 0.5) {
				isConnected = false;
			}
		}
	}

	private double getTotalLatencySeconds(LimelightResults results) {
		return (results.targetingResults.latency_capture + results.targetingResults.latency_pipeline +
				results.targetingResults.latency_jsonParse) / 1000.0;
	}

	private void handleFiducialTargets(double timestamp) {
		if (inputs.results.targetingResults.targets_Fiducials.length == 0) {
			return;
		}

		Pose2d robotPoseInLimelightCoordinates;
		List<LimelightTarget_Fiducial> fiducials = Arrays.asList(inputs.results.targetingResults.targets_Fiducials);
		if (fiducials.stream().allMatch(fiducial -> fiducial.ta >= kMinFiducialArea)) {
			robotPoseInLimelightCoordinates = inputs.robotPose2d;
		} else {
			Comparator<LimelightTarget_Fiducial> areaComparator = (f1, f2) -> Double.compare(f1.ta, f2.ta);
			LimelightTarget_Fiducial largestFiducial = Collections.max(fiducials, areaComparator);
			robotPoseInLimelightCoordinates = largestFiducial.getRobotPose_FieldSpace2D();
		}

		if (robotPoseInLimelightCoordinates.equals(Pose2d.identity()) || inputs.cameraPose3d_TargetSpace.equals(new Pose3d())) {
			return;
		}

		Pose2d estimatedRobotPoseMeters = FieldConversions.convertToField(Constants.kLimelightFieldOrigin, robotPoseInLimelightCoordinates);
		Pose2d estimatedRobotPoseInches = Units.metersToInches(estimatedRobotPoseMeters);

		// Only accept vision updates if they place the robot within our own community or loading zone
		if (!AllianceChooser.getCommunityBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation()) &&
				!AllianceChooser.getLoadingZoneBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation())) {
			return;
		}

		double cameraDistanceInches = Units.metersToInches(inputs.cameraPose3d_TargetSpace.getTranslation().getNorm());

		double translationalStdDev = translationalStandardDeviationRamp.calculate(cameraDistanceInches);
		double rotationalStdDev = rotationalStandardDeviationRamp.calculate(cameraDistanceInches);
		Matrix<N3, N1> standardDeviations = VecBuilder.fill(translationalStdDev, translationalStdDev, rotationalStdDev);
		Swerve.getInstance().addVisionMeasurement(estimatedRobotPoseMeters,  timestamp - getTotalLatencySeconds(inputs.results), standardDeviations);
	}

	private double coneLeftRightOffset = 0;
	private CircularBuffer coneLeftRightBuffer = new CircularBuffer(100);
	
	public void clearConeOffsetBuffer() {
		coneLeftRightBuffer.clear();
		coneLeftRightOffset = 0;
	}

	private void handleRetroTargets(double timestamp) {
		if (inputs.results.targetingResults.targets_Retro.length == 0) {
			return;
		}

		int pipelineIndex = (int) inputs.results.targetingResults.pipelineID;
		if (pipelineIndex == Pipeline.RETRO.index) {
			updateRobotPoseWithConePoles(timestamp);
		} else if (pipelineIndex == Pipeline.CONE.index) {
			Translation2d wristTipPosition = SuperstructureCoordinator.getInstance().getPosition().getWristTipPosition();
			com.team1323.lib.math.geometry.Pose3d cameraPose = new com.team1323.lib.math.geometry.Pose3d(
					new Vector3d(robotToCameraTransform.getTranslation().x(), robotToCameraTransform.getTranslation().y(), cameraHeightInches),
					cameraPitch);
			double distanceFromLimelight = cameraPose.getVector3d().add(new Vector3d(wristTipPosition.x(),0,wristTipPosition.y()).inverse()).magnitude();
			double currentFrameLeftRightOffset = -Math.tan(Math.toRadians(inputs.tx)) * 
									distanceFromLimelight;
			coneLeftRightBuffer.addValue(currentFrameLeftRightOffset);
			coneLeftRightOffset = coneLeftRightBuffer.getAverage();

			if(Math.abs(coneLeftRightOffset) >= 1.5) {
				Claw.getInstance().setCurrentConeOffset((Math.signum(coneLeftRightOffset) == -1) ? ConeOffset.Right : ConeOffset.Left);
			} else {
				Claw.getInstance().setCurrentConeOffset(ConeOffset.Center);
			}
		}
		
	}

	private void handleDetectorTargets(double timestamp) {
		if (inputs.results.targetingResults.targets_Detector.length == 0) {
			coneGoalTracker.update(timestamp, new ArrayList<>());
			return;
		}

		double observationTime = timestamp - getTotalLatencySeconds(inputs.results);
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(observationTime);

		if(inputs.results.targetingResults.pipelineID == Pipeline.DETECTOR.index) {
			/*for(int i = 0; i < results.targetingResults.targets_Detector.length; i++) {
				GridTracker.getInstance().addDetectedObject(results.targetingResults.targets_Detector[i]);
			}*/

			List<Translation2d> targetPositions = Arrays.stream(inputs.results.targetingResults.targets_Detector)
					.map(target -> {
						TargetInfo targetInfo = new TargetInfo(Rotation2d.fromDegrees(-target.tx).tan(), 
								Rotation2d.fromDegrees(target.ty).tan());
						Translation2d targetPosition = getRetroTargetPosition(targetInfo, kConeHeight, robotPose);

						return targetPosition;
					})
					.toList();
			coneGoalTracker.update(timestamp, targetPositions);

			List<Translation2d> goalTrackerPositions = coneGoalTracker.getTracks().stream()
						.map(report -> report.field_to_goal)
						.toList();
			Logger.getInstance().recordOutput(getLogKey("Cone Positions"), goalTrackerPositions.toString());
		}
	}

	private void updateConePolePosition(double timestamp) {
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(timestamp - getTotalLatencySeconds(inputs.results));
		double approximateDistanceToTarget = Math.abs(robotPose.getTranslation().x() - AllianceChooser.getAverageConePoleX());
		Rotation2d rotationToTarget = Rotation2d.fromDegrees(inputs.tx);

		Translation2d approximateTargetPosition = robotPose
				.transformBy(Pose2d.fromRotation(rotationToTarget))
				.transformBy(Pose2d.fromTranslation(new Translation2d(approximateDistanceToTarget, 0.0)))
				.getTranslation();
		Swerve.getInstance().addRetroObservation(approximateTargetPosition, timestamp);
	}

	private void updateRobotPoseWithConePoles(double timestamp) {
		final double kTYDecisionValue = 10.0;
		final double kMaxTXDifference = 7.0;

		double observationTimestamp = timestamp - getTotalLatencySeconds(inputs.results);
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(observationTimestamp);

		Comparator<LimelightTarget_Retro> areaComparator = (r1, r2) -> Double.compare(r2.ta, r1.ta);
		List<LimelightTarget_Retro> retroTargets = Arrays.stream(inputs.results.targetingResults.targets_Retro)
				.filter(r -> r.ta >= kMinRetroArea)
				.sorted(areaComparator)
				.limit(2)
				.toList();
		if (retroTargets.size() == 2 && Math.abs(retroTargets.get(0).tx - retroTargets.get(1).tx) > kMaxTXDifference) {
			LimelightTarget_Retro centeredTarget = Math.abs(retroTargets.get(0).tx) < Math.abs(retroTargets.get(1).tx) ?
					retroTargets.get(0) : retroTargets.get(1);
			retroTargets = Arrays.asList(centeredTarget);
		}

		double conePoleY = getNearestConePoleY(robotPose);
		for (LimelightTarget_Retro retroTarget : retroTargets) {
			boolean isHighPole = retroTarget.ty > kTYDecisionValue;
			double conePoleX = isHighPole ? AllianceChooser.getHighConePoleX() : AllianceChooser.getMidConePoleX();
			double conePoleHeight = isHighPole ? kHighConePoleHeight : kMidConePoleHeight;
			Translation2d polePosition = new Translation2d(conePoleX, conePoleY);
			updateRobotPoseWithConePole(retroTarget, polePosition, conePoleHeight, robotPose, observationTimestamp);
		}
	}

	private void updateRobotPoseWithConePole(LimelightTarget_Retro retroTarget, Translation2d truePolePosition, 
			double poleHeight, Pose2d robotPose, double observationTimestamp) {
		TargetInfo targetInfo = new TargetInfo(Rotation2d.fromDegrees(-retroTarget.tx).tan(), Rotation2d.fromDegrees(retroTarget.ty).tan());
		Translation2d estimatedPolePosition = getRetroTargetPosition(targetInfo, poleHeight, robotPose);
		Translation2d positionCorrection = new Translation2d(estimatedPolePosition, truePolePosition);
		Pose2d correctedRobotPose = new Pose2d(robotPose.getTranslation().translateBy(positionCorrection), robotPose.getRotation());
		Matrix<N3, N1> standardDeviations = VecBuilder.fill(2.0, 1.0, 100.0);

		if (poleHeight != kHighConePoleHeight || VerticalElevator.getInstance().getPosition() <= 14) {
			Swerve.getInstance().addVisionMeasurement(Units.inchesToMeters(correctedRobotPose), observationTimestamp, standardDeviations);
		} else if (poleHeight == kHighConePoleHeight) {
			correctedRobotPose = new Pose2d(new Translation2d(robotPose.getTranslation().x(), correctedRobotPose.getTranslation().y()), 
					correctedRobotPose.getRotation());
			standardDeviations = VecBuilder.fill(10000.0, 1.0, 100.0);
			Swerve.getInstance().addVisionMeasurement(Units.inchesToMeters(correctedRobotPose), observationTimestamp, standardDeviations);
		}

		// For debugging purposes
		if (poleHeight == kHighConePoleHeight) {
			Logger.getInstance().recordOutput(getLogKey("Cone High Pole Position"), new Translation2d(robotPose.getTranslation(), estimatedPolePosition).toString());
		} else if (poleHeight == kMidConePoleHeight) {
			Logger.getInstance().recordOutput(getLogKey("Cone Mid Pole Position"), new Translation2d(robotPose.getTranslation(), estimatedPolePosition).toString());
		}
	}

	private double getNearestConePoleY(Pose2d robotPose) {
		Comparator<Double> distanceToRobotComparator = (y1, y2) -> {
			double y1Distance = Math.abs(y1 - robotPose.getTranslation().y());
			double y2Distance = Math.abs(y2 - robotPose.getTranslation().y());

			return Double.compare(y1Distance, y2Distance);
		};

		return Collections.min(AllianceChooser.getConePoleYs(), distanceToRobotComparator);
	}

	public Translation2d getRetroTargetPosition(TargetInfo target, double physicalTargetHeight, Pose2d robotPose) {
		final double differentialHeight = physicalTargetHeight - cameraHeightInches;
		final Pose2d cameraPose = robotPose.transformBy(robotToCameraTransform);

		double ydeadband = target.getY();
		
		// Compensate for camera yaw
		double xyaw = target.getX() * cameraYaw.cos() + ydeadband * cameraYaw.sin();
		double yyaw = ydeadband * cameraYaw.cos() - target.getX() * cameraYaw.sin();
		double zyaw = target.getZ();
		
		// Compensate for camera pitch
		double xr = zyaw * cameraPitch.sin() + xyaw * cameraPitch.cos();
		double yr = yyaw;
		double zr = zyaw * cameraPitch.cos() - xyaw * cameraPitch.sin();
		
		// find intersection with the goal
		double scaling = differentialHeight / zr;
		double distance = Math.hypot(xr, yr) * scaling;
		Rotation2d angle = new Rotation2d(xr, yr, true);

		return cameraPose.transformBy(Pose2d.fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
				.getTranslation();
	}

	public synchronized Translation2d getNearestConePosition(Translation2d trueFieldPosition) {
		final double kNearestConeTolerance = 36.0;

		List<Translation2d> conePositions = coneGoalTracker.getTracks().stream()
				.map(report -> report.field_to_goal)
				.toList();
		if (conePositions.isEmpty()) {
			return Translation2d.identity();
		}

		Comparator<Translation2d> distanceComparator = (t1, t2) -> {
			double distance1 = t1.distance(trueFieldPosition);
			double distance2 = t2.distance(trueFieldPosition);

			return Double.compare(distance1, distance2);
		};
		Translation2d nearestConePosition = Collections.min(conePositions, distanceComparator);
		if (nearestConePosition.distance(trueFieldPosition) > kNearestConeTolerance) {
			return Translation2d.identity();
		}

		return nearestConePosition;
	}

	public Pose2d getRobotConePickupPosition(Translation2d trueConePosition) {
		Translation2d conePosition = getNearestConePosition(trueConePosition);
        Pose2d intakingPose = Pose2d.identity();    
		if (!conePosition.equals(Translation2d.identity())) {
			intakingPose = new Pose2d(conePosition, Swerve.getInstance().getPose().getRotation())
					.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
		}
		return intakingPose;
	}

	private String getLogKey(String entryName) {
		return String.format("%s/%s", name, entryName);
	}

	public boolean isConnected() {
		return isConnected;
	}

	public void setPipeline(Pipeline pipeline) {
		LimelightHelpers.setPipelineIndex(name, pipeline.index);
	}

	public enum Pipeline {
		FIDUCIAL(0), RETRO(1), CONE(3), DETECTOR(4);

		public final int index;

		private Pipeline(int index) {
			this.index = index;
		}
	}

	public static class LimelightInputs implements LoggableInputs {
		private static final double[] kDefaultPose2dArray = new double[3];
		private static final double[] kDefaultPose3dArray = new double[7];

		public double heartbeat;
		public String jsonDump;
		public double tx;
		public Pose2d robotPose2d = Pose2d.identity();
		public Pose3d cameraPose3d_TargetSpace = new Pose3d();
		// This should be parsed from the JSON dump input
		public LimelightResults results = new LimelightResults();
		
		@Override
		public void toLog(LogTable table) {
			table.put("Heartbeat", heartbeat);
			table.put("JSON Dump", jsonDump);
			table.put("TX", tx);
			table.put("Robot Pose 2D", new double[]{robotPose2d.getTranslation().x(), robotPose2d.getTranslation().y(), robotPose2d.getRotation().getRadians()});
			table.put("Camera Pose 3D - Target Space", new double[]{cameraPose3d_TargetSpace.getX(), cameraPose3d_TargetSpace.getY(), cameraPose3d_TargetSpace.getZ(),
					cameraPose3d_TargetSpace.getRotation().getQuaternion().getW(), cameraPose3d_TargetSpace.getRotation().getQuaternion().getX(),
					cameraPose3d_TargetSpace.getRotation().getQuaternion().getY(), cameraPose3d_TargetSpace.getRotation().getQuaternion().getZ()});
		}
		@Override
		public void fromLog(LogTable table) {
			heartbeat = table.getDouble("Heartbeat", heartbeat);
			jsonDump = table.getString("JSON Dump", jsonDump);
			tx = table.getDouble("TX", tx);

			double[] robotPoseArray = table.getDoubleArray("Robot Pose 2D", kDefaultPose2dArray);
			if (robotPoseArray.length == kDefaultPose2dArray.length) {
				robotPose2d = new Pose2d(new Translation2d(robotPoseArray[0], robotPoseArray[1]), Rotation2d.fromRadians(robotPoseArray[2]));
			}

			double[] cameraPoseArray = table.getDoubleArray("Camera Pose 3D - Target Space", kDefaultPose3dArray);
			if (cameraPoseArray.length == kDefaultPose3dArray.length) {
				cameraPose3d_TargetSpace = new Pose3d(cameraPoseArray[0], cameraPoseArray[1], cameraPoseArray[2], 
						new Rotation3d(new Quaternion(cameraPoseArray[3], cameraPoseArray[4], cameraPoseArray[5], cameraPoseArray[6])));
			}
		}
	}

	@AutoLog
	public static class LatencyInputs {
		public double jsonParseLatencyMs;
	}
}
