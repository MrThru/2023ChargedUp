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
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.GoalTrack;
import com.team1323.frc2023.vision.TargetInfo;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.math.Units;
import com.team1323.lib.math.geometry.Vector3d;
import com.team1323.lib.util.CircularBuffer;
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

	private static final Pose2d kRobotToCameraTransform = new Pose2d(new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset),
			Rotation2d.identity());
	private static final Rotation2d kCameraPitchCorrection = Rotation2d.fromDegrees(Constants.kCameraPitchAngleDegrees);
	private static final Rotation2d kCameraYawCorrection = Rotation2d.fromDegrees(Constants.kCameraYawAngleDegrees);
	private static final double kMidConePoleHeight = 24.125;
	private static final double kHighConePoleHeight = 43.875;
	private static final double kConeHeight = 7.5;

	private final GoalTrack coneGoalTrack = GoalTrack.makeNewTrack(0.0, Translation2d.identity(), 0);

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
			handleDetectorTargets(results, timestamp);
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

	private double coneLeftRightOffset = 0;
	private CircularBuffer coneLeftRightBuffer = new CircularBuffer(100);
	
	public void clearConeOffsetBuffer() {
		coneLeftRightBuffer.clear();
		coneLeftRightOffset = 0;
	}

	private void handleRetroTargets(LimelightResults results, double timestamp) {
		if (results.targetingResults.targets_Retro.length == 0) {
			return;
		}

		int pipelineIndex = (int) results.targetingResults.pipelineID;
		if (pipelineIndex == Pipeline.RETRO.index) {
			updateRobotPoseWithConePoles(results, timestamp);
		} else if (pipelineIndex == Pipeline.CONE.index) {
			Translation2d wristTipPosition = SuperstructureCoordinator.getInstance().getPosition().getWristTipPosition();
			double distanceFromLimelight = Constants.kCameraPose.getVector3d().add(new Vector3d(wristTipPosition.x(),0,wristTipPosition.y()).inverse()).magnitude();
			double currentFrameLeftRightOffset = -Math.tan(Math.toRadians(LimelightHelpers.getTX(kLimelightName))) * 
									distanceFromLimelight;
			coneLeftRightBuffer.addValue(currentFrameLeftRightOffset);
			coneLeftRightOffset = coneLeftRightBuffer.getAverage();	
		}
		
	}

	private void handleDetectorTargets(LimelightResults results, double timestamp) {
		if (results.targetingResults.targets_Detector.length == 0) {
			return;
		}

		double observationTime = timestamp - getTotalLatencySeconds(results);
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(observationTime);

		if(results.targetingResults.pipelineID == Pipeline.DETECTOR.index) {
			/*for(int i = 0; i < results.targetingResults.targets_Detector.length; i++) {
				GridTracker.getInstance().addDetectedObject(results.targetingResults.targets_Detector[i]);
			}*/

			TargetInfo targetInfo = new TargetInfo(Rotation2d.fromDegrees(-LimelightHelpers.getTX(kLimelightName)).tan(), 
					Rotation2d.fromDegrees(LimelightHelpers.getTY(kLimelightName)).tan());
			Translation2d conePosition = getRetroTargetPosition(targetInfo, kConeHeight, robotPose);
			coneGoalTrack.forceUpdate(timestamp, conePosition);
			SmartDashboard.putNumberArray("Cone Position", new double[]{conePosition.x(), conePosition.y()});
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
		final double kMaxTXDifference = 7.0;

		double observationTimestamp = timestamp - getTotalLatencySeconds(results);
		Pose2d robotPose = Swerve.getInstance().getPoseAtTime(observationTimestamp);

		Comparator<LimelightTarget_Retro> areaComparator = (r1, r2) -> Double.compare(r2.ta, r1.ta);
		List<LimelightTarget_Retro> retroTargets = Arrays.stream(results.targetingResults.targets_Retro)
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

		Swerve.getInstance().addVisionMeasurement(Units.inchesToMeters(correctedRobotPose), observationTimestamp, standardDeviations);

		// For debugging purposes
		if (poleHeight == kHighConePoleHeight) {
			SmartDashboard.putString("Cone Pole Position", new Translation2d(robotPose.getTranslation(), estimatedPolePosition).toString());
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
		final double differentialHeight = physicalTargetHeight - Constants.kCameraZOffset;
		final Pose2d cameraPose = robotPose.transformBy(kRobotToCameraTransform);

		double ydeadband = target.getY();
		
		// Compensate for camera yaw
		double xyaw = target.getX() * kCameraYawCorrection.cos() + ydeadband * kCameraYawCorrection.sin();
		double yyaw = ydeadband * kCameraYawCorrection.cos() - target.getX() * kCameraYawCorrection.sin();
		double zyaw = target.getZ();
		
		// Compensate for camera pitch
		double xr = zyaw * kCameraPitchCorrection.sin() + xyaw * kCameraPitchCorrection.cos();
		double yr = yyaw;
		double zr = zyaw * kCameraPitchCorrection.cos() - xyaw * kCameraPitchCorrection.sin();
		
		// find intersection with the goal
		double scaling = differentialHeight / zr;
		double distance = Math.hypot(xr, yr) * scaling;
		Rotation2d angle = new Rotation2d(xr, yr, true);

		return cameraPose.transformBy(Pose2d.fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
				.getTranslation();
	}

	public double getRetroConeLeftRightOffset() {
		return coneLeftRightOffset;
	}

	public Translation2d getConePosition() {
		coneGoalTrack.emptyUpdate();
		Translation2d conePosition = coneGoalTrack.getSmoothedPosition();

		if (conePosition == null) {
			return new Translation2d();
		}

		return conePosition;
	}

	public Pose2d getRobotConePickupPosition() {
		Translation2d conePosition = LimelightProcessor.getInstance().getConePosition();
        Pose2d intakingPose = Pose2d.identity();    
		if (!conePosition.equals(Translation2d.identity())) {
			intakingPose = new Pose2d(conePosition, Swerve.getInstance().getPose().getRotation())
					.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
		}
		return intakingPose;
	}

	public void setPipeline(Pipeline pipeline) {
		LimelightHelpers.setPipelineIndex(kLimelightName, pipeline.index);
	}

	public enum Pipeline {
		FIDUCIAL(0), RETRO(1), CONE(3), DETECTOR(4);

		public final int index;

		private Pipeline(int index) {
			this.index = index;
		}
	}
}
