package com.team1323.frc2023.loops;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONString;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.ObjectDetector;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import netscape.javascript.JSObject;

public class LimelightProcessor implements Loop {
	private static LimelightProcessor instance = new LimelightProcessor();
	public static LimelightProcessor getInstance() {
		return instance;
	}

	private ObjectDetector objectDetector;

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

	private final NetworkTable table;
	private final NetworkTableEntry ledMode;
	private final NetworkTableEntry pipeline;
	private final NetworkTableEntry camMode;
	private final NetworkTableEntry stream;
	private final NetworkTableEntry heartbeat;
	private final NetworkTableEntry latency;
	private final NetworkTableEntry seesTarget;
	private final NetworkTableEntry robotPose;
	private final NetworkTableEntry camPose;
	private final NetworkTableEntry tx, ty;
	private final NetworkTableEntry json;
	private final double[] zeroArray = new double[]{0, 0, 0, 0, 0, 0};
	private double previousHeartbeat = -1.0;
	
	private LimelightProcessor() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		ledMode = table.getEntry("ledMode");
		pipeline = table.getEntry("pipeline");
		camMode = table.getEntry("camMode");
		stream = table.getEntry("stream");
		heartbeat = table.getEntry("hb");
		latency = table.getEntry("tl");
		seesTarget = table.getEntry("tv");
		robotPose = table.getEntry("botpose");
		camPose = table.getEntry("camerapose_targetspace");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		json = table.getEntry("json");

		//objectDetector = new ObjectDetector();
	}
	
	@Override 
	public void onStart(double timestamp) {
		setStreamMode(0);
	}
	
	@Override 
	public void onLoop(double timestamp) {
		double currentHeartbeat = heartbeat.getDouble(previousHeartbeat);
		if (currentHeartbeat > previousHeartbeat && seesTarget()) {
			double[] robotPoseArray = robotPose.getDoubleArray(zeroArray);
			double[] camPoseArray = camPose.getDoubleArray(zeroArray);
			if (robotPoseArray.length == 6 && camPoseArray.length == 6) {
				Vector3d robotPositionInLimelightCoordinates = new Vector3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]);
				Vector3d robotPositionInOurCoordinates = FieldConversions.convertToField(Constants.kLimelightFieldOrigin, robotPositionInLimelightCoordinates);
				double horizontalAngle = tx.getDouble(0.0);
				double verticalAngle = ty.getDouble(0.0);
				boolean isInCorner = Math.abs(horizontalAngle) > 20.0 && Math.abs(verticalAngle) > 10.0;
				SmartDashboard.putBoolean("Is In Corner", isInCorner);
				if (isInCorner) {
					return;
				}
				if(Netlink.getBooleanValue("Limelight Disabled")) {
					return;
				}
				Rotation2d estimatedRobotHeading = isInCorner ? Swerve.getInstance().getHeading() : Rotation2d.fromDegrees(robotPoseArray[5]);
				Pose2d estimatedRobotPose = new Pose2d(robotPositionInOurCoordinates.x(), robotPositionInOurCoordinates.y(), estimatedRobotHeading);
				Pose2d estimatedRobotPoseInches = Units.metersToInches(estimatedRobotPose);

				if (!AllianceChooser.getCommunityBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation()) &&
						!AllianceChooser.getLoadingZoneBoundingBox().pointWithinBox(estimatedRobotPoseInches.getTranslation())) {
					return;
				}

				Vector3d camPoseVector = new Vector3d(camPoseArray[0], camPoseArray[1], camPoseArray[2]);
				double camDistanceInches = Units.metersToInches(camPoseVector.magnitude());

				double totalLatencySeconds = (latency.getDouble(0.0) / 1000.0) + Constants.kImageCaptureLatency;
	
				//SmartDashboard.putNumberArray("Path Pose", new double[]{robotPoseInches.getTranslation().x(), robotPoseInches.getTranslation().y(), robotPoseInches.getRotation().getDegrees()});
				double translationalStdDev = translationalStandardDeviationRamp.calculate(camDistanceInches);
				double rotationalStdDev = rotationalStandardDeviationRamp.calculate(camDistanceInches);
				Matrix<N3, N1> standardDeviations = VecBuilder.fill(translationalStdDev, translationalStdDev, rotationalStdDev);
				Swerve.getInstance().addVisionMeasurement(estimatedRobotPose,  timestamp - totalLatencySeconds, standardDeviations);
				/*if(json.getString("") != "") {
					try {
						JSONObject jsonDump = new JSONObject(json.getString(""));
						objectDetector.addDetectedObjects(jsonDump);
					} catch(JSONException error) {
						DriverStation.reportError(error.getMessage(), null);
					}
				}*/
				previousHeartbeat = currentHeartbeat;
			}
			
		}
	}
	
	@Override
	public void onStop(double timestamp) {}
	
	public void blink() {
		ledMode.setNumber(2);
	}
	
	public void ledOn(boolean on) {
		ledMode.setNumber(on ? 0 : 1);
	}
	
	public void setDriverMode() {
		camMode.setNumber(1);
	}
	
	public void setVisionMode() {
		camMode.setNumber(0);
	}

	public void setStreamMode(int id) {
		stream.setNumber(id);
	}
	
	public void setPipeline(int id) {
		pipeline.setNumber(id);
	}

	public void setPipeline(Pipeline p) {
		setPipeline(p.id);
		System.out.println("Pipeline set to " + p.id);
	}

	public enum Pipeline {
		APRIL_TAGS(0), GAME_PIECES(1);

		int id;
		private Pipeline(int id) {
			this.id = id;
		}
	}

	private boolean seesTarget() {
		return seesTarget.getDouble(0.0) == 1.0;
	}
}
