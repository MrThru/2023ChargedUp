package com.team1323.frc2023.loops;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.math.geometry.Vector3d;
import com.team1323.lib.util.FieldConversions;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightProcessor implements Loop {
	private static LimelightProcessor instance = new LimelightProcessor();
	public static LimelightProcessor getInstance() {
		return instance;
	}

	private final NetworkTable table;
	private final NetworkTableEntry ledMode;
	private final NetworkTableEntry pipeline;
	private final NetworkTableEntry camMode;
	private final NetworkTableEntry stream;
	private final NetworkTableEntry heartbeat;
	private final NetworkTableEntry latency;
	private final NetworkTableEntry seesTarget;
	private final NetworkTableEntry robotPose;
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
			if (robotPoseArray.length == 6) {
				Vector3d robotPositionInLimelightCoordinates = new Vector3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]);
				Vector3d robotPositionInOurCoordinates = FieldConversions.convertToField(Constants.kLimelightFieldOrigin, robotPositionInLimelightCoordinates);
				Pose2d estimatedRobotPose = new Pose2d(robotPositionInOurCoordinates.x(), robotPositionInOurCoordinates.y(), Rotation2d.fromDegrees(robotPoseArray[5]));
				double totalLatencySeconds = (latency.getDouble(0.0) / 1000.0) + Constants.kImageCaptureLatency;
	
				//Swerve.getInstance().addVisionMeasurement(estimatedRobotPose,  timestamp - totalLatencySeconds);
	
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
