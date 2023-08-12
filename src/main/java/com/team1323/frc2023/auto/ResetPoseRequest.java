package com.team1323.frc2023.auto;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Pose2d;

public class ResetPoseRequest extends Request {
	private final Pose2d newPose;
	private final Swerve swerve;

	public ResetPoseRequest(Pose2d bottomLeftPose, Quadrant quadrant) {
		this(AutoZones.mirror(bottomLeftPose, quadrant));
	}
	
	public ResetPoseRequest(Pose2d newPose){
		this.newPose = newPose;
		swerve = Swerve.getInstance();
	}

	@Override
	public void act() {
		swerve.setStartingPose(newPose);
		swerve.zeroSensors(newPose);
	}
}
