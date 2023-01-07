package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Swerve;
import com.team254.lib.geometry.Pose2d;

public class ResetPoseAction extends RunOnceAction{
	private Pose2d newPose;
	boolean leftStartingSide = true;

	Swerve swerve;
	
	public ResetPoseAction(Pose2d newPose){
		this.newPose = newPose;
		swerve = Swerve.getInstance();
	}

	@Override
	public void runOnce() {
		swerve.setStartingPose(newPose);
		swerve.zeroSensors(newPose);
	}

}
