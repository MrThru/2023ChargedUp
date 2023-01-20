package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Rotation2d;

public class SetTargetHeadingAction extends RunOnceAction{
	Rotation2d targetHeading;
	Swerve swerve;
	
	public SetTargetHeadingAction(Rotation2d targetHeading){
		this.targetHeading = targetHeading;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public void runOnce() {
		swerve.setPathHeading(targetHeading);
	}

}
