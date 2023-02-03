package com.team1323.frc2023.loops;

import com.team1323.frc2023.RobotState;
import com.team1323.frc2023.subsystems.swerve.Swerve;

public class RobotStateEstimator implements Loop{
	private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance(){
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator() {
	}
	
	RobotState robotState = RobotState.getInstance();
	Swerve swerve;

	@Override
	public void onStart(double timestamp) {
		swerve = Swerve.getInstance();
	}

	@Override
	public void onLoop(double timestamp) {
		//robotState.addObservations(timestamp, swerve.getPose(), swerve.getVelocity(), Rotation2d.identity());
		swerve.updateOdometry(timestamp);
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}