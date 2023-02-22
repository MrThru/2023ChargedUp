package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Translation2d;

public class WaitToPassYCoordinateAction implements Action{
	double startingYCoordinate;
	double targetYCoordinate;
	Swerve swerve;

	public WaitToPassYCoordinateAction(double bottomLeftY, Quadrant quadrant) {
		this(AutoZones.mirror(new Translation2d(0, bottomLeftY), quadrant).y());
	}
	
	public WaitToPassYCoordinateAction(double y){
		targetYCoordinate = y;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished() {
		return Math.signum(startingYCoordinate - targetYCoordinate) !=
				Math.signum(swerve.getPose().getTranslation().y() - targetYCoordinate);
	}

	@Override
	public void start() {
		startingYCoordinate = swerve.getPose().getTranslation().y();
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		
	}
}
