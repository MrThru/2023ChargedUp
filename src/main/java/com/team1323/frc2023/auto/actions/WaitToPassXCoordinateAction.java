package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Translation2d;

public class WaitToPassXCoordinateAction implements Action{
	double startingXCoordinate;
	double targetXCoordinate;
	Swerve swerve;

	public WaitToPassXCoordinateAction(double bottomLeftX, Quadrant quadrant) {
		this(AutoZones.mirror(new Translation2d(bottomLeftX, 0), quadrant).x());
	}
	
	public WaitToPassXCoordinateAction(double x){
		targetXCoordinate = x;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate) !=
				Math.signum(swerve.getPose().getTranslation().x() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = swerve.getPose().getTranslation().x();
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		
	}

}
