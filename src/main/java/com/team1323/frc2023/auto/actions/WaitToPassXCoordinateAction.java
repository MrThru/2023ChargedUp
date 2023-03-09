package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.util.Stopwatch;
import com.team254.lib.geometry.Translation2d;

public class WaitToPassXCoordinateAction implements Action{
	private final Swerve swerve;
	private final double timeoutSeconds;
	private final Stopwatch timeoutStopwatch = new Stopwatch();
	private final double targetXCoordinate;
	private double startingXCoordinate;

	public WaitToPassXCoordinateAction(double bottomLeftX, Quadrant quadrant) {
		this(bottomLeftX, quadrant, 15.0);
	}

	public WaitToPassXCoordinateAction(double bottomLeftX, Quadrant quadrant, double timeoutSeconds) {
		this(AutoZones.mirror(new Translation2d(bottomLeftX, 0), quadrant).x(), timeoutSeconds);
	}
	
	public WaitToPassXCoordinateAction(double x){
		this(x, 15.0);
	}

	public WaitToPassXCoordinateAction(double x, double timeoutSeconds){
		targetXCoordinate = x;
		this.timeoutSeconds = timeoutSeconds;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished() {
		return (Math.signum(startingXCoordinate - targetXCoordinate) !=
				Math.signum(swerve.getPose().getTranslation().x() - targetXCoordinate)) ||
				timeoutStopwatch.getTime() >= timeoutSeconds;
	}

	@Override
	public void start() {
		timeoutStopwatch.start();
		startingXCoordinate = swerve.getPose().getTranslation().x();
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		
	}

}
