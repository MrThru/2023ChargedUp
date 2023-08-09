package com.team1323.frc2023.requests.auto;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.util.Stopwatch;
import com.team254.lib.geometry.Translation2d;

public class WaitToPassXCoordinateRequest extends Request {
	private final Swerve swerve;
	private final double timeoutSeconds;
	private final Stopwatch timeoutStopwatch = new Stopwatch();
	private final double targetXCoordinate;
	private double startingXCoordinate;

	public WaitToPassXCoordinateRequest(double bottomLeftX, Quadrant quadrant, double timeoutSeconds) {
		this(AutoZones.mirror(new Translation2d(bottomLeftX, 0), quadrant).x(), timeoutSeconds);
	}

	public WaitToPassXCoordinateRequest(double x, double timeoutSeconds){
		targetXCoordinate = x;
		this.timeoutSeconds = timeoutSeconds;
		swerve = Swerve.getInstance();
	}

	@Override
	public void act() {
		timeoutStopwatch.start();
		startingXCoordinate = swerve.getPose().getTranslation().x();
	}
	
	@Override
	public boolean isFinished() {
		return (Math.signum(startingXCoordinate - targetXCoordinate) !=
				Math.signum(swerve.getPose().getTranslation().x() - targetXCoordinate)) ||
				timeoutStopwatch.getTime() >= timeoutSeconds;
	}
}
