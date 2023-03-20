package com.team1323.lib.util;

import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class SwerveHeadingController {
	private Rotation2d targetHeading;
	private double disabledTimestamp;
	private double lastUpdateTimestamp;
	private final double disableTimeLength = 0.2;
	private SynchronousPIDF stabilizationPID;
	private SynchronousPIDF snapPID;
	private SynchronousPIDF stationaryPID;
	
	public enum State{
		Off, Stabilize, Snap, TemporaryDisable, Stationary
	}
	private State currentState = State.Off;
	public State getState() {
		return currentState;
	}
	private void setState(State newState) {
		currentState = newState;
	}
	
	public SwerveHeadingController() {
		stabilizationPID = new SynchronousPIDF(0.0075, 0.0, 0.0005, 0.0);
		snapPID = new SynchronousPIDF(0.02, 0.0, 0.0, 0.0);
		stationaryPID = new SynchronousPIDF(0.009, 0.0, 0.0025, 0.0);
		
		targetHeading = Rotation2d.identity();
		lastUpdateTimestamp = Timer.getFPGATimestamp();
	}
	
	public void setStabilizationTarget(Rotation2d angle) {
		targetHeading = angle;
		setState(State.Stabilize);
	}
	
	public void setSnapTarget(Rotation2d angle) {
		targetHeading = angle;
		setState(State.Snap);
	}
	
	public void setStationaryTarget(Rotation2d angle) {
		targetHeading = angle;
		setState(State.Stationary);
	}
	
	public void disable() {
		setState(State.Off);
	}
	
	public void temporarilyDisable() {
		setState(State.TemporaryDisable);
		disabledTimestamp = Timer.getFPGATimestamp();
	}
	
	public Rotation2d getTargetHeading() {
		return targetHeading;
	}
	
	public double updateRotationCorrection(Rotation2d heading, double timestamp) {
		double correction = 0;
		double error = heading.rotateBy(targetHeading.inverse()).getDegrees();
		double dt = timestamp - lastUpdateTimestamp;
		
		switch(currentState) {
			case Off:
				
				break;
			case TemporaryDisable:
				targetHeading = heading;
				if(timestamp - disabledTimestamp >= disableTimeLength)
					setState(State.Stabilize);
				break;
			case Stabilize:
				correction = stabilizationPID.calculate(error, dt);
				break;
			case Snap:
				correction = snapPID.calculate(error, dt);
				break;
			case Stationary:
				correction = Util.limit(stationaryPID.calculate(error, dt), 0.5);
				break;
		}
		
		lastUpdateTimestamp = timestamp;
		return correction;
	}
	
}
