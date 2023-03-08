package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.lib.util.Stopwatch;

public class WaitForSuperstructureAction implements Action {
	private final double timeoutSeconds;
	private final Stopwatch timeoutStopwatch = new Stopwatch();
	private final Superstructure superstructure;
	
	public WaitForSuperstructureAction() {
		this(2.0);
	}

	public WaitForSuperstructureAction(double timeoutSeconds) {
		this.timeoutSeconds = timeoutSeconds;
		superstructure = Superstructure.getInstance();
	}

	@Override
	public boolean isFinished() {
		return superstructure.requestsCompleted() || timeoutStopwatch.getTime() >= timeoutSeconds;
	}

	@Override
	public void start() {		
		timeoutStopwatch.start();
	}

	@Override
	public void update() {		
	}

	@Override
	public void done() {
	}
	
}
