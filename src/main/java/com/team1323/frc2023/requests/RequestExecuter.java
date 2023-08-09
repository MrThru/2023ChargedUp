package com.team1323.frc2023.requests;

public class RequestExecuter {
	private Request activeRequest = null;
	private boolean startedCurrentRequest = false;

	public void request(Request r) {
		if (activeRequest != null) {
			activeRequest.cleanup();
		}
		activeRequest = r;
		startedCurrentRequest = false;
	}

    public void update() {
        if (activeRequest == null) {
            return;
        }

        if (!startedCurrentRequest && activeRequest.allowed()) {
            activeRequest.act();
            startedCurrentRequest = true;
        }

        if (startedCurrentRequest && activeRequest.isFinished()) {
            activeRequest = null;
        }
    }

	public boolean isFinished() { 
        return activeRequest == null;
    }
}
