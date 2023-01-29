package com.team1323.frc2023.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.requests.LambdaRequest;
import com.team1323.frc2023.subsystems.requests.ParallelRequest;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}

	public final Swerve swerve;
	public final VerticalElevator verticalElevator;
	public final HorizontalElevator horizontalElevator;
	public final Shoulder shoulder;
	public final Wrist wrist;
	
	public Superstructure() {
		swerve = Swerve.getInstance();
		verticalElevator = VerticalElevator.getInstance();
		horizontalElevator = HorizontalElevator.getInstance();
		shoulder = Shoulder.getInstance();
		wrist = Wrist.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}

	private Request activeRequest = null;
	private List<Request> queuedRequests = new ArrayList<>();
	
	private boolean newRequest = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequest(Request request){
		activeRequest = request;
		newRequest = true;
		allRequestsCompleted = false;
	}
	
	private void setQueue(List<Request> requests){
		clearQueue();
		for(Request request : requests) {
			queuedRequests.add(request);
		}
	}

	private void setQueue(Request request) {
		setQueue(Arrays.asList(request));
	}

	private void clearQueue() {
		queuedRequests.clear();
	}
	
	public void request(Request r){
		setActiveRequest(r);
		clearQueue();
	}
	
	public void request(Request active, Request queue){
		setActiveRequest(active);
		setQueue(queue);
	}
	
	public void queue(Request request){
		queuedRequests.add(request);
	}
	
	public void replaceQueue(Request request){
		setQueue(request);
	}

	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			if(newRequest && activeRequest != null) {
				activeRequest.act();
				newRequest = false;
			} 

			if(activeRequest == null) {
				if(queuedRequests.isEmpty()) {
					allRequestsCompleted = true;
				} else {
					setActiveRequest(queuedRequests.remove(0));
				}
			} else if(activeRequest.isFinished()) {
				activeRequest = null;
			}
		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
		
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
	}
	
	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }
        return false;
    }

	///// States /////
	public void neutralState() {
		request(new ParallelRequest(
			new LambdaRequest(()-> {
				verticalElevator.stop();
				horizontalElevator.stop();
				wrist.stop();
			})
		));
	}
		
}