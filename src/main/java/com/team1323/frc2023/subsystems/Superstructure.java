package com.team1323.frc2023.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.RobotState;
import com.team1323.frc2023.Constants.ScoringPositions;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.LambdaRequest;
import com.team1323.frc2023.subsystems.requests.ParallelRequest;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.requests.SequentialRequest;
import com.team1323.frc2023.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {

	private Compressor compressor;
	
	public Swerve swerve;
	public VerticalElevator verticalElevator;
	public HorizontalElevator horizontalElevator;
	public Wrist wrist;

	public Intake intake;

	public RobotState robotState;


	
	public Superstructure(){
		compressor = new Compressor(Ports.PNEUMATIC_HUB, PneumaticsModuleType.CTREPCM);

		swerve = Swerve.getInstance();
		verticalElevator = VerticalElevator.getInstance();
		horizontalElevator = HorizontalElevator.getInstance();
		wrist = Wrist.getInstance();
		intake = Intake.getInstance();

		robotState = RobotState.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}

	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
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
	
	public void enableCompressor(boolean enable){
		if (enable) {
			compressor.enableDigital();
		} else {
			compressor.disable();
		}
	}

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

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			@Override
			public boolean isFinished(){
				return robotState.seesTarget();
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
	private void intakeState(Intake.ControlState intakeState) {
		request(
			new SequentialRequest(
				scoringPositionRequest(ScoringPositions.INTAKE),
				intake.stateRequest(intakeState)
			)
		);
	}
	public void intakeConeState() {
		intakeState(Intake.ControlState.INTAKE_CONE);
	}
	public void intakeCubeState() {
		intakeState(Intake.ControlState.INTAKE_CUBE);
	}

	public void stowObjectState() {
		request(
			new ParallelRequest(
				horizontalElevator.extensionRequest(Constants.HorizontalElevator.kStowExtension),
				wrist.angleRequest(Constants.Wrist.kStowAngle)
			)
		);
	}

	private Request scoringPositionRequest(ScoringPositions scoringPosition) {
		return new ParallelRequest(
				horizontalElevator.extensionRequest(scoringPosition.horizontalExtension),
				verticalElevator.heightRequest(scoringPosition.verticalHeight),
				wrist.angleRequest(scoringPosition.wristAngle)
			);
	}
	public void setScoringPositionState(ScoringPositions scoringPosition) {
		request(scoringPositionRequest(scoringPosition));
	}

	
	public void neutralState() {
		request(new ParallelRequest(
			new LambdaRequest(()-> {
				verticalElevator.stop();
				wrist.stop();
				horizontalElevator.stop();
				intake.stop();
			})
		));
	}
		
}
