package com.team1323.frc2023.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.requests.EmptyRequest;
import com.team1323.frc2023.subsystems.requests.LambdaRequest;
import com.team1323.frc2023.subsystems.requests.ParallelRequest;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.requests.SequentialRequest;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}

	private final SuperstructureCoordinator coordinator;
	public final Swerve swerve;
	public final VerticalElevator verticalElevator;
	public final HorizontalElevator horizontalElevator;
	public final Shoulder shoulder;
	public final Tunnel tunnel;
	public final CubeIntake cubeIntake;
	public final Wrist wrist;
	public final Claw claw;
	
	public Superstructure() {
		coordinator = SuperstructureCoordinator.getInstance();
		swerve = Swerve.getInstance();
		verticalElevator = VerticalElevator.getInstance();
		horizontalElevator = HorizontalElevator.getInstance();
		shoulder = Shoulder.getInstance();
		wrist = Wrist.getInstance();
		cubeIntake = CubeIntake.getInstance();
		tunnel = Tunnel.getInstance();
		claw = Claw.getInstance();
		
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
		Translation2d wristTipPosition = SuperstructureCoordinator.getInstance().getPosition().getWristTipPosition();
		SmartDashboard.putNumberArray("Wrist Tip Position", new double[]{wristTipPosition.x(), wristTipPosition.y()});
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

	private Request choreographyRequest(ChoreographyProvider choreoProvider) {
		return new Request() {
			private Request choreography = new EmptyRequest();

			@Override
			public void act() {
				choreography = choreoProvider.generateChoreography();
				choreography.act();
			}

			@Override
			public boolean isFinished() {
				return choreography.isFinished();
			}
		};
	}

	private Prerequisite allSubsystemsOnTargetPrerequisite() {
		return new Prerequisite() {
			@Override
			public boolean met() {
				return verticalElevator.isOnTarget() &&
						horizontalElevator.isOnTarget() &&
						shoulder.isOnTarget() &&
						wrist.isOnTarget();
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

	public void intakeState(Tunnel.State tunnelState) {
		request(new ParallelRequest(
			verticalElevator.heightRequest(1),
			tunnel.stateRequest(tunnelState),
			cubeIntake.stateRequest(CubeIntake.State.INTAKE)
		));
	}
	public void postIntakeState() {
		request(new ParallelRequest(
			//tunnel.stateRequest(Tunnel.State.OFF),
			cubeIntake.stateRequest(CubeIntake.State.STOWED)
		));
	}

	public void neutralState() {
		request(new ParallelRequest(
			new LambdaRequest(()-> {
				verticalElevator.stop();
				horizontalElevator.stop();
				wrist.stop();
			})
		));
	}

	public void coneMidScoringSequence(Pose2d scoringPose) {
		request(new SequentialRequest(
			new ParallelRequest(
				swerve.visionPIDRequest(scoringPose, scoringPose.getRotation()),
				choreographyRequest(coordinator::getConeMidScoringChoreography)
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 24.0),
				new LambdaRequest(() -> claw.conformToState(Claw.ControlState.CONE_OUTAKE))
						.withPrerequisites(() -> swerve.getDistanceToTargetPosition() < 1.0,
								allSubsystemsOnTargetPrerequisite())
			),
			waitRequest(1.0),
			choreographyRequest(coordinator::getConeStowChoreography)
		));
	}

	public void coneHighScoringSequence(Pose2d scoringPose) {
		request(new SequentialRequest(
			new ParallelRequest(
				swerve.visionPIDRequest(scoringPose, scoringPose.getRotation()),
				choreographyRequest(coordinator::getConeHighScoringChoreography)
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 24.0)
			),
			new LambdaRequest(() -> claw.conformToState(Claw.ControlState.CONE_OUTAKE)),
			waitRequest(1.0),
			choreographyRequest(coordinator::getConeStowChoreography)
		));
	}
		
}
