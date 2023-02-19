package com.team1323.frc2023.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
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

	public boolean coneIntakingSequence = false;

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
		request(new EmptyRequest());
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
			tunnel.queueShutdownRequest(),
			cubeIntake.stateRequest(CubeIntake.State.STOWED)
		));
	}
	public void postCubeIntakeState() {
		request(new ParallelRequest(
			tunnel.stateRequest(Tunnel.State.SPIT),
			cubeIntake.stateRequest(CubeIntake.State.STOWED),
			new LambdaRequest(() -> {
				if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None) {
					claw.conformToState(Claw.ControlState.OFF);
				}
			})
		));
	}

	public void reverseSubsystemsState() {

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

	public void coneIntakeSequence() {
		request(new SequentialRequest(
			SuperstructureCoordinator.getInstance().getConeIntakeChoreography(),
			claw.stateRequest(Claw.ControlState.CONE_INTAKE),
			new ParallelRequest(
				getConeStowSequence(),
				new LambdaRequest(() -> {coneIntakingSequence = true;})
			).withPrerequisite(() -> (claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone)),
			new LambdaRequest(() -> {coneIntakingSequence = false;})
		));
	}

	public void coneStowSequence() {
		request(getConeStowSequence());
	}

	public Request objectAwareStow() {
		if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
			return coordinator.getConeStowChoreography();
		} else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube) {
			return coordinator.getCubeStowChoreography();
		} else  {
			return coordinator.getFullStowChoreography();
		}
	}

	private Request conditionalConeStow() {
		if (AllianceChooser.getCommunityBoundingBox().pointWithinBox(swerve.getPose().getTranslation())) {
			return coordinator.getCommunityConeHoldChoreography();
		}

		return coordinator.getConeStowChoreography();
	}

	private Request getConeStowSequence() {
		return new SequentialRequest(
			choreographyRequest(coordinator::getConeScanChoreography),
			new LambdaRequest(() -> LimelightProcessor.getInstance().clearConeOffsetBuffer()),
			new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.CONE)),
			waitRequest(1.0),
			new LambdaRequest(() -> ScoringPoses.updateConeLateralOffset()),
			new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL)),
			choreographyRequest(this::conditionalConeStow)
		);
	}

	private Request getHandOffCubeSequence() {
		return new SequentialRequest(
			SuperstructureCoordinator.getInstance().getCubeIntakeChoreography(),
			new ParallelRequest(
				tunnel.stateRequest(Tunnel.State.EJECT_ONE),
				claw.stateRequest(Claw.ControlState.CUBE_INTAKE)
			)
		);
	}
	public void handOffCubeState() {
		request(getHandOffCubeSequence());
	}

	public void shelfSequence(boolean left) {
		Pose2d shelfPose = ScoringPoses.getShelfPose(left);
		swerve.startVisionPID(shelfPose, shelfPose.getRotation(), false);

		request(new SequentialRequest(
			choreographyRequest(coordinator::getShelfChoreography)
					.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 24.0),
			claw.stateRequest(Claw.ControlState.CONE_INTAKE),
			swerve.startRobotCentricTrajectoryRequest(new Translation2d(-36.0, 0.0), shelfPose.getRotation(), 24.0)
					.withPrerequisite(() -> claw.getCurrentHoldingObject() == HoldingObject.Cone),
			new LambdaRequest(() -> swerve.resetVisionPID()),
			waitRequest(2.0),
			choreographyRequest(coordinator::getConeStowChoreography)
		));
	}

	public void shuttleIntakeSequence() {
		request(new SequentialRequest(
			coordinator.getShuttleChoreography(),
			claw.stateRequest(Claw.ControlState.CONE_INTAKE),
			getConeStowSequence().withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone)
		));
	}

	/*public void flipGroundCone() {
		request(new SequentialRequest(
			coordinator.getConeIntakeChoreography(),
			 
		));
	}*/

	private void scoringSequence(Pose2d scoringPose, ChoreographyProvider scoringChoreo, 
			Claw.ControlState clawScoringState, boolean useTrajectory, boolean useRetro) {
		Request switchToRetroRequest = useRetro ? 
				new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.RETRO))
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 18.0) :
				new EmptyRequest();

		request(new SequentialRequest(
			new ParallelRequest(
				swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), useTrajectory),
				switchToRetroRequest,
				choreographyRequest(scoringChoreo)
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 12.0)
			),
			new LambdaRequest(() -> claw.conformToState(clawScoringState)),
			new LambdaRequest(() -> swerve.stop()),
			new LambdaRequest(() -> swerve.resetVisionPID()),
			waitRequest(1.0),
			choreographyRequest(this::objectAwareStow)
		));
	}


	private Request getSuperstructureScorePositions(ChoreographyProvider scoringChoreo, 
			Claw.ControlState clawScoringState, ChoreographyProvider stowingChoreo) {
		return new SequentialRequest(
			choreographyRequest(scoringChoreo),
			new LambdaRequest(() -> claw.conformToState(clawScoringState)),
			waitRequest(1.0),
			choreographyRequest(stowingChoreo)
		);
	}
	public void setSuperstructureScorePositions(ChoreographyProvider scoringChoreo, 
			Claw.ControlState clawScoringState, ChoreographyProvider stowingChoreo) {
		request(getSuperstructureScorePositions(scoringChoreo, clawScoringState, stowingChoreo));
	}
	

	public void intakeCubeAndScore(ChoreographyProvider scoringChoreo,
			Claw.ControlState clawScoringState, ChoreographyProvider stowingChoreo) {
			request(new SequentialRequest(
				getHandOffCubeSequence(),
				getSuperstructureScorePositions(scoringChoreo, clawScoringState, stowingChoreo).
						withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube)
			));

	}
	
	public void scoringSequence(NodeLocation nodeLocation) {
		if (nodeLocation.row == Row.BOTTOM) {
			return;
		}

		Pose2d scoringPose = ScoringPoses.getScoringPose(nodeLocation);
		ChoreographyProvider scoringChoreo = coordinator::getConeStowChoreography;
		Claw.ControlState clawScoringState;
		boolean useTrajectory = true;
		boolean useRetro = true;
		
		if (Math.abs(swerve.getPose().getTranslation().y() - scoringPose.getTranslation().y()) < 36.0 &&
				nodeLocation.isEdgeColumn()) {
			useTrajectory = false;
		}

		if (nodeLocation.column == Column.CENTER) {
			clawScoringState = Claw.ControlState.CUBE_OUTAKE;
			useRetro = false;

			if (nodeLocation.row == Row.MIDDLE) {
				scoringChoreo = coordinator::getCubeMidScoringChoreography;
			} else if (nodeLocation.row == Row.TOP) {
				scoringChoreo = coordinator::getCubeHighScoringChoreography;
			}
		} else {
			clawScoringState = Claw.ControlState.CONE_OUTAKE;

			if (nodeLocation.row == Row.MIDDLE) {
				scoringChoreo = coordinator::getConeMidScoringChoreography;
			} else if (nodeLocation.row == Row.TOP) {
				scoringChoreo = coordinator::getConeHighScoringChoreography;
			}
		}

		scoringSequence(scoringPose, scoringChoreo, clawScoringState, useTrajectory, useRetro);
		//request(swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), useTrajectory));
	}

	public void coneMidScoringSequence(Pose2d scoringPose) {
		scoringSequence(scoringPose, coordinator::getConeMidScoringChoreography, 
				Claw.ControlState.CONE_OUTAKE, false, true);
	}

	public void coneHighScoringSequence(Pose2d scoringPose) {
		scoringSequence(scoringPose, coordinator::getConeHighScoringChoreography,
				Claw.ControlState.CONE_OUTAKE, false, true);
	}

	public void cubeLowScoringSequence(Pose2d scoringPose) {
		request(new SequentialRequest(
			swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), false),
			tunnel.stateRequest(Tunnel.State.EJECT_ONE),
			waitRequest(1.0),
			new LambdaRequest(() -> swerve.stop()),
			new LambdaRequest(() -> swerve.resetVisionPID())
		));
	}

	public void cubeMidScoringSequence(Pose2d scoringPose) {
		scoringSequence(scoringPose, coordinator::getCubeMidScoringChoreography,
				Claw.ControlState.CUBE_OUTAKE, false, false);
	}

	public void cubeHighScoringSequence(Pose2d scoringPose) {
		scoringSequence(scoringPose, coordinator::getCubeHighScoringChoreography,
				Claw.ControlState.CUBE_OUTAKE, false, false);
	}
}
