package com.team1323.frc2023.subsystems.superstructure;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Row;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.requests.EmptyRequest;
import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.ParallelRequest;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.requests.RequestExecuter;
import com.team1323.frc2023.requests.SequentialRequest;
import com.team1323.frc2023.requests.WaitRequest;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.VisionPIDController;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

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

	private final RequestExecuter requestExecuter = new RequestExecuter();
	
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
	}

	public boolean coneIntakingSequence = false;

	public synchronized void request(Request r) {
		requestExecuter.request(r);
	}

	public boolean areAllRequestsCompleted() { 
		return requestExecuter.isFinished();
	}

	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			/*InterpolatingDouble elevatorHeight = new InterpolatingDouble(verticalElevator.getPosition());
			InterpolatingDouble maxSwerveSpeed = Constants.kElevatorHeightToSwerveSpeedMap.getInterpolated(elevatorHeight);
			swerve.setMaxSpeed(maxSwerveSpeed.value);*/

			synchronized (Superstructure.this) {
				requestExecuter.update();
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

			@Override
			public String toString() {
				return String.format("ChoreographyRequest(choreo = %s)", choreography);
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
			verticalElevator.heightRequest(1.0),
			tunnel.stateRequest(tunnelState),
			cubeIntake.stateRequest(CubeIntake.State.INTAKE)
		));
	}
	public void communityIntakeState() {
		request(new ParallelRequest(
			new LambdaRequest(() -> {
				cubeIntake.conformToState(CubeIntake.State.INTAKE);
				tunnel.setState(Tunnel.State.COMMUNITY);
				verticalElevator.setPosition(1.66);
			})
		));
	}
	public void postIntakeState(double waitTime) {
		request(getPostIntakeState(waitTime));
	}

	public Request getPostIntakeState(double waitTime) {
		return new ParallelRequest(
			tunnel.queueShutdownRequest(),
			cubeIntake.stateRequest(CubeIntake.State.STOWED),
			new SequentialRequest(
				new WaitRequest(waitTime),
				verticalElevator.heightRequest(0.25)
			)
		);
	}


	public void reverseSubsystemsState() {
		request(new ParallelRequest(
			tunnel.stateRequest(Tunnel.State.REVERSE),
			new LambdaRequest(() -> {
				cubeIntake.setIntakeSpeed(-0.5);
			})
		));
	}


	public void neutralState() {
		request(new ParallelRequest(
			new LambdaRequest(()-> {
				verticalElevator.stop();
				horizontalElevator.stop();
				wrist.stop();
				tunnel.stop();
				cubeIntake.stop();
				claw.stop();
			})
		));
	}

	public void coneIntakeSequence() {
		request(getConeIntakeSequence());
	}
	public Request getConeIntakeSequence() {
		return new SequentialRequest(
			SuperstructureCoordinator.getInstance().getConeIntakeChoreography(),
			claw.stateRequest(Claw.ControlState.CONE_INTAKE),
			new ParallelRequest(
				choreographyRequest(this::objectAwareStow),
				new LambdaRequest(() -> {coneIntakingSequence = true;})
			).withPrerequisite(() -> (claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone)),
			new LambdaRequest(() -> {coneIntakingSequence = false;})
		).withCleanup(() -> {
			coneIntakingSequence = false;
			System.out.println("Cone Intake Sequence Reset");
		}
		);
	}

	public void coneIntakeWithoutScanSequence() {
		request(new SequentialRequest(
			new ParallelRequest(
				SuperstructureCoordinator.getInstance().getConeIntakeChoreography(),
				claw.stateRequest(Claw.ControlState.CONE_INTAKE)
			),
			choreographyRequest(this::objectAwareStow).withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone)
		));
	}

	public void coneStowSequence() {
		request(getConeStowSequence());
	}

	public Request objectAwareStow() {
		if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
			return new ParallelRequest(
				coordinator.getConeStowChoreography(),
				//coordinator.getConePreScoreChoreography(), 			
				new LambdaRequest(() -> {coneIntakingSequence = false;})
			);
		} else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube) {
			return coordinator.getCubeStowChoreography();
		} else {
			return coordinator.getFullStowChoreography(true);
		}
	}

	public void objectAwareStowSequence() {
		request(objectAwareStow());
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
			new WaitRequest(0.375),
			new LambdaRequest(() -> ScoringPoses.updateConeLateralOffset()),
			new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL)),
			choreographyRequest(this::conditionalConeStow)
		);
	}

	private Request getHandOffCubeSequence() {
		return new SequentialRequest(
			new LambdaRequest(() -> System.out.println("Starting cube handoff sequence")),
			claw.stateRequest(Claw.ControlState.CUBE_INTAKE),
			new LambdaRequest(() -> System.out.println("Set claw state to cube intake")),
			choreographyRequest(coordinator::getCubeIntakeChoreography),
			new LambdaRequest(() -> System.out.println("Finished cube intake choreography")),
			tunnel.stateRequest(Tunnel.State.SPIT_HANDOFF).withPrerequisite(() -> (Math.abs(claw.getRPM()) > 2500 || 
						claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube)),
			new LambdaRequest(() -> System.out.println("Set tunnel state to eject one"))
		);
	}

	public void handOffCubeState(ChoreographyProvider stowChoreo) {
		request(getHandOffCubeState(stowChoreo));
	}

	public Request getHandOffCubeState(ChoreographyProvider stowChoreo) {
		return new SequentialRequest(	
			getHandOffCubeSequence(),
			tunnel.stateRequest(Tunnel.State.SPIT_HANDOFF),
			new ParallelRequest(
				tunnel.stateRequest(Tunnel.State.OFF),
				choreographyRequest(stowChoreo)
			).withPrerequisite(
				() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube
			)
		);
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
			new WaitRequest(2.0),
			choreographyRequest(coordinator::getConeStowChoreography)
		));
	}
	public void manualShelfSequence() {
		request(new SequentialRequest(
			claw.stateRequest(Claw.ControlState.CONE_INTAKE),
			choreographyRequest(coordinator::getShelfChoreography)
			//choreographyRequest(this::objectAwareStow).withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone)
		));
	}

	public void shuttleIntakeSequence(boolean automaticStow) {
		request(getShuttleIntake(automaticStow));
	}

	public Request getShuttleIntake(boolean automaticStow) {
		Request stow = automaticStow ? 
			choreographyRequest(this::objectAwareStow).withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) :
			new EmptyRequest();
		Request shuttle = automaticStow ?
			coordinator.getAutoStowShuttleChoreography() : coordinator.getManualStowShuttleChoreography();
		return (new SequentialRequest(
			new ParallelRequest(
				shuttle,
				claw.stateRequest(Claw.ControlState.CONE_INTAKE)
			),
			stow
		));
	}
	/*public void flipGroundCone() {
		request(new SequentialRequest(
			coordinator.getConeIntakeChoreography(),
			 
		));
	}*/

	private void scoringSequence(Pose2d scoringPose, ChoreographyProvider scoringChoreo, 
			Claw.ControlState clawScoringState, VisionPIDController controller, boolean useTrajectory, boolean useRetro) {
		Request switchToRetroRequest = useRetro ? 
				new LambdaRequest(() -> LimelightProcessor.getInstance().setPipeline(Pipeline.RETRO))
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 18.0) :
				new EmptyRequest();
		request(new SequentialRequest(
			new ParallelRequest(
				swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), useTrajectory, true, controller),
				//choreographyRequest(coordinator::getConePreScoreChoreography),
				switchToRetroRequest,
				choreographyRequest(scoringChoreo)
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 12.0 || swerve.isVisionPIDDone())
			),
			new LambdaRequest(() -> {
				if (swerve.getState() == Swerve.ControlState.VISION_PID) {
					claw.conformToState(clawScoringState);
				}
			}),
			new LambdaRequest(() -> swerve.stop()),
			new LambdaRequest(() -> swerve.resetVisionPID()),
			choreographyRequest(this::objectAwareStow).withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.None)
		));
	}

	private void cubeScoringSequence(Pose2d scoringPose, ChoreographyProvider scoringChoreo,
			double horizontalExtension, double preemptiveScoreSeconds, VisionPIDController controller, 
			boolean useTrajectory, ChoreographyProvider stowingChoreo, boolean stopSwerveWhenDone) {
		request(getCubeScoringSequence(scoringPose, scoringChoreo, horizontalExtension, preemptiveScoreSeconds, 
				controller, useTrajectory, stowingChoreo, stopSwerveWhenDone));
	}

	private Request getCubeScoringSequence(Pose2d scoringPose, ChoreographyProvider scoringChoreo,
			double horizontalExtension, double preemptiveScoreSeconds, VisionPIDController controller, 
			boolean useTrajectory, ChoreographyProvider stowingChoreo, boolean stopSwerveWhenDone) {
		return new SequentialRequest(
			new LambdaRequest(() -> System.out.println("Started cube scoring sequence")),
			new ParallelRequest(
				swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), useTrajectory, false, controller),
				choreographyRequest(scoringChoreo)
						.withPrerequisite(() -> swerve.getDistanceToTargetPosition() < 12.0),
				new SequentialRequest(
					new LambdaRequest(() -> claw.conformToState(Claw.ControlState.CUBE_OUTAKE)),
					new WaitRequest(0.25),
					wrist.angleRequest(Constants.Wrist.kMaxControlAngle)
				).withPrerequisites(horizontalElevator.willReachPositionWithinTime(horizontalExtension, preemptiveScoreSeconds),
						() -> swerve.getDistanceToTargetPosition() < 4.0 || swerve.isVisionPIDDone())
			),
			new LambdaRequest(() -> {
				if (stopSwerveWhenDone) {
					swerve.stop();
				}
			}),
			new LambdaRequest(() -> swerve.resetVisionPID()),
			new LambdaRequest(() -> {
				System.out.println("Cube Ejected");
			}),
			choreographyRequest(stowingChoreo).withPrerequisite(() -> claw.getCurrentHoldingObject() == Claw.HoldingObject.None),
			new LambdaRequest(() -> {
				System.out.println("Superstructure Stowed");
			})
		);
	}

	private Request getSuperstructureScorePositions(ChoreographyProvider scoringChoreo, 
			Claw.ControlState clawScoringState, ChoreographyProvider stowingChoreo) {
		return new SequentialRequest(
			choreographyRequest(scoringChoreo),
			new LambdaRequest(() -> claw.conformToState(clawScoringState)),
			new WaitRequest(1.0),
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
		VisionPIDController visionPIDController = new VisionPIDBuilder().build();
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
				visionPIDController = new VisionPIDBuilder()
						.withTolerance(2.0)
						.withOnTargetTime(0.1)
						.build();
				scoringChoreo = coordinator::getConeMidScoringChoreography;
			} else if (nodeLocation.row == Row.TOP) {
				visionPIDController = new VisionPIDBuilder()
						.withOnTargetTime(0.25)
						.build();
				scoringChoreo = coordinator::getConeHighScoringChoreography;
			}
		}

		scoringSequence(scoringPose, scoringChoreo, clawScoringState, visionPIDController, useTrajectory, useRetro);
		//request(swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), useTrajectory));
	}

	public void coneMidScoringSequence(Pose2d scoringPose) {
		VisionPIDController controller = new VisionPIDBuilder()
				.withTolerance(2.0)
				.withOnTargetTime(0.1)
				.build();
		scoringSequence(scoringPose, coordinator::getConeMidScoringChoreography, 
				Claw.ControlState.CONE_OUTAKE, controller, false, true);
	}

	public void coneHighScoringSequence(Pose2d scoringPose) {
		scoringSequence(scoringPose, coordinator::getConeHighScoringChoreography,
				Claw.ControlState.CONE_OUTAKE, new VisionPIDBuilder().withOnTargetTime(0.25).build(), false, true);
	}

	public void cubeLowScoringSequence(Pose2d scoringPose) {
		request(new SequentialRequest(
			swerve.visionPIDRequest(scoringPose, scoringPose.getRotation(), false, true),
			tunnel.stateRequest(Tunnel.State.EJECT_ONE),
			new WaitRequest(1.0),
			new LambdaRequest(() -> swerve.stop()),
			new LambdaRequest(() -> swerve.resetVisionPID())
		));
	}

	public void cubeMidScoringSequence(Pose2d scoringPose, boolean stopSwerveWhenDone) {
		cubeScoringSequence(scoringPose, coordinator::getCubeMidScoringChoreography,
				SuperstructureCoordinator.kCubeMidScoringHorizontalExtension, 0.1, new VisionPIDBuilder().build(),
				false, this::objectAwareStow, stopSwerveWhenDone);
	}

	public void cubeHighScoringSequence(Pose2d scoringPose, VisionPIDController controller, boolean stopSwerveWhenDone) {
		cubeScoringSequence(scoringPose, coordinator::getCubeHighScoringChoreography,
				SuperstructureCoordinator.kCubeHighScoringHorizontalExtension, 0.0, controller, 
				false, this::objectAwareStow, stopSwerveWhenDone);
	}

	public void tripleCubeScoringSequence() {
		Request intakeCube = (claw.getCurrentHoldingObject() == Claw.HoldingObject.None) ? getHandOffCubeSequence() : new EmptyRequest();
		if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
			return;
		}
		request(new SequentialRequest(
			new LambdaRequest(() -> System.out.println("Starting triple cube score sequence")),
			intakeCube,
			new LambdaRequest(() -> System.out.println("Finished intaking cube")),
			new ParallelRequest(
				// tunnel.stateRequest(Tunnel.State.OFF),
				getCubeScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()), coordinator::getCubeMidScoringChoreography,
						SuperstructureCoordinator.kCubeMidScoringHorizontalExtension, 0.625, new VisionPIDBuilder().build(),
						false, coordinator::getHalfCubeStowChoreography, true)
			).withPrerequisite(() -> claw.getCurrentHoldingObject() == HoldingObject.Cube),
			new LambdaRequest(() -> System.out.println("Finished scoring mid cube")),
			getHandOffCubeSequence(),
			new LambdaRequest(() -> System.out.println("Finished handoff sequence for second cube")),
			new ParallelRequest(
				new LambdaRequest(() -> System.out.println("Entered parallel request for scoring high cube")),
				getCubeScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()), coordinator::getCubeHighScoringChoreography,
						SuperstructureCoordinator.kCubeHighScoringHorizontalExtension, 0.125, new VisionPIDBuilder().build(),
						false, this::objectAwareStow, true),
				tunnel.stateRequest(Tunnel.State.SPIT)
						.withPrerequisite(() -> shoulder.getPosition() > 0.0)
			).withPrerequisite(() -> claw.getCurrentHoldingObject() == HoldingObject.Cube)
		));
	}

	public void coneHighScoreManual() {
		request(new SequentialRequest(
			coordinator.getConeHighScoringChoreography(),
			claw.stateRequest(Claw.ControlState.CONE_OUTAKE),
			new WaitRequest(0.125)
		));
	}

	public void coneMidScoreManual() {
		request(new SequentialRequest(
			coordinator.getConeMidScoringChoreography(),
			claw.stateRequest(Claw.ControlState.CONE_OUTAKE)
		));
	}
}
