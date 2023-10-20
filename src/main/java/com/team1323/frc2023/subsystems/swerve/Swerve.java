package com.team1323.frc2023.subsystems.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.DriveMotionPlanner;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.gyros.Gyro;
import com.team1323.frc2023.subsystems.gyros.Pigeon2IMU;
import com.team1323.frc2023.subsystems.swerve.SwerveModule.SwerveMotorInfo;
import com.team1323.frc2023.vision.VisionPIDController;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.Units;
import com.team1323.lib.math.vectors.VectorField;
import com.team1323.lib.util.LogUtil;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.SwerveHeadingController;
import com.team1323.lib.util.SwerveInverseKinematics;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.wpilib.SwerveDriveKinematics;
import com.wpilib.SwerveDrivePoseEstimator;
import com.wpilib.SwerveModulePosition;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class Swerve extends Subsystem{
	//Instance declaration
	private static Swerve instance = null;
	public static Swerve getInstance(){
		if(instance == null)
		instance = new Swerve();
		return instance;
	}
	
	//Module declaration
	public SwerveModule frontRight, frontLeft, rearLeft, rearRight;
	private final List<SwerveModule> modules;
	
	//Evade maneuver variables
	Translation2d clockwiseCenter = new Translation2d();
	Translation2d counterClockwiseCenter = new Translation2d();
	boolean evading = false;
	boolean evadingToggled = false;
	public void toggleEvade(){
		evading = !evading;
		evadingToggled = true;
	}
	public void toggleEvade(boolean enabled) {
		evading = enabled;
		evadingToggled = true;
		startEvadeRevolve(Translation2d.fromPolar(Rotation2d.fromDegrees(45), 5).rotateBy(getHeading().inverse()));
	}
	
	//Heading controller methods
	Gyro pigeon;
	SwerveHeadingController headingController = new SwerveHeadingController();
	public void temporarilyDisableHeadingController(){
		headingController.temporarilyDisable();
	}
	public Rotation2d getTargetHeading(){
		return headingController.getTargetHeading();
	}
	
	//Vision dependencies
	Rotation2d visionTargetHeading = new Rotation2d();
	VisionPIDController visionPID = new VisionPIDBuilder().build();
	public boolean isTracking(){
		return currentState == ControlState.VISION_PID;
	}
	
	boolean needsToNotifyDrivers = false;
	public boolean needsToNotifyDrivers(){
		if(needsToNotifyDrivers){
			needsToNotifyDrivers = false;
			return true;
		}		
		return false;
	}

	BalanceController balanceController = new NonlinearBalanceController();
	
	TrajectoryGenerator generator;
	
	//Odometry variables
	Pose2d pose;
	double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;
	public Pose2d getPose(){
		return pose;
	}
	
	// WPILib odometry
	SwerveDrivePoseEstimator poseEstimator;
	private boolean isPoseEstimatorInitialized = false;
	
	// Module configuration variables (for beginnning of auto)
	boolean modulesReady = false;
	boolean alwaysConfigureModules = false;
	boolean moduleConfigRequested = false;
	public void requireModuleConfiguration(){
		modulesReady = false;
	}
	public void alwaysConfigureModules(){
		alwaysConfigureModules = true;
	}
	Pose2d startingPose = Constants.kRobotStartingPose;
	public void setStartingPose(Pose2d newPose){
		startingPose = newPose;
	}
	
	//Trajectory variables
	DriveMotionPlanner motionPlanner;
	public double getRemainingProgress(){
		if(motionPlanner != null && getState() == ControlState.TRAJECTORY){
			return motionPlanner.getIterator().getRemainingProgress() / (motionPlanner.getIterator().getProgress() + motionPlanner.getIterator().getRemainingProgress());
		}
		return 0.0;
	}
	double rotationScalar;
	public void setRotationScalar(double scalar) {
		rotationScalar = scalar;
	}
	double trajectoryStartTime = 0;
	Translation2d lastTrajectoryVector = new Translation2d();
	public Translation2d getLastTrajectoryVector(){ return lastTrajectoryVector; }
	boolean hasStartedFollowing = false;
	boolean hasFinishedPath = false;
	public boolean hasFinishedPath(){
		return hasFinishedPath;
	}
	
	//Experimental
	VectorField vf;

	private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
	
	private Swerve(){
		/**
		 * 			   	  Front
		 *      _________________________
		 * 	   | M1 |               | M0 |
		 * 	   |____|               |____|
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |____                _____|
		 * 	   | M2 |               | M3 |
		 *     |____|_______________|____|
		 * 		
		 */
		frontRight = new CoaxialSwerveModule(0, Constants.kFrontRightEncoderStartingPos, false, 
				new SwerveMotorInfo(Ports.FRONT_RIGHT_ROTATION, TalonFXInvertType.Clockwise),
				new SwerveMotorInfo(Ports.FRONT_RIGHT_DRIVE, TalonFXInvertType.CounterClockwise));
		frontLeft = new CoaxialSwerveModule(1, Constants.kFrontLeftEncoderStartingPos, false, 
				new SwerveMotorInfo(Ports.FRONT_LEFT_ROTATION, TalonFXInvertType.Clockwise),
				new SwerveMotorInfo(Ports.FRONT_LEFT_DRIVE, TalonFXInvertType.Clockwise));
		rearLeft = new CoaxialSwerveModule(2, Constants.kRearLeftEncoderStartingPos, false, 
				new SwerveMotorInfo(Ports.REAR_LEFT_ROTATION, TalonFXInvertType.Clockwise),
				new SwerveMotorInfo(Ports.REAR_LEFT_DRIVE, TalonFXInvertType.Clockwise));
		rearRight = new CoaxialSwerveModule(3, Constants.kRearRightEncoderStartingPos, false, 
				new SwerveMotorInfo(Ports.REAR_RIGHT_ROTATION, TalonFXInvertType.Clockwise),
				new SwerveMotorInfo(Ports.REAR_RIGHT_DRIVE, TalonFXInvertType.CounterClockwise));
		
		modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		
		pigeon = Pigeon2IMU.createRealOrSimulatedGyro(Ports.PIGEON, Ports.CANBUS);
		
		motionPlanner = new DriveMotionPlanner();
		
		SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			Units.inchesToMeters(Constants.kVehicleToModuleZero),
			Units.inchesToMeters(Constants.kVehicleToModuleOne),
			Units.inchesToMeters(Constants.kVehicleToModuleTwo),
			Units.inchesToMeters(Constants.kVehicleToModuleThree)
		);

		poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.identity(), 
				getModulePositions(), Pose2d.identity(), VecBuilder.fill(0.1, 0.1, 0.1), 
				VecBuilder.fill(0.1, 0.1, 0.1));
		pose = poseEstimator.getEstimatedPosition();
		distanceTraveled = 0;
		
		generator = TrajectoryGenerator.getInstance();
	}

	public void setModuleNeutralModes(NeutralMode mode) {
		modules.forEach(m -> m.setNeutralMode(mode));
	}
	
	//Teleop driving variables
	private Translation2d translationalVector = new Translation2d();
	private double rotationalInput = 0;
	private Translation2d lastDriveVector = new Translation2d();
	private final Translation2d rotationalVector = Translation2d.identity();
	private double lowPowerScalar = 0.6; //0.6
	public void setLowPowerScalar(double scalar){
		lowPowerScalar = scalar;
	}
	private double maxSpeedFactor = 1.0;
	public void setMaxSpeed(double max){
		maxSpeedFactor = max;
	}
	private final SlewRateLimiter inputMagnitudeLimiter = new SlewRateLimiter(2.0);
	private boolean useSlewLimiter = false;//Settings.kIsUsingCompBot;
	public void useSlewLimiter(boolean use) {
		useSlewLimiter = use;
	}
	private boolean updateInputs = true;
	public void enableInputs(boolean enable) {
		updateInputs = enable;
	}
	private boolean robotCentric = false;
	private boolean translationStickReset = false;
	
	//Swerve kinematics (exists in a separate class)
	private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
	public void setCenterOfRotation(Translation2d center){
		inverseKinematics.setCenterOfRotation(center);
	}
	
	//The swerve's various control states
	public enum ControlState{
		NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED,
		TRAJECTORY, VELOCITY, VISION_PID, BALANCE_PID
	}
	private ControlState currentState = ControlState.NEUTRAL;
	public ControlState getState(){
		return currentState;
	}
	public void setState(ControlState newState){
		currentState = newState;
	}
	
	/**
	* Main function used to send manual input during teleop.
	* @param x forward/backward input
	* @param y left/right input
	* @param rotate rotational input
	* @param robotCentric gyro use
	* @param lowPower scaled down output
	*/
	public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower) {
		if (AllianceChooser.getAlliance() != Alliance.Blue && !robotCentric) {
			x = -x;
			y = -y;
		}

		if (updateInputs) {
			Translation2d translationalInput = new Translation2d(x, y);
			if (useSlewLimiter) {
				double limitedMagnitude = inputMagnitudeLimiter.calculate(translationalInput.norm());
				translationalInput = Translation2d.fromPolar(translationalInput.direction(), limitedMagnitude);
			} else {
				inputMagnitudeLimiter.reset(translationalInput.norm());
			}
			double inputMagnitude = translationalInput.norm();
			
			/* Snap the translational input to its nearest pole, if it is within a certain threshold 
			of it. */
			double threshold = Math.toRadians(10.0);
			if(Math.abs(translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold){
				translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
			}
			
			/* Scale x and y by applying a power to the magnitude of the vector they create, in order
			to make the controls less sensitive at the lower end. */
			double translationalDeadband = 0.025;
			inputMagnitude = Util.scaledDeadband(inputMagnitude, 1.0, translationalDeadband);
			final double power = (lowPower) ? 2.0 : 1.5;
			inputMagnitude = Math.pow(inputMagnitude, power);
			inputMagnitude = Util.deadBand(inputMagnitude, 0.05);
			translationalInput = Translation2d.fromPolar(translationalInput.direction(), inputMagnitude);
			
			double rotationalDeadband = 0.1;
			rotate = Util.scaledDeadband(rotate, 1.0, rotationalDeadband);
			rotate = Math.pow(Math.abs(rotate), 1.75)*Math.signum(rotate);
			
			translationalInput = translationalInput.scale(maxSpeedFactor);
			rotate *= maxSpeedFactor;
			
			translationalVector = translationalInput;
			
			if(lowPower){
				translationalVector = translationalVector.scale(lowPowerScalar);
				rotate *= lowPowerScalar;
			}else{
				rotate *= 0.8;
			}
			
			if(!Util.epsilonEquals(rotate, 0.0) /*&& Util.epsilonEquals(rotationalInput, 0.0)*/){
				headingController.disable();
			}else if(Util.epsilonEquals(rotate, 0.0) && !Util.epsilonEquals(rotationalInput, 0.0)){
				headingController.temporarilyDisable();
			}
			
			rotationalInput = rotate;
			
			if(Util.epsilonEquals(translationalInput.norm(), 0.0) && !translationStickReset) {
				translationStickReset = true;
			}

			if(!Util.epsilonEquals(translationalInput.norm(), 0.0)){
				if (headingController.getState() == SwerveHeadingController.State.Snap || 
					headingController.getState() == SwerveHeadingController.State.Stationary)
					headingController.setStabilizationTarget(headingController.getTargetHeading());

				if(isTracking() || currentState == ControlState.POSITION){
					/*if (Math.hypot(x, y) >= 0.5) {
						stop();
					}*/
					if(Math.hypot(x, y) >= 0.5 && translationStickReset) {
						translationStickReset = false;
						System.out.println("VisionPID stopped due to Translation breakout");
						stop();
					}
				} else if(currentState != ControlState.MANUAL){
					setState(ControlState.MANUAL);
					
				}
			}else if(!Util.epsilonEquals(rotationalInput, 0.0)){
				if(currentState != ControlState.MANUAL && currentState != ControlState.TRAJECTORY && currentState != ControlState.VISION_PID && currentState != ControlState.POSITION){
					setState(ControlState.MANUAL);
				}
			}
			
			if(inputMagnitude > 0.1)
			lastDriveVector = new Translation2d(x, y);
			else if(translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0){
				lastDriveVector = rotationalVector;
			}
			
			this.robotCentric = robotCentric;
		}
	}
	
	//Possible new control method for rotation
	public Rotation2d averagedDirection = Rotation2d.identity();
	public void resetAveragedDirection(){ averagedDirection = pose.getRotation(); }
	public void setAveragedDirection(double degrees){ averagedDirection = Rotation2d.fromDegrees(degrees); }
	public final double rotationDirectionThreshold = Math.toRadians(5.0);
	public final double rotationDivision = 1.0;
	
	public void updateControllerDirection(Translation2d input){
		if(Util.epsilonEquals(input.norm(), 1.0, 0.1)){
			Rotation2d direction = input.direction();
			double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
			averagedDirection = Rotation2d.fromDegrees(roundedDirection);
		}
	}
	
	// Various methods to control the heading controller
	public void rotate(Rotation2d goalHeading) {
		if (AllianceChooser.getAlliance() != Alliance.Blue) {
			goalHeading = goalHeading.rotateBy(Rotation2d.fromDegrees(180.0));
		}

		if (translationalVector.x() == 0 && translationalVector.y() == 0) {
			rotateInPlace(goalHeading);
		} else {
			headingController.setStabilizationTarget(goalHeading);
		}
	}
	
	private void rotateInPlace(Rotation2d goalHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(goalHeading);
	}
	
	public void setPathHeading(Rotation2d goalHeading){
		headingController.setSnapTarget(goalHeading);
	}

	public boolean isGoingToPole() {
		Rotation2d targetHeading = getTargetHeading();
		return targetHeading.equals(Rotation2d.fromDegrees(0.0)) || targetHeading.equals(Rotation2d.fromDegrees(180.0));
	}

	public double closestPole() {
		if (Math.abs(pose.getRotation().distance(Rotation2d.fromDegrees(180.0))) < 
			Math.abs(pose.getRotation().distance(Rotation2d.fromDegrees(0.0))))
			return 180.0;
		return 0.0;
	}
	
	/** Sets MotionMagic targets for the drive motors */
	public void setPositionTarget(double directionDegrees, double magnitudeInches){
		setState(ControlState.POSITION);
		modules.forEach(m -> m.setClosedLoopPosition(Translation2d.fromPolar(Rotation2d.fromDegrees(directionDegrees), magnitudeInches)));
	}

	public void zukLockDrivePosition() {
		setState(ControlState.POSITION);
		frontRight.setOpenLoop(Translation2d.fromPolar(Rotation2d.fromDegrees(45.0), 0.0));
		frontLeft.setOpenLoop(Translation2d.fromPolar(Rotation2d.fromDegrees(-45.0), 0.0));
		rearLeft.setOpenLoop(Translation2d.fromPolar(Rotation2d.fromDegrees(45.0), 0.0));
		rearRight.setOpenLoop(Translation2d.fromPolar(Rotation2d.fromDegrees(-45.0), 0.0));
	}
	
	public void setOpenLoop(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			modules.get(i).setOpenLoop(driveVectors.get(i));
		}
	}
	
	public void setOpenLoopCoast(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			modules.get(i).setOpenLoopCoast(driveVectors.get(i).direction());
		}
	}
	
	public void setClosedLoopVelocity(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			// TODO: Consider reworking the inverse kinematics so that they accept and return vectors in
			// inches/second, rather than normalized vectors.
			modules.get(i).setClosedLoopVelocity(driveVectors.get(i).scale(Constants.kSwerveMaxSpeedInchesPerSecond));
		}
	}

	public void setClosedLoopVelocityStall(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			modules.get(i).setClosedLoopVelocityStall(driveVectors.get(i).direction());
		}
	}

	public void setVelocityMode(Rotation2d direction, double inchesPerSecond) {
		setState(ControlState.VELOCITY);
		setClosedLoopVelocity(modules.stream()
				.map(m -> Translation2d.fromPolar(direction, inchesPerSecond / Constants.kSwerveMaxSpeedInchesPerSecond))
				.toList());
	}
	
	/**
	* @return Whether or not at least one module has reached its MotionMagic setpoint
	*/
	public boolean arePositionsOnTarget(){
		boolean onTarget = false;
		for(SwerveModule m : modules){
			onTarget |= m.isDrivePositionOnTarget();
		}
		return onTarget;
	}
	
	/**
	* @return Whether or not all modules have reached their angle setpoints
	*/
	public boolean areModuleAnglesOnTarget(){
		boolean onTarget = true;
		for(SwerveModule m : modules){
			onTarget &= m.isAngleOnTarget();
		}
		return onTarget;
	}

	public boolean areModulesStuck() {
		final double kDesiredVelocityThreshold = 0.1;
		final double kStuckThreshold = 2.0;
		boolean stuck = true;
		for(SwerveModule m : modules) {
			double desiredVelocity = Math.abs(m.getDriveVelocitySetpoint() / Constants.kMaxFalconEncoderSpeed);
			double expectedVelocity = Math.abs(m.getDriveVoltage() / 12.0);
			if (desiredVelocity == 0.0) {
				desiredVelocity = expectedVelocity;
			}
			stuck &= (desiredVelocity >= kDesiredVelocityThreshold) && (expectedVelocity > kStuckThreshold * desiredVelocity);
		}
		return stuck;
	}
	
	/**
	* Sets a trajectory for the robot to follow
	* @param trajectory 
	* @param targetHeading Heading that the robot will rotate to during its path following
	* @param rotationScalar Scalar to increase or decrease the robot's rotation speed
	* @param followingCenter The point (relative to the robot) that will follow the trajectory
	*/
	public void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d targetHeading,
	double rotationScalar, Translation2d followingCenter){
		hasStartedFollowing = false;
		hasFinishedPath = false;
		moduleConfigRequested = false;
		motionPlanner.reset();
		motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
		motionPlanner.setFollowingCenter(followingCenter);
		inverseKinematics.setCenterOfRotation(followingCenter);
		setPathHeading(targetHeading);
		this.rotationScalar = rotationScalar;
		trajectoryStartTime = Timer.getFPGATimestamp();
		setState(ControlState.TRAJECTORY);
	}
	
	public void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d targetHeading,
	double rotationScalar){
		setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
	}
	
	public void setRobotCentricTrajectory(Translation2d relativeEndPos, Rotation2d targetHeading){
		setRobotCentricTrajectory(relativeEndPos, targetHeading, 45.0);
	}
	
	public void setRobotCentricTrajectory(Translation2d relativeEndPos, Rotation2d targetHeading, double defaultVel){
		modulesReady = true;
		Translation2d endPos = pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation();
		Rotation2d startHeading = endPos.translateBy(pose.getTranslation().inverse()).direction();
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(pose.getTranslation(), startHeading));	
		waypoints.add(new Pose2d(pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation(), startHeading));
		Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
		setTrajectory(trajectory, targetHeading, 1.0);
	}

	public void setFieldCentricTrajectory(Translation2d relativeEndPos, Rotation2d targetHeading, double defaultVel) {
		modulesReady = true;
		Translation2d endPos = pose.getTranslation().translateBy(relativeEndPos);
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(pose.getTranslation(), relativeEndPos.direction()));
		waypoints.add(new Pose2d(endPos, relativeEndPos.direction()));
		Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
		setTrajectory(trajectory, targetHeading, 1.0);
	}
	
	// Vision PID (new, simpler vision tracking system)
	public void startVisionPID(Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useTrajectory) {
		startVisionPID(desiredFieldPose, approachAngle, useTrajectory, new VisionPIDBuilder().build());
	}

	public void startVisionPID(Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useTrajectory, VisionPIDController controller) {
		visionPID = controller;
		visionPID.start(pose, desiredFieldPose, approachAngle, useTrajectory, false);
		rotationScalar = 0.75;
		translationStickReset = false;
		setState(ControlState.VISION_PID);
	}

	public Translation2d getVisionPIDTarget() {
		return visionPID.getTargetPosition();
	}

	public void setVisionPIDTarget(Translation2d targetPosition) {
		visionPID.setTargetPosition(targetPosition);
	}

	public void resetVisionPID() {
		visionPID.resetDistanceToTargetPosition();
	}

	public double getDistanceToTargetPosition() {
		return visionPID.getDistanceToTargetPosition();
	}

	public boolean isVisionPIDDone() {
		return getState() != ControlState.VISION_PID || visionPID.isDone();
	}

	public void addRetroObservation(Translation2d retroPosition, double timestamp) {
		visionPID.addRetroObservation(retroPosition, timestamp);
	}

	public void startBalancePID() {
		setState(ControlState.BALANCE_PID);
		balanceController.start(Rotation2d.fromDegrees(0));
	}
	public boolean balancePIDOnTarget() {
		return balanceController.isOnTarget();
	}
	
	/****************************************************/
	/* Vector Fields */
	public void setVectorField(VectorField vf_) {
		vf = vf_;
		setState(ControlState.VECTORIZED);
	}
	
	/** Determines which wheels the robot should rotate about in order to perform an evasive maneuver */
	int evadeClosestIndex = 0;
	public void determineEvasionWheels(){
		List<Translation2d> modulePositions = Constants.kModulePositions;
		int evadeClosestIndex = 0;
		for(int i = 0; i < modulePositions.size(); i++) {
			Translation2d currentModuleVector = translationalVector.rotateBy(pose.getRotation().inverse());
			if(modulePositions.get(i).distance(currentModuleVector) < modulePositions.get(evadeClosestIndex).distance(currentModuleVector)) {
				evadeClosestIndex = i;
			}
		}
		Translation2d targetModule = modulePositions.get(evadeClosestIndex).translateBy(Translation2d.fromPolar(modulePositions.get(evadeClosestIndex).direction(), 7.07));
		counterClockwiseCenter = targetModule;
		clockwiseCenter = targetModule;
	}



	double startingRotationValue = 0;
	final double rotationAmount = 135;
	int currentModuleIndex = 0;
	int totalModulesPivoted = 1;
	public void startEvadeRevolve(Translation2d revolveAround) {
		startingRotationValue = inputs.gyroYaw - 45;
		currentModuleIndex = 0;
		totalModulesPivoted = 1;
		for(int i = 0; i < Constants.kModulePositions.size(); i++) {
			if(Constants.kModulePositions.get(currentModuleIndex).distance(revolveAround) > Constants.kModulePositions.get(i).distance(revolveAround)) {
				currentModuleIndex = i;
			}
		}
		
	}
	
	
	public void updateEvadeRevolve() {
		// TODO: Uncomment this when the swerve's velocity reading is fixed to use SwerveModuleState instead of SwerveModulePosition
		/*
		double predictedRotation = inputs.gyroYaw + (velocity.dtheta * 180/Math.PI) * 0.1;
		double rotationDifference = predictedRotation pigeonAngle.getUnboundedDegrees() - startingRotationValue;
		if(Math.abs(rotationDifference) >= (rotationAmount * totalModulesPivoted * Math.signum(rotationalInput))) {
			totalModulesPivoted += Math.signum(rotationalInput);
			currentModuleIndex = (int) Util.boundToScope(0, 4, currentModuleIndex + Math.signum(rotationalInput));
		}
		clockwiseCenter = Constants.kModulePositions.get(currentModuleIndex);
		counterClockwiseCenter = Constants.kModulePositions.get((int) Util.boundToScope(0, 4, currentModuleIndex + 1));
		*/
	}

	/** Called every cycle to update the swerve based on its control state */
	public void updateControlCycle(double timestamp){
		double rotationCorrection = headingController.updateRotationCorrection(pose.getRotation(), timestamp);
		switch(currentState){
			case MANUAL:
				if(evading && evadingToggled){
					//determineEvasionWheels();
					updateEvadeRevolve();
					double sign = Math.signum(rotationalInput);
					if(sign == 1.0){
						inverseKinematics.setCenterOfRotation(clockwiseCenter);
					}else if(sign == -1.0){
						inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
					}
					evadingToggled = false;
				}else if(evading){
					updateEvadeRevolve();
					double sign = Math.signum(rotationalInput);
					if(sign == 1.0){
						inverseKinematics.setCenterOfRotation(clockwiseCenter);
					}else if(sign == -1.0){
						inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
					}
				}else if(evadingToggled){
					inverseKinematics.setCenterOfRotation(Translation2d.identity());
					evadingToggled = false;
				}
				if(translationalVector.equals(Translation2d.identity()) && rotationalInput == 0.0){
					if(lastDriveVector.equals(rotationalVector)){
						stop();
					}else{
						setOpenLoopCoast(inverseKinematics.updateDriveVectors(lastDriveVector,
								rotationCorrection, pose, robotCentric));
					}
				}else{
					setOpenLoop(inverseKinematics.updateDriveVectors(translationalVector,
							rotationalInput + rotationCorrection, pose, robotCentric));
				}
				break;
			case POSITION:
				/*if (moduleAnglesOnTarget() && !isDriveLocked) {
					modules.forEach((m) -> m.setDrivePositionTarget(0.0));
					this.isDriveLocked = true;
				}*/
				break;
			case ROTATION:
				setOpenLoop(inverseKinematics.updateDriveVectors(new Translation2d(), Util.deadBand(rotationCorrection, 0.1), pose, false));
				break;
			case VECTORIZED:
				Translation2d outputVectorV = vf.getVector(pose.getTranslation()).scale(0.25);
				Logger.getInstance().recordOutput("Swerve/Vector Direction", outputVectorV.direction().getDegrees());
				Logger.getInstance().recordOutput("Swerve/Vector Magnitude", outputVectorV.norm());
				//			System.out.println(outputVector.x()+" "+outputVector.y());
				setOpenLoop(inverseKinematics.updateDriveVectors(outputVectorV, rotationCorrection, getPose(), false));
				break;
			case TRAJECTORY:
				if(!motionPlanner.isDone()){
					Translation2d driveVector = motionPlanner.update(timestamp, pose);
					
					if(modulesReady){
						if(!hasStartedFollowing){
							if(moduleConfigRequested){
								zeroSensors(startingPose);
								System.out.println("Position reset for auto");
							}
							hasStartedFollowing = true;
						}
						double rotationInput = Util.deadBand(Util.limit(rotationCorrection*rotationScalar*driveVector.norm(), motionPlanner.getMaxRotationSpeed()), 0.01);
						if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
							driveVector = lastTrajectoryVector;
							setClosedLoopVelocityStall(inverseKinematics.updateDriveVectors(driveVector,
									rotationInput, pose, false));
						}else{
							setClosedLoopVelocity(inverseKinematics.updateDriveVectors(driveVector,
									rotationInput, pose, false));
						}
					}else if(!moduleConfigRequested){
						setClosedLoopVelocityStall(inverseKinematics.updateDriveVectors(driveVector,
								0.0, pose, false));
						moduleConfigRequested = true;
					}
					
					if(areModuleAnglesOnTarget() && !modulesReady){
						modulesReady = true;
						System.out.println("Modules Ready");
					}
					
					lastTrajectoryVector = driveVector;
				}else{
					if(!hasFinishedPath){ 
						System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
						hasFinishedPath = true;
						if(alwaysConfigureModules) requireModuleConfiguration();
					}
				}
				break;
			case VISION_PID:
				Pose2d visionPIDOutput = visionPID.update(pose, timestamp, timestamp - lastUpdateTimestamp);
				Translation2d driveVector = visionPIDOutput.getTranslation();
				Logger.getInstance().recordOutput("Swerve/Vision PID Output", driveVector.toString());
				if (!visionPIDOutput.getRotation().equals(headingController.getTargetHeading())) {
					setPathHeading(visionPIDOutput.getRotation());
				}
				if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
					driveVector = lastDriveVector;
					setClosedLoopVelocityStall(inverseKinematics.updateDriveVectors(driveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, false));
				}else{
					setClosedLoopVelocity(inverseKinematics.updateDriveVectors(driveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, false));
				}
				lastDriveVector = driveVector;
				break;
			case BALANCE_PID:
				Translation2d balanceDriveVector = balanceController.update(Rotation2d.fromDegrees(inputs.gyroRoll), timestamp);
				if(Util.epsilonEquals(balanceDriveVector.norm(), 0.0, Constants.kEpsilon)){
					balanceDriveVector = lastDriveVector;
					setClosedLoopVelocityStall(inverseKinematics.updateDriveVectors(balanceDriveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, true));
				}else{
					setClosedLoopVelocity(inverseKinematics.updateDriveVectors(balanceDriveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, true));
				}
				lastDriveVector = balanceDriveVector;
				break;
			case VELOCITY:
				break;
			case NEUTRAL:
				stop();
				break;
			case DISABLED:
				break;
			default:
				break;
		}
	}

	public void updateOdometry(double timestamp) {
		pose = Units.metersToInches(poseEstimator.updateWithTime(timestamp, Rotation2d.fromDegrees(inputs.gyroYaw), getModulePositions()));
		// TODO: Add a method to the pose estimator (or to the kinematics object) that takes a list of SwerveModuleStates (i.e., velocities)
		// and returns a Twist2d representing the overall velocity of the robot. This will yield a more accurate velocity for the robot than
		// the current method of using encoder deltas. This more accurate velocity will be useful for predicting robot motion, expecially
		// when using a turret to aim at a vision target.
	}

	public Pose2d getPoseAtTime(double timeSeconds) {
		return Units.metersToInches(poseEstimator.getEstimatedPositionAtTime(timeSeconds));
	}
	
	private final Loop loop = new Loop(){
		
		@Override
		public void onStart(double timestamp) {
			translationalVector = new Translation2d();
			lastDriveVector = rotationalVector;
			rotationalInput = 0;
			resetAveragedDirection();
			headingController.temporarilyDisable();
			stop();
			lastUpdateTimestamp = timestamp;
		}
		
		@Override
		public void onLoop(double timestamp) {
			updateControlCycle(timestamp);
			lastUpdateTimestamp = timestamp;
		}
		
		@Override
		public void onStop(double timestamp) {
			translationalVector = new Translation2d();
			rotationalInput = 0;
			stop();
		}
		
	};

	public Request visionPIDRequest(Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useTrajectory, boolean waitToFinish) {
		return visionPIDRequest(desiredFieldPose, approachAngle, useTrajectory, waitToFinish, new VisionPIDBuilder().build());
	}

	public Request visionPIDRequest(Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useTrajectory, boolean waitToFinish, VisionPIDController controller) {
		return new Request(){
		
			@Override
			public void act() {
				startVisionPID(desiredFieldPose, approachAngle, useTrajectory, controller);
				System.out.println("Vision Request Started");
			}

			@Override
			public boolean isFinished() {
				if (!waitToFinish) {
					return true;
				}
				
				if (visionPID.isDone()) {
					DriverStation.reportError("Swerve Vision PID Request Finished", false);
					return true;
				}
				return false;
			}

			@Override
			public String toString() {
				return "SwerveVisionPIDRequest()";
			}

		};
	}

	public Request robotCentricTrajectoryRequest(Translation2d relativeEndPos, Rotation2d targetHeading, double defaultVel){
		return new Request(){
			
			@Override
			public void act() {
				setRobotCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
			}
			
			@Override
			public boolean isFinished(){
				return (getState() == ControlState.TRAJECTORY && motionPlanner.isDone()) || getState() == ControlState.MANUAL;
			}
			
		};
	}
	
	public Request startRobotCentricTrajectoryRequest(Translation2d relativeEndPos, Rotation2d targetHeading, double defaultVel){
		return new Request(){
			
			@Override
			public void act() {
				setRobotCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
			}
			
		};
	}

	public Request fieldCentricTrajectoryRequest(Translation2d relativeEndPos, Rotation2d targetHeading, double defaultVel) {
		return new Request(){
			
			@Override
			public void act() {
				setFieldCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
			}
			
			@Override
			public boolean isFinished(){
				return (getState() == ControlState.TRAJECTORY && motionPlanner.isDone()) || getState() == ControlState.MANUAL;
			}
			
		};
	}
	
	public Request openLoopRequest(Translation2d input, double rotation){
		return new Request(){
			
			@Override
			public void act() {
				setState(ControlState.MANUAL);
				sendInput(input.x(), input.y(), rotation, false, false);
			}
			
		};
	}

	public Request setDriveMaxPowerRequest(double power) {
		return new Request() {

			@Override
			public void act() {
				setMaxSpeed(power);
			}
		};
	}

	public SwerveModulePosition[] getModulePositions() {
		return modules.stream()
				.map(SwerveModule::getPosition)
				.toArray(SwerveModulePosition[]::new);
	}

	public Rotation2d getHeading() {
		return pose.getRotation();
	}

	public void zeroModuleAngles() {
		modules.forEach((m) -> m.resetRotationToAbsolute());
	}

	public void forceZeroModuleAngles() {
		modules.forEach((m) -> m.forceResetRotationToAbsolute());
	}

	public void setRotationMotorZeroed(boolean isZeroed) {
		modules.forEach((m) -> m.setRotationMotorZeroed(isZeroed));
	}

	public void addVisionMeasurement(Pose2d estimatedRobotPose, double observationTimestamp, Matrix<N3, N1> standardDeviations) {
		if (Math.abs(estimatedRobotPose.getRotation().distance(pose.getRotation())) > Math.toRadians(45.0)) {
			temporarilyDisableHeadingController();
		}
		poseEstimator.addVisionMeasurement(estimatedRobotPose, observationTimestamp, standardDeviations);
	}
	
	@Override
	public void readPeriodicInputs() {
		modules.forEach(SwerveModule::readPeriodicInputs);
		inputs.gyroYaw = pigeon.getYaw();
		inputs.gyroRoll = pigeon.getRoll();
		Logger.getInstance().processInputs("Swerve", inputs);

		if (!isPoseEstimatorInitialized) {
			// In the swerve constructor, the pose estimator object is created with a default
			// gyro reading and swerve module states. In order for the heading and position of
			// the robot to be zeroed properly before the first enable, we need to reset the
			// pose estimator once, after all of the swerve's inputs have been properly read.
			zeroSensorsBasedOnAlliance();
			isPoseEstimatorInitialized = true;
		}
		updateOdometry(Timer.getFPGATimestamp());
	}
	
	@Override
	public void writePeriodicOutputs() {
		modules.forEach(SwerveModule::writePeriodicOutputs);
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	/** Puts all rotation and drive motors into open-loop mode */
	public void disable(){
		modules.forEach((m) -> m.disable());
		setState(ControlState.DISABLED);
	}
	
	@Override
	public void stop() {
		setState(ControlState.NEUTRAL);
		modules.forEach((m) -> m.stop());
	}
	
	@Override
	public void zeroSensors() {
		zeroSensors(Constants.kRobotStartingPose);
	}

	public void zeroSensorsBasedOnAlliance() {
		double heading = AllianceChooser.getAlliance() == Alliance.Blue ? 0.0 : 180.0;
		zeroSensors(Pose2d.fromRotation(Rotation2d.fromDegrees(heading)));
	}
	
	/** Sets the robot's internal position and heading to match that of the given pose. */
	public void zeroSensors(Pose2d startingPose){
		pose = startingPose;
		poseEstimator.resetPosition(Rotation2d.fromDegrees(inputs.gyroYaw), getModulePositions(), Units.inchesToMeters(startingPose));
		distanceTraveled = 0;
	}

	public void resetGyroRoll() {
		pigeon.resetRoll();
	}

	double prevSwerveVelocity = 0;
	double largestVelocity = 0;
	double lastTimestamp = 0;

	boolean neutralModeIsBrake = true;
	@Override
	public void outputTelemetry() {
		modules.forEach((m) -> m.outputTelemetry());
		LogUtil.recordPose2d("Swerve/Robot Pose", pose);
		LogUtil.recordRotation2d("Swerve/Heading", pose.getRotation());
		Logger.getInstance().recordOutput("Swerve/State", currentState.toString());

		if(Netlink.getBooleanValue("Swerve Coast Mode") && neutralModeIsBrake) {
			setModuleNeutralModes(NeutralMode.Coast);
			neutralModeIsBrake = false;
		} else if(!neutralModeIsBrake && !Netlink.getBooleanValue("Swerve Coast Mode")) {
			setModuleNeutralModes(NeutralMode.Brake);
			neutralModeIsBrake = true;
		}
	}

	@AutoLog
	public static class SwerveInputs {
		public double gyroYaw;
		public double gyroRoll;
	}
}
