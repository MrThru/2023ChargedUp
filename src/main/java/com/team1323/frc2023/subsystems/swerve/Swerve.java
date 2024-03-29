package com.team1323.frc2023.subsystems.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.DriveMotionPlanner;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.RobotState;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.gyros.Gyro;
import com.team1323.frc2023.subsystems.gyros.Pigeon2IMU;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.vision.VisionPIDController;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.Units;
import com.team1323.lib.math.vectors.VectorField;
import com.team1323.lib.util.DriveSignal;
import com.team1323.lib.util.Kinematics;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.SwerveHeadingController;
import com.team1323.lib.util.SwerveInverseKinematics;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.wpilib.SwerveDriveKinematics;
import com.wpilib.SwerveDrivePoseEstimator;
import com.wpilib.SwerveModulePosition;
import com.wpilib.SwerveModuleState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	List<SwerveModule> modules;
	List<SwerveModule> positionModules;
	
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
	RobotState robotState;
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
	
	//Name says it all
	TrajectoryGenerator generator;
	
	//Odometry variables
	Pose2d pose;
	Rotation2d pigeonAngle = Rotation2d.identity();
	Twist2d velocity = Twist2d.identity();
	double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;
	public synchronized Pose2d getPose(){
		return pose;
	}
	public Twist2d getVelocity() {
		return velocity;
	}
	
	// WPILib odometry
	SwerveDrivePoseEstimator poseEstimator;
	
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
	
	private Swerve(){
		/**
		 * 			   	  Front
		 *     __________________________
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
		frontRight = new PhoenixProSwerveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE,
		0, Constants.kFrontRightEncoderStartingPos, Constants.kVehicleToModuleZero, false);
		frontLeft = new PhoenixProSwerveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE,
		1, Constants.kFrontLeftEncoderStartingPos, Constants.kVehicleToModuleOne, false);
		rearLeft = new PhoenixProSwerveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE,
		2, Constants.kRearLeftEncoderStartingPos, Constants.kVehicleToModuleTwo, false);
		rearRight = new PhoenixProSwerveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE,
		3, Constants.kRearRightEncoderStartingPos, Constants.kVehicleToModuleThree, false);
		
		modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		positionModules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);

		frontLeft.invertDriveMotor(TalonFXInvertType.Clockwise);
		rearLeft.invertDriveMotor(TalonFXInvertType.Clockwise);
		frontRight.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		rearRight.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		modules.forEach(m -> m.invertRotationMotor(TalonFXInvertType.Clockwise));
		
		pigeon = Pigeon2IMU.getInstance();
		
		motionPlanner = new DriveMotionPlanner();
		
		robotState = RobotState.getInstance();
		
		SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			Units.inchesToMeters(Constants.kVehicleToModuleZero),
			Units.inchesToMeters(Constants.kVehicleToModuleOne),
			Units.inchesToMeters(Constants.kVehicleToModuleTwo),
			Units.inchesToMeters(Constants.kVehicleToModuleThree)
		);
		poseEstimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getYaw(), 
				getModulePositions(), Pose2d.identity(), VecBuilder.fill(0.1, 0.1, 0.1), 
				VecBuilder.fill(0.1, 0.1, 0.1));
		pose = poseEstimator.getEstimatedPosition();
		distanceTraveled = 0;
		
		generator = TrajectoryGenerator.getInstance();
	}

	public void setDriveNeutralMode(NeutralMode mode) {
		modules.forEach((m) -> m.setDriveNeutralMode(mode));
	}
	public void setRotationNeutralMode(NeutralMode mode) {
		modules.forEach((m) -> m.setRotationNeutralMode(mode));
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
	private SlewRateLimiter xInputLimiter = new SlewRateLimiter(Constants.kSwerveXInputRate);
	private SlewRateLimiter yInputLimiter = new SlewRateLimiter(Constants.kSwerveYInputRate);
	private boolean useSlewLimiter = false;
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
			if (useSlewLimiter) {
				x = xInputLimiter.calculate(x);
				y = yInputLimiter.calculate(y);
			} else {
				xInputLimiter.reset(x);
				yInputLimiter.reset(y);
			}
			Translation2d translationalInput = new Translation2d(x, y);
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
	
	public synchronized void updateControllerDirection(Translation2d input){
		if(Util.epsilonEquals(input.norm(), 1.0, 0.1)){
			Rotation2d direction = input.direction();
			double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
			averagedDirection = Rotation2d.fromDegrees(roundedDirection);
		}
	}
	
	// Various methods to control the heading controller
	public synchronized void rotate(Rotation2d goalHeading) {
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
		modules.forEach((m) -> m.setModuleAngle(directionDegrees));
		modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
	}
	
	/** Locks drive motors in place with MotionMagic */
	public void lockDrivePosition(){
		System.out.println("LOCKING MODULE POSITION");
		setState(ControlState.POSITION);
		isDriveLocked = false;
	}

	public void atomicLockDrivePosition() {
		setState(ControlState.NEUTRAL);
		modules.get(0).setModuleAngle(-45.0);
		modules.get(1).setModuleAngle(45.0);
		modules.get(2).setModuleAngle(-45.0);
		modules.get(3).setModuleAngle(45.0);
	}

	public void zukLockDrivePosition() {
		setState(ControlState.POSITION);
		modules.forEach((m) -> m.setDriveOpenLoop(0));;
		modules.get(0).setModuleAngle(45.0);
		modules.get(1).setModuleAngle(-45.0);
		modules.get(2).setModuleAngle(45.0);
		modules.get(3).setModuleAngle(-45.0);
		isDriveLocked = false;
	}
	
	/** Puts drive motors into closed-loop velocity mode */
	public void setVelocity(Rotation2d direction, double velocityInchesPerSecond){
		setState(ControlState.VELOCITY);
		modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
		modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
	}
	
	/** Configures each module to match its assigned vector */
	public void setDriveOutput(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
			}
		}
	}
	
	public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverride){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setDriveOpenLoop(-percentOutputOverride);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setDriveOpenLoop(percentOutputOverride);
			}
		}
	}
	public void setModulesDriveOuput(double output) {
		for(int i = 0; i < modules.size(); i++) {
			modules.get(i).setDriveOpenLoop(output);
		}
	}
	
	
	/** Configures each module to match its assigned vector, but puts the drive motors into closed-loop velocity mode */
	public void setVelocityDriveOutput(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setVelocitySetpoint(driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}
	
	public void setVelocityDriveOutput(List<Translation2d> driveVectors, double velocityOverride){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-velocityOverride);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setVelocitySetpoint(velocityOverride);
			}
		}
	}

	public void setVelocityDriveOutput(DriveSignal driveSignal, double velocityOverride) {
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveSignal.getWheelAzimuths()[i], modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-velocityOverride * Constants.kSwerveMaxSpeedInchesPerSecond);
			}else{
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees());
				modules.get(i).setVelocitySetpoint(velocityOverride * Constants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}

	public void setVelocityDriveOutput(DriveSignal driveSignal) {
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveSignal.getWheelAzimuths()[i], modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-driveSignal.getWheelSpeeds()[i] * Constants.kSwerveMaxSpeedInchesPerSecond);
			}else{
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees());
				modules.get(i).setVelocitySetpoint(driveSignal.getWheelSpeeds()[i] * Constants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}
	
	/** Sets only module angles to match their assigned vectors */
	public void setModuleAngles(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
			}
		}
	}

	public void setModuleAngles(DriveSignal driveSignal){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveSignal.getWheelAzimuths()[i], modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees() + 180.0);
			}else{
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees());
			}
		}
	}
	
	/**
	* @return Whether or not at least one module has reached its MotionMagic setpoint
	*/
	public boolean positionOnTarget(){
		boolean onTarget = false;
		for(SwerveModule m : modules){
			onTarget |= m.drivePositionOnTarget();
		}
		return onTarget;
	}
	
	/**
	* @return Whether or not all modules have reached their angle setpoints
	*/
	public boolean moduleAnglesOnTarget(){
		boolean onTarget = true;
		for(SwerveModule m : modules){
			onTarget &= m.angleOnTarget();
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
	public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d targetHeading,
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
	
	public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d targetHeading,
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

	public synchronized void setFieldCentricTrajectory(Translation2d relativeEndPos, Rotation2d targetHeading, double defaultVel) {
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

	public synchronized void startVisionPID(Pose2d desiredFieldPose, Rotation2d approachAngle, boolean useTrajectory, VisionPIDController controller) {
		visionPID = controller;
		visionPID.start(pose, desiredFieldPose, approachAngle, useTrajectory, false);
		rotationScalar = 0.75;
		translationStickReset = false;
		setState(ControlState.VISION_PID);
	}

	public synchronized Translation2d getVisionPIDTarget() {
		return visionPID.getTargetPosition();
	}

	public synchronized void setVisionPIDTarget(Translation2d targetPosition) {
		visionPID.setTargetPosition(targetPosition);
	}

	public synchronized void resetVisionPID() {
		visionPID.resetDistanceToTargetPosition();
	}

	public synchronized double getDistanceToTargetPosition() {
		return visionPID.getDistanceToTargetPosition();
	}

	public synchronized boolean isVisionPIDDone() {
		return getState() != ControlState.VISION_PID || visionPID.isDone();
	}

	public synchronized void addRetroObservation(Translation2d retroPosition, double timestamp) {
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
	public synchronized void setVectorField(VectorField vf_) {
		vf = vf_;
		setState(ControlState.VECTORIZED);
	}
	
	/** Determines which wheels the robot should rotate about in order to perform an evasive maneuver */
	int evadeClosestIndex = 0;
	public synchronized void determineEvasionWheels(){
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
	public synchronized void startEvadeRevolve(Translation2d revolveAround) {
		startingRotationValue = pigeonAngle.getUnboundedDegrees() - 45;
		currentModuleIndex = 0;
		totalModulesPivoted = 1;
		for(int i = 0; i < Constants.kModulePositions.size(); i++) {
			if(Constants.kModulePositions.get(currentModuleIndex).distance(revolveAround) > Constants.kModulePositions.get(i).distance(revolveAround)) {
				currentModuleIndex = i;
			}
		}
		
	}
	
	
	public synchronized void updateEvadeRevolve() {
		double predictedRotation = pigeonAngle.getUnboundedDegrees() + (velocity.dtheta * 180/Math.PI) * 0.1;
		double rotationDifference = predictedRotation /*pigeonAngle.getUnboundedDegrees()*/ - startingRotationValue;
		if(Math.abs(rotationDifference) >= (rotationAmount * totalModulesPivoted * Math.signum(rotationalInput))) {
			totalModulesPivoted += Math.signum(rotationalInput);
			currentModuleIndex = (int) Util.boundToScope(0, 4, currentModuleIndex + Math.signum(rotationalInput));
		}
		clockwiseCenter = Constants.kModulePositions.get(currentModuleIndex);
		counterClockwiseCenter = Constants.kModulePositions.get((int) Util.boundToScope(0, 4, currentModuleIndex + 1));
	}

	/** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();
		
		double averageDistance = 0.0;
		double[] distances = new double[4];
		for(SwerveModule m : positionModules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse()).norm();
			distances[m.moduleId] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();
		
		int minDevianceIndex = 0;
		double minDeviance = 100.0;
		List<SwerveModule> modulesToUse = new ArrayList<>();
		for(SwerveModule m : positionModules){
			double deviance = Math.abs(distances[m.moduleId] - averageDistance);
			if(deviance < minDeviance){
				minDeviance = deviance;
				minDevianceIndex = m.moduleId;
			}
			if(deviance <= 0.01){
				modulesToUse.add(m);
			}
		}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		
		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}
	
	/** Playing around with different methods of odometry. This will require the use of all four modules, however. */
	public synchronized void alternatePoseUpdate(){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();
		
		double[][] distances = new double[4][2];
		for(SwerveModule m : modules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
			distances[m.moduleId][0] = m.moduleId;
			distances[m.moduleId][1] = distance;
		}
		
		Arrays.sort(distances, new java.util.Comparator<double[]>() {
			public int compare(double[] a, double[] b) {
				return Double.compare(a[1], b[1]);
			}
		});
		List<SwerveModule> modulesToUse = new ArrayList<>();
		double firstDifference = distances[1][1] - distances[0][1];
		double secondDifference = distances[2][1] - distances[1][1];
		double thirdDifference = distances[3][1] - distances[2][1];
		if(secondDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
		}else if(thirdDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
		}else{
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
			modulesToUse.add(modules.get((int)distances[3][0]));
		}
		
		SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
		distanceTraveled += deltaPos;
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}
	
	boolean isDriveLocked = false;
	/** Called every cycle to update the swerve based on its control state */
	public synchronized void updateControlCycle(double timestamp){
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
					setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector,
					rotationCorrection, pose, robotCentric), 0.0);
				}
			}else{
				setDriveOutput(inverseKinematics.updateDriveVectors(translationalVector,
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
			setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(), Util.deadBand(rotationCorrection, 0.1), pose, false));
			break;
			case VECTORIZED:
			Translation2d outputVectorV = vf.getVector(pose.getTranslation()).scale(0.25);
			SmartDashboard.putNumber("Vector Direction", outputVectorV.direction().getDegrees());
			SmartDashboard.putNumber("Vector Magnitude", outputVectorV.norm());
			//			System.out.println(outputVector.x()+" "+outputVector.y());
			setDriveOutput(inverseKinematics.updateDriveVectors(outputVectorV, rotationCorrection, getPose(), false));
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
						setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
								rotationInput, pose, false), 0.0);
						//setVelocityDriveOutput(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), rotationInput, true), 0.0);
					}else{
						setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
								rotationInput, pose, false));
						//setVelocityDriveOutput(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), rotationInput, true));
					}
				}else if(!moduleConfigRequested){
					setModuleAngles(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), 0.0, true));
					moduleConfigRequested = true;
				}
				
				if(moduleAnglesOnTarget() && !modulesReady){
					modules.forEach((m) -> m.resetLastEncoderReading());
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
			SmartDashboard.putString("Swerve Vision PID Output", driveVector.toString());
			if (!visionPIDOutput.getRotation().equals(headingController.getTargetHeading())) {
				setPathHeading(visionPIDOutput.getRotation());
			}
			if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
				driveVector = lastDriveVector;
				setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, false), 0.0);
			}else{
				setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, false));
			}
			lastDriveVector = driveVector;
			break;
			case BALANCE_PID:
				Translation2d balanceDriveVector = balanceController.update(pigeon.getRoll(), timestamp);
				if(Util.epsilonEquals(balanceDriveVector.norm(), 0.0, Constants.kEpsilon)){
					balanceDriveVector = lastDriveVector;
					setVelocityDriveOutput(inverseKinematics.updateDriveVectors(balanceDriveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, true), 0.0);
				}else{
					setVelocityDriveOutput(inverseKinematics.updateDriveVectors(balanceDriveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, true));
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

	public synchronized void updateOdometry(double timestamp, double deltaTime) {
		pose = Units.metersToInches(poseEstimator.updateWithTime(timestamp, pigeon.getYaw(), getModulePositions()));
		velocity = Units.metersToInches(poseEstimator.getDeltaMeters().scaled(1.0 / deltaTime));
	}

	public synchronized Pose2d getPoseAtTime(double timeSeconds) {
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
	
	public Request velocityRequest(Rotation2d direction, double magnitude){
		return new Request(){
			
			@Override
			public void act() {
				setVelocity(direction, magnitude);
			}
			
		};
	}

	public Request lockDrivePositionRequest() {
		return new Request(){
		
			@Override
			public void act() {
				lockDrivePosition();
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
	
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.size()]; 
		for(int i = 0; i < modules.size(); i++) {
			states[i] = modules.get(i).getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.size()];
		for (int i = 0; i < modules.size(); i++) {
			positions[i] = new SwerveModulePosition(Units.inchesToMeters(modules.get(i).getDriveDistanceInches()), 
					modules.get(i).getModuleAngle());
		}
		return positions;
	}

	public double[] getModuleVelocities() {
		double[] velocities = new double[modules.size()];
		SwerveModuleState[] states = getModuleStates();
		for (int i = 0; i < modules.size(); i++) {
			velocities[i] = states[i].speedMetersPerSecond;
		}
		return velocities;
	}

	public Rotation2d[] getModuleAngles() {
		Rotation2d[] angles = new Rotation2d[modules.size()];
		SwerveModuleState[] states = getModuleStates();
		for (int i = 0; i < modules.size(); i++) {
			angles[i] = states[i].angle;
		}
		return angles;
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

	public synchronized void addVisionMeasurement(Pose2d estimatedRobotPose, double observationTimestamp, Matrix<N3, N1> standardDeviations) {
		if (Math.abs(estimatedRobotPose.getRotation().distance(pose.getRotation())) > Math.toRadians(45.0)) {
			temporarilyDisableHeadingController();
		}
		poseEstimator.addVisionMeasurement(estimatedRobotPose, observationTimestamp, standardDeviations);
	}
	
	@Override
	public void readPeriodicInputs() {
		modules.forEach((m) -> m.readPeriodicInputs());
		pigeonAngle = pigeon.getYaw();
	}
	
	@Override
	public void writePeriodicOutputs() {
		modules.forEach((m) -> m.writePeriodicOutputs());
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	/** Puts all rotation and drive motors into open-loop mode */
	public synchronized void disable(){
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
	
	/** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
	public synchronized void zeroSensors(Pose2d startingPose){
		modules.forEach((m) -> m.zeroSensors(startingPose));
		pose = startingPose;
		robotState.reset(Timer.getFPGATimestamp(), startingPose, Rotation2d.identity());
		poseEstimator.resetPosition(pigeon.getYaw(), getModulePositions(), Units.inchesToMeters(startingPose));
		distanceTraveled = 0;
	}
	double prevSwerveVelocity = 0;
	double largestVelocity = 0;
	double lastTimestamp = 0;

	boolean neutralModeIsBrake = true;
	@Override
	public void outputTelemetry() {
		modules.forEach((m) -> m.outputTelemetry());
		SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getDegrees()});
		
		SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
		

		SmartDashboard.putString("Swerve State", currentState.toString());

		if(Netlink.getBooleanValue("Swerve Coast Mode") && neutralModeIsBrake) {
			setDriveNeutralMode(NeutralMode.Coast);
			setRotationNeutralMode(NeutralMode.Coast);
			neutralModeIsBrake = false;
		} else if(!neutralModeIsBrake && !Netlink.getBooleanValue("Swerve Coast Mode")) {
			setDriveNeutralMode(NeutralMode.Brake);
			setRotationNeutralMode(NeutralMode.Brake);
			neutralModeIsBrake = true;
		}
		
		if(Settings.debugSwerve()){
			SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
			SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
			SmartDashboard.putString("Heading Controller", headingController.getState().toString());
			SmartDashboard.putNumber("Target Heading", getTargetHeading().getDegrees());
			SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
			SmartDashboard.putString("Robot Velocity", velocity.toString());
			SmartDashboard.putNumberArray("Pigeon YPR", pigeon.getYPR());
			SmartDashboard.putNumber("Gyro Yaw", pigeon.getYaw().getDegrees());
			SmartDashboard.putNumber("Gyro Pitch", pigeon.getRoll().getDegrees());

			double swerveVelocity =  velocity.norm() / 12.0;
			double currentTimestamp = Timer.getFPGATimestamp();
			SmartDashboard.putNumber("Robot Swerve Velocity", swerveVelocity);
			SmartDashboard.putNumber("Robot Swerve Acceleration", ((swerveVelocity - prevSwerveVelocity)));
			prevSwerveVelocity = swerveVelocity;
			lastTimestamp = currentTimestamp;
			if(swerveVelocity > largestVelocity)
				largestVelocity = swerveVelocity;
			SmartDashboard.putNumber("Robot Swerve Peak Velocity", largestVelocity);
		}
	}
}
