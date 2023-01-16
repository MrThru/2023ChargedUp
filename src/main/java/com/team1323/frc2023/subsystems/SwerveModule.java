package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.Settings;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyPhoenix5TalonFX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.wpilib.SwerveModuleState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SwerveModule extends Subsystem {
	DutyCycle rotationAbsoluteEncoder;
	int moduleId;
	String name = "Module ";
	int rotationSetpoint = 0;
	double driveSetpoint = 0;
	double encoderOffset;
	int encoderReverseFactor = 1;
	boolean isRotationEncoderFlipped = false;
	boolean rotationMotorZeroed = false;
	boolean useDriveEncoder = true;
	boolean moduleZeroedWitoutMagEnc = false;
	private double previousEncDistance = 0;
	private Translation2d position;
	private Translation2d startingPosition;
	private Pose2d estimatedRobotPose = new Pose2d();

	PeriodicIO periodicIO = new PeriodicIO();
	
	public SwerveModule(int moduleId, double encoderOffset, Translation2d startingPose, boolean flipAbsoluteEncoder) {
		name += (moduleId + " ");
		if (RobotBase.isReal()) {
			rotationAbsoluteEncoder = new DutyCycle(new DigitalInput(Ports.kModuleEncoders[moduleId]));
		}
			
		this.encoderOffset = encoderOffset;
		this.isRotationEncoderFlipped = flipAbsoluteEncoder;
		this.moduleId = moduleId;
		previousEncDistance = 0;
		position = startingPose;
		this.startingPosition = startingPose;
	}
	
	public abstract void invertDriveMotor(TalonFXInvertType invertType);
	
	public abstract void invertRotationMotor(TalonFXInvertType invertType);

	public void reverseRotationMagEncoder(boolean invert) {
		this.isRotationEncoderFlipped = invert;
	}
	
	public void reverseRotationSensor(boolean reverse){
		encoderReverseFactor = reverse ? -1 : 1;
	}
	
	public void disableDriveEncoder(){
		useDriveEncoder = false;
	}

	public abstract void setDriveNeutralMode(NeutralMode mode);

	public abstract void setRotationNeutralMode(NeutralMode mode);

	protected abstract void configureMotors();
	
	protected boolean isRotationSensorConnected(){
		if(RobotBase.isReal()){
			return rotationAbsoluteEncoder.getFrequency() != 0;
		}
		return true;
	}
	
	private double getRawAngle(){
		return encUnitsToDegrees(periodicIO.rotationPosition);
	}
	
	public Rotation2d getModuleAngle(){
		return Rotation2d.fromDegrees(getRawAngle());
	}
	
	public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
		Rotation2d normalizedAngle = getModuleAngle();
		return normalizedAngle.rotateBy(robotHeading);
	}
	
	public void setModuleAngle(double goalAngle){
		double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(), goalAngle);
		double setpoint = degreesToEncUnits(newAngle);
		periodicIO.rotationControlMode = ControlMode.MotionMagic;
		periodicIO.rotationDemand = setpoint;
	}
	
	public abstract boolean angleOnTarget();
	
	public void setRotationOpenLoop(double power){
		periodicIO.rotationControlMode = ControlMode.PercentOutput;
		periodicIO.rotationDemand = power;
	}
	
	/**
	* @param velocity Normalized value
	*/
	public void setDriveOpenLoop(double velocity){
		periodicIO.driveControlMode = ControlMode.PercentOutput;
		periodicIO.driveDemand = velocity;
	}

	protected abstract void setDriveProfileSlot(int slotIndex);
	
	public void setDrivePositionTarget(double deltaDistanceInches){
		setDriveProfileSlot(0);
		periodicIO.driveControlMode = ControlMode.MotionMagic;
		periodicIO.driveDemand = periodicIO.drivePosition + inchesToEncUnits(deltaDistanceInches);
	}
	
	public abstract boolean drivePositionOnTarget();
	
	public void setVelocitySetpoint(double inchesPerSecond){
		setDriveProfileSlot(1);
		periodicIO.driveControlMode = ControlMode.Velocity;
		periodicIO.driveDemand = inchesPerSecondToEncVelocity(inchesPerSecond);
	}

	private double getAbsoluteEncoderDegrees() {
		return (isRotationEncoderFlipped ? -1.0 : 1.0) * periodicIO.absoluteRotation;
	}
	
	public double getDriveDistanceInches(){
		return encUnitsToInches(periodicIO.drivePosition);
	}
	
	public double encUnitsToInches(double encUnits){
		return encUnits / Constants.kSwerveEncUnitsPerInch;
	}
	
	public double inchesToEncUnits(double inches){
		return inches * Constants.kSwerveEncUnitsPerInch;
	}
	
	public double encVelocityToInchesPerSecond(double encUnitsPer100ms){
		return encUnitsToInches(encUnitsPer100ms) * 10;
	}
	
	public double inchesPerSecondToEncVelocity(double inchesPerSecond){
		return inchesToEncUnits(inchesPerSecond / 10.0);
	}
	
	public double degreesToEncUnits(double degrees){
		return (degrees / 360.0) * Constants.kSwerveRotationReduction * Constants.kSwerveRotationEncoderResolution;
	}
	
	public double encUnitsToDegrees(double encUnits){
		return (encUnits / Constants.kSwerveRotationEncoderResolution) / Constants.kSwerveRotationReduction * 360.0;
	}
	
	public Translation2d getPosition(){
		return position;
	}
	
	public Pose2d getEstimatedRobotPose(){
		return estimatedRobotPose;
	}
	
	public SwerveModuleState getState() {
		return new SwerveModuleState(encVelocityToInchesPerSecond(periodicIO.velocity), getModuleAngle());
	}
	
	public void updatePose(Rotation2d robotHeading){
		double currentEncDistance = getDriveDistanceInches();
		double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleId];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(currentWheelAngle.cos()*deltaEncDistance, 
		currentWheelAngle.sin()*deltaEncDistance);
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(startingPosition).inverse());
		position = updatedPosition;
		estimatedRobotPose =  robotPose;
		previousEncDistance = currentEncDistance;
	}
	
	public void resetPose(Pose2d robotPose){
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
		position = modulePosition;
	}
	
	public void resetPose(){
		position = startingPosition;
	}
	
	public void resetLastEncoderReading(){
		previousEncDistance = getDriveDistanceInches();
	}
	
	@Override
	public void stop(){
		setDriveOpenLoop(0.0);
		setModuleAngle(getModuleAngle().getDegrees());
	}
	
	public void disable(){
		setDriveOpenLoop(0.0);
		setRotationOpenLoop(0.0);
	}

	protected abstract ErrorCode setRotationSensorPosition(double sensorPosition);
	
	int zeroCount = 0;
	public void resetRotationToAbsolute(){
		if (!rotationMotorZeroed) {
			ErrorCode rotationFalconCheck = setRotationSensorPosition(0.0);
			if (rotationFalconCheck == ErrorCode.OK) {
				if (isRotationSensorConnected() && RobotBase.isReal()) {
					setRotationSensorPosition(degreesToEncUnits(getAbsoluteEncoderDegrees() - encoderOffset));
					moduleZeroedWitoutMagEnc = false;
				} else {
					setRotationSensorPosition(0.0);
					DriverStation.reportError("MAG ENCODER FOR " + name + " WAS NOT CONNECTED UPON BOOT", false);
					moduleZeroedWitoutMagEnc = true;
				}
			} else {
				DriverStation.reportError("ROTATION FALCON NOT FOUND ON " + name, false);
				moduleZeroedWitoutMagEnc = true;
			}
			if (zeroCount < 50) {
				zeroCount++;
			} else {
				System.out.println("MODULE " + name + " ZEROED");
				System.out.println(name + "Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + encoderOffset + ", difference: " + (getAbsoluteEncoderDegrees() - encoderOffset) + ", degreesToEncUnits: " + degreesToEncUnits(getAbsoluteEncoderDegrees() - encoderOffset));
				rotationMotorZeroed = true;
			}
		}
	}

	public void setRotationMotorZeroed(boolean isZeroed) {
		rotationMotorZeroed = isZeroed;
	}
	
	@Override
	public void zeroSensors() {
		zeroSensors(new Pose2d());
	}
	
	public void zeroSensors(Pose2d robotPose) {
		resetPose(robotPose);
		estimatedRobotPose = robotPose;
		previousEncDistance = getDriveDistanceInches();
	}

	public double getModuleVelocity() {
		return encVelocityToInchesPerSecond(periodicIO.velocity);
	}
	
	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
		SmartDashboard.putNumber(name + "Absolute Angle", getAbsoluteEncoderDegrees());
		SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
		SmartDashboard.putNumber(name + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));

		if(Settings.debugSwerve()){
			if (RobotBase.isReal()) {
				SmartDashboard.putNumber(name + "Absolute Angle", getAbsoluteEncoderDegrees());
			}
			SmartDashboard.putNumber(name + "Rotation Encoder", periodicIO.rotationPosition);
			SmartDashboard.putNumber(name + "Drive Voltage", periodicIO.driveVoltage);
			SmartDashboard.putNumber(name + "Velocity - in", encVelocityToInchesPerSecond(periodicIO.velocity));
			SmartDashboard.putNumber(name + "Raw Velocity", periodicIO.velocity);
		}
	}
	
	public static class PeriodicIO{
		//Inputs
		public double rotationPosition = 0;
		public double drivePosition = 0;
		public double velocity = 0;
		public double driveVoltage = 0.0;
		public double absoluteRotation = 0.0;
		
		
		//Outputs
		public ControlMode rotationControlMode = ControlMode.PercentOutput;
		public ControlMode driveControlMode = ControlMode.PercentOutput;
		public double rotationDemand;
		public double driveDemand;
	}

}
