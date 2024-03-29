package com.team1323.frc2023.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team254.drivers.LazyPhoenix5TalonFX;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class Phoenix5SwerveModule extends SwerveModule {
	private final LazyPhoenix5TalonFX rotationMotor, driveMotor;

    public Phoenix5SwerveModule(int rotationPort, int drivePort, int moduleId, 
            double encoderOffset, Translation2d startingPose, boolean flipAbsoluteEncoder) {
        super(moduleId, encoderOffset, startingPose, flipAbsoluteEncoder);
		rotationMotor = new LazyPhoenix5TalonFX(rotationPort, Ports.CANBUS);
		driveMotor = new LazyPhoenix5TalonFX(drivePort, Ports.CANBUS);
		configureMotors();
    }
    
    @Override
	protected void configureMotors(){
		rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		rotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 10);
		rotationMotor.setNeutralMode(NeutralMode.Brake);
		rotationMotor.configVoltageCompSaturation(7.0, 10);
		rotationMotor.enableVoltageCompensation(true);
		rotationMotor.configForwardSoftLimitEnable(false, 10);
		rotationMotor.configReverseSoftLimitEnable(false, 10);
		rotationMotor.configMotionAcceleration((int)(Constants.kSwerveRotationMaxSpeedEncUnits*12.5), 10);
		rotationMotor.configMotionCruiseVelocity((int)(Constants.kSwerveRotationMaxSpeedEncUnits), 10);
		rotationMotor.selectProfileSlot(0, 0);
		//Slot 1 is for normal use
		rotationMotor.config_kP(0, 1.55, 10); // 1.55
		rotationMotor.config_kI(0, 0.0, 10);
		rotationMotor.config_kD(0, 5.0, 10); // 5.0
		rotationMotor.config_kF(0, 1023.0/Constants.kSwerveRotationMaxSpeedEncUnits, 10);
		//Slot 2 is reserved for the beginning of auto
		rotationMotor.config_kP(1, 8.0, 10);
		rotationMotor.config_kI(1, 0.0, 10);
		rotationMotor.config_kD(1, 200.0, 10);
		rotationMotor.config_kF(1, 1023.0/Constants.kSwerveRotation10VoltMaxSpeed, 10);
		rotationMotor.configAllowableClosedloopError(0, 50, Constants.kCANTimeoutMs);
		rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));
		if(!isRotationSensorConnected()){
			DriverStation.reportError(name + "rotation encoder not detected!", false);
			hasEmergency = true;
		}
		resetRotationToAbsolute();

		driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		driveMotor.setSelectedSensorPosition(0, 0, 10);
		driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 10);
		driveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms, 10);
		driveMotor.configVelocityMeasurementWindow(32, 10);
		driveMotor.configForwardSoftLimitEnable(false, 10);
		driveMotor.configReverseSoftLimitEnable(false, 10);
		driveMotor.configNominalOutputForward(0/12.0, 10);
		driveMotor.configNominalOutputReverse(0/12.0, 10);
		driveMotor.configVoltageCompSaturation(12.0, 10);
		driveMotor.enableVoltageCompensation(true);
		driveMotor.configOpenloopRamp(0.2, 10);
		driveMotor.configClosedloopRamp(0.0);
		driveMotor.configAllowableClosedloopError(0, 0, 10);
		driveMotor.setNeutralMode(NeutralMode.Brake);
		// Slot 0 is reserved for MotionMagic
		driveMotor.selectProfileSlot(0, 0);
		driveMotor.config_kP(0, 0.18, 10);
		driveMotor.config_kI(0, 0.0, 10);
		driveMotor.config_kD(0, 3.6, 10);
		driveMotor.config_kF(0, 1023.0/Constants.kMaxFalconEncoderSpeed, 10);
		driveMotor.configMotionCruiseVelocity(Constants.kMaxFalconEncoderSpeed*0.9, 10);
		driveMotor.configMotionAcceleration(Constants.kMaxFalconEncoderSpeed, 10);
		// Slot 1 corresponds to velocity mode
		driveMotor.config_kP(1, 0.11, 10);
		driveMotor.config_kI(1, 0.0, 10);
		driveMotor.config_kD(1, 0.0, 10);
		driveMotor.config_kF(1, 1023.0/Constants.kMaxFalconEncoderSpeed, 10);
	}

    @Override
	public void invertDriveMotor(TalonFXInvertType invertType){
		driveMotor.setInverted(invertType);
	}
	
    @Override
	public void invertRotationMotor(TalonFXInvertType invertType){
		rotationMotor.setInverted(invertType);
	}

    @Override
	public void setDriveNeutralMode(NeutralMode mode) {
		driveMotor.setNeutralMode(mode);
	}

    @Override
	public void setRotationNeutralMode(NeutralMode mode) {
		rotationMotor.setNeutralMode(mode);
	}
	
    @Override
	public boolean angleOnTarget(){
		double error = encUnitsToDegrees(Math.abs(rotationMotor.getClosedLoopError(0)));
		return error < Constants.kSwerveModuleRotationTolerance;
	}

    @Override
	public boolean drivePositionOnTarget(){
		if(driveMotor.getControlMode() == ControlMode.MotionMagic) {
            return encUnitsToInches(Math.abs(periodicIO.driveDemand - periodicIO.drivePosition)) < Constants.kSwerveMotionMagicTolerance;
        }
		return false;
	}

    @Override
    protected void setDriveProfileSlot(int slotIndex) {
        driveMotor.selectProfileSlot(slotIndex, 0);
    }

    @Override
    protected ErrorCode setRotationSensorPosition(double sensorPosition) {
        return rotationMotor.setSelectedSensorPosition(sensorPosition);
    }

	@Override
	public double getDriveVoltage() {
		return driveMotor.getMotorOutputVoltage();
	}

	@Override
	public void readPeriodicInputs() {
		periodicIO.velocity = driveMotor.getSelectedSensorVelocity(0);
		periodicIO.rotationPosition = rotationMotor.getSelectedSensorPosition(0);
		periodicIO.drivePosition = driveMotor.getSelectedSensorPosition(0);
		if (RobotBase.isReal()) {
            periodicIO.absoluteRotation = rotationAbsoluteEncoder.getOutput() * 360.0;
		}
		drivePositionBuffer.addValue(encUnitsToInches(periodicIO.drivePosition));
	}
	
	@Override
	public void writePeriodicOutputs() {
		rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
		driveMotor.set(periodicIO.driveControlMode, periodicIO.driveDemand);
	}
}
