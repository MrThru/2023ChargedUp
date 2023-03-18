package com.team1323.frc2023.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team254.drivers.LazyPhoenix5TalonFX;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class PhoenixProSwerveModule extends SwerveModule {
    private static final double kFalconSensorResolution = 2048.0;

    private final LazyPhoenix5TalonFX rotationMotor;
    private final TalonFX driveMotor;

    private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private final VoltageOut driveVoltageOutRequest = new VoltageOut(0.0, true, false);
    private final MotionMagicVoltage driveMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0, true, 0.0, 1, false);
    private ControlRequest currentDriveControlRequest = driveVoltageOutRequest;

    public PhoenixProSwerveModule(int rotationPort, int drivePort, int moduleId, 
            double encoderOffset, Translation2d startingPose, boolean flipAbsoluteEncoder) {
        super(moduleId, encoderOffset, startingPose, flipAbsoluteEncoder);
        rotationMotor = new LazyPhoenix5TalonFX(rotationPort, Ports.CANBUS);
        driveMotor = new TalonFX(drivePort, Ports.CANBUS);
        configureMotors();
    }

    @Override
    protected void configureMotors() {
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

        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Slot 0 is reserved for MotionMagic
        driveConfiguration.Slot0.kP = 0.18;
        driveConfiguration.Slot0.kI = 0.0;
        driveConfiguration.Slot0.kD = 3.6;
        driveConfiguration.Slot0.kV = 1.0 / Constants.kMaxFalconRotationsPerSecond;
        // Slot 1 corresponds to velocity mode
        driveConfiguration.Slot1.kP = 0.11;
        driveConfiguration.Slot1.kI = 0.0;
        driveConfiguration.Slot1.kD = 0.0;
        driveConfiguration.Slot1.kV = 12.0 / (Constants.kMaxFalconRotationsPerSecond * 0.95);
        driveMotor.getConfigurator().apply(driveConfiguration, Constants.kCANTimeoutMs);
        driveMotor.setRotorPosition(0.0);
    }

    @Override
    public void invertDriveMotor(TalonFXInvertType invertType) {
        driveConfiguration.MotorOutput.Inverted = (invertType == TalonFXInvertType.Clockwise) ? 
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    @Override
    public void invertRotationMotor(TalonFXInvertType invertType) {
		rotationMotor.setInverted(invertType);
    }

    @Override
    public void setDriveNeutralMode(NeutralMode mode) {
        driveConfiguration.MotorOutput.NeutralMode = (mode == NeutralMode.Brake) ?
                NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    @Override
    public void setRotationNeutralMode(NeutralMode mode) {
		rotationMotor.setNeutralMode(mode);
    }

    @Override
    public boolean angleOnTarget() {
		double error = encUnitsToDegrees(Math.abs(rotationMotor.getClosedLoopError(0)));
		return error < Constants.kSwerveModuleRotationTolerance;
    }

    @Override
    protected void setDriveProfileSlot(int slotIndex) {
        // Not necessary with Phoenix Pro
    }

    @Override
    public boolean drivePositionOnTarget() {
        if (currentDriveControlRequest.equals(driveMotionMagicRequest)) {
            return encUnitsToInches(Math.abs(periodicIO.driveDemand - periodicIO.drivePosition)) < Constants.kSwerveMotionMagicTolerance;
        }
        return false;
    }

    @Override
    protected ErrorCode setRotationSensorPosition(double sensorPosition) {
        return rotationMotor.setSelectedSensorPosition(sensorPosition);
    }

    @Override
    public double getDriveVoltage() {
        return driveMotor.getSupplyVoltage().getValue();
    }

    private double rotationsPerSecondToEncUnitsPer100Ms(double rps) {
        return rps * kFalconSensorResolution / 10.0;
    }

    private double encUnitsPer100MsToRotationsPerSecond(double encUnitsPer100Ms) {
        return encUnitsPer100Ms / kFalconSensorResolution * 10.0;
    }
    
    @Override
    public void readPeriodicInputs() {
		periodicIO.rotationPosition = rotationMotor.getSelectedSensorPosition(0);
        periodicIO.drivePosition = driveMotor.getRotorPosition().getValue() * kFalconSensorResolution;
        periodicIO.velocity = rotationsPerSecondToEncUnitsPer100Ms(driveMotor.getRotorVelocity().getValue());
		if (RobotBase.isReal()) {
            periodicIO.absoluteRotation = rotationAbsoluteEncoder.getOutput() * 360.0;
		}
    }

    @Override
    public void writePeriodicOutputs() {
		rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);

        switch (periodicIO.driveControlMode) {
            case PercentOutput:
                driveVoltageOutRequest.Output = periodicIO.driveDemand * 12.0;
                currentDriveControlRequest = driveVoltageOutRequest;
                break;
            case MotionMagic:
                driveMotionMagicRequest.Position = periodicIO.driveDemand / kFalconSensorResolution;
                currentDriveControlRequest = driveMotionMagicRequest;
                break;
            case Velocity:
                driveVelocityRequest.Velocity = encUnitsPer100MsToRotationsPerSecond(periodicIO.driveDemand);
                currentDriveControlRequest = driveVelocityRequest;
                break;
            default:
                System.out.println("Unexpected drive control mode! " + periodicIO.driveControlMode.toString());
                break;
        }
        driveMotor.setControl(currentDriveControlRequest);
    }
}
