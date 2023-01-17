package com.team1323.frc2023.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class PhoenixProSwerveModule extends SwerveModule {
    private static final double kFalconSensorResolution = 2048.0;

    private final TalonFX rotationMotor, driveMotor;

    private final TalonFXConfiguration rotationConfiguration = new TalonFXConfiguration();
    private final VoltageOut rotationVoltageOutRequest = new VoltageOut(0.0, true, false);
    private final MotionMagicVoltage rotationMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false);
    private ControlRequest currentRotationControlRequest = rotationVoltageOutRequest;

    private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private final VoltageOut driveVoltageOutRequest = new VoltageOut(0.0, true, false);
    private final MotionMagicVoltage driveMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0, true, 0.0, 1, false);
    private ControlRequest currentDriveControlRequest = driveVoltageOutRequest;

    public PhoenixProSwerveModule(int rotationPort, int drivePort, int moduleId, 
            double encoderOffset, Translation2d startingPose, boolean flipAbsoluteEncoder) {
        super(moduleId, encoderOffset, startingPose, flipAbsoluteEncoder);
        rotationMotor = new TalonFX(rotationPort);
        driveMotor = new TalonFX(drivePort);
        configureMotors();
    }

    @Override
    protected void configureMotors() {
        rotationConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rotationConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotationConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        rotationConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.kSwerveRotationMaxSpeed;
        rotationConfiguration.MotionMagic.MotionMagicAcceleration = Constants.kSwerveRotationMaxAcceleration;
        rotationConfiguration.Slot0.kP = 1.55;
        rotationConfiguration.Slot0.kI = 0.0;
        rotationConfiguration.Slot0.kD = 5.0;
        rotationConfiguration.Slot0.kV = 1.0 / Constants.kMaxFalconRotationsPerSecond;
        rotationMotor.getConfigurator().apply(rotationConfiguration, Constants.kCANTimeoutMs);
		resetRotationToAbsolute();
		if(!isRotationSensorConnected()){
			DriverStation.reportError(name + "rotation encoder not detected!", false);
			hasEmergency = true;
		}

        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
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
        driveConfiguration.Slot1.kV = 1.0 / Constants.kMaxFalconRotationsPerSecond;
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
        rotationConfiguration.MotorOutput.Inverted = (invertType == TalonFXInvertType.Clockwise) ? 
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        rotationMotor.getConfigurator().apply(rotationConfiguration);
    }

    @Override
    public void setDriveNeutralMode(NeutralMode mode) {
        driveConfiguration.MotorOutput.NeutralMode = (mode == NeutralMode.Brake) ?
                NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    @Override
    public void setRotationNeutralMode(NeutralMode mode) {
        rotationConfiguration.MotorOutput.NeutralMode = (mode == NeutralMode.Brake) ?
                NeutralModeValue.Brake : NeutralModeValue.Coast;
        rotationMotor.getConfigurator().apply(rotationConfiguration);
    }

    @Override
    public boolean angleOnTarget() {
        double error = rotationMotor.getClosedLoopError().getValue();
        error = Math.abs(error) * 360.0;
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
        rotationMotor.setRotorPosition(sensorPosition / kFalconSensorResolution);
        return ErrorCode.OK;
    }

    private double rotationsPerSecondToEncUnitsPer100Ms(double rps) {
        return rps * kFalconSensorResolution / 10.0;
    }

    private double encUnitsPer100MsToRotationsPerSecond(double encUnitsPer100Ms) {
        return encUnitsPer100Ms / kFalconSensorResolution * 10.0;
    }
    
    @Override
    public void readPeriodicInputs() {
        periodicIO.rotationPosition = rotationMotor.getRotorPosition().getValue() * kFalconSensorResolution;
        periodicIO.drivePosition = driveMotor.getRotorPosition().getValue() * kFalconSensorResolution;
        periodicIO.velocity = rotationsPerSecondToEncUnitsPer100Ms(driveMotor.getRotorVelocity().getValue());
		if (RobotBase.isReal()) {
            periodicIO.absoluteRotation = rotationAbsoluteEncoder.getOutput() * 360.0;
		}
    }

    @Override
    public void writePeriodicOutputs() {
        switch (periodicIO.rotationControlMode) {
            case PercentOutput:
                rotationVoltageOutRequest.Output = periodicIO.rotationDemand * 7.0;
                currentRotationControlRequest = rotationVoltageOutRequest;
                break;
            case MotionMagic:
                rotationMotionMagicRequest.Position = periodicIO.rotationDemand / kFalconSensorResolution;
                currentRotationControlRequest = rotationMotionMagicRequest;
                break;
            default:
                System.out.println("Unexpected rotation control mode! " + periodicIO.rotationControlMode.toString());
                break;
        }

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

        rotationMotor.setControl(currentRotationControlRequest);
        driveMotor.setControl(currentDriveControlRequest);
    }
}
